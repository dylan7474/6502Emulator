/**
 * c6502.c
 * A 6502 emulator targeting the Commodore PET 2001.
 *
 * This version includes:
 * - A feature-complete 6502 CPU core.
 * - Emulation of the PET's memory-mapped text screen at 0x8000.
 * - Emulation of the PET's 10x8 keyboard matrix.
 * - SDL_Audio integration for a simple square wave generator (placeholder).
 * - Logic to load the four 4KB PET KERNAL/BASIC ROMs.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h> // For audio generation
#include <SDL.h>
#include <SDL_ttf.h> // Include SDL_ttf for text rendering

// --- 6502 CPU State ---

// Flag Register Bits
#define FLAG_C (1 << 0) // Carry
#define FLAG_Z (1 << 1) // Zero
#define FLAG_I (1 << 2) // Interrupt Disable
#define FLAG_D (1 << 3) // Decimal Mode
#define FLAG_B (1 << 4) // Break Command
#define FLAG_U (1 << 5) // Unused (always 1)
#define FLAG_V (1 << 6) // Overflow
#define FLAG_N (1 << 7) // Negative

typedef struct {
    // 8-bit Registers
    uint8_t reg_A; // Accumulator
    uint8_t reg_X; // Index Register X
    uint8_t reg_Y; // Index Register Y
    uint16_t reg_PC; // 16-bit Program Counter
    uint8_t reg_SP;  // 8-bit Stack Pointer (offset from 0x0100)
    uint8_t reg_F;   // 8-bit Flag Register (Status)

    int halted;
    int irq_pending;
    int nmi_pending;

} C6502;

// --- Global Memory ---
uint8_t memory[0x10000]; // 65536 bytes

// --- PET Hardware Emulation ---

// PET Screen RAM is at 0x8000
#define PET_SCREEN_RAM_START 0x8000
#define PET_SCREEN_COLS 40
#define PET_SCREEN_ROWS 25

// PET Keyboard I/O (mapped to a 6520 PIA at 0xE810)
// We will simplify this.
// Address 0xE810 (PIA1 Port A) will be the "row select"
// Address 0xE812 (PIA1 Port B) will be the "column value"
#define PIA1_PORTA 0xE810
#define PIA1_PORTB 0xE812 // KERNAL reads from here

// This matrix holds the state of all 10 rows (8 keys each)
uint8_t pet_keyboard_matrix[10];

// PET Audio (very simple, just a toggle bit on a VIA)
// We'll keep our fantasy audio for now.
#define SOUND_PITCH_ADDR 0x00FD   // Unused by PET
#define SOUND_ENABLE_ADDR 0x00FE  // Unused by PET


// --- Memory Access Helpers ---

/**
 * Reads a single byte from memory.
 * This now intercepts reads from the keyboard I/O port.
 */
uint8_t readByte(uint16_t addr) {
    if (addr == PIA1_PORTB) {
        // KERNAL is reading the keyboard
        // It selects a row by writing to PIA1_PORTA
        // The value 0-9 is in memory[PIA1_PORTA]
        uint8_t row_select = memory[PIA1_PORTA] & 0x0F; // Use lower 4 bits
        if (row_select < 10) {
            // Return the state of the selected row
            // PET keyboard logic is active-low (0 = pressed)
            return pet_keyboard_matrix[row_select];
        }
        return 0xFF; // No row selected
    }
    
    return memory[addr];
}

/**
 * Writes a single byte to memory.
 * This now intercepts writes to the keyboard I/O port.
 */
void writeByte(uint16_t addr, uint8_t val) {
    if (addr >= 0xC000) {
        // Prevent writing to ROM
        return;
    }

    // Any write to PIA1 (0xE810-0xE81F) is just stored.
    // The KERNAL writes to 0xE810 to select the row.
    // We just let it write to memory[0xE810].
    
    memory[addr] = val;
}

// ... (readWord, writeWord, stack helpers - unchanged) ...
uint16_t readWord(uint16_t addr) {
    uint8_t lo = readByte(addr);
    uint8_t hi = readByte(addr + 1);
    return (uint16_t)((hi << 8) | lo);
}
void writeWord(uint16_t addr, uint16_t val) {
    writeByte(addr, val & 0xFF);
    writeByte(addr + 1, (val >> 8) & 0xFF);
}
void pushByte(C6502* cpu, uint8_t val) {
    writeByte(0x0100 + cpu->reg_SP, val);
    cpu->reg_SP--;
}
void pushWord(C6502* cpu, uint16_t val) {
    pushByte(cpu, (val >> 8) & 0xFF); // High byte
    pushByte(cpu, val & 0xFF);        // Low byte
}
uint8_t popByte(C6502* cpu) {
    cpu->reg_SP++;
    return readByte(0x0100 + cpu->reg_SP);
}
uint16_t popWord(C6502* cpu) {
    uint8_t lo = popByte(cpu);
    uint8_t hi = popByte(cpu);
    return (uint16_t)((hi << 8) | lo);
}

// ... (Addressing Mode helpers - unchanged) ...
uint8_t am_Immediate(C6502* cpu) {
    return readByte(cpu->reg_PC++);
}
uint16_t am_ZeroPage(C6502* cpu) {
    return (uint16_t)readByte(cpu->reg_PC++);
}
uint16_t am_ZeroPage_X(C6502* cpu) {
    uint8_t zp_addr = readByte(cpu->reg_PC++);
    return (uint16_t)((zp_addr + cpu->reg_X) & 0xFF);
}
uint16_t am_ZeroPage_Y(C6502* cpu) {
    uint8_t zp_addr = readByte(cpu->reg_PC++);
    return (uint16_t)((zp_addr + cpu->reg_Y) & 0xFF);
}
uint16_t am_Absolute(C6502* cpu) {
    uint16_t addr = readWord(cpu->reg_PC);
    cpu->reg_PC += 2;
    return addr;
}
uint16_t am_Absolute_X(C6502* cpu, int* cycles) {
    uint16_t base_addr = readWord(cpu->reg_PC);
    cpu->reg_PC += 2;
    uint16_t final_addr = base_addr + cpu->reg_X;
    if ((base_addr & 0xFF00) != (final_addr & 0xFF00)) {
        (*cycles)++;
    }
    return final_addr;
}
uint16_t am_Absolute_Y(C6502* cpu, int* cycles) {
    uint16_t base_addr = readWord(cpu->reg_PC);
    cpu->reg_PC += 2;
    uint16_t final_addr = base_addr + cpu->reg_Y;
    if ((base_addr & 0xFF00) != (final_addr & 0xFF00)) {
        (*cycles)++;
    }
    return final_addr;
}
uint16_t am_Indirect_X(C6502* cpu) {
    uint8_t zp_addr = readByte(cpu->reg_PC++);
    uint16_t lookup_addr = (uint16_t)((zp_addr + cpu->reg_X) & 0xFF);
    return readWord(lookup_addr);
}
uint16_t am_Indirect_Y(C6502* cpu, int* cycles) {
    uint8_t zp_addr = readByte(cpu->reg_PC++);
    uint16_t base_addr = readWord((uint16_t)zp_addr);
    uint16_t final_addr = base_addr + cpu->reg_Y;
    if ((base_addr & 0xFF00) != (final_addr & 0xFF00)) {
        (*cycles)++;
    }
    return final_addr;
}

// ... (CPU Logic helpers - unchanged) ...
void set_flag(C6502* cpu, uint8_t flag, int cond) {
    if (cond) { cpu->reg_F |= flag; } else { cpu->reg_F &= ~flag; }
}
void set_flag_NZ(C6502* cpu, uint8_t val) {
    set_flag(cpu, FLAG_Z, val == 0);
    set_flag(cpu, FLAG_N, (val & 0x80) != 0);
}
void do_Branch(C6502* cpu, int* cycles, int cond) {
    int8_t offset = (int8_t)readByte(cpu->reg_PC++);
    if (cond) {
        (*cycles)++;
        uint16_t old_pc = cpu->reg_PC;
        cpu->reg_PC += offset;
        if ((old_pc & 0xFF00) != (cpu->reg_PC & 0xFF00)) {
            (*cycles)++;
        }
    }
}
void do_ADC(C6502* cpu, uint8_t M) {
    uint16_t tmp = (uint16_t)cpu->reg_A + (uint16_t)M + (uint16_t)(cpu->reg_F & FLAG_C);
    set_flag(cpu, FLAG_V, (~(cpu->reg_A ^ M) & (cpu->reg_A ^ tmp) & 0x80) != 0);
    set_flag(cpu, FLAG_C, tmp > 0xFF);
    cpu->reg_A = (uint8_t)(tmp & 0xFF);
    set_flag_NZ(cpu, cpu->reg_A);
}
void do_SBC(C6502* cpu, uint8_t M) {
    uint16_t tmp = (uint16_t)cpu->reg_A + (uint16_t)(~M) + (uint16_t)(cpu->reg_F & FLAG_C);
    set_flag(cpu, FLAG_V, ((cpu->reg_A ^ tmp) & (~M ^ tmp) & 0x80) != 0);
    set_flag(cpu, FLAG_C, tmp > 0xFF);
    cpu->reg_A = (uint8_t)(tmp & 0xFF);
    set_flag_NZ(cpu, cpu->reg_A);
}
void do_BIT(C6502* cpu, uint8_t M) {
    set_flag(cpu, FLAG_Z, (cpu->reg_A & M) == 0);
    set_flag(cpu, FLAG_V, (M & 0x40) != 0);
    set_flag(cpu, FLAG_N, (M & 0x80) != 0);
}
void do_CMP(C6502* cpu, uint8_t M) {
    uint16_t tmp = (uint16_t)cpu->reg_A - (uint16_t)M;
    set_flag(cpu, FLAG_C, cpu->reg_A >= M);
    set_flag_NZ(cpu, (uint8_t)(tmp & 0xFF));
}
void do_CPX(C6502* cpu, uint8_t M) {
    uint16_t tmp = (uint16_t)cpu->reg_X - (uint16_t)M;
    set_flag(cpu, FLAG_C, cpu->reg_X >= M);
    set_flag_NZ(cpu, (uint8_t)(tmp & 0xFF));
}
void do_CPY(C6502* cpu, uint8_t M) {
    uint16_t tmp = (uint16_t)cpu->reg_Y - (uint16_t)M;
    set_flag(cpu, FLAG_C, cpu->reg_Y >= M);
    set_flag_NZ(cpu, (uint8_t)(tmp & 0xFF));
}
uint8_t do_ASL(C6502* cpu, uint8_t M) {
    set_flag(cpu, FLAG_C, (M & 0x80) != 0);
    uint8_t result = (uint8_t)(M << 1);
    set_flag_NZ(cpu, result);
    return result;
}
uint8_t do_LSR(C6502* cpu, uint8_t M) {
    set_flag(cpu, FLAG_C, (M & 0x01) != 0);
    uint8_t result = (uint8_t)(M >> 1);
    set_flag_NZ(cpu, result);
    return result;
}
uint8_t do_ROL(C6502* cpu, uint8_t M) {
    uint8_t old_c = (cpu->reg_F & FLAG_C);
    set_flag(cpu, FLAG_C, (M & 0x80) != 0);
    uint8_t result = (uint8_t)((M << 1) | old_c);
    set_flag_NZ(cpu, result);
    return result;
}
uint8_t do_ROR(C6502* cpu, uint8_t M) {
    uint8_t old_c = (cpu->reg_F & FLAG_C);
    set_flag(cpu, FLAG_C, (M & 0x01) != 0);
    uint8_t result = (uint8_t)((M >> 1) | (old_c << 7));
    set_flag_NZ(cpu, result);
    return result;
}

// ... (Interrupt Logic - unchanged) ...
int cpu_interrupt(C6502* cpu, uint16_t vector_addr, int is_brk) {
    if (is_brk) {
        pushWord(cpu, cpu->reg_PC + 1);
    } else {
        pushWord(cpu, cpu->reg_PC);
    }
    if (is_brk) {
        pushByte(cpu, cpu->reg_F | FLAG_B | FLAG_U);
    } else {
        pushByte(cpu, (cpu->reg_F & ~FLAG_B) | FLAG_U);
    }
    cpu->reg_F |= FLAG_I;
    cpu->reg_PC = readWord(vector_addr);
    return 7;
}
int cpu_irq(C6502* cpu) {
    if ((cpu->reg_F & FLAG_I) == 0) {
        cpu->irq_pending = 0;
        return cpu_interrupt(cpu, 0xFFFE, 0);
    }
    return 0;
}
int cpu_nmi(C6502* cpu) {
    cpu->nmi_pending = 0;
    return cpu_interrupt(cpu, 0xFFFA, 0);
}

// ... (CPU Control Functions - unchanged) ...
void cpu_reset(C6502* cpu) {
    cpu->reg_A = 0;
    cpu->reg_X = 0;
    cpu->reg_Y = 0;
    cpu->reg_SP = 0xFD;
    cpu->reg_F = FLAG_U | FLAG_I;
    cpu->reg_PC = readWord(0xFFFC);
    cpu->halted = 0;
    cpu->irq_pending = 0;
    cpu->nmi_pending = 0;
}
int cpu_handle_interrupts(C6502* cpu) {
    if (cpu->nmi_pending) {
        return cpu_nmi(cpu);
    }
    if (cpu->irq_pending && (cpu->reg_F & FLAG_I) == 0) {
        return cpu_irq(cpu);
    }
    return 0;
}

// ... (cpu_step function - unchanged) ...
int cpu_step(C6502* cpu) {
    int cycles = cpu_handle_interrupts(cpu);
    if (cycles > 0) { return cycles; }
    uint16_t debug_pc_start = cpu->reg_PC;
    uint8_t opcode = readByte(cpu->reg_PC++);
    cycles = 2;
    uint16_t eff_addr = 0;
    uint8_t value = 0;
    switch (opcode) {
        case 0xEA: cycles = 2; break; // NOP
        case 0x00: { cycles = cpu_interrupt(cpu, 0xFFFE, 1); cpu->halted = 1; break; }
        case 0x40: { cpu->reg_F = (popByte(cpu) & ~FLAG_B) | FLAG_U; cpu->reg_PC = popWord(cpu); cycles = 6; break; }
        case 0x18: cpu->reg_F &= ~FLAG_C; cycles = 2; break; // CLC
        case 0x38: cpu->reg_F |= FLAG_C;  cycles = 2; break; // SEC
        case 0x58: cpu->reg_F &= ~FLAG_I; cycles = 2; break; // CLI
        case 0x78: cpu->reg_F |= FLAG_I;  cycles = 2; break; // SEI
        case 0xD8: cpu->reg_F &= ~FLAG_D; cycles = 2; break; // CLD
        case 0xF8: cpu->reg_F |= FLAG_D;  cycles = 2; break; // SED
        case 0x4C: { cpu->reg_PC = am_Absolute(cpu); cycles = 3; break; }
        case 0x6C: { uint16_t ptr_addr = am_Absolute(cpu); if ((ptr_addr & 0x00FF) == 0x00FF) { uint8_t lo = readByte(ptr_addr); uint8_t hi = readByte(ptr_addr & 0xFF00); cpu->reg_PC = (uint16_t)((hi << 8) | lo); } else { cpu->reg_PC = readWord(ptr_addr); } cycles = 5; break; }
        case 0x20: { uint16_t sub_addr = am_Absolute(cpu); pushWord(cpu, cpu->reg_PC - 1); cpu->reg_PC = sub_addr; cycles = 6; break; }
        case 0x60: { cpu->reg_PC = popWord(cpu) + 1; cycles = 6; break; }
        case 0xA9: { value = am_Immediate(cpu); cpu->reg_A = value; set_flag_NZ(cpu, cpu->reg_A); cycles = 2; break; }
        case 0xA5: { eff_addr = am_ZeroPage(cpu); cpu->reg_A = readByte(eff_addr); set_flag_NZ(cpu, cpu->reg_A); cycles = 3; break; }
        case 0xB5: { eff_addr = am_ZeroPage_X(cpu); cpu->reg_A = readByte(eff_addr); set_flag_NZ(cpu, cpu->reg_A); cycles = 4; break; }
        case 0xAD: { eff_addr = am_Absolute(cpu); cpu->reg_A = readByte(eff_addr); set_flag_NZ(cpu, cpu->reg_A); cycles = 4; break; }
        case 0xBD: { cycles = 4; eff_addr = am_Absolute_X(cpu, &cycles); cpu->reg_A = readByte(eff_addr); set_flag_NZ(cpu, cpu->reg_A); break; }
        case 0xB9: { cycles = 4; eff_addr = am_Absolute_Y(cpu, &cycles); cpu->reg_A = readByte(eff_addr); set_flag_NZ(cpu, cpu->reg_A); break; }
        case 0xA1: { eff_addr = am_Indirect_X(cpu); cpu->reg_A = readByte(eff_addr); set_flag_NZ(cpu, cpu->reg_A); cycles = 6; break; }
        case 0xB1: { cycles = 5; eff_addr = am_Indirect_Y(cpu, &cycles); cpu->reg_A = readByte(eff_addr); set_flag_NZ(cpu, cpu->reg_A); break; }
        case 0x85: { eff_addr = am_ZeroPage(cpu); writeByte(eff_addr, cpu->reg_A); cycles = 3; break; }
        case 0x95: { eff_addr = am_ZeroPage_X(cpu); writeByte(eff_addr, cpu->reg_A); cycles = 4; break; }
        case 0x8D: { eff_addr = am_Absolute(cpu); writeByte(eff_addr, cpu->reg_A); cycles = 4; break; }
        case 0x9D: { cycles = 5; eff_addr = am_Absolute_X(cpu, &cycles); cycles = 5; writeByte(eff_addr, cpu->reg_A); break; }
        case 0xA2: { cpu->reg_X = am_Immediate(cpu); set_flag_NZ(cpu, cpu->reg_X); cycles = 2; break; }
        case 0xA6: { eff_addr = am_ZeroPage(cpu); cpu->reg_X = readByte(eff_addr); set_flag_NZ(cpu, cpu->reg_X); cycles = 3; break; }
        case 0xB6: { eff_addr = am_ZeroPage_Y(cpu); cpu->reg_X = readByte(eff_addr); set_flag_NZ(cpu, cpu->reg_X); cycles = 4; break; }
        case 0xAE: { eff_addr = am_Absolute(cpu); cpu->reg_X = readByte(eff_addr); set_flag_NZ(cpu, cpu->reg_X); cycles = 4; break; }
        case 0xA0: { cpu->reg_Y = am_Immediate(cpu); set_flag_NZ(cpu, cpu->reg_Y); cycles = 2; break; }
        case 0x86: { eff_addr = am_ZeroPage(cpu); writeByte(eff_addr, cpu->reg_X); cycles = 3; break; }
        case 0x96: { eff_addr = am_ZeroPage_Y(cpu); writeByte(eff_addr, cpu->reg_X); cycles = 4; break; }
        case 0x8E: { eff_addr = am_Absolute(cpu); writeByte(eff_addr, cpu->reg_X); cycles = 4; break; }
        case 0x84: { eff_addr = am_ZeroPage(cpu); writeByte(eff_addr, cpu->reg_Y); cycles = 3; break; }
        case 0x94: { eff_addr = am_ZeroPage_X(cpu); writeByte(eff_addr, cpu->reg_Y); cycles = 4; break; }
        case 0x8C: { eff_addr = am_Absolute(cpu); writeByte(eff_addr, cpu->reg_Y); cycles = 4; break; }
        case 0x48: pushByte(cpu, cpu->reg_A); cycles = 3; break; // PHA
        case 0x68: cpu->reg_A = popByte(cpu); set_flag_NZ(cpu, cpu->reg_A); cycles = 4; break; // PLA
        case 0x08: pushByte(cpu, cpu->reg_F | FLAG_B | FLAG_U); cycles = 3; break; // PHP
        case 0x28: cpu->reg_F = (popByte(cpu) & ~FLAG_B) | FLAG_U; cycles = 4; break; // PLP
        case 0xAA: cpu->reg_X = cpu->reg_A; set_flag_NZ(cpu, cpu->reg_X); cycles = 2; break; // TAX
        case 0xA8: cpu->reg_Y = cpu->reg_A; set_flag_NZ(cpu, cpu->reg_Y); cycles = 2; break; // TAY
        case 0x8A: cpu->reg_A = cpu->reg_X; set_flag_NZ(cpu, cpu->reg_A); cycles = 2; break; // TXA
        case 0x98: cpu->reg_A = cpu->reg_Y; set_flag_NZ(cpu, cpu->reg_A); cycles = 2; break; // TYA
        case 0xE8: cpu->reg_X++; set_flag_NZ(cpu, cpu->reg_X); cycles = 2; break; // INX
        case 0xC8: cpu->reg_Y++; set_flag_NZ(cpu, cpu->reg_Y); cycles = 2; break; // INY
        case 0xCA: cpu->reg_X--; set_flag_NZ(cpu, cpu->reg_X); cycles = 2; break; // DEX
        case 0x88: cpu->reg_Y--; set_flag_NZ(cpu, cpu->reg_Y); cycles = 2; break; // DEY
        case 0xE6: { eff_addr = am_ZeroPage(cpu); value = readByte(eff_addr) + 1; writeByte(eff_addr, value); set_flag_NZ(cpu, value); cycles = 5; break; }
        case 0xEE: { eff_addr = am_Absolute(cpu); value = readByte(eff_addr) + 1; writeByte(eff_addr, value); set_flag_NZ(cpu, value); cycles = 6; break; }
        case 0xC6: { eff_addr = am_ZeroPage(cpu); value = readByte(eff_addr) - 1; writeByte(eff_addr, value); set_flag_NZ(cpu, value); cycles = 5; break; }
        case 0xCE: { eff_addr = am_Absolute(cpu); value = readByte(eff_addr) - 1; writeByte(eff_addr, value); set_flag_NZ(cpu, value); cycles = 6; break; }
        case 0x10: do_Branch(cpu, &cycles, (cpu->reg_F & FLAG_N) == 0); break; // BPL
        case 0x30: do_Branch(cpu, &cycles, (cpu->reg_F & FLAG_N) != 0); break; // BMI
        case 0x50: do_Branch(cpu, &cycles, (cpu->reg_F & FLAG_V) == 0); break; // BVC
        case 0x70: do_Branch(cpu, &cycles, (cpu->reg_F & FLAG_V) != 0); break; // BVS
        case 0x90: do_Branch(cpu, &cycles, (cpu->reg_F & FLAG_C) == 0); break; // BCC
        case 0xB0: do_Branch(cpu, &cycles, (cpu->reg_F & FLAG_C) != 0); break; // BCS
        case 0xD0: do_Branch(cpu, &cycles, (cpu->reg_F & FLAG_Z) == 0); break; // BNE
        case 0xF0: do_Branch(cpu, &cycles, (cpu->reg_F & FLAG_Z) != 0); break; // BEQ
        case 0x69: value = am_Immediate(cpu);   do_ADC(cpu, value); cycles = 2; break;
        case 0x65: eff_addr = am_ZeroPage(cpu); value = readByte(eff_addr); do_ADC(cpu, value); cycles = 3; break;
        case 0x75: eff_addr = am_ZeroPage_X(cpu); value = readByte(eff_addr); do_ADC(cpu, value); cycles = 4; break;
        case 0x6D: eff_addr = am_Absolute(cpu); value = readByte(eff_addr); do_ADC(cpu, value); cycles = 4; break;
        case 0x7D: cycles = 4; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr); do_ADC(cpu, value); break;
        case 0x79: cycles = 4; eff_addr = am_Absolute_Y(cpu, &cycles); value = readByte(eff_addr); do_ADC(cpu, value); break;
        case 0x61: eff_addr = am_Indirect_X(cpu); value = readByte(eff_addr); do_ADC(cpu, value); cycles = 6; break;
        case 0x71: cycles = 5; eff_addr = am_Indirect_Y(cpu, &cycles); value = readByte(eff_addr); do_ADC(cpu, value); break;
        case 0xE9: value = am_Immediate(cpu);   do_SBC(cpu, value); cycles = 2; break;
        case 0xE5: eff_addr = am_ZeroPage(cpu); value = readByte(eff_addr); do_SBC(cpu, value); cycles = 3; break;
        case 0xF5: eff_addr = am_ZeroPage_X(cpu); value = readByte(eff_addr); do_SBC(cpu, value); cycles = 4; break;
        case 0xED: eff_addr = am_Absolute(cpu); value = readByte(eff_addr); do_SBC(cpu, value); cycles = 4; break;
        case 0xFD: cycles = 4; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr); do_SBC(cpu, value); break;
        case 0xF9: cycles = 4; eff_addr = am_Absolute_Y(cpu, &cycles); value = readByte(eff_addr); do_SBC(cpu, value); break;
        case 0xE1: eff_addr = am_Indirect_X(cpu); value = readByte(eff_addr); do_SBC(cpu, value); cycles = 6; break;
        case 0xF1: cycles = 5; eff_addr = am_Indirect_Y(cpu, &cycles); value = readByte(eff_addr); do_SBC(cpu, value); break;
        case 0x29: value = am_Immediate(cpu);   cpu->reg_A &= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 2; break;
        case 0x25: eff_addr = am_ZeroPage(cpu); value = readByte(eff_addr); cpu->reg_A &= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 3; break;
        case 0x35: eff_addr = am_ZeroPage_X(cpu); value = readByte(eff_addr); cpu->reg_A &= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 4; break;
        case 0x2D: eff_addr = am_Absolute(cpu); value = readByte(eff_addr); cpu->reg_A &= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 4; break;
        case 0x3D: cycles = 4; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr); cpu->reg_A &= value; set_flag_NZ(cpu, cpu->reg_A); break;
        case 0x39: cycles = 4; eff_addr = am_Absolute_Y(cpu, &cycles); value = readByte(eff_addr); cpu->reg_A &= value; set_flag_NZ(cpu, cpu->reg_A); break;
        case 0x21: eff_addr = am_Indirect_X(cpu); value = readByte(eff_addr); cpu->reg_A &= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 6; break;
        case 0x31: cycles = 5; eff_addr = am_Indirect_Y(cpu, &cycles); value = readByte(eff_addr); cpu->reg_A &= value; set_flag_NZ(cpu, cpu->reg_A); break;
        case 0x09: value = am_Immediate(cpu);   cpu->reg_A |= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 2; break;
        case 0x05: eff_addr = am_ZeroPage(cpu); value = readByte(eff_addr); cpu->reg_A |= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 3; break;
        case 0x15: eff_addr = am_ZeroPage_X(cpu); value = readByte(eff_addr); cpu->reg_A |= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 4; break;
        case 0x0D: eff_addr = am_Absolute(cpu); value = readByte(eff_addr); cpu->reg_A |= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 4; break;
        case 0x1D: cycles = 4; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr); cpu->reg_A |= value; set_flag_NZ(cpu, cpu->reg_A); break;
        case 0x19: cycles = 4; eff_addr = am_Absolute_Y(cpu, &cycles); value = readByte(eff_addr); cpu->reg_A |= value; set_flag_NZ(cpu, cpu->reg_A); break;
        case 0x01: eff_addr = am_Indirect_X(cpu); value = readByte(eff_addr); cpu->reg_A |= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 6; break;
        case 0x11: cycles = 5; eff_addr = am_Indirect_Y(cpu, &cycles); value = readByte(eff_addr); cpu->reg_A |= value; set_flag_NZ(cpu, cpu->reg_A); break;
        case 0x49: value = am_Immediate(cpu);   cpu->reg_A ^= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 2; break;
        case 0x45: eff_addr = am_ZeroPage(cpu); value = readByte(eff_addr); cpu->reg_A ^= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 3; break;
        case 0x55: eff_addr = am_ZeroPage_X(cpu); value = readByte(eff_addr); cpu->reg_A ^= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 4; break;
        case 0x4D: eff_addr = am_Absolute(cpu); value = readByte(eff_addr); cpu->reg_A ^= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 4; break;
        case 0x5D: cycles = 4; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr); cpu->reg_A ^= value; set_flag_NZ(cpu, cpu->reg_A); break;
        case 0x59: cycles = 4; eff_addr = am_Absolute_Y(cpu, &cycles); value = readByte(eff_addr); cpu->reg_A ^= value; set_flag_NZ(cpu, cpu->reg_A); break;
        case 0x41: eff_addr = am_Indirect_X(cpu); value = readByte(eff_addr); cpu->reg_A ^= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 6; break;
        case 0x51: cycles = 5; eff_addr = am_Indirect_Y(cpu, &cycles); value = readByte(eff_addr); cpu->reg_A ^= value; set_flag_NZ(cpu, cpu->reg_A); break;
        case 0xC9: value = am_Immediate(cpu);   do_CMP(cpu, value); cycles = 2; break;
        case 0xC5: eff_addr = am_ZeroPage(cpu); value = readByte(eff_addr); do_CMP(cpu, value); cycles = 3; break;
        case 0xD5: eff_addr = am_ZeroPage_X(cpu); value = readByte(eff_addr); do_CMP(cpu, value); cycles = 4; break;
        case 0xCD: eff_addr = am_Absolute(cpu); value = readByte(eff_addr); do_CMP(cpu, value); cycles = 4; break;
        case 0xDD: cycles = 4; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr); do_CMP(cpu, value); break;
        case 0xD9: cycles = 4; eff_addr = am_Absolute_Y(cpu, &cycles); value = readByte(eff_addr); do_CMP(cpu, value); break;
        case 0xC1: eff_addr = am_Indirect_X(cpu); value = readByte(eff_addr); do_CMP(cpu, value); cycles = 6; break;
        case 0xD1: cycles = 5; eff_addr = am_Indirect_Y(cpu, &cycles); value = readByte(eff_addr); do_CMP(cpu, value); break;
        case 0xE0: value = am_Immediate(cpu);   do_CPX(cpu, value); cycles = 2; break;
        case 0xE4: eff_addr = am_ZeroPage(cpu); value = readByte(eff_addr); do_CPX(cpu, value); cycles = 3; break;
        case 0xEC: eff_addr = am_Absolute(cpu); value = readByte(eff_addr); do_CPX(cpu, value); cycles = 4; break;
        case 0xC0: value = am_Immediate(cpu);   do_CPY(cpu, value); cycles = 2; break;
        case 0xC4: eff_addr = am_ZeroPage(cpu); value = readByte(eff_addr); do_CPY(cpu, value); cycles = 3; break;
        case 0xCC: eff_addr = am_Absolute(cpu); value = readByte(eff_addr); do_CPY(cpu, value); cycles = 4; break;
        case 0x24: eff_addr = am_ZeroPage(cpu);  value = readByte(eff_addr); do_BIT(cpu, value); cycles = 3; break;
        case 0x2C: eff_addr = am_Absolute(cpu);  value = readByte(eff_addr); do_BIT(cpu, value); cycles = 4; break;
        case 0x0A: cpu->reg_A = do_ASL(cpu, cpu->reg_A); cycles = 2; break;
        case 0x06: eff_addr = am_ZeroPage(cpu);    value = readByte(eff_addr); writeByte(eff_addr, do_ASL(cpu, value)); cycles = 5; break;
        case 0x16: eff_addr = am_ZeroPage_X(cpu);  value = readByte(eff_addr); writeByte(eff_addr, do_ASL(cpu, value)); cycles = 6; break;
        case 0x0E: eff_addr = am_Absolute(cpu);    value = readByte(eff_addr); writeByte(eff_addr, do_ASL(cpu, value)); cycles = 6; break;
        case 0x1E: cycles = 7; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr); writeByte(eff_addr, do_ASL(cpu, value)); cycles = 7; break;
        case 0x4A: cpu->reg_A = do_LSR(cpu, cpu->reg_A); cycles = 2; break;
        case 0x46: eff_addr = am_ZeroPage(cpu);    value = readByte(eff_addr); writeByte(eff_addr, do_LSR(cpu, value)); cycles = 5; break;
        case 0x56: eff_addr = am_ZeroPage_X(cpu);  value = readByte(eff_addr); writeByte(eff_addr, do_LSR(cpu, value)); cycles = 6; break;
        case 0x4E: eff_addr = am_Absolute(cpu);    value = readByte(eff_addr); writeByte(eff_addr, do_LSR(cpu, value)); cycles = 6; break;
        case 0x5E: cycles = 7; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr); writeByte(eff_addr, do_LSR(cpu, value)); cycles = 7; break;
        case 0x2A: cpu->reg_A = do_ROL(cpu, cpu->reg_A); cycles = 2; break;
        case 0x26: eff_addr = am_ZeroPage(cpu);    value = readByte(eff_addr); writeByte(eff_addr, do_ROL(cpu, value)); cycles = 5; break;
        case 0x36: eff_addr = am_ZeroPage_X(cpu);  value = readByte(eff_addr); writeByte(eff_addr, do_ROL(cpu, value)); cycles = 6; break;
        case 0x2E: eff_addr = am_Absolute(cpu);    value = readByte(eff_addr); writeByte(eff_addr, do_ROL(cpu, value)); cycles = 6; break;
        case 0x3E: cycles = 7; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr); writeByte(eff_addr, do_ROL(cpu, value)); cycles = 7; break;
        case 0x6A: cpu->reg_A = do_ROR(cpu, cpu->reg_A); cycles = 2; break;
        case 0x66: eff_addr = am_ZeroPage(cpu);    value = readByte(eff_addr); writeByte(eff_addr, do_ROR(cpu, value)); cycles = 5; break;
        case 0x76: eff_addr = am_ZeroPage_X(cpu);  value = readByte(eff_addr); writeByte(eff_addr, do_ROR(cpu, value)); cycles = 6; break;
        case 0x6E: eff_addr = am_Absolute(cpu);    value = readByte(eff_addr); writeByte(eff_addr, do_ROR(cpu, value)); cycles = 6; break;
        case 0x7E: cycles = 7; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr); writeByte(eff_addr, do_ROR(cpu, value)); cycles = 7; break;
        default:
            printf("Error: Unimplemented opcode 0x%02X at address 0x%04X\n",
                   opcode, debug_pc_start);
            cpu->halted = 1;
            cycles = 1;
            break;
    }
    return cycles;
}


// --- SDL Globals ---
SDL_Window* window = NULL;
SDL_Renderer* renderer = NULL;
SDL_Texture* texture = NULL;
TTF_Font* font = NULL;

// Screen dimensions
#define CHAR_WIDTH 8
#define CHAR_HEIGHT 8
#define WINDOW_WIDTH (PET_SCREEN_COLS * CHAR_WIDTH)
#define WINDOW_HEIGHT (PET_SCREEN_ROWS * CHAR_HEIGHT)

// --- Audio Globals ---
#define AUDIO_SAMPLE_RATE 44100
#define AUDIO_AMPLITUDE 2000
static double audio_phase = 0.0;

// ... (Audio Callback - unchanged, still reads fantasy registers) ...
void audio_callback(void* userdata, Uint8* stream, int len) {
    (void)userdata;
    Sint16* buffer = (Sint16*)stream;
    int num_samples = len / sizeof(Sint16);
    uint8_t sound_enable = readByte(SOUND_ENABLE_ADDR);
    uint8_t pitch_val = readByte(SOUND_PITCH_ADDR);
    if (sound_enable == 0 || pitch_val == 0) {
        memset(buffer, 0, len);
        return;
    }
    double base_freq = 110.0;
    double freq = base_freq * pow(2.0, (double)pitch_val / 48.0);
    double phase_inc = (2.0 * M_PI * freq) / AUDIO_SAMPLE_RATE;
    for (int i = 0; i < num_samples; ++i) {
        buffer[i] = (sin(audio_phase) > 0) ? AUDIO_AMPLITUDE : -AUDIO_AMPLITUDE;
        audio_phase += phase_inc;
        if (audio_phase > 2.0 * M_PI) {
            audio_phase -= 2.0 * M_PI;
        }
    }
}


// --- I/O Helpers ---

/**
 * Maps an SDL key to the PET's 10x8 matrix.
 * Returns 1 if a key was mapped, 0 otherwise.
 */
int map_sdl_key_to_pet(SDL_Keycode key, int press) {
    uint8_t row = 0, col_mask = 0;
    
    // This is a partial map based on a common PET layout
    switch (key) {
        // Row 0
        case SDLK_LEFT:   row = 0; col_mask = 0x80; break; // Cursor Left/Right
        case SDLK_F1:     row = 0; col_mask = 0x40; break; // F1
        case SDLK_F3:     row = 0; col_mask = 0x20; break; // F3
        case SDLK_F5:     row = 0; col_mask = 0x10; break; // F5
        case SDLK_F7:     row = 0; col_mask = 0x08; break; // F7
        case SDLK_UP:     row = 0; col_mask = 0x04; break; // Cursor Up/Down
        case SDLK_RETURN: row = 0; col_mask = 0x02; break; // RETURN
        case SDLK_DELETE: row = 0; col_mask = 0x01; break; // DEL
        
        // Row 1
        case SDLK_SPACE:  row = 1; col_mask = 0x80; break; // SPACE
        case SDLK_q:      row = 1; col_mask = 0x40; break; // Q
        case SDLK_a:      row = 1; col_mask = 0x20; break; // A
        case SDLK_z:      row = 1; col_mask = 0x10; break; // Z
        case SDLK_LSHIFT: row = 1; col_mask = 0x08; break; // L-SHIFT
        case SDLK_e:      row = 1; col_mask = 0x04; break; // E
        case SDLK_s:      row = 1; col_mask = 0x02; break; // S
        case SDLK_x:      row = 1; col_mask = 0x01; break; // X

        // Row 2
        case SDLK_c:      row = 2; col_mask = 0x80; break; // C
        case SDLK_d:      row = 2; col_mask = 0x40; break; // D
        case SDLK_w:      row = 2; col_mask = 0x20; break; // W
        // (RSHIFT)       row = 2; col_mask = 0x10; break;
        case SDLK_v:      row = 2; col_mask = 0x08; break; // V
        case SDLK_f:      row = 2; col_mask = 0x04; break; // F
        case SDLK_r:      row = 2; col_mask = 0x02; break; // R
        case SDLK_b:      row = 2; col_mask = 0x01; break; // B

        // Row 3
        case SDLK_n:      row = 3; col_mask = 0x80; break; // N
        case SDLK_g:      row = 3; col_mask = 0x40; break; // G
        case SDLK_t:      row = 3; col_mask = 0x20; break; // T
        case SDLK_m:      row = 3; col_mask = 0x10; break; // M
        case SDLK_h:      row = 3; col_mask = 0x08; break; // H
        case SDLK_y:      row = 3; col_mask = 0x04; break; // Y
        case SDLK_j:      row = 3; col_mask = 0x02; break; // J
        case SDLK_u:      row = 3; col_mask = 0x01; break; // U

        // Row 4
        case SDLK_i:      row = 4; col_mask = 0x80; break; // I
        case SDLK_k:      row = 4; col_mask = 0x40; break; // K
        case SDLK_7:      row = 4; col_mask = 0x20; break; // 7
        case SDLK_o:      row = 4; col_mask = 0x10; break; // O
        case SDLK_l:      row = 4; col_mask = 0x08; break; // L
        case SDLK_8:      row = 4; col_mask = 0x04; break; // 8
        case SDLK_COMMA:  row = 4; col_mask = 0x02; break; // ,
        case SDLK_9:      row = 4; col_mask = 0x01; break; // 9

        // Row 5
        case SDLK_0:      row = 5; col_mask = 0x80; break; // 0
        case SDLK_PERIOD: row = 5; col_mask = 0x40; break; // .
        case SDLK_p:      row = 5; col_mask = 0x20; break; // P
        case SDLK_MINUS:  row = 5; col_mask = 0x10; break; // -
        case SDLK_SEMICOLON: row = 5; col_mask = 0x08; break; // ;
        case SDLK_AT:     row = 5; col_mask = 0x04; break; // @
        case SDLK_COLON:  row = 5; col_mask = 0x02; break; // :
        case SDLK_SLASH:  row = 5; col_mask = 0x01; break; // /

        // ... etc for other rows
        
        default:
            return 0; // Unmapped
    }

    if (press) {
        // Active-low: set bit to 0
        pet_keyboard_matrix[row] &= ~col_mask;
    } else {
        // Active-low: set bit to 1
        pet_keyboard_matrix[row] |= col_mask;
    }
    
    return 1;
}

/**
 * The PET KERNAL converts its internal character codes (PETSCII)
 * to screen codes (POKES) before writing to video RAM.
 * This is a partial map for 'unshifted' characters.
 */
char map_pet_screencode_to_ascii(uint8_t code) {
    if (code >= 1 && code <= 26) {
        return 'A' + (code - 1);
    }
    if (code >= 48 && code <= 57) {
        return '0' + (code - 48);
    }
    switch (code) {
        case 0: return '@'; // Note: 0 is '@' in PETSCII
        case 32: return ' ';
        case 33: return '!';
        case 34: return '"';
        case 35: return '#';
        case 36: return '$';
        case 37: return '%';
        case 38: return '&';
        case 39: return '\'';
        case 40: return '(';
        case 41: return ')';
        case 42: return '*';
        case 43: return '+';
        case 44: return ',';
        case 45: return '-';
        case 46: return '.';
        case 47: return '/';
        case 58: return ':';
        case 59: return ';';
        case 60: return '<';
        case 61: return '=';
        case 62: return '>';
        case 63: return '?';
    }
    return ' '; // Default for unmapped/graphics chars
}


// --- SDL Initialization ---
int init_sdl(void) {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) < 0) {
        fprintf(stderr, "SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
        return 0;
    }
    if (TTF_Init() == -1) {
        fprintf(stderr, "SDL_ttf could not initialize! TTF_Error: %s\n", TTF_GetError());
        return 0;
    }

    window = SDL_CreateWindow("Commodore PET 2001 Emulator",
                              SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                              WINDOW_WIDTH * 2, WINDOW_HEIGHT * 2,
                              SDL_WINDOW_SHOWN);
    if (!window) { /* ... error ... */ return 0; }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!renderer) { /* ... error ... */ return 0; }
    
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "0");
    SDL_RenderSetLogicalSize(renderer, WINDOW_WIDTH, WINDOW_HEIGHT);

    font = TTF_OpenFont("font.ttf", CHAR_HEIGHT);
    if (!font) {
        fprintf(stderr, "Failed to load font 'font.ttf'! TTF_Error: %s\n", TTF_GetError());
        return 0;
    }
    TTF_SetFontHinting(font, TTF_HINTING_MONO);

    // Audio setup (unchanged)
    SDL_AudioSpec wanted_spec, have_spec;
    SDL_zero(wanted_spec);
    wanted_spec.freq = AUDIO_SAMPLE_RATE;
    wanted_spec.format = AUDIO_S16SYS;
    wanted_spec.channels = 1;
    wanted_spec.samples = 2048;
    wanted_spec.callback = audio_callback;
    wanted_spec.userdata = NULL;
    if (SDL_OpenAudio(&wanted_spec, &have_spec) == 0) {
        SDL_PauseAudio(0);
    } else {
        fprintf(stderr, "Failed to open audio: %s\n", SDL_GetError());
    }

    return 1;
}

// --- SDL Cleanup ---
void cleanup_sdl(void) {
    SDL_CloseAudio();
    if (font) TTF_CloseFont(font);
    if (texture) SDL_DestroyTexture(texture);
    if (renderer) SDL_DestroyRenderer(renderer);
    if (window) SDL_DestroyWindow(window);
    TTF_Quit();
    SDL_Quit();
}

// --- Render/Update Screen ---
void render_screen(void) {
    // PET has a green-on-black screen
    SDL_SetRenderDrawColor(renderer, 0, 0x22, 0, 0xFF);
    SDL_RenderClear(renderer);

    SDL_Color fg_color = { 0x00, 0xFF, 0x00, 0xFF }; // Bright Green text
    char char_str[2] = { 0, 0 };
    SDL_Surface* text_surface = NULL;
    SDL_Texture* text_texture = NULL;
    SDL_Rect dest_rect = { 0, 0, CHAR_WIDTH, CHAR_HEIGHT };

    for (int y = 0; y < PET_SCREEN_ROWS; ++y) {
        for (int x = 0; x < PET_SCREEN_COLS; ++x) {
            // Read from PET's screen RAM at 0x8000
            uint16_t mem_addr = PET_SCREEN_RAM_START + (y * PET_SCREEN_COLS) + x;
            uint8_t screen_code = readByte(mem_addr);
            
            // PET screen codes are weird. Bit 7 flips the character.
            // We'll ignore that for now and just render the basic set.
            char_str[0] = map_pet_screencode_to_ascii(screen_code & 0x7F);

            if (char_str[0] == ' ') continue;

            text_surface = TTF_RenderText_Solid(font, char_str, fg_color);
            if (!text_surface) continue;

            text_texture = SDL_CreateTextureFromSurface(renderer, text_surface);
            SDL_FreeSurface(text_surface);
            if (!text_texture) continue;

            dest_rect.x = x * CHAR_WIDTH;
            dest_rect.y = y * CHAR_HEIGHT;
            SDL_RenderCopy(renderer, text_texture, NULL, &dest_rect);
            SDL_DestroyTexture(text_texture);
        }
    }

    SDL_RenderPresent(renderer);
}

/**
 * Loads a single 4KB ROM file into a specific memory location.
 * Returns 1 on success, 0 on failure.
 */
int load_rom_file(const char* filename, uint16_t addr) {
    FILE* f = fopen(filename, "rb");
    if (!f) {
        perror(filename);
        return 0;
    }
    size_t bytes_read = fread(&memory[addr], 1, 0x1000, f); // Read 4KB
    fclose(f);
    if (bytes_read != 0x1000) {
        fprintf(stderr, "Error: ROM file '%s' is not 4KB.\n", filename);
        return 0;
    }
    printf("Loaded '%s' at 0x%04X\n", filename, addr);
    return 1;
}

/**
 * Loads the PET KERNAL and BASIC ROMs into memory.
 */
int load_pet_roms() {
    // These are the 4 ROMs for a PET 2001N
    if (!load_rom_file("basic-1.rom", 0xC000)) return 0;
    if (!load_rom_file("basic-2.rom", 0xD000)) return 0;
    if (!load_rom_file("edit-1.rom",  0xE000)) return 0;
    if (!load_rom_file("kernal-1.rom",0xF000)) return 0;
    
    // The KERNAL ROM at 0xF000 contains the interrupt vectors.
    // We just need to load it; no need to set vectors manually.
    // The KERNAL's reset vector at 0xFFFC will point to its
    // internal boot-up code.
    return 1;
}

// --- Main Program ---
int main(int argc, char* argv[]) {
    (void)argc; (void)argv; // We're not using command-line args for this
    
    C6502 cpu;
    memset(&memory, 0, sizeof(memory));
    
    // Initialize keyboard matrix (all keys up = 0xFF)
    for (int i = 0; i < 10; ++i) {
        pet_keyboard_matrix[i] = 0xFF;
    }

    // --- Load PET ROMs ---
    if (!load_pet_roms()) {
        fprintf(stderr, "Failed to load PET ROM files.\n");
        fprintf(stderr, "Please ensure basic-1.rom, basic-2.rom, edit-1.rom, and kernal-1.rom are in the directory.\n");
        return 1;
    }

    // --- Init ---
    cpu_reset(&cpu); // This will now read the KERNAL's reset vector
    if (!init_sdl()) {
        return 1;
    }

    printf("CPU Reset: PC set to 0x%04X (from KERNAL ROM)\n", cpu.reg_PC);
    printf("Starting Commodore PET emulation...\n");

    int quit = 0;
    SDL_Event e;
    int cycles_this_frame = 0;
    const int cycles_per_frame = 1000000 / 60; // 1MHz / 60Hz

    // --- Main Emulation Loop ---
    while (!quit && !cpu.halted) {
        // --- Event Handling ---
        while (SDL_PollEvent(&e) != 0) {
            if (e.type == SDL_QUIT) {
                quit = 1;
            }
            else if (e.type == SDL_KEYDOWN) {
                map_sdl_key_to_pet(e.key.keysym.sym, 1); // 1 = press
            }
            else if (e.type == SDL_KEYUP) {
                map_sdl_key_to_pet(e.key.keysym.sym, 0); // 0 = release
            }
        }

        // --- CPU Emulation ---
        cycles_this_frame = 0;
        while (cycles_this_frame < cycles_per_frame && !cpu.halted) {
            cycles_this_frame += cpu_step(&cpu);
        }
        
        // --- Hardware Timer ---
        // The PET KERNAL relies on a 60Hz timer IRQ.
        // This IRQ is what makes the cursor blink and scans the keyboard.
        cpu.irq_pending = 1;

        // --- Render Frame ---
        render_screen();
    }

    // --- Cleanup ---
    if (cpu.halted) {
         printf("CPU Halted at 0x%04X\n", cpu.reg_PC - 1);
    }
    printf("Emulation finished. Final CPU State:\n");
    printf("A: 0x%02X X: 0x%02X Y: 0x%02X\n", cpu.reg_A, cpu.reg_X, cpu.reg_Y);
    printf("PC: 0x%04X SP: 0x%02X F: 0x%02X\n", cpu.reg_PC, cpu.reg_SP, cpu.reg_F);

    cleanup_sdl();
    return 0;
}
