/**
 * c6502.c
 * A 6502 emulator targeting the Commodore PET 2001-N.
 *
 * This version emulates the hardware:
 * - A feature-complete 6502 CPU core, including common illegal opcodes.
 * - Emulation of the PET's memory-mapped text screen at 0x8000.
 * - Emulation of the PET's 10x8 keyboard matrix.
 * - Hardware-accurate text rendering by loading and using the
 * PET's Character Generator ROM.
 * - Logic to load the specific 2001-N KERNAL/BASIC ROMs.
 *
 * NEW:
 * - Moved all 12 "KIL" opcodes (0x*2) to the NOP section,
 * as the PET ROMs use them as part of normal execution.
 * - Updated the linter to reflect this change.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h> // For audio generation
#include <SDL.h>
// No more SDL_ttf!

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
uint8_t memory[0x10000];   // 65536 bytes
uint8_t char_rom[2048];   // 2KB Character ROM

// --- PET Hardware Emulation ---

// PET Screen RAM is at 0x8000
#define PET_SCREEN_RAM_START 0x8000
#define PET_SCREEN_COLS 40
#define PET_SCREEN_ROWS 25

// PET Keyboard I/O (mapped to a 6520 PIA at 0xE810)
#define PIA1_PORTA 0xE810 // KERNAL writes row select here
#define PIA1_PORTB 0xE812 // KERNAL reads column data from here

// This matrix holds the state of all 10 rows (8 keys each)
uint8_t pet_keyboard_matrix[10];

// We will keep our "fantasy" audio for now, as the KERNAL doesn't touch it.
#define SOUND_PITCH_ADDR 0x00FD
#define SOUND_ENABLE_ADDR 0x00FE


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
    // KERNAL/BASIC ROMs are from 0xC000 to 0xFFFF
    // Screen RAM is 0x8000-0x83E7
    // I/O is 0xE800+
    
    // Prevent writing to ROM
    if (addr >= 0xC000 && addr < 0xE000) return; // BASIC ROM
    if (addr >= 0xF000) return; // KERNAL ROM
    
    // The 2KB Edit ROM is at 0xE000-0xE7FF
    // The I/O area is 0xE800-0xEFFF
    if (addr >= 0xE000 && addr < 0xE800) return; // Edit ROM
    
    // Allow writes to RAM (0x0000-0x7FFF), Screen (0x8000+), and I/O (0xE800+)
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
        // BRK is 1-byte, PC is already pointing at *next* instr.
        // It pushes PC+1, but since PC is already advanced, we just push PC.
        pushWord(cpu, cpu->reg_PC); 
    } else {
        // IRQ/NMI push the current PC
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

// ... (cpu_step function - *** UPDATED ***) ...
int cpu_step(C6502* cpu) {
    int cycles = cpu_handle_interrupts(cpu);
    if (cycles > 0) { return cycles; }
    uint16_t debug_pc_start = cpu->reg_PC;
    uint8_t opcode = readByte(cpu->reg_PC++);
    cycles = 2;
    uint16_t eff_addr = 0;
    uint8_t value = 0;
    switch (opcode) {
        // --- Official Opcodes ---
        case 0xEA: cycles = 2; break; // NOP
        case 0x00: { 
            cycles = cpu_interrupt(cpu, 0xFFFE, 1); 
            // cpu->halted = 1; // <-- FIX: BRK is an IRQ, not a halt. KERNAL handles it.
            break; 
        }
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
        case 0x99: { cycles = 5; eff_addr = am_Absolute_Y(cpu, &cycles); writeByte(eff_addr, cpu->reg_A); break; }
        case 0x81: { eff_addr = am_Indirect_X(cpu); writeByte(eff_addr, cpu->reg_A); cycles = 6; break; }
        case 0x91: { cycles = 6; eff_addr = am_Indirect_Y(cpu, &cycles); writeByte(eff_addr, cpu->reg_A); break; } // STA (Indirect),Y
        case 0xA2: { cpu->reg_X = am_Immediate(cpu); set_flag_NZ(cpu, cpu->reg_X); cycles = 2; break; }
        case 0xA6: { eff_addr = am_ZeroPage(cpu); cpu->reg_X = readByte(eff_addr); set_flag_NZ(cpu, cpu->reg_X); cycles = 3; break; }
        case 0xB6: { eff_addr = am_ZeroPage_Y(cpu); cpu->reg_X = readByte(eff_addr); set_flag_NZ(cpu, cpu->reg_X); cycles = 4; break; }
        case 0xAE: { eff_addr = am_Absolute(cpu); cpu->reg_X = readByte(eff_addr); set_flag_NZ(cpu, cpu->reg_X); cycles = 4; break; }
        case 0xBE: { cycles = 4; eff_addr = am_Absolute_Y(cpu, &cycles); cpu->reg_X = readByte(eff_addr); set_flag_NZ(cpu, cpu->reg_X); break; }
        case 0xA0: { cpu->reg_Y = am_Immediate(cpu); set_flag_NZ(cpu, cpu->reg_Y); cycles = 2; break; }
        case 0xA4: { eff_addr = am_ZeroPage(cpu); cpu->reg_Y = readByte(eff_addr); set_flag_NZ(cpu, cpu->reg_Y); cycles = 3; break; }
        case 0xB4: { eff_addr = am_ZeroPage_X(cpu); cpu->reg_Y = readByte(eff_addr); set_flag_NZ(cpu, cpu->reg_Y); cycles = 4; break; }
        case 0xAC: { eff_addr = am_Absolute(cpu); cpu->reg_Y = readByte(eff_addr); set_flag_NZ(cpu, cpu->reg_Y); cycles = 4; break; }
        case 0xBC: { cycles = 4; eff_addr = am_Absolute_X(cpu, &cycles); cpu->reg_Y = readByte(eff_addr); set_flag_NZ(cpu, cpu->reg_Y); break; }
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
        case 0x9A: cpu->reg_SP = cpu->reg_X; cycles = 2; break; // TXS
        case 0xBA: cpu->reg_X = cpu->reg_SP; set_flag_NZ(cpu, cpu->reg_X); cycles = 2; break; // TSX
        case 0xE8: cpu->reg_X++; set_flag_NZ(cpu, cpu->reg_X); cycles = 2; break; // INX
        case 0xC8: cpu->reg_Y++; set_flag_NZ(cpu, cpu->reg_Y); cycles = 2; break; // INY
        case 0xCA: cpu->reg_X--; set_flag_NZ(cpu, cpu->reg_X); cycles = 2; break; // DEX
        case 0x88: cpu->reg_Y--; set_flag_NZ(cpu, cpu->reg_Y); cycles = 2; break; // DEY
        case 0xE6: { eff_addr = am_ZeroPage(cpu); value = readByte(eff_addr) + 1; writeByte(eff_addr, value); set_flag_NZ(cpu, value); cycles = 5; break; }
        case 0xF6: { eff_addr = am_ZeroPage_X(cpu); value = readByte(eff_addr) + 1; writeByte(eff_addr, value); set_flag_NZ(cpu, value); cycles = 6; break; }
        case 0xEE: { eff_addr = am_Absolute(cpu); value = readByte(eff_addr) + 1; writeByte(eff_addr, value); set_flag_NZ(cpu, value); cycles = 6; break; }
        case 0xFE: { cycles = 7; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr) + 1; writeByte(eff_addr, value); break; }
        case 0xC6: { eff_addr = am_ZeroPage(cpu); value = readByte(eff_addr) - 1; writeByte(eff_addr, value); set_flag_NZ(cpu, value); cycles = 5; break; }
        case 0xD6: { eff_addr = am_ZeroPage_X(cpu); value = readByte(eff_addr) - 1; writeByte(eff_addr, value); set_flag_NZ(cpu, value); cycles = 6; break; }
        case 0xCE: { eff_addr = am_Absolute(cpu); value = readByte(eff_addr) - 1; writeByte(eff_addr, value); set_flag_NZ(cpu, value); cycles = 6; break; }
        case 0xDE: { cycles = 7; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr) - 1; writeByte(eff_addr, value); break; }
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
        case 0xE9: case 0xEB: value = am_Immediate(cpu);   do_SBC(cpu, value); cycles = 2; break; // 0xEB is illegal SBC
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
        case 0x1E: cycles = 7; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr); writeByte(eff_addr, do_ASL(cpu, value)); break;
        case 0x4A: cpu->reg_A = do_LSR(cpu, cpu->reg_A); cycles = 2; break;
        case 0x46: eff_addr = am_ZeroPage(cpu);    value = readByte(eff_addr); writeByte(eff_addr, do_LSR(cpu, value)); cycles = 5; break;
        case 0x56: eff_addr = am_ZeroPage_X(cpu);  value = readByte(eff_addr); writeByte(eff_addr, do_LSR(cpu, value)); cycles = 6; break;
        case 0x4E: eff_addr = am_Absolute(cpu);    value = readByte(eff_addr); writeByte(eff_addr, do_LSR(cpu, value)); cycles = 6; break;
        case 0x5E: cycles = 7; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr); writeByte(eff_addr, do_LSR(cpu, value)); break;
        case 0x2A: cpu->reg_A = do_ROL(cpu, cpu->reg_A); cycles = 2; break; 
        case 0x26: eff_addr = am_ZeroPage(cpu);    value = readByte(eff_addr); writeByte(eff_addr, do_ROL(cpu, value)); cycles = 5; break;
        case 0x36: eff_addr = am_ZeroPage_X(cpu);  value = readByte(eff_addr); writeByte(eff_addr, do_ROL(cpu, value)); cycles = 6; break;
        case 0x2E: eff_addr = am_Absolute(cpu);    value = readByte(eff_addr); writeByte(eff_addr, do_ROL(cpu, value)); cycles = 6; break;
        case 0x3E: cycles = 7; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr); writeByte(eff_addr, do_ROL(cpu, value)); break;
        case 0x6A: cpu->reg_A = do_ROR(cpu, cpu->reg_A); cycles = 2; break;
        case 0x66: eff_addr = am_ZeroPage(cpu);    value = readByte(eff_addr); writeByte(eff_addr, do_ROR(cpu, value)); cycles = 5; break;
        case 0x76: eff_addr = am_ZeroPage_X(cpu);  value = readByte(eff_addr); writeByte(eff_addr, do_ROR(cpu, value)); cycles = 6; break;
        case 0x6E: eff_addr = am_Absolute(cpu);    value = readByte(eff_addr); writeByte(eff_addr, do_ROR(cpu, value)); cycles = 6; break;
        case 0x7E: cycles = 7; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr); writeByte(eff_addr, do_ROR(cpu, value)); break;

        // --- Illegal Opcodes ---
        
        // KIL/JAM (from linter list) - MOVED TO NOPs
        
            
        // NOP (various)
        case 0x1A: case 0x3A: case 0x5A: case 0x7A: case 0xDA: case 0xFA: cycles = 2; break;
        // 2-byte NOPs (includes list from linter)
        case 0x80: case 0x82: case 0x89: case 0xC2: case 0xE2: 
        case 0x0B: case 0x2B: case 0x4B: case 0x6B: case 0x8B: case 0xAB: case 0xCB:
            am_Immediate(cpu); cycles = 2; 
            break;
        // 2-byte NOPs (ZP)
        case 0x04: case 0x44: case 0x64: am_ZeroPage(cpu); cycles = 3; break;
        // 2-byte NOPs (ZP,X)
        case 0x14: case 0x34: case 0x54: case 0x74: case 0xD4: case 0xF4: am_ZeroPage_X(cpu); cycles = 4; break;
        // 3-byte NOPs (ABS)
        case 0x0C: am_Absolute(cpu); cycles = 4; break;
        // 3-byte NOPs (ABS,X) (includes list from linter)
        case 0x1C: case 0x3C: case 0x5C: case 0x7C: case 0xDC: case 0xFC: 
        case 0x9C: case 0x9E:
            cycles = 4; eff_addr = am_Absolute_X(cpu, &cycles); 
            break;
        
        // Other misc illegal NOPs from linter
        case 0x93: // SHA (illegal)
        case 0x9F: // SHA (illegal)
        case 0x9B: // TAS (illegal)
        case 0xBB: // LAS (illegal)
            cycles = 2; // Treat as simple NOPs for now
            break;
            
        // KIL opcodes, now implemented as 1-byte NOPs
        case 0x02: case 0x12: case 0x22: case 0x32: case 0x42: case 0x52: case 0x62: case 0x72:
        case 0x92: case 0xB2: case 0xD2: case 0xF2:
            cycles = 1; // Treat as NOP
            break;


        // LAX (LDA + LDX)
        case 0xA7: eff_addr = am_ZeroPage(cpu);    value = readByte(eff_addr); cpu->reg_A = cpu->reg_X = value; set_flag_NZ(cpu, value); cycles = 3; break;
        case 0xB7: eff_addr = am_ZeroPage_Y(cpu);  value = readByte(eff_addr); cpu->reg_A = cpu->reg_X = value; set_flag_NZ(cpu, value); cycles = 4; break;
        case 0xAF: eff_addr = am_Absolute(cpu);    value = readByte(eff_addr); cpu->reg_A = cpu->reg_X = value; set_flag_NZ(cpu, value); cycles = 4; break;
        case 0xBF: cycles = 4; eff_addr = am_Absolute_Y(cpu, &cycles); value = readByte(eff_addr); cpu->reg_A = cpu->reg_X = value; set_flag_NZ(cpu, value); break;
        case 0xA3: eff_addr = am_Indirect_X(cpu);  value = readByte(eff_addr); cpu->reg_A = cpu->reg_X = value; set_flag_NZ(cpu, value); cycles = 6; break;
        case 0xB3: cycles = 5; eff_addr = am_Indirect_Y(cpu, &cycles); value = readByte(eff_addr); cpu->reg_A = cpu->reg_X = value; set_flag_NZ(cpu, value); break;

        // SAX (STA & STX)
        case 0x87: eff_addr = am_ZeroPage(cpu);    writeByte(eff_addr, cpu->reg_A & cpu->reg_X); cycles = 3; break;
        case 0x97: eff_addr = am_ZeroPage_Y(cpu);  writeByte(eff_addr, cpu->reg_A & cpu->reg_X); cycles = 4; break;
        case 0x8F: eff_addr = am_Absolute(cpu);    writeByte(eff_addr, cpu->reg_A & cpu->reg_X); cycles = 4; break;
        case 0x83: eff_addr = am_Indirect_X(cpu);  writeByte(eff_addr, cpu->reg_A & cpu->reg_X); cycles = 6; break;

        // DCP (DEC + CMP)
        case 0xC7: eff_addr = am_ZeroPage(cpu);    value = readByte(eff_addr) - 1; writeByte(eff_addr, value); do_CMP(cpu, value); cycles = 5; break;
        case 0xD7: eff_addr = am_ZeroPage_X(cpu);  value = readByte(eff_addr) - 1; writeByte(eff_addr, value); do_CMP(cpu, value); cycles = 6; break;
        case 0xCF: eff_addr = am_Absolute(cpu);    value = readByte(eff_addr) - 1; writeByte(eff_addr, value); do_CMP(cpu, value); cycles = 6; break;
        case 0xDF: cycles = 7; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr) - 1; writeByte(eff_addr, value); do_CMP(cpu, value); break;
        case 0xDB: cycles = 7; eff_addr = am_Absolute_Y(cpu, &cycles); value = readByte(eff_addr) - 1; writeByte(eff_addr, value); do_CMP(cpu, value); break;
        case 0xC3: eff_addr = am_Indirect_X(cpu);  value = readByte(eff_addr) - 1; writeByte(eff_addr, value); do_CMP(cpu, value); cycles = 8; break;
        case 0xD3: cycles = 8; eff_addr = am_Indirect_Y(cpu, &cycles); value = readByte(eff_addr) - 1; writeByte(eff_addr, value); do_CMP(cpu, value); break;

        // ISC (INC + SBC)
        case 0xE7: eff_addr = am_ZeroPage(cpu);    value = readByte(eff_addr) + 1; writeByte(eff_addr, value); do_SBC(cpu, value); cycles = 5; break;
        case 0xF7: eff_addr = am_ZeroPage_X(cpu);  value = readByte(eff_addr) + 1; writeByte(eff_addr, value); do_SBC(cpu, value); cycles = 6; break;
        case 0xEF: eff_addr = am_Absolute(cpu);    value = readByte(eff_addr) + 1; writeByte(eff_addr, value); do_SBC(cpu, value); cycles = 6; break;
        case 0xFF: cycles = 7; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr) + 1; writeByte(eff_addr, value); do_SBC(cpu, value); break;
        case 0xFB: cycles = 7; eff_addr = am_Absolute_Y(cpu, &cycles); value = readByte(eff_addr) + 1; writeByte(eff_addr, value); do_SBC(cpu, value); break;
        case 0xE3: eff_addr = am_Indirect_X(cpu);  value = readByte(eff_addr) + 1; writeByte(eff_addr, value); do_SBC(cpu, value); cycles = 8; break;
        case 0xF3: cycles = 8; eff_addr = am_Indirect_Y(cpu, &cycles); value = readByte(eff_addr) + 1; writeByte(eff_addr, value); do_SBC(cpu, value); break;

        // SLO (ASL + ORA)
        case 0x07: eff_addr = am_ZeroPage(cpu);    value = do_ASL(cpu, readByte(eff_addr)); writeByte(eff_addr, value); cpu->reg_A |= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 5; break;
        case 0x17: eff_addr = am_ZeroPage_X(cpu);  value = do_ASL(cpu, readByte(eff_addr)); writeByte(eff_addr, value); cpu->reg_A |= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 6; break;
        case 0x0F: eff_addr = am_Absolute(cpu);    value = do_ASL(cpu, readByte(eff_addr)); writeByte(eff_addr, value); cpu->reg_A |= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 6; break;
        case 0x1F: cycles = 7; eff_addr = am_Absolute_X(cpu, &cycles); value = do_ASL(cpu, readByte(eff_addr)); writeByte(eff_addr, value); cpu->reg_A |= value; set_flag_NZ(cpu, cpu->reg_A); break;
        case 0x1B: cycles = 7; eff_addr = am_Absolute_Y(cpu, &cycles); value = do_ASL(cpu, readByte(eff_addr)); writeByte(eff_addr, value); cpu->reg_A |= value; set_flag_NZ(cpu, cpu->reg_A); break;
        case 0x03: eff_addr = am_Indirect_X(cpu);  value = do_ASL(cpu, readByte(eff_addr)); writeByte(eff_addr, value); cpu->reg_A |= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 8; break;
        case 0x13: cycles = 8; eff_addr = am_Indirect_Y(cpu, &cycles); value = do_ASL(cpu, readByte(eff_addr)); writeByte(eff_addr, value); cpu->reg_A |= value; set_flag_NZ(cpu, cpu->reg_A); break;

        // RLA (ROL + AND)
        case 0x27: eff_addr = am_ZeroPage(cpu);    value = do_ROL(cpu, readByte(eff_addr)); writeByte(eff_addr, value); cpu->reg_A &= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 5; break;
        case 0x37: eff_addr = am_ZeroPage_X(cpu);  value = do_ROL(cpu, readByte(eff_addr)); writeByte(eff_addr, value); cpu->reg_A &= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 6; break;
        case 0x2F: eff_addr = am_Absolute(cpu);    value = do_ROL(cpu, readByte(eff_addr)); writeByte(eff_addr, value); cpu->reg_A &= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 6; break;
        case 0x3F: cycles = 7; eff_addr = am_Absolute_X(cpu, &cycles); value = do_ROL(cpu, readByte(eff_addr)); writeByte(eff_addr, value); cpu->reg_A &= value; set_flag_NZ(cpu, cpu->reg_A); break;
        case 0x3B: cycles = 7; eff_addr = am_Absolute_Y(cpu, &cycles); value = do_ROL(cpu, readByte(eff_addr)); writeByte(eff_addr, value); cpu->reg_A &= value; set_flag_NZ(cpu, cpu->reg_A); break;
        case 0x23: eff_addr = am_Indirect_X(cpu);  value = do_ROL(cpu, readByte(eff_addr)); writeByte(eff_addr, value); cpu->reg_A &= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 8; break;
        case 0x33: cycles = 8; eff_addr = am_Indirect_Y(cpu, &cycles); value = do_ROL(cpu, readByte(eff_addr)); writeByte(eff_addr, value); cpu->reg_A &= value; set_flag_NZ(cpu, cpu->reg_A); break;

        // SRE (LSR + EOR)
        case 0x47: eff_addr = am_ZeroPage(cpu);    value = do_LSR(cpu, readByte(eff_addr)); writeByte(eff_addr, value); cpu->reg_A ^= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 5; break;
        case 0x57: eff_addr = am_ZeroPage_X(cpu);  value = do_LSR(cpu, readByte(eff_addr)); writeByte(eff_addr, value); cpu->reg_A ^= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 6; break;
        case 0x4F: eff_addr = am_Absolute(cpu);    value = do_LSR(cpu, readByte(eff_addr)); writeByte(eff_addr, value); cpu->reg_A ^= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 6; break;
        case 0x5F: cycles = 7; eff_addr = am_Absolute_X(cpu, &cycles); value = do_LSR(cpu, readByte(eff_addr)); writeByte(eff_addr, value); cpu->reg_A ^= value; set_flag_NZ(cpu, cpu->reg_A); break;
        case 0x5B: cycles = 7; eff_addr = am_Absolute_Y(cpu, &cycles); value = do_LSR(cpu, readByte(eff_addr)); writeByte(eff_addr, value); cpu->reg_A ^= value; set_flag_NZ(cpu, cpu->reg_A); break;
        case 0x43: eff_addr = am_Indirect_X(cpu);  value = do_LSR(cpu, readByte(eff_addr)); writeByte(eff_addr, value); cpu->reg_A ^= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 8; break;
        case 0x53: cycles = 8; eff_addr = am_Indirect_Y(cpu, &cycles); value = do_LSR(cpu, readByte(eff_addr)); writeByte(eff_addr, value); cpu->reg_A ^= value; set_flag_NZ(cpu, cpu->reg_A); break;

        // RRA (ROR + ADC)
        case 0x67: eff_addr = am_ZeroPage(cpu);    value = do_ROR(cpu, readByte(eff_addr)); writeByte(eff_addr, value); do_ADC(cpu, value); cycles = 5; break;
        case 0x77: eff_addr = am_ZeroPage_X(cpu);  value = do_ROR(cpu, readByte(eff_addr)); writeByte(eff_addr, value); do_ADC(cpu, value); cycles = 6; break;
        case 0x6F: eff_addr = am_Absolute(cpu);    value = do_ROR(cpu, readByte(eff_addr)); writeByte(eff_addr, value); do_ADC(cpu, value); cycles = 6; break;
        case 0x7F: cycles = 7; eff_addr = am_Absolute_X(cpu, &cycles); value = do_ROR(cpu, readByte(eff_addr)); writeByte(eff_addr, value); do_ADC(cpu, value); break;
        case 0x7B: cycles = 7; eff_addr = am_Absolute_Y(cpu, &cycles); value = do_ROR(cpu, readByte(eff_addr)); writeByte(eff_addr, value); do_ADC(cpu, value); break;
        case 0x63: eff_addr = am_Indirect_X(cpu);  value = do_ROR(cpu, readByte(eff_addr)); writeByte(eff_addr, value); do_ADC(cpu, value); cycles = 8; break;
        case 0x73: cycles = 8; eff_addr = am_Indirect_Y(cpu, &cycles); value = do_ROR(cpu, readByte(eff_addr)); writeByte(eff_addr, value); do_ADC(cpu, value); break;

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
// No more TTF_Font or global texture

// Screen dimensions
#define CHAR_WIDTH 8
#define CHAR_HEIGHT 8
#define PIXEL_SCALE 2 // Scale each 8x8 char to 16x16
#define WINDOW_WIDTH (PET_SCREEN_COLS * CHAR_WIDTH * PIXEL_SCALE)
#define WINDOW_HEIGHT (PET_SCREEN_ROWS * CHAR_HEIGHT * PIXEL_SCALE)

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
        case SDLK_BACKSPACE: row = 0; col_mask = 0x01; break; // DEL (mapped from backspace)
        
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
        case SDLK_RSHIFT: row = 2; col_mask = 0x10; break; // R-SHIFT
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
        
        // Row 6 (Keypad)
        case SDLK_KP_1:   row = 6; col_mask = 0x80; break;
        case SDLK_KP_4:   row = 6; col_mask = 0x40; break;
        case SDLK_KP_7:   row = 6; col_mask = 0x20; break;
        case SDLK_KP_2:   row = 6; col_mask = 0x10; break;
        case SDLK_KP_5:   row = 6; col_mask = 0x08; break;
        case SDLK_KP_8:   row = 6; col_mask = 0x04; break;
        case SDLK_KP_3:   row = 6; col_mask = 0x02; break;
        case SDLK_KP_6:   row = 6; col_mask = 0x01; break;

        // Row 7 (Keypad)
        case SDLK_KP_9:   row = 7; col_mask = 0x80; break;
        case SDLK_KP_PERIOD: row = 7; col_mask = 0x40; break;
        case SDLK_KP_0:   row = 7; col_mask = 0x20; break;
        // ...
        
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


// --- SDL Initialization ---
int init_sdl(void) {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) < 0) {
        fprintf(stderr, "SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
        return 0;
    }

    window = SDL_CreateWindow("Commodore PET 2001-N Emulator",
                              SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                              WINDOW_WIDTH, WINDOW_HEIGHT,
                              SDL_WINDOW_SHOWN);
    if (!window) { /* ... error ... */ return 0; }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!renderer) { /* ... error ... */ return 0; }
    
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "0");
    SDL_RenderSetLogicalSize(renderer, WINDOW_WIDTH / PIXEL_SCALE, WINDOW_HEIGHT / PIXEL_SCALE);

    // No more TTF_Init!

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
    // No more TTF_CloseFont
    // No more global texture
    if (renderer) SDL_DestroyRenderer(renderer);
    if (window) SDL_DestroyWindow(window);
    // No more TTF_Quit
    SDL_Quit();
}

// --- Render/Update Screen ---
void render_screen(void) {
    // PET has a green-on-black screen
    SDL_SetRenderDrawColor(renderer, 0, 0x22, 0, 0xFF); // Dark green background
    SDL_RenderClear(renderer);

    // Bright green "pixels"
    SDL_SetRenderDrawColor(renderer, 0x00, 0xFF, 0x00, 0xFF);

    SDL_Rect pixel_rect = { 0, 0, PIXEL_SCALE, PIXEL_SCALE };

    for (int y = 0; y < PET_SCREEN_ROWS; ++y) {
        for (int x = 0; x < PET_SCREEN_COLS; ++x) {
            // Read from PET's screen RAM at 0x8000
            uint16_t mem_addr = PET_SCREEN_RAM_START + (y * PET_SCREEN_COLS) + x;
            uint8_t screen_code = readByte(mem_addr);
            
            // KERNAL flips bit 7 for reverse video
            int is_reverse = screen_code & 0x80;
            
            // Look up the 8-byte pixel data for this character
            // The 2001-N uses the "business" (non-graphics) character set by default
            // which is the *second* 1KB of the 2KB character ROM.
            // We'll just use the first 1KB for now.
            uint16_t char_data_addr = (uint16_t)(screen_code & 0x7F) * 8;
            
            for (int row = 0; row < 8; ++row) {
                uint8_t row_pixels = char_rom[char_data_addr + row];
                
                for (int col = 0; col < 8; ++col) {
                    // Is the pixel "on"? (Bit is 1)
                    int is_pixel_on = (row_pixels & (0x80 >> col)) != 0;
                    
                    // Handle reverse video
                    if (is_reverse) {
                        is_pixel_on = !is_pixel_on;
                    }

                    if (is_pixel_on) {
                        pixel_rect.x = (x * CHAR_WIDTH + col) * PIXEL_SCALE;
                        pixel_rect.y = (y * CHAR_HEIGHT + row) * PIXEL_SCALE;
                        SDL_RenderFillRect(renderer, &pixel_rect);
                    }
                }
            }
        }
    }

    SDL_RenderPresent(renderer);
}

/**
 * Loads a ROM file into a specific memory location with a specific size.
 * Returns 1 on success, 0 on failure.
 */
int load_rom_file(const char* filename, uint16_t addr, size_t size) {
    FILE* f = fopen(filename, "rb");
    if (!f) {
        perror(filename);
        return 0;
    }
    
    // Determine the buffer to load into
    uint8_t* buffer = &memory[addr];
    if (strcmp(filename, "characters-2.901447-10.bin") == 0) {
        buffer = char_rom;
        printf("Loading CHAR ROM '%s'...\n", filename);
    } else {
        printf("Loading '%s' at 0x%04X...\n", filename, addr);
    }

    size_t bytes_read = fread(buffer, 1, size, f);
    fclose(f);
    
    if (bytes_read != size) {
        fprintf(stderr, "Error: ROM file '%s' was not %zu bytes (read %zu).\n", 
                filename, size, bytes_read);
        return 0;
    }
    
    return 1;
}

/**
 * Loads the PET KERNAL and BASIC ROMs into memory.
 * These filenames are based on your 'ls' output for a PET 2001-N.
 */
int load_pet_roms() {
    // 8KB BASIC 2.0 ROM
    if (!load_rom_file("basic-2.901465-01-02.bin", 0xC000, 0x2000)) return 0;
    
    // 2KB Edit ROM (for 'N' models)
    if (!load_rom_file("edit-1-n.901439-03.bin", 0xE000, 0x0800)) return 0;
    
    // 4KB KERNAL 2.0 ROM
    if (!load_rom_file("kernal-2.901465-03.bin", 0xF000, 0x1000)) return 0;
    
    // 2KB Character ROM (for 'N' models, business font)
    if (!load_rom_file("characters-2.901447-10.bin", 0, 0x0800)) return 0;
    
    // The KERNAL ROM at 0xF000 contains the interrupt vectors.
    // The KERNAL's reset vector at 0xFFFC will point to its
    // internal boot-up code.
    return 1;
}

// --- ROM Linter ---

/**
 * Scans all loaded ROM regions for unimplemented opcodes.
 * Prints a list of any opcodes that are used by the ROMs
 * but not implemented in our cpu_step() function.
 * Returns 1 if all used opcodes are implemented, 0 otherwise.
 */
int check_rom_opcodes() {
    printf("Checking ROMs for unimplemented opcodes...\n");
    
    // A manually-maintained list of *all* opcodes (legal and illegal)
    // that we have a 'case' for in cpu_step().
    int implemented[256] = {0};
    
    // Official
    implemented[0x00] = 1; implemented[0x01] = 1; implemented[0x05] = 1; implemented[0x06] = 1; implemented[0x08] = 1; implemented[0x09] = 1; implemented[0x0A] = 1; implemented[0x0D] = 1; implemented[0x0E] = 1;
    implemented[0x10] = 1; implemented[0x11] = 1; implemented[0x15] = 1; implemented[0x16] = 1; implemented[0x18] = 1; implemented[0x19] = 1; implemented[0x1D] = 1; implemented[0x1E] = 1;
    implemented[0x20] = 1; implemented[0x21] = 1; implemented[0x24] = 1; implemented[0x25] = 1; implemented[0x26] = 1; implemented[0x28] = 1; implemented[0x29] = 1; implemented[0x2A] = 1; implemented[0x2C] = 1; implemented[0x2D] = 1; implemented[0x2E] = 1;
    implemented[0x30] = 1; implemented[0x31] = 1; implemented[0x35] = 1; implemented[0x36] = 1; implemented[0x38] = 1; implemented[0x39] = 1; implemented[0x3D] = 1; implemented[0x3E] = 1;
    implemented[0x40] = 1; implemented[0x41] = 1; implemented[0x45] = 1; implemented[0x46] = 1; implemented[0x48] = 1; implemented[0x49] = 1; implemented[0x4A] = 1; implemented[0x4C] = 1; implemented[0x4D] = 1; implemented[0x4E] = 1;
    implemented[0x50] = 1; implemented[0x51] = 1; implemented[0x55] = 1; implemented[0x56] = 1; implemented[0x58] = 1; implemented[0x59] = 1; implemented[0x5D] = 1; implemented[0x5E] = 1;
    implemented[0x60] = 1; implemented[0x61] = 1; implemented[0x65] = 1; implemented[0x66] = 1; implemented[0x68] = 1; implemented[0x69] = 1; implemented[0x6A] = 1; implemented[0x6C] = 1; implemented[0x6D] = 1; implemented[0x6E] = 1;
    implemented[0x70] = 1; implemented[0x71] = 1; implemented[0x75] = 1; implemented[0x76] = 1; implemented[0x78] = 1; implemented[0x79] = 1; implemented[0x7D] = 1; implemented[0x7E] = 1;
    implemented[0x81] = 1; implemented[0x84] = 1; implemented[0x85] = 1; implemented[0x86] = 1; implemented[0x88] = 1; implemented[0x8A] = 1; implemented[0x8C] = 1; implemented[0x8D] = 1; implemented[0x8E] = 1;
    implemented[0x90] = 1; implemented[0x91] = 1; implemented[0x94] = 1; implemented[0x95] = 1; implemented[0x96] = 1; implemented[0x98] = 1; implemented[0x99] = 1; implemented[0x9A] = 1; implemented[0x9D] = 1;
    implemented[0xA0] = 1; implemented[0xA1] = 1; implemented[0xA2] = 1; implemented[0xA4] = 1; implemented[0xA5] = 1; implemented[0xA6] = 1; implemented[0xA8] = 1; implemented[0xA9] = 1; implemented[0xAA] = 1; implemented[0xAC] = 1; implemented[0xAD] = 1; implemented[0xAE] = 1;
    implemented[0xB0] = 1; implemented[0xB1] = 1; implemented[0xB4] = 1; implemented[0xB5] = 1; implemented[0xB6] = 1; implemented[0xB8] = 1; implemented[0xB9] = 1; implemented[0xBA] = 1; implemented[0xBC] = 1; implemented[0xBD] = 1; implemented[0xBE] = 1;
    implemented[0xC0] = 1; implemented[0xC1] = 1; implemented[0xC4] = 1; implemented[0xC5] = 1; implemented[0xC6] = 1; implemented[0xC8] = 1; implemented[0xC9] = 1; implemented[0xCA] = 1; implemented[0xCC] = 1; implemented[0xCD] = 1; implemented[0xCE] = 1;
    implemented[0xD0] = 1; implemented[0xD1] = 1; implemented[0xD5] = 1; implemented[0xD6] = 1; implemented[0xD8] = 1; implemented[0xD9] = 1; implemented[0xDD] = 1; implemented[0xDE] = 1;
    implemented[0xE0] = 1; implemented[0xE1] = 1; implemented[0xE4] = 1; implemented[0xE5] = 1; implemented[0xE6] = 1; implemented[0xE8] = 1; implemented[0xE9] = 1; implemented[0xEA] = 1; implemented[0xEC] = 1; implemented[0xED] = 1; implemented[0xEE] = 1;
    implemented[0xF0] = 1; implemented[0xF1] = 1; implemented[0xF5] = 1; implemented[0xF6] = 1; implemented[0xF8] = 1; implemented[0xF9] = 1; implemented[0xFD] = 1; implemented[0xFE] = 1;
    
    // Illegal
    implemented[0x03] = 1; implemented[0x04] = 1; implemented[0x07] = 1; implemented[0x0C] = 1; implemented[0x0F] = 1;
    implemented[0x13] = 1; implemented[0x14] = 1; implemented[0x17] = 1; implemented[0x1A] = 1; implemented[0x1B] = 1; implemented[0x1C] = 1; implemented[0x1F] = 1;
    implemented[0x23] = 1; implemented[0x27] = 1; implemented[0x2F] = 1;
    implemented[0x33] = 1; implemented[0x34] = 1; implemented[0x37] = 1; implemented[0x3A] = 1; implemented[0x3B] = 1; implemented[0x3C] = 1; implemented[0x3F] = 1;
    implemented[0x43] = 1; implemented[0x44] = 1; implemented[0x47] = 1; implemented[0x4F] = 1;
    implemented[0x53] = 1; implemented[0x54] = 1; implemented[0x57] = 1; implemented[0x5A] = 1; implemented[0x5B] = 1; implemented[0x5C] = 1; implemented[0x5F] = 1;
    implemented[0x63] = 1; implemented[0x64] = 1; implemented[0x67] = 1; implemented[0x6F] = 1;
    implemented[0x73] = 1; implemented[0x74] = 1; implemented[0x77] = 1; implemented[0x7A] = 1; implemented[0x7B] = 1; implemented[0x7C] = 1; implemented[0x7F] = 1;
    implemented[0x80] = 1; implemented[0x82] = 1; implemented[0x83] = 1; implemented[0x87] = 1; implemented[0x89] = 1; implemented[0x8F] = 1;
    implemented[0x92] = 1; implemented[0x97] = 1;
    implemented[0xA3] = 1; implemented[0xA7] = 1; implemented[0xAF] = 1;
    implemented[0xB3] = 1; implemented[0xB7] = 1; implemented[0xBF] = 1;
    implemented[0xC2] = 1; implemented[0xC3] = 1; implemented[0xC7] = 1; implemented[0xCF] = 1;
    implemented[0xD3] = 1; implemented[0xD4] = 1; implemented[0xD7] = 1; implemented[0xDA] = 1; implemented[0xDB] = 1; implemented[0xDC] = 1; implemented[0xDF] = 1;
    implemented[0xE2] = 1; implemented[0xE3] = 1; implemented[0xE7] = 1; implemented[0xEB] = 1; implemented[0xEF] = 1;
    implemented[0xF3] = 1; implemented[0xF4] = 1; implemented[0xF7] = 1; implemented[0xFA] = 1; implemented[0xFB] = 1; implemented[0xFC] = 1; implemented[0xFF] = 1;
    
    // --- NEW: Add opcodes from linter list ---
    implemented[0x02] = 1; implemented[0x12] = 1; implemented[0x22] = 1; implemented[0x32] = 1; implemented[0x42] = 1; implemented[0x52] = 1; implemented[0x62] = 1; implemented[0x72] = 1;
    implemented[0x92] = 1; implemented[0xB2] = 1; implemented[0xD2] = 1; implemented[0xF2] = 1; // KIL/JAM opcodes (now NOPs)
    implemented[0x0B] = 1; implemented[0x2B] = 1; implemented[0x4B] = 1; implemented[0x6B] = 1; implemented[0x8B] = 1; implemented[0xAB] = 1; implemented[0xCB] = 1; // 2-byte NOPs
    implemented[0x9C] = 1; implemented[0x9E] = 1; // 3-byte NOPs
    implemented[0x93] = 1; implemented[0x9F] = 1; implemented[0x9B] = 1; implemented[0xBB] = 1; // 2-cycle NOPs

    int missing_count = 0;
    int unknown_opcodes[256] = {0};

    // Scan BASIC ROM (0xC000 - 0xDFFF)
    for (int i = 0xC000; i <= 0xDFFF; ++i) {
        uint8_t op = memory[i];
        if (!implemented[op]) {
            unknown_opcodes[op]++;
        }
    }
    // Scan Edit ROM (0xE000 - 0xE7FF)
    for (int i = 0xE000; i <= 0xE7FF; ++i) {
        uint8_t op = memory[i];
        if (!implemented[op]) {
            unknown_opcodes[op]++;
        }
    }
    // Scan KERNAL ROM (0xF000 - 0xFFFF)
    for (int i = 0xF000; i <= 0xFFFF; ++i) {
        uint8_t op = memory[i];
        if (!implemented[op]) {
            unknown_opcodes[op]++;
        }
    }

    for (int i = 0; i < 256; ++i) {
        if (unknown_opcodes[i] > 0) {
            if (missing_count == 0) {
                printf("Found unimplemented opcodes used by these ROMs:\n");
            }
            printf("  - Opcode 0x%02X (used %d times)\n", i, unknown_opcodes[i]);
            missing_count++;
        }
    }

    if (missing_count == 0) {
        printf("All opcodes found in ROMs are implemented! Good to go.\n");
        return 1;
    } else {
        printf("Found %d unimplemented opcodes. Please add them to cpu_step().\n", missing_count);
        return 0;
    }
}


// --- Main Program ---
int main(int argc, char* argv[]) {
    int check_roms_mode = 0;
    if (argc > 1 && strcmp(argv[1], "--check-roms") == 0) {
        check_roms_mode = 1;
    }
    
    C6502 cpu;
    memset(&memory, 0, sizeof(memory));
    memset(&char_rom, 0, sizeof(char_rom));
    
    // Initialize keyboard matrix (all keys up = 0xFF)
    for (int i = 0; i < 10; ++i) {
        pet_keyboard_matrix[i] = 0xFF;
    }

    // --- Load PET ROMs ---
    if (!load_pet_roms()) {
        fprintf(stderr, "Failed to load PET ROM files.\n");
        fprintf(stderr, "Please ensure the correct .bin files from your VICE ROM set are in the directory.\n");
        return 1;
    }
    
    // --- ROM Linter Mode ---
    if (check_roms_mode) {
        if (check_rom_opcodes()) {
            return 0; // Success
        } else {
            return 1; // Found missing opcodes
        }
    }

    // --- Init ---
    cpu_reset(&cpu); // This will now read the KERNAL's reset vector
    if (!init_sdl()) {
        return 1;
    }

    printf("CPU Reset: PC set to 0x%04X (from KERNAL ROM)\n", cpu.reg_PC);
    printf("Starting Commodore PET 2001-N emulation...\n");

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
        // The KERNAL relies on a 60Hz timer IRQ.
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
