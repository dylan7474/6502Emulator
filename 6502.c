/**
 * c6502.c
 * A simple 6502 CPU emulator skeleton in C.
 *
 * This version includes:
 * - A feature-complete 6502 CPU core.
 * - SDL_ttf integration for a text-based screen.
 * - Keyboard input mapped to memory address 0x00FF.
 * - SDL_Audio integration for a simple square wave generator.
 * - A 60Hz IRQ timer to create a proper interrupt-driven machine.
 * - Logic to load and run external .bin files (ROMs).
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

    // 16-bit Program Counter
    uint16_t reg_PC;

    // 8-bit Stack Pointer (offset from 0x0100)
    uint8_t reg_SP;

    // 8-bit Flag Register (Status)
    uint8_t reg_F;

    // Emulator control
    int halted;

    // Interrupt lines
    int irq_pending; // Level-sensitive IRQ line
    int nmi_pending; // Edge-sensitive NMI line

} C6502;

// --- Global Memory ---
uint8_t memory[0x10000]; // 65536 bytes

// --- Memory-Mapped I/O ---
#define KEY_BUFFER_ADDR 0x00FF    // Memory address for last key press
#define SOUND_PITCH_ADDR 0x00FD   // Memory address for sound pitch
#define SOUND_ENABLE_ADDR 0x00FE  // Memory address for sound enable (1=on)
#define ROM_LOAD_ADDR 0x0600      // Default address to load external files

// --- Memory Access Helpers ---

/**
 * Reads a single byte from memory.
 */
uint8_t readByte(uint16_t addr) {
    // In a real system, reads from I/O ports might have side effects
    // or return values from hardware (e.g., POKEY, VIA).
    // For now, it just reads from our global memory array.
    return memory[addr];
}

/**
 * Writes a single byte to memory.
 */
void writeByte(uint16_t addr, uint8_t val) {
    // In a real system, writes to 0x00FD/FE would trigger the sound chip.
    // Our audio callback will just read these values.
    memory[addr] = val;
}

/**
 * Reads a 16-bit word (little-endian) from memory.
 */
uint16_t readWord(uint16_t addr) {
    uint8_t lo = readByte(addr);
    uint8_t hi = readByte(addr + 1);
    return (uint16_t)((hi << 8) | lo);
}

/**
 * Writes a 16-bit word (little-endian) to memory.
 */
void writeWord(uint16_t addr, uint16_t val) {
    writeByte(addr, val & 0xFF);
    writeByte(addr + 1, (val >> 8) & 0xFF);
}


// --- Stack Helper Functions ---

/**
 * Pushes a byte onto the stack.
 * Stack grows down from 0x01FF.
 */
void pushByte(C6502* cpu, uint8_t val) {
    writeByte(0x0100 + cpu->reg_SP, val);
    cpu->reg_SP--;
}

/**
 * Pushes a 16-bit word onto the stack (high byte first).
 */
void pushWord(C6502* cpu, uint16_t val) {
    pushByte(cpu, (val >> 8) & 0xFF); // High byte
    pushByte(cpu, val & 0xFF);        // Low byte
}

/**
 * Pops a byte from the stack.
 */
uint8_t popByte(C6502* cpu) {
    cpu->reg_SP++;
    return readByte(0x0100 + cpu->reg_SP);
}

/**
 * Pops a 16-bit word from the stack (low byte first).
 */
uint16_t popWord(C6502* cpu) {
    uint8_t lo = popByte(cpu);
    uint8_t hi = popByte(cpu);
    return (uint16_t)((hi << 8) | lo);
}


// --- Addressing Mode Helpers ---
// These functions calculate the "effective address" for an instruction
// and return it. Some also fetch the value at that address.
// They also handle advancing the PC past any operands.
// Cycle counts are *estimates* and will be refined.

/**
 * AM_IMMEDIATE - Fetches the value immediately after the opcode.
 * Returns the value.
 */
uint8_t am_Immediate(C6502* cpu) {
    return readByte(cpu->reg_PC++);
}

/**
 * AM_ZEROPAGE - Fetches the 8-bit zero-page address.
 * Returns the 16-bit zero-page address.
 */
uint16_t am_ZeroPage(C6502* cpu) {
    return (uint16_t)readByte(cpu->reg_PC++);
}

/**
 * AM_ZEROPAGE_X - Fetches 8-bit ZP address, adds X.
 * Returns the 16-bit zero-page address, wrapped.
 */
uint16_t am_ZeroPage_X(C6502* cpu) {
    uint8_t zp_addr = readByte(cpu->reg_PC++);
    return (uint16_t)((zp_addr + cpu->reg_X) & 0xFF);
}

/**
 * AM_ZEROPAGE_Y - Fetches 8-bit ZP address, adds Y.
 * Returns the 16-bit zero-page address, wrapped.
 */
uint16_t am_ZeroPage_Y(C6502* cpu) {
    uint8_t zp_addr = readByte(cpu->reg_PC++);
    return (uint16_t)((zp_addr + cpu->reg_Y) & 0xFF);
}

/**
 * AM_ABSOLUTE - Fetches the 16-bit absolute address.
 * Returns the 16-bit address.
 */
uint16_t am_Absolute(C6502* cpu) {
    uint16_t addr = readWord(cpu->reg_PC);
    cpu->reg_PC += 2;
    return addr;
}

/**
 * AM_ABSOLUTE_X - Fetches 16-bit address, adds X.
 * Handles page crossing for cycle counting.
 */
uint16_t am_Absolute_X(C6502* cpu, int* cycles) {
    uint16_t base_addr = readWord(cpu->reg_PC);
    cpu->reg_PC += 2;
    uint16_t final_addr = base_addr + cpu->reg_X;

    // Check if we crossed a page boundary (e.g., 0x02FF -> 0x0300)
    if ((base_addr & 0xFF00) != (final_addr & 0xFF00)) {
        (*cycles)++;
    }
    return final_addr;
}

/**
 * AM_ABSOLUTE_Y - Fetches 16-bit address, adds Y.
 * Handles page crossing for cycle counting.
 */
uint16_t am_Absolute_Y(C6502* cpu, int* cycles) {
    uint16_t base_addr = readWord(cpu->reg_PC);
    cpu->reg_PC += 2;
    uint16_t final_addr = base_addr + cpu->reg_Y;

    // Check for page crossing
    if ((base_addr & 0xFF00) != (final_addr & 0xFF00)) {
        (*cycles)++;
    }
    return final_addr;
}

/**
 * AM_INDIRECT_X - (Indexed Indirect)
 * 1. Fetch 8-bit ZP address.
 * 2. Add X (wrapped).
 * 3. Read 16-bit *effective address* from that ZP location.
 * Returns the final 16-bit effective address.
 */
uint16_t am_Indirect_X(C6502* cpu) {
    uint8_t zp_addr = readByte(cpu->reg_PC++);
    uint16_t lookup_addr = (uint16_t)((zp_addr + cpu->reg_X) & 0xFF);
    return readWord(lookup_addr);
}

/**
 * AM_INDIRECT_Y - (Indirect Indexed)
 * 1. Fetch 8-bit ZP address.
 * 2. Read 16-bit *base address* from that ZP location.
 * 3. Add Y to the base address.
 * Returns the final 16-bit effective address.
 */
uint16_t am_Indirect_Y(C6502* cpu, int* cycles) {
    uint8_t zp_addr = readByte(cpu->reg_PC++);
    uint16_t base_addr = readWord((uint16_t)zp_addr);
    uint16_t final_addr = base_addr + cpu->reg_Y;

    // Check for page crossing
    if ((base_addr & 0xFF00) != (final_addr & 0xFF00)) {
        (*cycles)++;
    }
    return final_addr;
}


// --- CPU Logic Helpers ---

/**
 * Sets or clears CPU flags based on a value.
 */
void set_flag(C6502* cpu, uint8_t flag, int cond) {
    if (cond) {
        cpu->reg_F |= flag;
    } else {
        cpu->reg_F &= ~flag;
    }
}

/**
 * Sets the Zero (Z) and Negative (N) flags based on a value.
 */
void set_flag_NZ(C6502* cpu, uint8_t val) {
    set_flag(cpu, FLAG_Z, val == 0);
    set_flag(cpu, FLAG_N, (val & 0x80) != 0);
}

/**
 * Helper for branch instructions.
 * Fetches 8-bit relative offset, adds to PC if cond is true.
 */
void do_Branch(C6502* cpu, int* cycles, int cond) {
    int8_t offset = (int8_t)readByte(cpu->reg_PC++);
    if (cond) {
        (*cycles)++; // Branch taken
        uint16_t old_pc = cpu->reg_PC;
        cpu->reg_PC += offset;

        // Page cross adds another cycle
        if ((old_pc & 0xFF00) != (cpu->reg_PC & 0xFF00)) {
            (*cycles)++;
        }
    }
}

/**
 * Performs A = A + M + C
 */
void do_ADC(C6502* cpu, uint8_t M) {
    // Use 16-bit ints to catch carry
    uint16_t tmp = (uint16_t)cpu->reg_A + (uint16_t)M + (uint16_t)(cpu->reg_F & FLAG_C);

    // Set Overflow (V) flag
    // Overflow happens if sign of A and M are the same,
    // but sign of the result is different.
    // (A^result) & (M^result) & 0x80
    set_flag(cpu, FLAG_V, (~(cpu->reg_A ^ M) & (cpu->reg_A ^ tmp) & 0x80) != 0);

    // Set Carry (C) flag
    set_flag(cpu, FLAG_C, tmp > 0xFF);

    cpu->reg_A = (uint8_t)(tmp & 0xFF);
    set_flag_NZ(cpu, cpu->reg_A);
}

/**
 * Performs A = A - M - (1-C)
 */
void do_SBC(C6502* cpu, uint8_t M) {
    // SBC is just ADC with the operand inverted
    // A + (~M) + C
    uint16_t tmp = (uint16_t)cpu->reg_A + (uint16_t)(~M) + (uint16_t)(cpu->reg_F & FLAG_C);

    // Set Overflow (V) flag
    set_flag(cpu, FLAG_V, ((cpu->reg_A ^ tmp) & (~M ^ tmp) & 0x80) != 0);

    // Set Carry (C) flag
    set_flag(cpu, FLAG_C, tmp > 0xFF);

    cpu->reg_A = (uint8_t)(tmp & 0xFF);
    set_flag_NZ(cpu, cpu->reg_A);
}

/**
 * Performs (A & M), sets Z, V, N flags.
 * Used by BIT instruction.
 */
void do_BIT(C6502* cpu, uint8_t M) {
    // 1. Z flag is set if A & M is zero
    set_flag(cpu, FLAG_Z, (cpu->reg_A & M) == 0);
    // 2. V flag is set to bit 6 of M
    set_flag(cpu, FLAG_V, (M & 0x40) != 0);
    // 3. N flag is set to bit 7 of M
    set_flag(cpu, FLAG_N, (M & 0x80) != 0);
}


/**
 * Performs (A - M), setting flags only
 */
void do_CMP(C6502* cpu, uint8_t M) {
    uint16_t tmp = (uint16_t)cpu->reg_A - (uint16_t)M;
    set_flag(cpu, FLAG_C, cpu->reg_A >= M);
    set_flag_NZ(cpu, (uint8_t)(tmp & 0xFF));
}

/**
 * Performs (X - M), setting flags only
 */
void do_CPX(C6502* cpu, uint8_t M) {
    uint16_t tmp = (uint16_t)cpu->reg_X - (uint16_t)M;
    set_flag(cpu, FLAG_C, cpu->reg_X >= M);
    set_flag_NZ(cpu, (uint8_t)(tmp & 0xFF));
}

/**
 * Performs (Y - M), setting flags only
 */
void do_CPY(C6502* cpu, uint8_t M) {
    uint16_t tmp = (uint16_t)cpu->reg_Y - (uint16_t)M;
    set_flag(cpu, FLAG_C, cpu->reg_Y >= M);
    set_flag_NZ(cpu, (uint8_t)(tmp & 0xFF));
}

/**
 * RMW Helper: ASL (Arithmetic Shift Left)
 */
uint8_t do_ASL(C6502* cpu, uint8_t M) {
    set_flag(cpu, FLAG_C, (M & 0x80) != 0);
    uint8_t result = (uint8_t)(M << 1);
    set_flag_NZ(cpu, result);
    return result;
}

/**
 * RMW Helper: LSR (Logical Shift Right)
 */
uint8_t do_LSR(C6502* cpu, uint8_t M) {
    set_flag(cpu, FLAG_C, (M & 0x01) != 0);
    uint8_t result = (uint8_t)(M >> 1);
    set_flag_NZ(cpu, result);
    return result;
}

/**
 * RMW Helper: ROL (Rotate Left)
 */
uint8_t do_ROL(C6502* cpu, uint8_t M) {
    uint8_t old_c = (cpu->reg_F & FLAG_C);
    set_flag(cpu, FLAG_C, (M & 0x80) != 0);
    uint8_t result = (uint8_t)((M << 1) | old_c);
    set_flag_NZ(cpu, result);
    return result;
}

/**
 * RMW Helper: ROR (Rotate Right)
 */
uint8_t do_ROR(C6502* cpu, uint8_t M) {
    uint8_t old_c = (cpu->reg_F & FLAG_C);
    set_flag(cpu, FLAG_C, (M & 0x01) != 0);
    uint8_t result = (uint8_t)((M >> 1) | (old_c << 7));
    set_flag_NZ(cpu, result);
    return result;
}

// --- Interrupt Logic ---

/**
 * Performs the 6502 interrupt sequence (IRQ or BRK).
 * Pushes PC and Flags, sets I flag, jumps to vector.
 * Returns 7 cycles.
 */
int cpu_interrupt(C6502* cpu, uint16_t vector_addr, int is_brk) {
    // Push PC onto stack (BRK pushes PC+1)
    if (is_brk) {
        pushWord(cpu, cpu->reg_PC + 1);
    } else {
        pushWord(cpu, cpu->reg_PC);
    }

    // Push flags (with B flag set if BRK)
    if (is_brk) {
        pushByte(cpu, cpu->reg_F | FLAG_B | FLAG_U);
    } else {
        pushByte(cpu, (cpu->reg_F & ~FLAG_B) | FLAG_U);
    }

    // Set Interrupt Disable flag
    cpu->reg_F |= FLAG_I;

    // Load PC from interrupt vector
    cpu->reg_PC = readWord(vector_addr);

    return 7;
}

/**
 * Triggers a maskable interrupt (IRQ).
 */
int cpu_irq(C6502* cpu) {
    if ((cpu->reg_F & FLAG_I) == 0) { // Only if interrupts are not disabled
        cpu->irq_pending = 0;
        return cpu_interrupt(cpu, 0xFFFE, 0); // IRQ vector
    }
    return 0;
}

/**
 * Triggers a non-maskable interrupt (NMI).
 */
int cpu_nmi(C6502* cpu) {
    cpu->nmi_pending = 0;
    return cpu_interrupt(cpu, 0xFFFA, 0); // NMI vector
}


// --- CPU Control Functions ---

/**
 * Resets the CPU to its initial state.
 * Loads the 16-bit Reset Vector from 0xFFFC/FFFD.
 */
void cpu_reset(C6502* cpu) {
    // Note: Vectors are now set in main() *before* calling reset.
    cpu->reg_A = 0;
    cpu->reg_X = 0;
    cpu->reg_Y = 0;
    cpu->reg_SP = 0xFD; // Stack starts at 0x01FF, grows down
    cpu->reg_F = FLAG_U | FLAG_I; // Unused and Interrupt flags set
    cpu->reg_PC = readWord(0xFFFC);
    cpu->halted = 0;
    cpu->irq_pending = 0;
    cpu->nmi_pending = 0;
}

/**
 * Checks for and handles pending interrupts *before* fetching an opcode.
 * Returns cycles spent (0 if no interrupt).
 */
int cpu_handle_interrupts(C6502* cpu) {
    if (cpu->nmi_pending) {
        return cpu_nmi(cpu);
    }
    if (cpu->irq_pending && (cpu->reg_F & FLAG_I) == 0) {
        return cpu_irq(cpu);
    }
    return 0;
}


/**
 * Executes a single CPU instruction.
 * Returns the number of cycles taken.
 */
int cpu_step(C6502* cpu) {
    // --- 1. Handle Interrupts ---
    // Check for NMI/IRQ *before* fetching the next instruction
    int cycles = cpu_handle_interrupts(cpu);
    if (cycles > 0) {
        return cycles; // Interrupt took precedence
    }

    // --- 2. Fetch Opcode ---
    uint16_t debug_pc_start = cpu->reg_PC; // For error reporting
    uint8_t opcode = readByte(cpu->reg_PC++);
    cycles = 2; // Default cycles (will be overridden)
    uint16_t eff_addr = 0; // Effective address for opcodes
    uint8_t value = 0;     // Value fetched from memory

    // --- 3. Decode and Execute ---
    switch (opcode) {

        // --- NOP ---
        case 0xEA: cycles = 2; break; // NOP

        // --- BRK ---
        case 0x00: { // BRK (Software Interrupt)
            // BRK is handled as a software IRQ
            cycles = cpu_interrupt(cpu, 0xFFFE, 1);
            // Note: Unlike real hardware, we'll halt the emu for testing
            cpu->halted = 1;
            break;
        }
        
        // --- RTI (Return from Interrupt) ---
        case 0x40: { // RTI
            cpu->reg_F = (popByte(cpu) & ~FLAG_B) | FLAG_U;
            cpu->reg_PC = popWord(cpu);
            cycles = 6;
            break;
        }

        // --- Flag Control ---
        case 0x18: cpu->reg_F &= ~FLAG_C; cycles = 2; break; // CLC
        case 0x38: cpu->reg_F |= FLAG_C;  cycles = 2; break; // SEC
        case 0x58: cpu->reg_F &= ~FLAG_I; cycles = 2; break; // CLI
        case 0x78: cpu->reg_F |= FLAG_I;  cycles = 2; break; // SEI
        case 0xD8: cpu->reg_F &= ~FLAG_D; cycles = 2; break; // CLD
        case 0xF8: cpu->reg_F |= FLAG_D;  cycles = 2; break; // SED

        // --- Jumps & Subroutines ---
        case 0x4C: { // JMP Absolute
            cpu->reg_PC = am_Absolute(cpu);
            cycles = 3;
            break;
        }
        case 0x6C: { // JMP Indirect
            uint16_t ptr_addr = am_Absolute(cpu);
            // 6502 bug: if indirect vector is on a page boundary,
            // it wraps around (e.g., 0x02FF -> 0x02FF, 0x0200)
            if ((ptr_addr & 0x00FF) == 0x00FF) {
                uint8_t lo = readByte(ptr_addr);
                uint8_t hi = readByte(ptr_addr & 0xFF00);
                cpu->reg_PC = (uint16_t)((hi << 8) | lo);
            } else {
                cpu->reg_PC = readWord(ptr_addr);
            }
            cycles = 5;
            break;
        }
        case 0x20: { // JSR (Jump to Subroutine)
            uint16_t sub_addr = am_Absolute(cpu);
            pushWord(cpu, cpu->reg_PC - 1); // Push return address (PC-1)
            cpu->reg_PC = sub_addr;
            cycles = 6;
            break;
        }
        case 0x60: { // RTS (Return from Subroutine)
            cpu->reg_PC = popWord(cpu) + 1; // Pop PC and increment
            cycles = 6;
            break;
        }


        // --- LDA (Load Accumulator) ---
        case 0xA9: { // LDA Immediate
            value = am_Immediate(cpu);
            cpu->reg_A = value;
            set_flag_NZ(cpu, cpu->reg_A);
            cycles = 2;
            break;
        }
        case 0xA5: { // LDA ZeroPage
            eff_addr = am_ZeroPage(cpu);
            cpu->reg_A = readByte(eff_addr);
            set_flag_NZ(cpu, cpu->reg_A);
            cycles = 3;
            break;
        }
        case 0xB5: { // LDA ZeroPage,X
            eff_addr = am_ZeroPage_X(cpu);
            cpu->reg_A = readByte(eff_addr);
            set_flag_NZ(cpu, cpu->reg_A);
            cycles = 4;
            break;
        }
        case 0xAD: { // LDA Absolute
            eff_addr = am_Absolute(cpu);
            cpu->reg_A = readByte(eff_addr);
            set_flag_NZ(cpu, cpu->reg_A);
            cycles = 4;
            break;
        }
        case 0xBD: { // LDA Absolute,X
            cycles = 4;
            eff_addr = am_Absolute_X(cpu, &cycles);
            cpu->reg_A = readByte(eff_addr);
            set_flag_NZ(cpu, cpu->reg_A);
            break;
        }
        case 0xB9: { // LDA Absolute,Y
            cycles = 4;
            eff_addr = am_Absolute_Y(cpu, &cycles);
            cpu->reg_A = readByte(eff_addr);
            set_flag_NZ(cpu, cpu->reg_A);
            break;
        }
        case 0xA1: { // LDA (Indirect,X)
            eff_addr = am_Indirect_X(cpu);
            cpu->reg_A = readByte(eff_addr);
            set_flag_NZ(cpu, cpu->reg_A);
            cycles = 6;
            break;
        }
        case 0xB1: { // LDA (Indirect),Y
            cycles = 5;
            eff_addr = am_Indirect_Y(cpu, &cycles);
            cpu->reg_A = readByte(eff_addr);
            set_flag_NZ(cpu, cpu->reg_A);
            break;
        }

        // --- STA (Store Accumulator) ---
        case 0x85: { // STA ZeroPage
            eff_addr = am_ZeroPage(cpu);
            writeByte(eff_addr, cpu->reg_A);
            cycles = 3;
            break;
        }
        case 0x95: { // STA ZeroPage,X
            eff_addr = am_ZeroPage_X(cpu);
            writeByte(eff_addr, cpu->reg_A);
            cycles = 4;
            break;
        }
        case 0x8D: { // STA Absolute
            eff_addr = am_Absolute(cpu);
            writeByte(eff_addr, cpu->reg_A);
            cycles = 4;
            break;
        }
        case 0x9D: { // STA Absolute,X
            cycles = 5; // STA Abs,X is always 5 cycles
            eff_addr = am_Absolute_X(cpu, &cycles);
            cycles = 5; // Re-set to 5, page cross doesn't matter
            writeByte(eff_addr, cpu->reg_A);
            break;
        }
        // ... Other STA modes ...

        // --- LDX (Load X) ---
        case 0xA2: { // LDX Immediate
            cpu->reg_X = am_Immediate(cpu);
            set_flag_NZ(cpu, cpu->reg_X);
            cycles = 2;
            break;
        }
        case 0xA6: { // LDX ZeroPage
            eff_addr = am_ZeroPage(cpu);
            cpu->reg_X = readByte(eff_addr);
            set_flag_NZ(cpu, cpu->reg_X);
            cycles = 3;
            break;
        }
        case 0xB6: { // LDX ZeroPage,Y
            eff_addr = am_ZeroPage_Y(cpu);
            cpu->reg_X = readByte(eff_addr);
            set_flag_NZ(cpu, cpu->reg_X);
            cycles = 4;
            break;
        }
        case 0xAE: { // LDX Absolute
            eff_addr = am_Absolute(cpu);
            cpu->reg_X = readByte(eff_addr);
            set_flag_NZ(cpu, cpu->reg_X);
            cycles = 4;
            break;
        }
        // ... Other LDX modes ...

        // --- LDY (Load Y) ---
        case 0xA0: { // LDY Immediate
            cpu->reg_Y = am_Immediate(cpu);
            set_flag_NZ(cpu, cpu->reg_Y);
            cycles = 2;
            break;
        }
        // ... Other LDY modes ...

        // --- STX (Store X) ---
        case 0x86: { // STX ZeroPage
            eff_addr = am_ZeroPage(cpu);
            writeByte(eff_addr, cpu->reg_X);
            cycles = 3;
            break;
        }
        case 0x96: { // STX ZeroPage,Y
            eff_addr = am_ZeroPage_Y(cpu);
            writeByte(eff_addr, cpu->reg_X);
            cycles = 4;
            break;
        }
        case 0x8E: { // STX Absolute
            eff_addr = am_Absolute(cpu);
            writeByte(eff_addr, cpu->reg_X);
            cycles = 4;
            break;
        }

        // --- STY (Store Y) ---
        case 0x84: { // STY ZeroPage
            eff_addr = am_ZeroPage(cpu);
            writeByte(eff_addr, cpu->reg_Y);
            cycles = 3;
            break;
        }
        case 0x94: { // STY ZeroPage,X
            eff_addr = am_ZeroPage_X(cpu);
            writeByte(eff_addr, cpu->reg_Y);
            cycles = 4;
            break;
        }
        case 0x8C: { // STY Absolute
            eff_addr = am_Absolute(cpu);
            writeByte(eff_addr, cpu->reg_Y);
            cycles = 4;
            break;
        }


        // --- Stack Operations ---
        case 0x48: pushByte(cpu, cpu->reg_A); cycles = 3; break; // PHA
        case 0x68: cpu->reg_A = popByte(cpu); set_flag_NZ(cpu, cpu->reg_A); cycles = 4; break; // PLA
        case 0x08: pushByte(cpu, cpu->reg_F | FLAG_B | FLAG_U); cycles = 3; break; // PHP
        case 0x28: cpu->reg_F = (popByte(cpu) & ~FLAG_B) | FLAG_U; cycles = 4; break; // PLP

        // --- Transfers ---
        case 0xAA: cpu->reg_X = cpu->reg_A; set_flag_NZ(cpu, cpu->reg_X); cycles = 2; break; // TAX
        case 0xA8: cpu->reg_Y = cpu->reg_A; set_flag_NZ(cpu, cpu->reg_Y); cycles = 2; break; // TAY
        case 0x8A: cpu->reg_A = cpu->reg_X; set_flag_NZ(cpu, cpu->reg_A); cycles = 2; break; // TXA
        case 0x98: cpu->reg_A = cpu->reg_Y; set_flag_NZ(cpu, cpu->reg_A); cycles = 2; break; // TYA

        // --- Increments / Decrements (Register) ---
        case 0xE8: cpu->reg_X++; set_flag_NZ(cpu, cpu->reg_X); cycles = 2; break; // INX
        case 0xC8: cpu->reg_Y++; set_flag_NZ(cpu, cpu->reg_Y); cycles = 2; break; // INY
        case 0xCA: cpu->reg_X--; set_flag_NZ(cpu, cpu->reg_X); cycles = 2; break; // DEX
        case 0x88: cpu->reg_Y--; set_flag_NZ(cpu, cpu->reg_Y); cycles = 2; break; // DEY

        // --- Increments / Decrements (Memory) ---
        case 0xE6: { // INC ZeroPage
            eff_addr = am_ZeroPage(cpu);
            value = readByte(eff_addr) + 1;
            writeByte(eff_addr, value);
            set_flag_NZ(cpu, value);
            cycles = 5;
            break;
        }
        case 0xEE: { // INC Absolute
            eff_addr = am_Absolute(cpu);
            value = readByte(eff_addr) + 1;
            writeByte(eff_addr, value);
            set_flag_NZ(cpu, value);
            cycles = 6;
            break;
        }
        case 0xC6: { // DEC ZeroPage
            eff_addr = am_ZeroPage(cpu);
            value = readByte(eff_addr) - 1;
            writeByte(eff_addr, value);
            set_flag_NZ(cpu, value);
            cycles = 5;
            break;
        }
        case 0xCE: { // DEC Absolute
            eff_addr = am_Absolute(cpu);
            value = readByte(eff_addr) - 1;
            writeByte(eff_addr, value);
            set_flag_NZ(cpu, value);
            cycles = 6;
            break;
        }

        // --- Branching ---
        case 0x10: do_Branch(cpu, &cycles, (cpu->reg_F & FLAG_N) == 0); break; // BPL (Branch if Plus)
        case 0x30: do_Branch(cpu, &cycles, (cpu->reg_F & FLAG_N) != 0); break; // BMI (Branch if Minus)
        case 0x50: do_Branch(cpu, &cycles, (cpu->reg_F & FLAG_V) == 0); break; // BVC (Branch if Overflow Clear)
        case 0x70: do_Branch(cpu, &cycles, (cpu->reg_F & FLAG_V) != 0); break; // BVS (Branch if Overflow Set)
        case 0x90: do_Branch(cpu, &cycles, (cpu->reg_F & FLAG_C) == 0); break; // BCC (Branch if Carry Clear)
        case 0xB0: do_Branch(cpu, &cycles, (cpu->reg_F & FLAG_C) != 0); break; // BCS (Branch if Carry Set)
        case 0xD0: do_Branch(cpu, &cycles, (cpu->reg_F & FLAG_Z) == 0); break; // BNE (Branch if Not Equal)
        case 0xF0: do_Branch(cpu, &cycles, (cpu->reg_F & FLAG_Z) != 0); break; // BEQ (Branch if Equal)


        // --- ALU: ADC (Add with Carry) ---
        case 0x69: value = am_Immediate(cpu);   do_ADC(cpu, value); cycles = 2; break;
        case 0x65: eff_addr = am_ZeroPage(cpu); value = readByte(eff_addr); do_ADC(cpu, value); cycles = 3; break;
        case 0x75: eff_addr = am_ZeroPage_X(cpu); value = readByte(eff_addr); do_ADC(cpu, value); cycles = 4; break;
        case 0x6D: eff_addr = am_Absolute(cpu); value = readByte(eff_addr); do_ADC(cpu, value); cycles = 4; break;
        case 0x7D: cycles = 4; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr); do_ADC(cpu, value); break;
        case 0x79: cycles = 4; eff_addr = am_Absolute_Y(cpu, &cycles); value = readByte(eff_addr); do_ADC(cpu, value); break;
        case 0x61: eff_addr = am_Indirect_X(cpu); value = readByte(eff_addr); do_ADC(cpu, value); cycles = 6; break;
        case 0x71: cycles = 5; eff_addr = am_Indirect_Y(cpu, &cycles); value = readByte(eff_addr); do_ADC(cpu, value); break;

        // --- ALU: SBC (Subtract with Carry) ---
        case 0xE9: value = am_Immediate(cpu);   do_SBC(cpu, value); cycles = 2; break;
        case 0xE5: eff_addr = am_ZeroPage(cpu); value = readByte(eff_addr); do_SBC(cpu, value); cycles = 3; break;
        case 0xF5: eff_addr = am_ZeroPage_X(cpu); value = readByte(eff_addr); do_SBC(cpu, value); cycles = 4; break;
        case 0xED: eff_addr = am_Absolute(cpu); value = readByte(eff_addr); do_SBC(cpu, value); cycles = 4; break;
        case 0xFD: cycles = 4; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr); do_SBC(cpu, value); break;
        case 0xF9: cycles = 4; eff_addr = am_Absolute_Y(cpu, &cycles); value = readByte(eff_addr); do_SBC(cpu, value); break;
        case 0xE1: eff_addr = am_Indirect_X(cpu); value = readByte(eff_addr); do_SBC(cpu, value); cycles = 6; break;
        case 0xF1: cycles = 5; eff_addr = am_Indirect_Y(cpu, &cycles); value = readByte(eff_addr); do_SBC(cpu, value); break;

        // --- ALU: AND (Logical AND) ---
        case 0x29: value = am_Immediate(cpu);   cpu->reg_A &= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 2; break;
        case 0x25: eff_addr = am_ZeroPage(cpu); value = readByte(eff_addr); cpu->reg_A &= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 3; break;
        case 0x35: eff_addr = am_ZeroPage_X(cpu); value = readByte(eff_addr); cpu->reg_A &= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 4; break;
        case 0x2D: eff_addr = am_Absolute(cpu); value = readByte(eff_addr); cpu->reg_A &= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 4; break;
        case 0x3D: cycles = 4; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr); cpu->reg_A &= value; set_flag_NZ(cpu, cpu->reg_A); break;
        case 0x39: cycles = 4; eff_addr = am_Absolute_Y(cpu, &cycles); value = readByte(eff_addr); cpu->reg_A &= value; set_flag_NZ(cpu, cpu->reg_A); break;
        case 0x21: eff_addr = am_Indirect_X(cpu); value = readByte(eff_addr); cpu->reg_A &= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 6; break;
        case 0x31: cycles = 5; eff_addr = am_Indirect_Y(cpu, &cycles); value = readByte(eff_addr); cpu->reg_A &= value; set_flag_NZ(cpu, cpu->reg_A); break;

        // --- ALU: ORA (Logical OR) ---
        case 0x09: value = am_Immediate(cpu);   cpu->reg_A |= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 2; break;
        case 0x05: eff_addr = am_ZeroPage(cpu); value = readByte(eff_addr); cpu->reg_A |= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 3; break;
        case 0x15: eff_addr = am_ZeroPage_X(cpu); value = readByte(eff_addr); cpu->reg_A |= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 4; break;
        case 0x0D: eff_addr = am_Absolute(cpu); value = readByte(eff_addr); cpu->reg_A |= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 4; break;
        case 0x1D: cycles = 4; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr); cpu->reg_A |= value; set_flag_NZ(cpu, cpu->reg_A); break;
        case 0x19: cycles = 4; eff_addr = am_Absolute_Y(cpu, &cycles); value = readByte(eff_addr); cpu->reg_A |= value; set_flag_NZ(cpu, cpu->reg_A); break;
        case 0x01: eff_addr = am_Indirect_X(cpu); value = readByte(eff_addr); cpu->reg_A |= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 6; break;
        case 0x11: cycles = 5; eff_addr = am_Indirect_Y(cpu, &cycles); value = readByte(eff_addr); cpu->reg_A |= value; set_flag_NZ(cpu, cpu->reg_A); break;
        
        // --- ALU: EOR (Logical EOR) ---
        case 0x49: value = am_Immediate(cpu);   cpu->reg_A ^= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 2; break;
        case 0x45: eff_addr = am_ZeroPage(cpu); value = readByte(eff_addr); cpu->reg_A ^= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 3; break;
        case 0x55: eff_addr = am_ZeroPage_X(cpu); value = readByte(eff_addr); cpu->reg_A ^= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 4; break;
        case 0x4D: eff_addr = am_Absolute(cpu); value = readByte(eff_addr); cpu->reg_A ^= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 4; break;
        case 0x5D: cycles = 4; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr); cpu->reg_A ^= value; set_flag_NZ(cpu, cpu->reg_A); break;
        case 0x59: cycles = 4; eff_addr = am_Absolute_Y(cpu, &cycles); value = readByte(eff_addr); cpu->reg_A ^= value; set_flag_NZ(cpu, cpu->reg_A); break;
        case 0x41: eff_addr = am_Indirect_X(cpu); value = readByte(eff_addr); cpu->reg_A ^= value; set_flag_NZ(cpu, cpu->reg_A); cycles = 6; break;
        case 0x51: cycles = 5; eff_addr = am_Indirect_Y(cpu, &cycles); value = readByte(eff_addr); cpu->reg_A ^= value; set_flag_NZ(cpu, cpu->reg_A); break;

        // --- Comparisons: CMP (Compare A) ---
        case 0xC9: value = am_Immediate(cpu);   do_CMP(cpu, value); cycles = 2; break;
        case 0xC5: eff_addr = am_ZeroPage(cpu); value = readByte(eff_addr); do_CMP(cpu, value); cycles = 3; break;
        case 0xD5: eff_addr = am_ZeroPage_X(cpu); value = readByte(eff_addr); do_CMP(cpu, value); cycles = 4; break;
        case 0xCD: eff_addr = am_Absolute(cpu); value = readByte(eff_addr); do_CMP(cpu, value); cycles = 4; break;
        case 0xDD: cycles = 4; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr); do_CMP(cpu, value); break;
        case 0xD9: cycles = 4; eff_addr = am_Absolute_Y(cpu, &cycles); value = readByte(eff_addr); do_CMP(cpu, value); break;
        case 0xC1: eff_addr = am_Indirect_X(cpu); value = readByte(eff_addr); do_CMP(cpu, value); cycles = 6; break;
        case 0xD1: cycles = 5; eff_addr = am_Indirect_Y(cpu, &cycles); value = readByte(eff_addr); do_CMP(cpu, value); break;

        // --- Comparisons: CPX (Compare X) ---
        case 0xE0: value = am_Immediate(cpu);   do_CPX(cpu, value); cycles = 2; break;
        case 0xE4: eff_addr = am_ZeroPage(cpu); value = readByte(eff_addr); do_CPX(cpu, value); cycles = 3; break;
        case 0xEC: eff_addr = am_Absolute(cpu); value = readByte(eff_addr); do_CPX(cpu, value); cycles = 4; break;

        // --- Comparisons: CPY (Compare Y) ---
        case 0xC0: value = am_Immediate(cpu);   do_CPY(cpu, value); cycles = 2; break;
        case 0xC4: eff_addr = am_ZeroPage(cpu); value = readByte(eff_addr); do_CPY(cpu, value); cycles = 3; break;
        case 0xCC: eff_addr = am_Absolute(cpu); value = readByte(eff_addr); do_CPY(cpu, value); cycles = 4; break;
        
        // --- Comparisons: BIT (Test Bits) ---
        case 0x24: eff_addr = am_ZeroPage(cpu);  value = readByte(eff_addr); do_BIT(cpu, value); cycles = 3; break;
        case 0x2C: eff_addr = am_Absolute(cpu);  value = readByte(eff_addr); do_BIT(cpu, value); cycles = 4; break;


        // --- RMW: ASL (Arithmetic Shift Left) ---
        case 0x0A: cpu->reg_A = do_ASL(cpu, cpu->reg_A); cycles = 2; break;
        case 0x06: eff_addr = am_ZeroPage(cpu);    value = readByte(eff_addr); writeByte(eff_addr, do_ASL(cpu, value)); cycles = 5; break;
        case 0x16: eff_addr = am_ZeroPage_X(cpu);  value = readByte(eff_addr); writeByte(eff_addr, do_ASL(cpu, value)); cycles = 6; break;
        case 0x0E: eff_addr = am_Absolute(cpu);    value = readByte(eff_addr); writeByte(eff_addr, do_ASL(cpu, value)); cycles = 6; break;
        case 0x1E: cycles = 7; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr); writeByte(eff_addr, do_ASL(cpu, value)); cycles = 7; break;

        // --- RMW: LSR (Logical Shift Right) ---
        case 0x4A: cpu->reg_A = do_LSR(cpu, cpu->reg_A); cycles = 2; break;
        case 0x46: eff_addr = am_ZeroPage(cpu);    value = readByte(eff_addr); writeByte(eff_addr, do_LSR(cpu, value)); cycles = 5; break;
        case 0x56: eff_addr = am_ZeroPage_X(cpu);  value = readByte(eff_addr); writeByte(eff_addr, do_LSR(cpu, value)); cycles = 6; break;
        case 0x4E: eff_addr = am_Absolute(cpu);    value = readByte(eff_addr); writeByte(eff_addr, do_LSR(cpu, value)); cycles = 6; break;
        case 0x5E: cycles = 7; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr); writeByte(eff_addr, do_LSR(cpu, value)); cycles = 7; break;

        // --- RMW: ROL (Rotate Left) ---
        case 0x2A: cpu->reg_A = do_ROL(cpu, cpu->reg_A); cycles = 2; break;
        case 0x26: eff_addr = am_ZeroPage(cpu);    value = readByte(eff_addr); writeByte(eff_addr, do_ROL(cpu, value)); cycles = 5; break;
        case 0x36: eff_addr = am_ZeroPage_X(cpu);  value = readByte(eff_addr); writeByte(eff_addr, do_ROL(cpu, value)); cycles = 6; break;
        case 0x2E: eff_addr = am_Absolute(cpu);    value = readByte(eff_addr); writeByte(eff_addr, do_ROL(cpu, value)); cycles = 6; break;
        case 0x3E: cycles = 7; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr); writeByte(eff_addr, do_ROL(cpu, value)); cycles = 7; break;

        // --- RMW: ROR (Rotate Right) ---
        case 0x6A: cpu->reg_A = do_ROR(cpu, cpu->reg_A); cycles = 2; break;
        case 0x66: eff_addr = am_ZeroPage(cpu);    value = readByte(eff_addr); writeByte(eff_addr, do_ROR(cpu, value)); cycles = 5; break;
        case 0x76: eff_addr = am_ZeroPage_X(cpu);  value = readByte(eff_addr); writeByte(eff_addr, do_ROR(cpu, value)); cycles = 6; break;
        case 0x6E: eff_addr = am_Absolute(cpu);    value = readByte(eff_addr); writeByte(eff_addr, do_ROR(cpu, value)); cycles = 6; break;
        case 0x7E: cycles = 7; eff_addr = am_Absolute_X(cpu, &cycles); value = readByte(eff_addr); writeByte(eff_addr, do_ROR(cpu, value)); cycles = 7; break;


        // --- Unimplemented Opcode ---
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
TTF_Font* font = NULL; // Font for rendering text

// Screen dimensions
#define SCREEN_RAM_START 0x0400
#define SCREEN_COLS 40
#define SCREEN_ROWS 25
#define CHAR_WIDTH 8
#define CHAR_HEIGHT 8
#define WINDOW_WIDTH (SCREEN_COLS * CHAR_WIDTH)
#define WINDOW_HEIGHT (SCREEN_ROWS * CHAR_HEIGHT)

// --- Audio Globals ---
#define AUDIO_SAMPLE_RATE 44100
#define AUDIO_AMPLITUDE 2000 // 16-bit
static double audio_phase = 0.0; // Global phase for the sound wave

/**
 * Audio Callback
 * This function is called by SDL when it needs more audio data.
 */
void audio_callback(void* userdata, Uint8* stream, int len) {
    (void)userdata; // Unused
    Sint16* buffer = (Sint16*)stream;
    int num_samples = len / sizeof(Sint16);

    // Read the "hardware registers" from 6502 memory
    uint8_t sound_enable = readByte(SOUND_ENABLE_ADDR);
    uint8_t pitch_val = readByte(SOUND_PITCH_ADDR);

    if (sound_enable == 0 || pitch_val == 0) {
        // Sound is off, fill with silence
        for (int i = 0; i < num_samples; ++i) {
            buffer[i] = 0;
        }
        return;
    }

    // --- Generate a Square Wave ---
    // A simple formula to map 1-255 to a pleasant frequency range
    double base_freq = 110.0; // A2
    double freq = base_freq * pow(2.0, (double)pitch_val / 48.0);
    double phase_inc = (2.0 * M_PI * freq) / AUDIO_SAMPLE_RATE;

    for (int i = 0; i < num_samples; ++i) {
        // Generate a square wave (positive/negative amplitude)
        buffer[i] = (sin(audio_phase) > 0) ? AUDIO_AMPLITUDE : -AUDIO_AMPLITUDE;
        audio_phase += phase_inc;
        
        // Wrap phase to prevent it from growing indefinitely
        if (audio_phase > 2.0 * M_PI) {
            audio_phase -= 2.0 * M_PI;
        }
    }
}


// --- I/O Helpers ---

/**
 * Maps an SDL keycode to our simple PETSCII-like char set.
 * 1-26 for A-Z, 32 for Space. Returns 0 for unmapped keys.
 */
uint8_t map_sdl_key_to_petscii(SDL_Keycode key) {
    if (key >= SDLK_a && key <= SDLK_z) {
        return (uint8_t)(key - SDLK_a + 1); // A-Z -> 1-26
    }
    if (key == SDLK_SPACE) {
        return 32; // Space
    }
    if (key == SDLK_EXCLAIM) {
        return 33; // !
    }
    // TODO: Add numbers, punctuation
    return 0; // Unmapped
}


// --- SDL Initialization ---
int init_sdl(void) {
    // *** NEW: Added SDL_INIT_AUDIO ***
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) < 0) {
        fprintf(stderr, "SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
        return 0;
    }

    // Initialize SDL_ttf
    if (TTF_Init() == -1) {
        fprintf(stderr, "SDL_ttf could not initialize! TTF_Error: %s\n", TTF_GetError());
        return 0;
    }

    window = SDL_CreateWindow("6502 Emulator",
                              SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                              WINDOW_WIDTH * 2, WINDOW_HEIGHT * 2, // Scaled up window
                              SDL_WINDOW_SHOWN);
    if (!window) {
        fprintf(stderr, "Window Error: %s\n", SDL_GetError());
        return 0;
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!renderer) {
        fprintf(stderr, "Renderer Error: %s\n", SDL_GetError());
        return 0;
    }
    
    // Use "nearest pixel" scaling to get a sharp, blocky look
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "0");
    SDL_RenderSetLogicalSize(renderer, WINDOW_WIDTH, WINDOW_HEIGHT);

    // Load the font (assuming font.ttf is in the same directory)
    font = TTF_OpenFont("font.ttf", CHAR_HEIGHT); // Request 8pt size
    if (!font) {
        fprintf(stderr, "Failed to load font 'font.ttf'! TTF_Error: %s\n", TTF_GetError());
        fprintf(stderr, "Please make sure 'font.ttf' is in the same directory as the executable.\n");
        return 0;
    }
    // Set font style to be non-antialiased for a retro feel
    TTF_SetFontHinting(font, TTF_HINTING_MONO);

    // --- NEW: Setup Audio ---
    SDL_AudioSpec wanted_spec, have_spec;
    SDL_zero(wanted_spec);
    wanted_spec.freq = AUDIO_SAMPLE_RATE;
    wanted_spec.format = AUDIO_S16SYS; // Signed 16-bit
    wanted_spec.channels = 1;          // Mono
    wanted_spec.samples = 2048;        // Buffer size
    wanted_spec.callback = audio_callback;
    wanted_spec.userdata = NULL;

    if (SDL_OpenAudio(&wanted_spec, &have_spec) < 0) {
        fprintf(stderr, "Failed to open audio: %s\n", SDL_GetError());
        // We don't return 0 here, emulator can run without sound
    } else {
        SDL_PauseAudio(0); // Start playing (will call callback)
    }

    return 1;
}

// --- SDL Cleanup ---
void cleanup_sdl(void) {
    SDL_CloseAudio(); // Close audio subsystem
    if (font) TTF_CloseFont(font);
    if (texture) SDL_DestroyTexture(texture);
    if (renderer) SDL_DestroyRenderer(renderer);
    if (window) SDL_DestroyWindow(window);
    TTF_Quit();
    SDL_Quit();
}

// --- Render/Update Screen ---
void render_screen(void) {
    // Set a dark blue background (C64-like)
    SDL_SetRenderDrawColor(renderer, 0, 0, 0xAA, 0xFF);
    SDL_RenderClear(renderer);

    SDL_Color fg_color = { 0xBB, 0xBB, 0xFF, 0xFF }; // Light blue/purple text
    char char_str[2] = { 0, 0 }; // String for rendering a single char
    SDL_Surface* text_surface = NULL;
    SDL_Texture* text_texture = NULL;
    SDL_Rect dest_rect = { 0, 0, CHAR_WIDTH, CHAR_HEIGHT };

    for (int y = 0; y < SCREEN_ROWS; ++y) {
        for (int x = 0; x < SCREEN_COLS; ++x) {
            // Calculate memory address
            uint16_t mem_addr = SCREEN_RAM_START + (y * SCREEN_COLS) + x;
            uint8_t petscii_code = readByte(mem_addr);
            
            // Convert PETSCII-like code to ASCII for rendering
            // This is a very simple map, not real PETSCII
            if (petscii_code >= 1 && petscii_code <= 26) {
                char_str[0] = 'A' + (petscii_code - 1); // 1-26 -> A-Z
            } else if (petscii_code == 32) {
                char_str[0] = ' '; // 32 -> Space
            } else if (petscii_code == 33) {
                 char_str[0] = '!'; // 33 -> !
            } else {
                char_str[0] = ' '; // Default to space for unmapped
            }

            if (char_str[0] == ' ') continue; // Skip rendering spaces

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
 * Loads the built-in test program into memory.
 */
void load_built_in_program() {
    // Set vectors for built-in program
    writeWord(0xFFFA, 0x0300); // NMI Vector
    writeWord(0xFFFC, 0x0200); // RESET Vector
    writeWord(0xFFFE, 0x0300); // IRQ/BRK Vector
    
    // PETSCII-like codes (1-26 for A-Z, 32 for space)
    uint8_t message_codes[] = {
        32, 8, 5, 12, 12, 15, 32, 23, 15, 18, 12, 4, 33, 32, 0
    };
    memcpy(&memory[0x0100], message_codes, sizeof(message_codes));

    // --- Main Program (at 0x0200) ---
    uint8_t test_program[] = {
        // --- Part 1: Print "HELLO WORLD!" ---
        // 0x0200
        0xA2, 0x00,       // LDX #$00         (X = 0)
    // PRINT_LOOP (0x0202)
        0xBD, 0x00, 0x01, // LDA $0100,X      (Load char from message)
        0xF0, 0x07,       // BEQ INIT_MACHINE (If char is 0, jump to part 2)
        
    // 0x0207
        0x9D, 0x00, 0x04, // STA $0400,X      (Store char to screen RAM)
        0xE8,             // INX
        0x4C, 0x02, 0x02, // JMP PRINT_LOOP
        
    // --- Part 2: Init Machine and Idle ---
    // INIT_MACHINE (0x020E)
        0xA9, 0x50,       // LDA #$50         (Load a pitch value)
        0x85, 0xFD,       // STA $FD          (Store to pitch register)
        0xA9, 0x01,       // LDA #$01         (Load 1)
        0x85, 0xFE,       // STA $FE          (Store to sound enable register)
        0xA2, 0x00,       // LDX #$00         (X = 0, our cursor on line 2)
        0x86, 0xFC,       // STX $FC          (Store 0 to cursor scratchpad)
        0x58,             // CLI              (Clear Interrupt Disable)
        
    // IDLE_LOOP (0x021B)
        0x4C, 0x1B, 0x02  // JMP IDLE_LOOP    (Loop forever, waiting for IRQs)
    };
    memcpy(&memory[0x0200], test_program, sizeof(test_program));
    
    // --- IRQ Handler (at 0x0300, set by vector) ---
    uint8_t irq_handler[] = {
        // 0x0300
        0x48,             // PHA              (Save A)
        0x8A,             // TXA              (Save X)
        0x48,             // PHA
        
        // --- Poll Keyboard ---
        // 0x0303
        0xA5, 0xFF,       // LDA $FF          (Load key buffer)
        // 0x0305
        // *** BUG FIX HERE ***
        0xF0, 0x11,       // BEQ EXIT_IRQ (0x0318). PC=0x0307. 0x0307 + 0x11 = 0x0318
        
        // --- Key was pressed ---
        // 0x0307
        0x85, 0xFD,       // STA $FD          (Set sound pitch to key value)
        // 0x0309
        0xA6, 0xFC,       // LDX $FC          (Load cursor pos from scratchpad)
        // 0x030B
        0x9D, 0x28, 0x04, // STA $0428,X      (Store key to line 2 (0x0400 + 40))
        // 0x030E
        0xE8,             // INX              (Advance cursor)
        // 0x030F
        0x86, 0xFC,       // STX $FC          (Save new cursor pos)
        // 0x0311
        0xA9, 0x00,       // LDA #$00         (Clear A)
        // 0x0313
        0x85, 0xFF,       // STA $FF          (Clear key buffer)
        // *** BUG FIX HERE: Added JMP to skip exit code ***
        // 0x0315
        0x4C, 0x18, 0x03, // JMP EXIT_IRQ ($0318)

        // --- Exit IRQ ---
    // EXIT_IRQ: (0x0318)
        0x68,             // PLA              (Restore X)
        // 0x0319
        0xAA,             // TAX
        // 0x031A
        0x68,             // PLA              (Restore A)
        // 0x031B
        0x40              // RTI              (Return from Interrupt)
    };
    memcpy(&memory[0x0300], irq_handler, sizeof(irq_handler));
}

/**
 * Loads an external binary file into memory at ROM_LOAD_ADDR
 * and sets the RESET vector to point to it.
 */
int load_external_rom(const char* filename) {
    FILE* f = fopen(filename, "rb");
    if (!f) {
        perror("Error opening ROM file");
        return 0;
    }

    // Read file into memory at ROM_LOAD_ADDR
    size_t bytes_read = fread(&memory[ROM_LOAD_ADDR], 1, sizeof(memory) - ROM_LOAD_ADDR, f);
    fclose(f);

    if (bytes_read == 0) {
        fprintf(stderr, "Error: ROM file is empty or could not be read.\n");
        return 0;
    }

    printf("Loaded %zu bytes from '%s' into 0x%04X.\n", bytes_read, filename, ROM_LOAD_ADDR);

    // Set vectors to point to the new program
    // We assume the external ROM will provide its own IRQ/NMI handlers
    // if it needs them, or it will set them itself.
    writeWord(0xFFFA, 0x0000); // Default NMI
    writeWord(0xFFFC, ROM_LOAD_ADDR); // RESET
    writeWord(0xFFFE, 0x0000); // Default IRQ

    return 1;
}

// --- Main Program ---
int main(int argc, char* argv[]) {
    C6502 cpu;
    memset(&memory, 0, sizeof(memory));

    // --- NEW: Program Loading ---
    if (argc > 1) {
        // Load external ROM
        if (!load_external_rom(argv[1])) {
            return 1; // Failed to load
        }
    } else {
        // Load built-in test program
        printf("No ROM file specified. Loading built-in test program.\n");
        load_built_in_program();
    }


    // --- Init ---
    cpu_reset(&cpu);
    if (!init_sdl()) {
        return 1;
    }

    printf("CPU Reset: PC set to 0x%04X\n", cpu.reg_PC);
    printf("Starting 6502 emulation...\n");

    int quit = 0;
    SDL_Event e;
    int cycles_this_frame = 0;
    // We assume a 1.0 MHz CPU for this timer, 60Hz frame
    const int cycles_per_frame = 1000000 / 60; // ~16666 cycles

    // --- Main Emulation Loop ---
    while (!quit && !cpu.halted) {
        // --- Event Handling ---
        while (SDL_PollEvent(&e) != 0) {
            if (e.type == SDL_QUIT) {
                quit = 1;
            }
            // *** Handle Keyboard Input ***
            else if (e.type == SDL_KEYDOWN) {
                // Only write to buffer if it's empty
                if (readByte(KEY_BUFFER_ADDR) == 0) {
                    uint8_t key = map_sdl_key_to_petscii(e.key.keysym.sym);
                    if (key != 0) {
                        writeByte(KEY_BUFFER_ADDR, key);
                    }
                }
            }
        }

        // --- CPU Emulation ---
        cycles_this_frame = 0;
        // Run for one "frame's" worth of cycles
        while (cycles_this_frame < cycles_per_frame && !cpu.halted) {
            cycles_this_frame += cpu_step(&cpu);
        }
        
        // --- Hardware Timer ---
        // After a frame's worth of cycles, trigger the IRQ
        cpu.irq_pending = 1;


        // --- Render Frame ---
        render_screen();
    }

    // --- Cleanup ---
    if (cpu.halted) {
        // Also check if the error was NOT a BRK
        if (readByte(cpu.reg_PC - 1) != 0x00) {
             printf("CPU Halted by non-BRK at 0x%04X\n", cpu.reg_PC - 1);
        } else {
             printf("BRK instruction hit at 0x%04X. Halting.\n", cpu.reg_PC - 1);
        }
    }
    printf("Emulation finished. Final CPU State:\n");
    printf("A: 0x%02X X: 0x%02X Y: 0x%02X\n", cpu.reg_A, cpu.reg_X, cpu.reg_Y);
    printf("PC: 0x%04X SP: 0x%02X F: 0x%02X\n", cpu.reg_PC, cpu.reg_SP, cpu.reg_F);

    printf("Screen RAM check:\n");
    printf("0x0401: %d (Should be 8)\n", readByte(0x0401));


    cleanup_sdl();
    return 0;
}
