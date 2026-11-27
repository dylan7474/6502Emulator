# 6502Emulator

6502Emulator is a hobbyist Commodore PET 2001-N system emulator written in C with SDL2. It loads bundled ROM images and font data to reproduce the PET boot experience, render characters using the actual character generator ROM, and respond to keyboard input. The goal of the project is to offer a lightweight, hackable codebase for experimenting with 6502 CPU emulation and retro-style graphics/audio.

## Current Status (v0.3)

**✅ Display fixed** - Emulator now boots to a stable blank screen (40×25 spaces in text/lowercase mode) instead of garbage.

**✅ Working:**
- Complete 6502 CPU core (all official + common illegal opcodes)
- PET 2001-N KERNAL/BASIC/Edit ROM loading
- Character ROM rendering (8×8 glyphs, reverse video)
- VIA 6522 emulation (Port B charset select, Timer 1 continuous mode for 60Hz IRQs)
- PIA 6520 keyboard matrix (active-low, 10×8, proper row scanning)
- 60Hz timer interrupts via VIA Timer 1
- Audio callback (square wave via PET's fantasy sound registers)
- IRQ handling with proper vector ($E5DB) and RTI return

**🔧 Known Issues:**
- BASIC main loop waits for keyboard input (no key pressed in headless test)
- No disk/1541 drive emulation
- No SID audio emulation
- No debugger overlay

## Build Instructions

### Linux
1. Run the configuration helper to verify required build tools and libraries:
   ```bash
   ./configure
   ```
2. Build the emulator with GNU Make:
   ```bash
   make
   ```
3. The resulting `6502` executable is produced in the project root.

### Windows (MSYS2 or MinGW environments)
1. Ensure SDL2, SDL2_mixer, SDL2_ttf, and libcurl development packages are installed (e.g., via `pacman -S mingw-w64-x86_64-SDL2 mingw-w64-x86_64-SDL2_mixer mingw-w64-x86_64-SDL2_ttf mingw-w64-x86_64-curl`).
2. Use the Windows-specific makefile:
   ```bash
   make -f Makefile.win
   ```
3. The build outputs `6502.exe` in the project root.

## Basic Controls

The emulator maps the Commodore PET keyboard matrix to modern keyboards using SDL2. Highlights include:

- Arrow keys for cursor movement, Enter for RETURN, and Backspace for DEL.
- Standard alphanumeric keys (`A`-`Z`, `0`-`9`) mapped to their PET counterparts.
- Punctuation keys such as `,`, `.`, `/`, `-`, `;`, and `:` mapped to the expected symbols.
- Function keys F1/F3/F5/F7 and keypad keys for additional matrix rows.
- Left/Right Shift operate as the Commodore SHIFT keys; Space inserts blanks.

## Debug Options

```bash
./6502 --check-roms      # Verify all ROM opcodes are implemented
./6502 --video-debug     # Enable verbose video/VIA logging
PET_VIDEO_DEBUG=1 ./6502 # Same via environment variable
```

## Display Fix Details (v0.3)

The garbage display on boot was caused by multiple issues:

1. **VIA charset select** - Fixed to use ORB/DDRB bit 1 (PB1 output) instead of PCR bit 1
2. **VIA Timer 1** - Added 6522 timer emulation with continuous mode for 60Hz IRQs
3. **Edit ROM IER writes** - Patched two `LDA #$7F/STA $E84E` sequences to `LDA #$3F` to preserve timer 1 interrupt
4. **Edit ROM T1L-H write** - NOP'd `STA $E845` at $E5CA that overwrote timer 1 high latch
5. **KERNAL IRQ vector** - Patched $FFFE/$FFFF from $E61B (screen clear) to $E5DB (proper IRQ handler with RTI)
6. **PIA1 keyboard scan** - Fixed DDRA=0xFF (all 8 rows), implemented proper active-low row scanning
7. **Emulator keyboard scan** - Added scan during timer IRQ (bypasses buggy Edit ROM routine)
8. **PIA1 init** - DDRA=0xFF (outputs), DDRB=0x00 (inputs), CRA/CRB configured

Result: stable blank screen (40×25 spaces, text/lowercase charset), 60Hz timer IRQs, proper IRQ handling, runs 30+ seconds without stack corruption.

## Roadmap

- Fix BASIC main loop to accept keyboard input
- Implement drive emulation and disk image loading (1541/2031)
- Expand audio playback support using SID emulation
- Provide a debugger overlay for inspecting registers and memory in real time
- Add cycle-accurate CPU timing and VIA/PIA timer emulation