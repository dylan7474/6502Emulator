# 6502Emulator

6502Emulator is a hobbyist Commodore PET 2001-N system emulator written in C with SDL2. It loads bundled ROM images and font data to reproduce the PET boot experience, render characters using the actual character generator ROM, and respond to keyboard input. The goal of the project is to offer a lightweight, hackable codebase for experimenting with 6502 CPU emulation and retro-style graphics/audio.

## Current Status (v0.4)

**✅ Boots to BASIC.** The emulator now starts like a real 32K PET 2001-N with BASIC 2:

```
### COMMODORE BASIC ###

 31743 BYTES FREE

READY.
```

Typing, line editing, LIST/RUN, floating-point maths, the TI/TI$ jiffy clock, RUN/STOP and both character sets work. No ROM bytes are patched.

**Working:**
- 6502 CPU core (all official + common illegal opcodes, decimal mode for ADC/SBC)
- PET 2001-N memory map: 32K RAM, 1K screen mirrored to $8FFF, open bus at $9000-$BFFF
- BASIC 2 (901465-01/02), Editor (901447-24) and KERNAL (901465-03) ROMs
- PIA1 keyboard: 10x8 matrix, row number on PA0-3, diagnostic sense PA7 high
- 60 Hz IRQ from vertical retrace (PIA1 CB1), retrace input on VIA PB5
- VIA 6522 timers 1 and 2, charset select on CA2 (POKE 59468,12 / 14)
- Sound from the VIA shift register on CB2 (POKE 59467,16 : POKE 59466,15 : POKE 59464,pitch)
- Character ROM rendering with reverse video, scaled to a resizable window

**🔧 Not yet:**
- Cassette (datasette) and IEEE-488 disk drives
- Loading .PRG files
- Debugger overlay

### Why it didn't boot before (fixed in v0.4)
The repo loaded the **BASIC 1** editor ROM (`edit-1-n.901439-03`) next to the **BASIC 2** KERNAL. The KERNAL's reset path and IRQ vector ($E61B) jump into the editor ROM, so every call landed in the middle of unrelated code. The earlier "fixes" (patching the IRQ vector, NOP'ing editor code, forcing a VIA timer) worked around that mismatch. v0.4 loads the matching `edit-2-n.901447-24` ROM, removes all ROM patches and models the I/O the way the 2001-N hardware does. It also fixes an SBC carry bug (carry was always set) and a render bug that only drew the top-left quarter of the screen.

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
1. Ensure the SDL2 development package is installed (e.g., via `pacman -S mingw-w64-x86_64-SDL2`).
2. Build with the standard Makefile (cross-compiles via MinGW):
   ```bash
   make CC=x86_64-w64-mingw32-gcc
   ```
3. The build outputs `6502.exe` in the project root.

## Basic Controls

The PET 2001-N has a graphics keyboard: letters are upper case, and SHIFT+letter gives a graphics character (or a capital in text mode). Characters you type are sent to the matching PET key, so `"`, `$`, `(`, `*` and so on work as printed on your keyboard.

| PC key | PET key |
|---|---|
| Letters, digits, punctuation | the same character |
| Shift+letter | SHIFT+letter (graphics / capitals) |
| Enter | RETURN |
| Backspace / Insert | DEL / INST |
| Arrow keys | cursor keys |
| Home / Shift+Home or Delete | HOME / CLR |
| Esc | RUN/STOP |
| Tab / Shift+Tab | RVS ON / OFF |

## Command-line Options

```bash
./6502                                   # normal windowed run
./6502 --type '10 print "hi"\nrun\n'      # type text after boot (\n = RETURN, {stop} = RUN/STOP)
./6502 --headless 60 --type 'print 2+2\n' # no window: boot, type, run 60 frames, print the screen
./6502 --check-roms                      # verify all ROM opcodes are implemented
./6502 --video-debug                     # verbose video/VIA logging (or PET_VIDEO_DEBUG=1)
```

## Roadmap

- Load .PRG files straight into memory
- Cassette (.TAP) and IEEE-488 drive emulation (2040/4040 disk images)
- Provide a debugger overlay for inspecting registers and memory in real time
- Add cycle-accurate CPU timing and VIA/PIA timer emulation