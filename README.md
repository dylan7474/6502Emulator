# 6502Emulator

6502Emulator is a hobbyist Commodore PET 2001-N system emulator written in C with SDL2. It loads bundled ROM images and font data to reproduce the PET boot experience, render characters using the actual character generator ROM, and respond to keyboard input. The goal of the project is to offer a lightweight, hackable codebase for experimenting with 6502 CPU emulation and retro-style graphics/audio.

## Current Status (v0.5)

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
- Loading .PRG files (`--load`, or drag a file onto the window)

**🔧 Not yet:**
- Cassette (datasette) and IEEE-488 disk drives
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

## Loading Programs

PET software is usually distributed as `.prg` files: two bytes of load address followed by the data.

- **From the command line:** `./6502 --load file.prg` loads it as soon as BASIC is ready; add `--run` to start it.
- **While running:** drag a `.prg` file onto the window, then type `RUN`.

A BASIC program (load address $0401) also gets BASIC's start/variable/array pointers set, just as `LOAD` does, so `LIST`, `RUN` and editing work straight away. A machine-code file is copied to its own address; the emulator prints the `SYS` address to start it. Files must fit in RAM/screen memory ($0000-$83FF). Programs written for BASIC 4 PETs (4000/8000 series) or for 80 columns may not run on a 2001-N.

## Command-line Options

```bash
./6502                                   # normal windowed run
./6502 --load game.prg --run            # load a .PRG after boot, then type RUN
./6502 --type '10 print "hi"\nrun\n'      # type text after boot (\n = RETURN, {stop} = RUN/STOP)
./6502 --headless 60 --type 'print 2+2\n' # no window: boot, type, run 60 frames, print the screen
./6502 --check-roms                      # verify all ROM opcodes are implemented
./6502 --video-debug                     # verbose video/VIA logging (or PET_VIDEO_DEBUG=1)
```

## Roadmap

### Done
- **v0.4** — boots to `READY.` with the correct BASIC 2 editor ROM, no ROM patches; real 2001-N I/O (retrace IRQ, keyboard matrix, VIA timers, charset, CB2 sound); SBC/decimal-mode CPU fixes; full-screen rendering; `--headless` / `--type` testing
- **v0.5** — `.PRG` loading (`--load`, `--run`, drag and drop)

### Next (in rough priority order)
1. **Automated tests** — a `make test` target that boots headless, types BASIC and checks the screen text, so regressions show up straight away
2. **LOAD and SAVE from BASIC** — trap the KERNAL tape routines so `SAVE "NAME"` writes `NAME.prg` and `LOAD "NAME"` reads it from a folder on the host
3. **Held keys for games** — the keyboard currently taps each key; games that read the matrix (e.g. `PEEK(151)`) need keys held down while the PC key is down
4. **Debugger overlay** — registers, a memory view, breakpoints and single-step, toggled with a function key
5. **Cassette (.TAP) emulation** — real tape timing on the cassette lines, for software that only exists as tape images
6. **IEEE-488 disk drives** — a 2040/4040 drive on device 8 with `.D64` images (8050/`.D80` later)
7. **Other PET models** — a ROM-set option for the original 2001 (BASIC 1) and the 4000/8000 series (BASIC 4, 80 columns via the 6545 CRTC)
8. **Timing accuracy** — check per-instruction cycle counts and page-crossing penalties against a test suite (e.g. Klaus Dormann's 6502 functional tests)
