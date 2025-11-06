# 6502Emulator

6502Emulator is a hobbyist Commodore 64-inspired system emulator written in C with SDL2. It loads bundled ROM images and font data to reproduce the C64 boot experience, render characters, and respond to keyboard input. The goal of the project is to offer a lightweight, hackable codebase for experimenting with 6502 CPU emulation and retro-style graphics/audio.

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

The emulator maps the Commodore 64 keyboard matrix to modern keyboards using SDL2. Highlights include:

- Arrow keys for cursor movement, Enter for RETURN, and Backspace for DEL.
- Standard alphanumeric keys (`A`-`Z`, `0`-`9`) mapped to their C64 counterparts.
- Punctuation keys such as `,`, `.`, `/`, `-`, `;`, and `:` mapped to the expected symbols.
- Function keys F1/F3/F5/F7 and keypad keys for additional matrix rows.
- Left/Right Shift operate as the Commodore SHIFT keys; Space inserts blanks.

## Roadmap

- Improve CPU timing accuracy and add cycle-by-cycle tests.
- Implement drive emulation and disk image loading.
- Expand audio playback support using additional SID test ROMs.
- Provide a debugger overlay for inspecting registers and memory in real time.
