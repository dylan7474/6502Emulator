# Agent Guidance

## Project overview
- This repository contains a single-file C implementation (`6502.c`) of a Commodore PET 2001-N emulator built on SDL2.
- ROM images (`*.bin`) are treated as read-only data that must be mapped into the 6502 address space exactly as the hardware expects.
- The emulator currently boots but video output is corrupted, so future work will likely focus on video timing, character set selection, or screen RAM handling.

## Coding style
- Follow the existing C conventions in `6502.c`: four-space indentation, brace-on-same-line for function definitions, and `snake_case` identifiers.
- Keep helper functions grouped by purpose (addressing modes, ALU helpers, rendering, SDL setup, etc.). When adding new helpers, place them near related helpers and add brief comments that mirror the surrounding style.
- Avoid introducing C++ features or additional dependencies beyond SDL2/SDL2_mixer/SDL2_ttf/libcurl without explicit justification.

## Build & verification
- Use `./configure` to confirm the required SDL2 tooling is available before building.
- Build the emulator with `make` (or `make -f Makefile.win` on Windows/MSYS2). The Linux makefile relies on `sdl2-config` being in PATH.
- There is no automated test suite; manual verification typically involves running the produced `6502` binary and observing the PET boot screen. Document any manual testing steps in your PR/summary.

## Assets & resources
- Character ROM (`characters-2.901447-10.bin`), BASIC/KERNAL ROMs, and the optional font must remain byte-for-byte identical. Do not edit these binary files in place.
- The `Resources` text file documents the upstream ROM source; keep this reference intact.

## Troubleshooting tips
- Video issues often stem from incorrect handling of screen RAM (0x8000–0x83E7) or the character set select register at 0xE84C; double-check writes to these areas when debugging.
- Keyboard handling uses a 10x8 matrix with active-low bits. Ensure any new input code preserves this convention.
