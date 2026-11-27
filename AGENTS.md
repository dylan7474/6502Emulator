# Agent Guidance

## Project overview
- This repository contains a single-file C implementation (`6502.c`) of a Commodore PET 2001-N emulator built on SDL2.
- ROM images (`*.bin`) are treated as read-only data that must be mapped into the 6502 address space exactly as the hardware expects.
- The emulator boots to the BASIC 2 READY prompt. Use `./6502 --headless N --type "..."` to test without a window (prints the screen as text).

## Coding style
- Follow the existing C conventions in `6502.c`: four-space indentation, brace-on-same-line for function definitions, and `snake_case` identifiers.
- Keep helper functions grouped by purpose (addressing modes, ALU helpers, rendering, SDL setup, etc.). When adding new helpers, place them near related helpers and add brief comments that mirror the surrounding style.
- Avoid introducing C++ features or additional dependencies beyond SDL2 without explicit justification.

## Build & verification
- Use `./configure` to confirm the required SDL2 tooling is available before building.
- Build the emulator with `make` (or `make CC=x86_64-w64-mingw32-gcc` on Windows/MSYS2). The Linux makefile relies on `sdl2-config` being in PATH.
- There is no automated test suite; manual verification typically involves running the produced `6502` binary and observing the PET boot screen. Document any manual testing steps in your PR/summary.

## Assets & resources
- Character ROM (`characters-2.901447-10.bin`), BASIC/KERNAL ROMs, and the optional font must remain byte-for-byte identical. Do not edit these binary files in place.
- The `Resources` text file documents the upstream ROM source; keep this reference intact.

## Troubleshooting tips
- The editor ROM must match the KERNAL (BASIC 2 = edit-2-n.901447-24). A mismatched editor ROM looks like video/IRQ bugs; never patch ROM bytes to work around it.
- Keyboard: PIA1 PA0-3 is a row NUMBER (0-9, via a 74145 decoder), PB0-7 the active-low columns. The 60 Hz IRQ is PIA1 CB1 (vertical retrace), acknowledged by reading $E812. Charset is VIA CA2 (POKE 59468,12/14).
