# Project Review and Next Steps (April 28, 2026)

## Current status

The emulator builds and starts, and the CPU core appears broad enough to execute the shipped PET ROM set (`--check-roms` reports no missing opcodes). The major remaining blocker is hardware accuracy around PET I/O and video pipeline behavior.

## What appears healthy

- Build prerequisites and compile path are in place (`./configure`, `make`).
- Reset vector boot path is wired to KERNAL ROM reset vector.
- ROM loading and memory protection for ROM regions are implemented.
- Keyboard matrix infrastructure exists and uses active-low bit semantics.
- Character ROM is loaded and a full 40x25 text matrix is rendered each frame.

## Most likely causes of the current corrupted video behavior

### 1) VIA register modeling is too simplified (high priority)
The current code treats `0xE84C` directly as a character-set selector and stores only a single global flag. On real PET hardware, this is part of a VIA with data direction and output registers that interact; KERNAL behavior may depend on reads/writes to multiple VIA addresses.

**Impact:** Character-set selection can be wrong or unstable, producing consistent but incorrect glyph interpretation.

### 2) PIA keyboard scanning model is too simplified (high priority)
`readByte(PIA1_PORTB)` currently decodes a direct row index from low bits of `memory[PIA1_PORTA]`. Real 6520 keyboard scanning is bitmask-driven and mediated by DDR configuration and line strobes, not simple numeric row indexing.

**Impact:** Boot routine may read incorrect key matrix values, which can perturb KERNAL init flow and leave screen RAM contents inconsistent.

### 3) Screen code -> glyph mapping assumptions need verification (high priority)
Renderer currently maps `screen_code & 0x7F` directly to `char_rom[index * 8]`, with optional +1024 offset for the second character set.

**Impact:** If PET screen code translation differs from current assumptions, glyphs render as repeating garbage despite valid CPU execution.

### 4) No timing model beyond coarse frame cycle budget (medium priority)
The emulator executes `~1MHz/60` cycles and then injects IRQ every frame, but does not model VIA/PIA timers or precise interrupt sources.

**Impact:** Cursor, keyboard scan, and display update pacing can diverge from ROM expectations.

## Recommended phased plan

### Phase 1: Add observability before changing behavior
1. Add optional trace logging for writes/reads in `0xE800-0xE84F` (PIA/VIA).
2. Add optional periodic dump of:
   - `0x8000-0x83E7` screen RAM histogram
   - current charset control register state
   - top-left 5x5 screen codes
3. Add a deterministic "headless smoke" mode that runs N frames and prints those diagnostics.

**Goal:** Confirm whether corruption starts from wrong screen codes vs wrong glyph decoding.

### Phase 2: Implement minimal, accurate PET I/O register layer
1. Replace direct memory-as-register behavior with explicit structs for:
   - PIA1 (`PORTA`, `PORTB`, `DDRA`, `DDRB`, control bits as needed)
   - VIA (`ORB`, `ORA`, `DDRB`, `DDRA`, IFR/IER minimally where ROM depends on it)
2. Route `readByte`/`writeByte` for `0xE810-0xE813` and `0xE840-0xE84F` through those handlers.
3. Ensure character set selection derives from the correct VIA output latch + DDR semantics, not a standalone global flag.

**Goal:** Let ROM interact with I/O similarly to real hardware, reducing undefined behavior.

### Phase 3: Validate text rendering against known-good PET behavior
1. Verify which 1KB half of `characters-2.901447-10.bin` should be active at cold boot.
2. Verify reverse-video semantics against PET docs.
3. Add a tiny diagnostic mode to render by directly writing known test patterns into screen RAM (A-Z, punctuation, reverse video) to isolate renderer correctness from CPU/KERNAL behavior.

**Goal:** Separate renderer correctness from boot/runtime state.

### Phase 4: Timing and input quality
1. Move from coarse per-frame IRQ injection to IRQ source behavior that better matches VIA timing.
2. Refine key mapping completeness and PET matrix correctness (especially modifiers and function/edit keys).

**Goal:** Reach stable interactive READY prompt and basic typing reliability.

## Definition of done (suggested)

- Cold boot consistently reaches readable PET startup text and `READY.` prompt.
- Keyboard input can type BASIC commands and receive expected responses.
- No repeated garbage glyph fields on idle screen.
- Headless smoke mode exits cleanly and reports stable screen diagnostics over repeated runs.

## Immediate next implementation task

Implement **Phase 1 observability** first (trace + headless diagnostics). This gives high leverage and avoids speculative fixes to the rendering path before proving where state first diverges.
