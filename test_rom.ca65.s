; ==================================================================
; TEST_ROM.ASM
; A simple test program for the c6502 emulator.
;
; Assemble with an online assembler (like https://www.masswerk.at/6502/assembler.html)
; Set the origin to $0600. (e.g., add "* = $0600" at the top)
; Download the binary/prg file and run: ./c6502 test_rom.bin
; ==================================================================

; --- Hardware Equates ---
KEY_BUFFER      = $FF
SOUND_PITCH     = $FD
SOUND_ENABLE    = $FE
CURSOR_POS      = $FC  ; ZP scratchpad for cursor
SCREEN_START    = $0400
SCREEN_LINE_2   = $0428

; --- Vectors ---
; The loader in main() sets the RESET vector to $0600.
; We must set the IRQ vector ourself.
.org $0600

RESET:
; --- Part 1: Setup Vectors ---
LDA #<IRQ_HANDLER  ; Load low byte of IRQ address
STA $FFFE          ; Store to IRQ vector low
LDA #>IRQ_HANDLER  ; Load high byte
STA $FFFF          ; Store to IRQ vector high
; (We ignore NMI for this test)

; --- Part 2: Print "HELLO WORLD!" ---
LDX #$00           ; X = 0


PRINT_LOOP:
LDA MESSAGE,X      ; Load char from message
BEQ INIT_MACHINE   ; If char is 0, jump to part 2
STA SCREEN_START,X ; Store char to screen RAM
INX
JMP PRINT_LOOP

; --- Part 3: Init Machine and Idle ---
INIT_MACHINE:
LDA #$50           ; Load a pitch value
STA SOUND_PITCH    ; Store to pitch register
LDA #$01           ; Load 1
STA SOUND_ENABLE   ; Store to sound enable register
LDX #$00           ; X = 0, our cursor on line 2
STX CURSOR_POS     ; Store 0 to cursor scratchpad
CLI                ; Clear Interrupt Disable (Enable IRQs)

IDLE_LOOP:
JMP IDLE_LOOP      ; Loop forever, waiting for IRQs

; --- Data ---
MESSAGE:
.byte 32, 8, 5, 12, 12, 15, 32, 23, 15, 18, 12, 4, 33, 32, 0 ; " HELLO WORLD! "

; ==================================================================
; IRQ HANDLER
; This code runs 60 times per second.
; ==================================================================
IRQ_HANDLER:
PHA                ; Save A
TXA                ; Save X
PHA

; --- Poll Keyboard ---
LDA KEY_BUFFER     ; Load key buffer
BEQ EXIT_IRQ       ; If 0, skip to exit

; --- Key was pressed ---
STA SOUND_PITCH    ; Set sound pitch to key value
LDX CURSOR_POS     ; Load cursor pos from scratchpad
STA SCREEN_LINE_2,X; Store key to line 2 (0x0400 + 40)
INX                ; Advance cursor
STX CURSOR_POS     ; Save new cursor pos
LDA #$00           ; Clear A
STA KEY_BUFFER     ; Clear key buffer
JMP EXIT_IRQ       ; Done


EXIT_IRQ:
PLA                ; Restore X
TAX
PLA                ; Restore A
RTI                ; Return from Interrupt
