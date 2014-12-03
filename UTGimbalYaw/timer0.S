; This file provides some common timing functions such as wait, check if some
; time has elapsed, and measure how much time has elapsed. This file assumes
; that TIMER0 has been initialized to produce a an overflow interrupt at exactly
; 1kHz.

; In addition to being functional, this file was an attempt to learn assembly
; programming for with avr-gcc, so contains some comments meant to be
; informational to a beginner.

; The following references were very helpful in making this file:
; 8-bit AVR Instruction Set
; Atmel AT1886: Mixing Assembly and C with AVRGCC (has many errors though)
; ATmega48A/PA/88A/PA/168A/PA/328/P Datasheet (Instruction Set Summary)

; Encapsulating the include in a .nolist statement prevents a bunch of
; unnecessary output in the .lst file.
.nolist
#include <avr/io.h>
.list

; This is actually not necessary because avr-gcc seems to treat all unknown
; symbols as an external. It's included as evidence to the reader that this
; symbol is declared elsewhere.
.extern ms_timestamp_

; I'm not sure if this is the right thing to do. The .lst files that come from c
; files have __SREG__ hard-coded to an address (0x3f). I don't know where that
; comes from and whether or not it is device specific. This might be more
; generic.
__SREG__ = _SFR_IO_ADDR(SREG)

; This interrupt handler increments a 16-bit word every time TIMER0 reaches the
; value in OCR0A (which should occur at 1kHz).
;;
; Normally, when a function is called from c, it makes sure that registers R0,
; R18-27 and R30-31 are freed up for the next function to use. This is not true
; for interrupt handlers, which are called immediately. When an interrupt
; occurs, the interrupt flag is automatically cleared and the program counter is
; pushed to the stack and set to the appropriate place in the interrupt vector,
; which should contain a jump statement to a handler. No other changes to SREG
; or other registers are made. The status of SREG and the contents of any
; register may be critical to correct execution of whatever process was just
; interrupted and therefore must be preserved. In other words, and register that
; will be modified by this interrupt handler (including SREG) must first be
; preserved, typically by pushing it to the stack before it is used, then
; popping it back before returning.
;;
; Note that interrupt handlers return with the special reti instruction, which
; sets the I-bit in SREG AFTER the program counter is repopulated with the value
; that was on the stack.
;;
; Note that ms_timestamp_ is a global variable and therefore has a constant
; place in memory. I.E. it's memory location is a constant that can be used with
; instructions like lds and sts.
;;
; My approach to this function:
;;
; To speed up this function, I first load and increment only the lower 8-bits of
; the timestamp. If the low bits overflow, then I load, increment and restore
; the upper 8-bits. This means that 255 out of 256 times, this routine is much
; faster than loading, incrementing and restoring both bytes. Furthermore, the 1
; bad case of 256 is only 4 cycles longer than always loading, incrementing and
; restoring both bytes (2 cycles for breq and 2 cycles for rjmp).
;;
; Register R1 is used to hold SREG so that it can be restored upon completion of
; this interrupt handler. The c compiler (avr-gcc) assumes that R1 always
; contains 0x00, so rather than push a register to the stack to make space to
; store SREG and then pop it's contents back after we're done (4 cycles) it is
; better to use R1 and then just clear it when we're done (1 cycle).
; Unfortunately, this is not feasible since the clr instruction sets the Z flag
; in SREG, thereby defeating the goal of preserving SREGthe ldi instruction does
; not, but ldi can only be used with register R16-R31. To get around this, I
; decided to copy (mov) R1 to R26 once R26 was not needed anymore, clear R1,
; then copy R26 to SREG. This mov adds an extra cycle, but it is still faster
; than pushing and popping by 2 cycles.
.global TIMER0_COMPA_vect
TIMER0_COMPA_vect:
  in r1, __SREG__  ; Save SREG in R1 (normally reserved for 0)
  push r26  ; Save R26 (X-) to the stack
  lds r26, ms_timestamp_  ; Load the lower byte of ms_timestamp
  inc r26  ; Increment the lower byte
  breq MS_HI  ; If r26 rolled over to 0, then branch to MS_HI
MS_SAVE:
  sts ms_timestamp_, r26  ; Save lower byte to ms_timestamp
  mov r26, r1  ; (because clr will set the Z flag)
  clr r1  ; Restore R1 to 0
  out __SREG__, r26  ; Restore the state of SREG
  pop r26  ; Restore R26 (X-) from the stack
  reti;
MS_HI:
  push r27  ; Save R27 (X+) to the stack
  lds r27, ms_timestamp_ + 1  ; Load the upper byte of ms_timestamp
  inc r27  ; Increment the upper byte of ms_timestamp
  sts ms_timestamp_ + 1, r27  ; Save the upper byte to ms_timestamp
  pop r27  ; Restore R27 (X+) from the stack
  rjmp MS_SAVE  ; Jump back to save lower byte

; uint16_t SetDelay (uint16_t t);
;;
; This function returns a timestamp corresponding to "t" ms in the future. This
; timestamp can be checked against the current timestamp to see if a certain
; amount of time has passed. This function works for delays up to 32767 ms.
;;
; Note that ms_timestamp_ could be updated by interrupt between the 2 load (lds)
; instructions, giving unexpected results. Interrupts are disabled and re-
; enabled to avoid this.
;;
; Note that parameters are passed to functions in chunks not smaller than
; 16-bits. The parameters are passed in the order R25:24, R23:22, and so on to
; R9:8, after which arguments are put on the stack. 8-bit parameters just use
; the lower bit of a pair. The result is passed back in registers R25:18, also
; arranged in 2-byte pairs so that the result fits into [R19 R18 R21 R20 R23 R22
; R25 R24] aligned to the right (weird).
;;
; Note that in this case the input and output are the same registers, so the
; timestamp can be subtracted from the input in place and doesn't require any
; subsequent move.
.global SetDelay
SetDelay:
  cli
  lds r26, ms_timestamp_
  lds r27, ms_timestamp_ + 1
  sei
  adiw r26, 1  ; Add 1 in case we are close to the next tick.
  add r24, r26
  adc r25, r27
  ret

; int8_t CheckDelay (uint16_t t);
;;
; This function compares a timestamp to the current timestamp and returns a 1 if
; the timestamp is in the past. This function works for delays up to 32767 ms.
;;
; Note that the brmi and similar instructions do not work as is written in the
; "8-bit AVR Instruction Set" reference. Instead, "brmi ." will jump to the next
; instruction (essentially a nop x 2), "brmi .+2" will skip 1 instruction
; (assuming it is a 1-word instruction), "brmi .+4" will skip 2 and so on.
;;
; My approach to this function:
;;
; Subtract the current timestamp from the target timestamp. If the result is
; negative, then the target timestamp should have elapsed.
.global CheckDelay
CheckDelay:
  cli
  lds r26, ms_timestamp_
  lds r27, ms_timestamp_ + 1
  sei
  sub r24, r26
  sbc r25, r27
  ldi r24, 0x01  ; Set the return value to 1
  brmi .+2  ; If the result is negative, then skip an instruction.
  ldi r24, 0x00  ; Set the return value to 0
  ret

; uint16_t GetDelay(uint16_t *last_time);
;;
; This function returns the amount of time that has elapsed since the timestamp
; "last_time" has occurred. This function works for time periods up to 32767 ms.
; The function also automatically updates last_time so that it can be easily be
; called periodically.
.global GetDelay
GetDelay:
  movw r30, r24  ; Copy address of "last_time" to the Z register pair (R31:30)
  cli
  lds r24, ms_timestamp_
  lds r25, ms_timestamp_ + 1
  sei
  ld r26, Z  ; Load the byte at Z (lower bytes of "last_time") to R24
  ldd r27, Z + 1  ; Load the byte at Z+1 (upper bytes of "last_time") to R25
  st Z, r24  ; Store the current timestamp to "last_time"
  std Z + 1, r25
  sub r24,r26  ; Subtract "last_time" from the current timestamp
  sbc r25,r27
  ret
