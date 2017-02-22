;/*****************************************************************************/
; OSasm.asm: low-level OS commands, written in assembly                       */
; Runs on LM4F120/TM4C123
; A very simple real time operating system with minimal features.
; Daniel Valvano
; January 29, 2015
;
; This example accompanies the book
;  "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
;  ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015
;
;  Programs 4.4 through 4.12, section 4.2
;
;Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
;    You may use, edit, run or distribute this file
;    as long as the above copyright notice remains
; THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
; OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
; MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
; VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
; OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
; For more information about my classes, my research, and my books, see
; http://users.ece.utexas.edu/~valvano/
; */

        .thumb
        .text
        .align 2


        .global  RunPt            ; currently running thread
        .global  OS_DisableInterrupts
        .global  OS_EnableInterrupts
        .global  StartOS
        .global  SysTick_Handler
        .global  OS_Wait
        .global  OS_bWait
        .global  OS_Signal
        .global  OS_bSignal


OS_DisableInterrupts:  .asmfunc
        CPSID   I
        BX      LR
       .endasmfunc

OS_EnableInterrupts:  .asmfunc
        CPSIE   I
        BX      LR
       .endasmfunc

SysTick_Handler:  .asmfunc     ; 1) Saves R0-R3,R12,LR,PC,PSR
    CPSID   I                  ; 2) Prevent interrupt during switch
    PUSH    {R4-R11}           ; 3) Save remaining regs r4-11
    LDR     R0, RunPtAddr      ; 4) R0=pointer to RunPt, old thread
    LDR     R1, [R0]           ;    R1 = RunPt
    STR     SP, [R1]           ; 5) Save SP into TCB
    LDR     R1, [R1,#4]        ; 6) R1 = RunPt->next
    STR     R1, [R0]           ;    RunPt = R1
    LDR     SP, [R1]           ; 7) new thread SP; SP = RunPt->sp;
    POP     {R4-R11}           ; 8) restore regs r4-11
    CPSIE   I                  ; 9) tasks run with interrupts enabled
    BX      LR                 ; 10) restore R0-R3,R12,LR,PC,PSR
   .endasmfunc
RunPtAddr .field RunPt,32

StartOS:  .asmfunc
    LDR     R0, RunPtAddr      ; currently running thread
    LDR     R2, [R0]           ; R2 = value of RunPt
    LDR     SP, [R2]           ; new thread SP; SP = RunPt->stackPointer;
    POP     {R4-R11}           ; restore regs r4-11
    POP     {R0-R3}            ; restore regs r0-3
    POP     {R12}
    POP     {LR}               ; discard LR from initial stack
    POP     {LR}               ; start location
    POP     {R1}               ; discard PSR
    CPSIE   I                  ; Enable interrupts at processor level
    BX      LR                 ; start first thread
   .endasmfunc

   ; void OS_Wait(Sema4Type *semaPt)
OS_Wait_store_back:
    STREX R2, R1, [R0]
OS_Wait: .asmfunc
    LDREX R1, [R0]
    CMP R1, #0
    BLT OS_Wait_store_back
    ADD R1, R1, #-1
    STREX R2, R1, [R0]
    CMP R2, #0
    BNE OS_Wait
    BX LR
    .endasmfunc

; void OS_bWait(Sema4Type *semaPt)
OS_bWait_store_back:
    STREX R2, R1, [R0]
OS_bWait: .asmfunc
    LDREX R1, [R0]
    CMP R1, #0
    BNE OS_bWait_store_back
    MOV R1, #-1
    STREX R2, R1, [R0]
    CMP R2, #0
    BNE OS_bWait
    BX LR
    .endasmfunc

; void OS_bSignal(Sema4Type *semaPt)
OS_Signal: .asmfunc
    LDREX R1, [R0]
    ADD R1, R1, #1
    STREX R2, R1, [R0]
    CMP R2, #0
    BNE OS_Signal
    BX LR
    .endasmfunc

; void OS_bSignal(Sema4Type *semaPt)
OS_bSignal: .asmfunc
    LDREX R1, [R0]
    MOV R1, #0
    STREX R2, R1, [R0]
    CMP R2, #0
    BNE OS_bSignal
    BX LR
    .endasmfunc

   .end
