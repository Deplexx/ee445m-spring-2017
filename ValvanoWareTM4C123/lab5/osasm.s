;/*****************************************************************************/
; OSasm.s: low-level OS commands, written in assembly
; Mar 30, 2017
; Dung Nguyen & Nico Cortes


        AREA |.text|, CODE, READONLY, ALIGN=2
        THUMB
        REQUIRE8
        PRESERVE8

        EXTERN  RunPt
        EXPORT  StartOS
        EXPORT  SysTick_Handler
		EXPORT  PendSV_Handler
		EXPORT  SVC_Handler
		EXPORT  JumpAsm
		
		    ALIGN
PF1    EQU     0x40025008
PF2    EQU     0x40025010

;DisableInterrupts
;        CPSID   I
;        BX      LR

;EnableInterrupts
;        CPSIE   I
;        BX      LR

SysTick_Handler                ; 1) Saves R0-R3,R12,LR,PC,PSR
    CPSID   I                  ; 2) Prevent interrupt during switch
	LDR     R0, =PF1           ; toggle heartbeat
	LDR     R1, [R0]
	EOR     R1, #0x02
	STR     R1, [R0]
	EOR     R1, #0x02
	STR     R1, [R0]
    PUSH    {R4-R11}           ; 3) Save remaining regs r4-11
	MOV     R0, #0xD000D000
    PUSH    {R0}
	ADD     SP, #4
    LDR     R0, =RunPt         ; 4) R0=pointer to RunPt, old thread
    LDR     R1, [R0]           ;    R1 = RunPt
    STR     SP, [R1]           ; 5) Save SP into TCB
	
	LDR     R3, [R1,#4]        ; R3 is an iterator, R3 = RunPt->next
	MOV     R5, R3             ; R5 nextThread
	MOV     R4, #0x7FFFFFFF    ; R4 priority

SysTick_Priority
	LDR     R2, [R1,#16]       ; R3->sleep
    CMP     R2, #0
	BNE     SysTick_Next
	LDR     R2, [R1,#28]       ; R3->blocked
	CMP     R2, #0
	BNE     SysTick_Next
	LDR     R2, [R1,#24]       ; R3->pri
	CMP     R2, R4             ; R2 < R4 ?
	BGE     SysTick_Next
SysTick_Update
    MOV     R5, R3
	MOV     R4, R2
SysTick_Next
    LDR     R3, [R3,#4]        ; increment
	CMP     R3, R1
	BNE     SysTick_Priority   ; R3 != RunPt
	STR     R5, [R0]           ; update RunPt
	LDR     SP, [R5]
    	
;SysTick_Next_Thread
;    LDR     R1, [R1,#4]        ; 6) R1 = RunPt->next
;    LDR     R2, [R1,#16]       ; RunPt->next->sleep
;    CMP     R2, #0
;    BNE     SysTick_Next_Thread
;	LDR     R2, [R1, #28]      ; RunPt->next->blocked
;	CMP     R2, #0             ; is thread blocked?
;	BNE     SysTick_Next_Thread
;   STR     R1, [R0]           ;    RunPt = R1
;    LDR     SP, [R1]           ; 7) new thread SP; SP = RunPt->sp;

    POP     {R4-R11}           ; 8) restore regs r4-11
	LDR     R0, =PF1           ; toggle heartbeat
	LDR     R1, [R0]
	EOR     R1, #0x02
	STR     R1, [R0]
    CPSIE   I                  ; 9) tasks run with interrupts enabled
    BX      LR                 ; 10) restore R0-R3,R12,LR,PC,PSR
	
PendSV_Handler                ; 1) Saves R0-R3,R12,LR,PC,PSR
    CPSID   I                  ; 2) Prevent interrupt during switch
	LDR     R0, =PF2
	LDR     R1, [R0]
	EOR     R1, #0x04
	STR     R1, [R0]
	EOR     R1, #0x04
	STR     R1, [R0]
    PUSH    {R4-R11}           ; 3) Save remaining regs r4-11
	MOV     R0, #0xB000B000
    PUSH    {R0}
	ADD     SP, #4
    LDR     R0, =RunPt         ; 4) R0=pointer to RunPt, old thread
    LDR     R1, [R0]           ;    R1 = RunPt
    STR     SP, [R1]           ; 5) Save SP into TCB

	LDR     R3, [R1,#4]        ; R3 is an iterator, R3 = RunPt->next
	MOV     R1, R3
	MOV     R5, R3             ; R5 nextThread
	MOV     R4, #0x7FFFFFFF    ; R4 priority

PendSV_Priority
	LDR     R2, [R1,#16]       ; R3->sleep
    CMP     R2, #0
	BNE     PendSV_Next
	LDR     R2, [R1,#28]       ; R3->blocked
	CMP     R2, #0
	BNE     PendSV_Next
	LDR     R2, [R1,#24]       ; R3->pri
	CMP     R2, R4             ; R2 < R4 ?
	BGE     PendSV_Next
PendSV_Update
    MOV     R5, R3
	MOV     R4, R2
PendSV_Next
    LDR     R3, [R3,#4]        ; increment
	CMP     R3, R1
	BNE     PendSV_Priority   ; R3 != RunPt
	STR     R5, [R0]           ; update RunPt
	LDR     SP, [R5]

;PendSV_Next_Thread
;    LDR     R1, [R1,#4]        ; 6) R1 = RunPt->next
;    LDR     R2, [R1,#16]       ; RunPt->next->sleep
;    CMP     R2, #0
;    BNE     PendSV_Next_Thread
;	LDR     R2, [R1, #28]      ; RunPt->next->blocked
;	CMP     R2, #0             ; is thread blocked?
;	BNE     PendSV_Next_Thread
;    STR     R1, [R0]           ;    RunPt = R1
;    LDR     SP, [R1]           ; 7) new thread SP; SP = RunPt->sp;

    POP     {R4-R11}           ; 8) restore regs r4-11
	LDR     R0, =PF2
	LDR     R1, [R0]
	EOR     R1, #0x04
	STR     R1, [R0]
    CPSIE   I                  ; 9) tasks run with interrupts enabled
    BX      LR                 ; 10) restore R0-R3,R12,LR,PC,PSR
	
StartOS
    LDR     R0, =RunPt         ; currently running thread
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

JumpAsm
	ADD		R0, #30
	BX		R0

SVC_Handler
    BX      LR
;0 OS_Id
;1 OS_Kill
;2 OS_Sleep
;3 OS_Time
;4 OS_AddThread

    ALIGN
    END
