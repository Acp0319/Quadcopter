ICSRaddr 		EQU  	0xE000ED04
PendSVSet 		EQU 	0x10000000
SHPR3Addr 		EQU		0xE000ED22
PendSVPrio		EQU		0xFF
	
RCC_BASE        EQU     0x40023800
RCC_CR          EQU     RCC_BASE + 0x00
RCC_PLLCFGR     EQU     RCC_BASE + 0x04
RCC_CFGR        EQU     RCC_BASE + 0x08
RCC_AHB1ENR     EQU     RCC_BASE + 0x30
FLASH_ACR       EQU     0x40023C00    ; Flash访问控制寄存器

SYSTICK_BASE    EQU     0xE000E010
SYSTICK_CTRL    EQU     SYSTICK_BASE + 0x00
SYSTICK_LOAD    EQU     SYSTICK_BASE + 0x04
SYSTICK_VAL     EQU     SYSTICK_BASE + 0x08

	extern OSRunning
	extern OSPrioCur
	extern OSPrioHighRdy
	extern OSTCBCur
	extern OSTCBHighRdy
	extern OS_CPU_ExceptStkBase
    extern OS_KA_BASEPRI_Boundary

	AREA |.text|, CODE, READONLY, ALIGN=2 
	THUMB 
	REQUIRE8 
	PRESERVE8 
		
OSCtxSw PROC
	EXPORT OSCtxSw
	LDR R0,=ICSRaddr
	LDR R1,=PendSVSet
	STR R1,[R0]
	BX LR 
	ENDP
		
OSIntCtxSw PROC
	EXPORT OSIntCtxSw
	LDR R0,=ICSRaddr
	LDR R1,=PendSVSet
	STR R1,[R0]
	BX LR 
	ENDP
		
OSStartHighRdy PROC
	EXPORT OSStartHighRdy
	CPSID I	
	
	;设置PendSV优先级为最低
	LDR R0,=SHPR3Addr
	LDR R1,=PendSVPrio
	STRB R1,[R0]
		
	MOVS R0,#0
	MSR PSP,R0
	
	;-----------------------------------------------------------------------------------------------------------
	LDR     R0, =OS_CPU_ExceptStkBase
    LDR     R1, [R0]
    MSR     MSP, R1
	;-----------------------------------------------------------------------------------------------------------
	
	LDR R0,=OSRunning      ;BOOLEAN OSRunning;
	MOV R1,#1
	STRB R1,[R0]
	
	LDR R0,=OSTCBCur
	LDR R0,[R0]
	LDR R0,[R0]
	MSR PSP,R0
	
	MRS R0,CONTROL
	ORR R0,R0,#2
	MSR CONTROL,R0
	ISB			;作用是?
	
	LDMFD SP!,{R4-R11}
	LDMFD SP!,{LR}
	LDMFD SP!,{R0-R3}
	LDMFD SP!,{R12,LR}
	LDMFD SP!,{R1,R2}
	CPSIE I
	BX R1 
	ENDP
		
OS_CPU_SR_Restore PROC
	EXPORT OS_CPU_SR_Restore
	CPSID I
	MSR BASEPRI,R0
	DSB
	ISB
	CPSIE I
	BX LR 
	ENDP
		
OS_CPU_SR_Save PROC
	EXPORT OS_CPU_SR_Save
	CPSID I
	PUSH {R1}
	MRS R1,BASEPRI
	MSR BASEPRI,R0		;屏蔽优先级数值大于等于4的中断，与OS_CPU_SR_Save传入的参数有关
	DSB
	ISB					;清空流水线
	MOV R0,R1			;R0存储返回值
	POP {R1}
	CPSIE I
	BX LR 
	ENDP

OS_CPU_PendSVHandler
	EXPORT OS_CPU_PendSVHandler
    CPSID   I
    MOV32   R2, OS_KA_BASEPRI_Boundary
    LDR     R1, [R2]
    MSR     BASEPRI, R1
    DSB
    ISB
    CPSIE   I

    MRS     R0, PSP
    STMFD   R0!, {R4-R11, R14}

    LDR     R5, =OSTCBCur
    LDR     R1, [R5]
    STR     R0, [R1]

    MOV     R4, LR
    ;BL      OSTaskSwHook

    LDR     R0, =OSPrioCur
    LDR     R1, =OSPrioHighRdy
    LDRB    R2, [R1]
    STRB    R2, [R0]

    LDR     R1, =OSTCBHighRdy
    LDR     R2, [R1]
    STR     R2, [R5]

    ORR     LR,  R4, #0x04
    LDR     R0,  [R2]
    LDMFD   R0!, {R4-R11, R14}
    MSR     PSP, R0

    MOV32   R2, #0
    CPSID   I
    MSR     BASEPRI, R2
    DSB
    ISB
    CPSIE   I
    BX      LR

    ALIGN
		
OSTaskDelHook PROC
	EXPORT OSTaskDelHook
	BX LR 
	ENDP
		
OSTimeTickHook PROC
	EXPORT OSTimeTickHook
	BX LR 
	ENDP
		
OSTaskStatHook PROC
	EXPORT OSTaskStatHook
	BX LR 
	ENDP
	
	ALIGN	
	END
