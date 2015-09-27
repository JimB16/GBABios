/*
Hello and Welcome to a dissassembled GBABios

"blown" is my own bl-command, because the most
bls go to an adress that isn't already
dissassembled, so I had to write a function
that makes a command that goes to an address
that isn't already included.
		blown	0x3710, 0x2CC
Here we're at address 0x2CC and want to jump to 0x3710

The same is with some b-commands, but here
I've only included this commands as data.
		.hword	0xE020
		@b		0x2F3C

And some add/sub-commands have other binary-data
with gas (then with the assembler that Nintendo used).
So I made a Halfword of the original data
and commented the dissassembled command.
		.hword	0x1C80
		@add		r0, r0, #2



*/

.include "gball.s"

.text
.global _start

/*
y = current adress, where the command is
x = address where the jump goes to
*/
.macro	blown	x=0, y=0
.hword	0b1111000000000000 | ((((\x-\y)>>12)&0b11111111111))
.hword	0b1111100000000000 | ((((\x-(\y+2))>>1)&0b11111111111)-1)
.endm

.macro	blown2	x=0, y=0
.hword	0b1111000000000000 | ((((\x-\y)>>12)&0b11111111111))
.hword	0b1111100000000000 | ((((\x-(\y+2))>>1)&0b11111111111)-1)
.endm

.align
.arm

_start:
		b		Reset				@ Reset [0x68]
		b		DebugHandler		@ Undefined Instruction [0x1C]
		b		SWIHandler			@ Software Interrupt (SWI [0x140])
		b		DebugHandler		@ Prefetch Abort [0x1C]
		b		DebugHandler		@ Data Abort [0x1C]
		b		DebugHandler		@ Reserved [0x1C]
		b		IRQHandler			@ Normal Interrupt (IRQ [0x128])
DebugHandler:						@ Fast Interrupt (FIQ)
		ldr		sp, b1B8+0xC		@ 0x1C4 =0x03007FF0
		stmdb	sp!, {r12, lr}
		mrs		r12, SPSR
		mrs		r14, CPSR
		stmdb	sp!, {r12, r14}
		mov		r12, #0x08000000
		ldrb	r14, [r12, #0x9C]
		cmp		r14, #0xA5			@ If debugging is enabled
		bne		b54
		ldreqb	r14, [r12, #0xB4]	@ load Device type
		andeqs	r14, r14, #0x80
		adr		r14, b54
		ldrne	pc, =0x09FE2000		@ 1MBIT DACS [0x274]
		ldreq	pc, =0x09FFC000		@ 8MBIT DACS [0x278]
b54:
		ldr		sp, b1B8+0x8		@ 0x1C0 =0x03007FE0
		ldmia	sp!, {r12, r14}
		msr		SPSR, r12
		ldmia	sp!, {r12, lr}
		subs	pc, lr, #0x4

Reset:
		cmp		lr, #0x0
		moveq	lr, #0x4
/*
4000300h - POSTFLG - BYTE - Undocumented - Post Boot / Debug Control (R/W)
After initial reset, the GBA BIOS initializes the register to 01h, and any further execution of the Reset vector (00000000h) will pass control to the Debug vector (0000001Ch) when sensing the register to be still set to 01h.

  Bit   Expl.
  0     Undocumented. First Boot Flag  (0=First, 1=Further)
  1-7   Undocumented. Not used.
*/
		mov		r12, #0x04000000
		ldrb	r12, [r12, #0x300]
		teq		r12, #0x1			@ If first start
		mrseq	r12, CPSR
		orreq	r12, r12, #0xC0
		msreq	CPSR, r12
		beq		DebugHandler
HardReset:
		mov		r0, #0xDF			@ Systemmode | ARMMode | FIQ + IRQ disable
		msr		CPSR, r0
		mov		r4, #0x04000000		@ IME = 0
		strb	r4, [r4, #0x208]
		bl		bE0
		adr		r0, b300				@ add		r0, pc, #0x258		@ r0 = 0x300
		str		r0, [sp, #0xFC]
		ldr		r0, =b1928+1			@ [0x27C] = 0x1929
		adr		lr, SoftReset			@ add		lr, pc, #0x0		@ lr = 0xB4
		bx		r0

@ bB4
SoftReset:
		mov		r4, #0x04000000
		ldrb	r2, [r4, #-6]
		bl		bE0
		cmp		r2, #0x0
		ldmdb	r4, {r0-r12}
		movne	lr, #0x02000000
		moveq	lr, #0x08000000
		mov		r0, #0x1F			@ Systemmode
		msr		CPSR, r0
		mov		r0, #0x0
		bx		lr
bE0:
		mov		r0, #0xD3			@ Supervisor (SWI) Mode | FIQ IRQ disable
		msr		CPSR, r0
		ldr		sp, b1B8+0x8		@ 0x1C0 (= 0x03007FA0)
		mov		lr, #0x0
		msr		SPSR, lr
		mov		r0, #0xD2			@ IRQ Mode | FIQ + IRQ disable
		msr		CPSR, r0
		ldr		sp, b1B8+0x4		@ 0x1BC (= 0x03007FA0)
		mov		lr, #0x0
b100:
		msr		SPSR, lr
		mov		r0, #0x5F			@ System Mode | FIQ disabled
		msr		CPSR, r0
		ldr		sp, b1B8			@ 0x1B8 (= 0x03007F00)
		add		r0, pc, #0x1
		bx		r0
.thumb
	@ Reset memory from 0x03FFFFE0 to 0x04000000
		mov		r0, #0
		ldr		r1, =-0x200		@ 0x280
b120:
		str		r0, [r4, r1]
		.hword	0x1D09
		@add		r1, r1, #0x4
		blt		b120
		bx		lr

.arm
IRQHandler:
		stmdb	sp!, {r0-r3, r12, lr}	@ save registers to SP_irq
		mov		r0, #0x04000000			@ ptr+4 to 03FFFFFC (mirror of 03007FFC)
		add		lr, pc, #0x0			@ retadr for USER handler $+8=138h
		ldr		pc, [r0, #-4]			@ jump to [03FFFFFC] USER handler
		ldmia	sp!, {r0-r3, r12, lr}	@ restore registers from SP_irq
		subs	pc, lr, #0x4			@ return from IRQ (PC=LR-4, CPSR=SPSR)

SWIHandler:
		stmdb	sp!, {r11, r12, lr}
		ldrb	r12, [lr, #-2]
		adr		r11, b1C8
		ldr		r12, [r11, r12, lsl #2]
		mrs		r11, SPSR
		stmdb	sp!, {r11}
		and		r11, r11, #0x80
		orr		r11, r11, #0x1F
		msr		CPSR, r11
		stmdb	sp!, {r2, lr}
		adr		lr, b170
		bx		r12
b170:
		ldmia	sp!, {r2, lr}
		mov		r12, #0xD3
		msr		CPSR, r12
		ldmia	sp!, {r11}
		msr		SPSR, r11
		ldmia	sp!, {r11, r12, lr}
		movs	pc, lr

		mov		r12, #0x04000000		@ DISPCNT - LCD Control (Read/Write)
		mov		r2, #0x4
		strb	r2, [r12, #1]
		mov		r2, #0x8
		strb	r2, [r12]
Halt:
		mov		r2, #0x0
		b		CustomHalt
Stop:
		mov		r2, #0x80
CustomHalt:
		mov		r12, #0x04000000
		strb	r2, [r12, #0x301]		@ 4000301h - HALTCNT - BYTE - Undocumented - Low Power Mode Control (W)
		bx		lr

b1B8:
		.word	0x03007F00
		.word	0x03007FA0
		.word	0x03007FE0
		.word	0x03007FF0
b1C8:
		.word	SoftReset				@ SoftReset [0xB4]
		.word	RegisterRamReset+1		@ RegisterRamReset [0x9C3]
		.word	Halt					@ Halt [0x1A0]
		.word	Stop					@ Stop/Sleep [0x1A8]
		.word	IntrWait				@ IntrWait [0x330]
		.word	VBlankIntrWait			@ VBlankIntrWait [0x328]
		.word	Div						@ Div [0x3B4]
		.word	DivArm					@ DivArm [0x3A8]
		.word	Sqrt					@ Sqrt [0x404]
		.word	ArcTan					@ ArcTan [0x474]
		.word	ArcTan2+1				@ ArcTan2 [0x4FD]
		.word	CpuSet+1				@ CpuSet [0xB4D]
		.word	CpuFastSet				@ CpuFastSet [0xBC4]
		.word	GetBiosChecksum			@ GetBiosChecksum [0x378]
		.word	BgAffineSet				@ BgAffineSet [0xC2C]
		.word	ObjAffineSet			@ ObjAffineSet [0xCE0]
		.word	BitUnPack				@ BitUnPack [0xF60]
		.word	LZ77UnCompWram			@ LZ77UnCompWram [0x10FC]
		.word	LZ77UnCompVram			@ LZ77UnCompVram [0x1194]
		.word	HuffUnComp				@ HuffUnComp [0x1014]
		.word	RlUnCompWram+1			@ RlUnCompWram [0x1279]
		.word	RlUnCompVram+1			@ RlUnCompVram [0x12C1]
		.word	Diff8bitUnFilterWram+1	@ Diff8bitUnFilterWram [0x1333]
		.word	Diff8bitUnFilterVram+1	@ Diff8bitUnFilterVram [0x135D]
		.word	Diff16bitUnFilter+1		@ Diff16bitUnFilter [0x1399]
		.word	SoundBias+1				@ SoundBias [0x801]
		.word	SoundDriverInit+1		@ SoundDriverInit [0x1665]
		.word	SoundDriverMode+1		@ SoundDriverMode [0x179D]
		.word	SoundDriverMain+1		@ SoundDriverMain [0x1DC5]
		.word	SoundDriverVSync+1		@ SoundDriverVSync [0x210D]
		.word	SoundChannelClear+1		@ SoundChannelClear [0x1825]
		.word	MidiKey2Freq+1			@ MidiKey2Freq [0x18D9]
		.word	SoundWhatever0+1		@ SoundWhatever0 [0x13C5]
		.word	SoundWhatever1+1		@ SoundWhatever1 [0x1435]
		.word	SoundWhatever2+1		@ SoundWhatever2 [0x14C1]
		.word	SoundWhatever3+1		@ SoundWhatever3 [0x14FD]
		.word	SoundWhatever4+1		@ SoundWhatever4 [0x1515]
		.word	MultiBoot+1				@ MultiBoot [0x28CF]
		.word	HardReset				@ HardReset [0x8C]
		.word	CustomHalt				@ CustomHalt [0x1AC]
		.word	SoundDriverVSyncOff+1	@ SoundDriverVSyncOff [0x1879]
		.word	SoundDriverVSyncOn+1	@ SoundDriverVSyncOn [0x18C9]
		.word	SoundGetJumpList+1		@ SoundGetJumpList [0x2693]
b274:
.pool

.thumb
.globl b284
.thumb_func
b284:
		mov		r4, #4
		lsl		r4, r4, #24
		mov		r5, #5
		lsl		r5, r5, #24
		mov		r6, #6
		lsl		r6, r6, #24
		mov		r1, #0
		mov		r0, #194
		mov		r2, r4
		add		r2, #0x80
		strb	r0, [r2, #2]
		strb	r0, [r2, #9]
		mov		r0, #0xFF
		.hword	0x1C80
		@add		r0, r0, #2
		mov		r2, #160
		mov		r3, #144
		str		r6, [sp]
		mov		r7, #240
		str		r7, [sp, #4]
		bl		b79E
		mov		r0, #131
		lsl		r0, r0, #7
		strh	r0, [r4, #0xC]
		ldr		r0, b2F0+0x4				@ 0x2F4
		str		r0, [r4, #0x28]
		asr		r0, r0, #16
		lsl		r0, r0, #11
		str		r0, [r4, #44]
		ldr		r3, b2F0+0x8				@ 0x2F8
		str		r3, [r5]
		ldrh	r3, [r5]
		ldr		r7, b2F0+0xC				@ 0x2FC
b2C6:
		lsr		r2, r4, #17
		add		r2, r2, r4
		strh	r7, [r2, #2]
@ 0xfa20f003
		blown2	0x3710, 0x2CC
		mov		r0, #4
		strb	r0, [r4, #1]
		strb	r0, [r4]
		sub		r3, r3, r7
		strh	r3, [r5]
		bgt		b2C6
		mvn		r0, r1
		str		r0, [sp, #8]
		add		r4, #212
		add		r1, sp, #8
		str		r1, [r4]
		str		r6, [r4, #4]
		ldr		r1, b2F0					@ 0x2F0
		str		r1, [r4, #8]
		blown	0x3718, 0x2EC
b2F0:
.pool
		.word	0x85006000
		.word	0xFFFFD800
		.word	0x7FFF7BDE
		.word	0x00000C63
.arm
b300:
		mov		r3, #0x04000000
		ldr		r2, [r3, #512]
		and		r2, r2, r2, lsr #16
		ands	r1, r2, #0x80
		ldrne	r0, [pc, #1952]
		andeq	r1, r2, #0x1
		ldreq	r0, [pc, #1948]
		streqh	r2, [r3, #-8]
		strb	r1, [r3, #514]
		bx		r0

VBlankIntrWait:
		mov		r0, #0x1
		mov		r1, #0x1
IntrWait:
		stmdb	sp!, {r4,lr}
		mov		r3, #0x0
		mov		r4, #0x1
		cmp		r0, #0x0
		blne	b358
b344:
		strb	r3, [r12, #0x301]
		bl		b358
		beq		b344
		ldmia	sp!, {r4,lr}
		bx		lr
b358:
		mov		r12, #0x04000000
		strb	r3, [r12, #0x208]
		ldrh	r2, [r12, #-8]
		ands	r0, r1, r2
		eorne	r2, r2, r0
		strneh	r2, [r12, #-8]
		strb	r4, [r12, #0x208]
		bx		lr

GetBiosChecksum:
		mov		r0, #0x0
		mov		r3, #0x0
b380:
		mov		r12, #0xDF
		ldmia	r3!, {r2}
		msr		CPSR_fc, r12
		add		r0, r0, r2
		movs	r1, r3, lsr #14
		beq		b380
		bx		lr
.thumb
.globl b39C
.thumb_func
b39C:
		cmp		r0, #0
		bgt		b3A2
		neg		r0, r0
b3A2:
		bx		lr

.globl	DivThumb
.thumb_func
DivThumb:
b3A4:
		adr		r3, Div
		bx		r3

/*
SWI 07h - DivArm
Same as "SWI 06h Div", but incoming parameters are exchanged, r1/r0 (r0=Denom, r1=number). For compatibility with ARM's library. Slightly slower (3 clock cycles) than SWI 06h.
*/
.arm
DivArm:
		mov		r3, r0
		mov		r0, r1
		mov		r1, r3

/*
SWI 06h - Div
Signed Division, r0/r1.
  r0  signed 32bit Number
  r1  signed 32bit Denom

Return:
  r0  Number DIV Denom ;signed
  r1  Number MOD Denom ;signed
  r3  ABS (Number DIV Denom) ;unsigned
  
For example, incoming -1234, 10 should return -123, -4, +123.
The function usually gets caught in an endless loop upon division by zero.
*/
Div:
@ r0 and r1 must be positive
@ r12 saves if result is positive or negative
		ands	r3, r1, #0x80000000
		rsbmi	r1, r1, #0x0			@ r1 must be positive for calculation
		eors	r12, r3, r0, asr #32	@ if(r0 and r1 are both positive or negative) {r12 = 0}
										@ and if(r0 is negative) {Carry is Set}
		rsbcs	r0, r0, #0x0			@ r0 must be positive for calculation
		movs	r2, r1
b3C8:
		cmp		r2, r0, lsr #1
		movls	r2, r2, lsl #1
		bcc		b3C8
b3D4:
		cmp		r0, r2
		adc		r3, r3, r3
		subcs	r0, r0, r2
		teq		r2, r1
		movne	r2, r2, lsr #1
		bne		b3D4

		mov		r1, r0
		mov		r0, r3
		movs	r12, r12, lsl #1
		rsbcs	r0, r0, #0x0			@ neg r0 if
		rsbmi	r1, r1, #0x0			@ neg r1 if
		bx		lr

/*
SWI 08h - Sqrt
Calculate square root.
  r0   unsigned 32bit number

Return:
  r0   unsigned 16bit number

The result is an integer value, so Sqrt(2) would return 1, to avoid this inaccuracy, shift left incoming number by 2*N as much as possible (the result is then shifted left by 1*N). Ie. Sqrt(2 shl 30) would return 1.41421 shl 15.
*/
Sqrt:
		stmdb	sp!, {r4}
		mov		r12, r0
		mov		r1, #0x1
b410:
		cmp		r0, r1
		movhi	r0, r0, lsr #1
		movhi	r1, r1, lsl	#1
		bhi		b410
b420:
		mov		r0, r12
		mov		r4, r1
		mov		r3, #0x0
		mov		r2, r1
b430:
		cmp		r2, r0, lsr #1
		movls	r2, r2, lsl	#1
		bcc		b430
b43C:
		cmp		r0, r2
		adc		r3, r3, r3
		subcs	r0, r0, r2
		teq		r2, r1
		movne	r2, r2, lsr #1
		bne		b43C
		add		r1, r1, r3
		movs	r1, r1, lsr #1
		cmp		r1, r4
		bcc		b420
		mov		r0, r4
		ldmia	sp!, {r4}
		bx		lr

/*
SWI 09h - ArcTan
Calculates the arc tangent.
  r0   Tan, 16bit (1bit sign, 1bit integral part, 14bit decimal part)

Return:
  r0   "-PI/2<THETA/<PI/2" in a range of C000h-4000h.

Note: there is a problem in accuracy with "THETA<-PI/4, PI/4<THETA".
*/
.globl ArcTanThumb
.thumb_func
.thumb
ArcTanThumb:
b470:
		adr		r3, ArcTan
		bx		r3
.arm
ArcTan:
		mul		r1, r0, r0
		mov		r1, r1, asr #14
		rsb		r1, r1, #0x0
		mov		r3, #0xA9
		mul		r3, r1, r3
		mov		r3, r3, asr #14
		add		r3, r3, #0x390
		mul		r3, r1, r3
		mov		r3, r3, asr #14
		add		r3, r3, #0x900
		add		r3, r3, #0x1C
		mul		r3, r1, r3
		mov		r3, r3, asr #14
		add		r3, r3, #0xF00
		add		r3, r3, #0xB6
		mul		r3, r1, r3
		mov		r3, r3, asr #14
		add		r3, r3, #0x1600
		add		r3, r3, #0xAA
		mul		r3, r1, r3
		mov		r3, r3, asr	#14
		add		r3, r3, #0x2000
		add		r3, r3, #0x81
		mul		r3, r1, r3
		mov		r3, r3, asr #14
		add		r3, r3, #0x3600
		add		r3, r3, #0x51
		mul		r3, r1, r3
		mov		r3, r3, asr #14
		add		r3, r3, #0xA200
		add		r3, r3, #0xF9
		mul		r0, r3, r0
		mov		r0, r0, asr #16
		bx		lr

/*
SWI 0Ah - ArcTan2
Calculates the arc tangent after correction processing.
Use this in normal situations.
  r0   X, 16bit (1bit sign, 1bit integral part, 14bit decimal part)
  r1   Y, 16bit (1bit sign, 1bit integral part, 14bit decimal part)

Return:
  r0   0000h-FFFFh for 0<=THETA<2PI.
*/
.thumb
ArcTan2:
		push	{r4-r7, lr}
		cmp		r1, #0
		bne		b510
		cmp		r0, #0
		blt		b50A
		mov		r0, #0
		b		b59E
b50A:
		mov		r0, #0x80
		lsl		r0, r0, #8
		b		b59E
b510:
		cmp		r0, #0
		bne		b524
		cmp		r1, #0
		blt		b51E
		mov		r0, #64
		lsl		r0, r0, #8
		b		b59E
b51E:
		mov		r0, #192
		lsl		r0, r0, #8
		b		b59E
b524:
		mov		r2, r0
		lsl		r2, r2, #14
		mov		r3, r1
		lsl		r3, r3, #14
		neg		r4, r0
		neg		r5, r1
		mov		r6, #64
		lsl		r6, r6, #8
		lsl		r7, r6, #1
		cmp		r1, #0
		blt		b572
		cmp		r0, #0
		blt		b55E
		cmp		r0, r1
		blt		b550
		mov		r1, r0
		mov		r0, r3
		bl		DivThumb
		bl		ArcTanThumb
		b		b59E
b550:
		mov		r0, r2
		bl		DivThumb
		bl		ArcTanThumb
		sub		r0, r6, r0
		b		b59E
b55E:
		cmp		r4, r1
		blt		b550
b562:
		mov		r1, r0
		mov		r0, r3
		bl		DivThumb
		bl		ArcTanThumb
		add		r0, r7, r0
		b		b59E
b572:
		cmp		r0, #0
		bgt		b58A
		cmp		r4, r5
		bgt		b562
b57A:
		mov		r0, r2
		bl		DivThumb
		bl		ArcTanThumb
		add		r6, r6, r7
		sub		r0, r6, r0
		b		b59E
b58A:
		cmp		r0, r5
		blt		b57A
		mov		r1, r0
		mov		r0, r3
		bl		DivThumb
		bl		ArcTanThumb
		add		r7, r7, r7
		add		r0, r7, r0
b59E:
		pop		{r4-r7}
		pop		{r3}
		bx		r3

@ Maybe check of Cartridge Key Number MSBs
@ Cartridgeheader 0x9E
.globl b5A4
.thumb_func
b5A4:
		push	{r3-r6, lr}
		mov		r6, #8
		lsl		r6, r6, #24
		mov		r5, #0x9E
		add		r5, r5, r6
		sub		r0, r5, #1
		mov		r1, #27
		bl		b6AC
		mov		r4, #12
		mul		r4, r0
		ldrb	r3, [r5]
		lsl		r3, r3, #30
		lsr		r3, r3, #30
		mov		r2, #0x30
		mul		r2, r3
		add		r4, r4, r2
		adr		r5, b5EC			@add	r5, pc, #36
		add		r5, r5, r4
		mov		r4, #0
b5CC:
		mov		r0, r4
		bl		b6CE
		cmp		r4, #3
		blt		b5E4
		cmp		r4, #9
		bge		b5E4
		ldrh	r1, [r5]
		lsl		r1, r1, #1
		orr		r1, r6
		ldrh	r0, [r1]
		@add		r5, r5, #2
		.hword	0x1CAD
b5E4:
		@add		r4, r4, #1
		.hword	0x1C64
		cmp		r4, #11
		bne		b5CC
		pop		{r3-r6, pc}
b5EC:
		.hword	0x479B, 0x7426, 0x11BC, 0x6D4F
		.hword	0x11BD, 0x32F1, 0x7FD9, 0x2CE7
		.hword	0x5DA5, 0x11BD, 0x4610, 0x5DA4
		.hword	0x4E90, 0x6173, 0x2A84, 0x4E91
		.hword	0x106A, 0x75FE, 0x29C8, 0x7839
		.hword	0x420E, 0x5D1B, 0x7838, 0x12A8
		.hword	0x3F7D, 0x67B9, 0x26F3, 0x54EF
		.hword	0x7C23, 0x26F2, 0x6BC6, 0x4137
		.hword	0x15AB, 0x730D, 0x6BC7, 0x3B4F
		.hword	0x5F24, 0x3DDA, 0x253F, 0x1749
		.hword	0x3DDB, 0x70E6, 0x746C, 0x30F7
		.hword	0x531F, 0x6738, 0x531E, 0x1A51
		.hword	0x1971, 0x5B7D, 0x4ED6, 0x1970
		.hword	0x3F27, 0x75CB, 0x3D62, 0x128C
		.hword	0x74B8, 0x2FAD, 0x74B9, 0x64FD
		.hword	0x6C9A, 0x4F3A, 0x276D, 0x73EF
		.hword	0x38B1, 0x4F3B, 0x571E, 0x7EA3
		.hword	0x6249, 0x3587, 0x1B7C, 0x3586
		.hword	0x7AFB, 0x67E4, 0x5C92, 0x67E5
		.hword	0x2BCA, 0x438C, 0x2E6F, 0x587F
		.hword	0x14B7, 0x2E6E, 0x4CB9, 0x6FA2
		.hword	0x38F0, 0x719E, 0x475A, 0x1F3C
		.hword	0x6AD8, 0x475B, 0x5199, 0x3264
		.hword	0x7B41, 0x49EF, 0x5198, 0x1CD7

b6AC:
		push	{r4, r5, lr}
		mov		r4, #3
		mov		r3, #0
b6B2:
		ldrb	r2, [r0, #0]
		ror		r3, r4
		mov		r5, #4
b6B8:
		eor		r3, r2
		lsl		r2, r2, #8
		.hword	0x1E6D
		@sub		r5, r5, #1
		bgt		b6B8
		.hword	0x1C40
		@add		r0, r0, #1
		.hword	0x1E49
		@sub		r1, r1, #1
		bgt		b6B2
		mov		r0, r3
		lsl		r0, r0, #27
		lsr		r0, r0, #30
		pop		{r4, r5, pc}
b6CE:
		push	{r4, lr}
		mov		r4, #20
		mul		r4, r0
		mov		r3, #8
		lsl		r3, r3, #24
		add		r0, r3, #4
		add		r0, r0, r4
		ldr		r1, =0x03000088			@ 0xAC0
		add		r1, r1, r4
		mov		r2, #10
		bl		CpuSet
		pop		{r4, pc}

.globl b6E8
.thumb_func
b6E8:
		push	{r4-r6, lr}
		ldr		r1, =0x3290				@ 0xAC4
		mov		r6, #0
b6EE:
		mov		r4, #255
		cmp		r6, #152
		bne		b6F6
		mov		r4, #123
b6F6:
		cmp		r6, #154
		bne		b6FC
		mov		r4, #252
b6FC:
		cmp		r6, #156
		bge		b70E
		ldrb	r2, [r0, r6]
		ldrb	r3, [r1, r6]
		and		r2, r4
		.hword	0x1C76
		@add		r6, r6, #1
		cmp		r2, r3
		beq		b6EE
		b		b722
b70E:
		mov		r4, #25
b710:
		ldrb	r2, [r0, r6]
		add		r4, r4, r2
		.hword	0x1C76
		@add		r6, r6, #1
		cmp		r6, #186
		blt		b710
		lsl		r0, r4, #24
		bne		b722
		mov		r0, #0
		b		0x724
b722:
		mov		r0, #1
		pop		{r4-r6, pc}

.globl b726
.thumb_func
b726:
		ldr		r3, =0x03003580			@ 0xAC8
		mov		r2, #8
		mov		r0, #126
		neg		r0, r0
b72E:
		str		r0, [r3, r2]
		add		r2, #16
		cmp		r2, #120
		blt		b72E
		bx		lr

.globl b738
.thumb_func
b738:
		push	{r6, lr}
		sub		r3, r0, #3
		lsl		r6, r3, #2
		mul		r6, r2
		mov		r3, #64
		sub		r3, r3, r2
		mul		r6, r3
		.hword	0x1EC0
		@sub		r0, r0, #3
		mov		r3, #24
		mul		r3, r0
		lsl		r3, r3, #8
		sub		r6, r6, r3
		str		r6, [r1]
		cmp		r2, #47
		bgt		b766
		mov		r6, #26
		mul		r6, r2
		sub		r2, #72
		mul		r6, r2
		mov		r3, #104
		lsl		r3, r3, #8
		add		r6, r6, r3
		str		r6, [r1, #4]
b766:
		pop		{r6, pc}

.globl b768
.thumb_func
b768:
		push	{r4-r7, lr}
		mov		r7, r1
		ldmia	r0!, {r4-r6}
		add		r6, #128
		mov		r1, r6
		mov		r0, #128
		lsl		r0, r0, #16
		bl		DivThumb
		lsl		r3, r6, #1
		strh	r3, [r7, #12]
		strh	r3, [r7, #14]
		mov		r1, #127
		lsl		r1, r1, #7
		str		r1, [r7]
		str		r1, [r7, #4]
		asr		r1, r4, #8
		mul		r1, r0
		asr		r1, r1, #16
		add		r1, #120
		strh	r1, [r7, #8]
		asr		r1, r5, #8
		mul		r1, r0
		asr		r1, r1, #16
		add		r1, #80
		strh	r1, [r7, #10]
		pop		{r4-r7, pc}

.globl b79E
.thumb_func
b79E:
		push	{r4-r7, lr}
		ldr		r4, [sp, #20]
		ldr		r5, [sp, #24]
		mov		r7, #0
b7A6:
		mov		r6, #0
b7A8:
		strh	r0, [r4, r6]
		add		r0, r0, r1
		.hword	0x1CB6
		@add		r6, r6, #2
		cmp		r6, r2
		blt		b7A8
		add		r4, r4, r5
		.hword	0x1C7F
		@add		r7, r7, #1
		cmp		r7, r3
		blt		b7A6
		pop		{r4-r7, pc}

.globl b7BC
.thumb_func
b7BC:
		push	{r4-r7, lr}
		mov		r7, #2
b7C0:
		ldr		r4, =0x3200				@ 0xACC
		lsl		r3, r0, #1
		add		r3, r3, r0
		add		r3, r3, r7
		lsl		r3, r3, #2
		add		r3, r3, r4
		ldr		r5, [r3, #4]
		ldr		r6, [r3, #16]
		mov		r3, #32
		sub		r3, r3, r1
		mul		r3, r5
		mul		r6, r1
		add		r3, r3, r6
		lsr		r4, r3, #5
		mov		r6, #31
		lsl		r3, r6, #20
		and		r3, r4
		lsr		r5, r3, #10
		lsl		r3, r6, #10
		and		r3, r4
		lsr		r3, r3, #5
		orr		r3, r5
		and		r4, r6
		orr		r4, r3
		add		r3, r2, r7
		lsl		r6, r3, #1
		ldr		r3, =0x05000200			@ 0xAD0
		add		r3, r6, r3
		strh	r4, [r3]
		.hword	0x1E7F
		@sub		r7, r7, #1
		bge		b7C0
		pop		{r4-r7, pc}

.globl SoundBias
.thumb_func
SoundBias:
b800:
		mov		r1, #2
		lsl		r1, r1, #8
		mov		r12, r1
		ldr		r3, =0x04000088			@ 0xAD4
		ldrh	r2, [r3]
		ldr		r3, =0x04000088			@ 0xAD4
		lsl		r1, r2, #22
		lsr		r1, r1, #22
		cmp		r0, #0
		beq		b81C
		cmp		r1, r12
		bge		b82C
		.hword	0x1C92
		@add		r2, r2, #2
		b		b822
b81C:
		cmp		r1, #0
		ble		b82C
		.hword	0x1E92
		@sub		r2, r2, #2
b822:
		strh	r2, [r3]
		mov		r2, #8
b826:
		.hword	0x1E52
		@sub		r2, r2, #1
		bpl		b826
		b		b800
b82C:
		bx		lr

.globl b82E
.thumb_func
b82E:
		ldr		r1, =0x03000564			@ 0xAD8
		mov		r2, #55
		lsl		r2, r2, #4
		ldr		r0, =0x332C				@ 0xADC
		b		b858
.globl b838
.thumb_func
b838:
		ldr		r1, =0x03000564			@ 0xAD8
		mov		r2, #36
		ldr		r0, =0x326C				@ 0xAE0
		b		b858
.globl b840
.thumb_func
b840:
		mov		r1, #7
		lsl		r1, r1, #24
		mov		r2, #80
		ldr		r0, =0x369C				@ 0xAE4
		b		b858
.globl b84A
.thumb_func
b84A:
		ldr		r1, =0x05000038			@ 0xAE8
		cmp		r0, #0
		beq		b854
		lsl		r0, r0, #9
		add		r1, r1, r0
b854:
		mov		r2, #8
		ldr		r0, =0x3264				@ 0xAEC
b858:
		push	{r4, r5, lr}
		add		r2, r2, r1
b85C:
		ldr		r3, =0x3200				@ 0xACC
		cmp		r0, r3
		blt		b872
		mov		r3, #4
		lsl		r3, r3, #12
		cmp		r0, r3
		bge		b872
		ldmia	r0!, {r3}
		stmia	r1!, {r3}
		cmp		r1, r2
		blt		b85C
b872:
		pop		{r4, r5, pc}

.globl b874
.thumb_func
b874:
		push	{r4-r7, lr}
		sub		sp, #20
		ldr		r1, =0x30C0				@ 0xAF0
		ldmia	r1!, {r5, r7}
		add		r0, sp, #8
		stmia	r0!, {r5, r7}
		ldr		r0, =0x0BFE1FE0			@ 0xAF4
		ldr		r3, =0x080000B4			@ 0xAF8
		ldrb	r3, [r3]
		lsr		r3, r3, #7
		bne		b88C
		ldr		r0, =0x0BFFFFE0			@ 0xAFC
b88C:
		ldr		r1, =0x03000564			@ 0xAD8
		mov		r2, #10
		bl		CpuSet
		bl		b5A4
		ldr		r1, =0x03000088			@ 0xAC0
		mov		r3, r1
		add		r3, #174
		ldrb	r0, [r3]
		cmp		r0, #150
		beq		b8B0
		ldr		r2, =0x85000027			@ 0xB00
		asr		r3, r2, #31
		str		r3, [sp, #16]
		add		r0, sp, #16
		bl		CpuSet
b8B0:
		bl		b82E
		ldr		r0, =0x03000564			@ 0xAD8
		ldr		r1, =0x03001564			@ 0xB04
		bl		HuffUnCompThumb
		ldr		r0, =0x03001564			@ 0xB04
		ldr		r1, =0x03000564			@ 0xAD8
		bl		LZ77UnCompWramThumb
		mov		r7, #0
b8C6:
		lsl		r0, r7, #2
		str		r0, [sp, #12]
		ldr		r2, =0x03000564			@ 0xAD8
		lsl		r0, r7, #8
		add		r0, r0, r2
		ldr		r3, =0x06000040			@ 0xB08
		lsl		r1, r7, #10
		add		r1, r1, r3
		add		r2, sp, #8
		bl		BitUnPackThumb
		.hword	0x1C7F
		@add		r7, r7, #1
		cmp		r7, #8
		blt		b8C6
		mov		r7, #14
b8E4:
		mov		r4, #3
b8E6:
		ldr		r3, =0x06000040			@ 0xB08
		lsl		r0, r7, #1
		add		r0, r0, r4
		lsl		r0, r0, #8
		add		r0, r0, r3
		ldr		r3, =0x30B0				@ 0xB0C
		ldrh	r2, [r3, r7]
		ldr		r3, =0x06010000			@ 0xB10
		lsl		r1, r4, #4
		add		r1, r1, r2
		lsl		r1, r1, #6
		add		r1, r1, r3
		mov		r2, #128
		bl		CpuSet
		.hword	0x1E64
		@sub		r4, r4, #1
		bge		b8E6
		.hword	0x1EBF
		@sub		r7, r7, #2
		bge		b8E4
		ldr		r0, =0x03000088			@ [pc, #432]			@ 0xAC0
		bl		b94A
		bl		b974
		bl		b982
		mov		r2, #32
		str		r2, [sp, #4]
		ldr		r1, =0x0600B880			@ 0xB14
		str		r1, [sp]
		mov		r3, #4
		mov		r2, #4
		ldr		r1, =0x0202				@ 0xB18
		ldr		r0, =0x7271				@ 0xB1C
		bl		b79E
		mov		r1, #5
		lsl		r1, r1, #24
		mvn		r0, r1
		strh	r0, [r1]
		mov		r0, #0
		bl		b84A
		mov		r0, #1
		bl		b84A
		bl		b840
		add		sp, #20
		pop		{r4-r7, pc}

.globl b94A
.thumb_func
b94A:
		push	{r0, r4-r7, lr}
		ldr		r4, =0x03007FF7			@ 0xB20
		strb	r0, [r4]
		bl		b838
		ldr		r0, [sp]
		ldr		r1, =0x03000588			@ 0xB24
		mov		r2, #78
		bl		CpuSet
		ldr		r0, =0x03000564			@ 0xAD8
		ldr		r1, =0x03001564			@ 0xB04
		bl		HuffUnCompThumb
		ldr		r0, =0x03001564			@ 0xB04
		ldr		r2, =0xD082				@ 0xB28
		str		r2, [r0]
		ldr		r1, =0x03000564			@ [pc, #360]			@ 0xAD8
		bl		Diff16bitUnFilter
		pop		{r0, r4-r7, pc}

.globl b974
.thumb_func
b974:
		push	{r0, r4-r7, lr}
		ldr		r0, =0x03000564			@ 0xAD8
		ldr		r1, =0x03001564			@ 0xB04
		ldr		r2, =0x30C8				@ 0xB2C
		bl		BitUnPackThumb
		pop		{r0, r4-r7, pc}

.globl b982
.thumb_func
b982:
		push	{r0, r4-r7, lr}
		ldr		r6, =0x03001564			@ 0xB04
		ldr		r4, =0x060024C0			@ 0xB30
		mov		r7, #2
b98A:
		mov		r5, #52
b98C:
		ldmia	r6!, {r0-r3}
		stmia	r4!, {r0-r3}
		.hword	0x1E6D
		@sub		r5, r5, #1
		bgt		b98C
		add		r4, #192
		.hword	0x1E7F
		@sub		r7, r7, #1
		bgt		b98A
		mov		r7, #3
b99C:
		lsl		r3, r7, #10
		ldr		r0, =0x06002040			@ 0xB34
		add		r0, r0, r3
		ldr		r1, =0x06016800			@ 0xB38
		add		r1, r1, r3
		mov		r2, #1
		lsl		r2, r2, #8
		bl		CpuFastSetThumb
		.hword	0x1E7F
		@sub		r7, r7, #1
		bgt		b99C
		mov		r0, sp
		str		r7, [r0]
		ldr		r1, =0x03000564			@ 0xAD8
		mov		r2, #8
		lsl		r2, r2, #8
		bl		bAB2
		pop		{r0, r4-r7, pc}

/*
SWI 01h (GBA) - RegisterRamReset
Resets the I/O registers and RAM specified in ResetFlags. However, it does not clear the CPU internal RAM area from 3007E00h-3007FFFh.

  r0  ResetFlags
       Bit   Expl.
       0     Clear 256K on-board WRAM  ;-don't use when returning to WRAM
       1     Clear 32K in-chip WRAM    ;-excluding last 200h bytes
       2     Clear Palette
       3     Clear VRAM
       4     Clear OAM              ;-zerofilled! does NOT disable OBJs!
       5     Reset SIO registers    ;-switches to general purpose mode!
       6     Reset Sound registers
       7     Reset all other registers (except SIO, Sound)

Return: No return value.
Bug: LSBs of SIODATA32 are always destroyed, even if Bit5 of R0 was cleared.
The function always switches the screen into forced blank by setting DISPCNT=0080h (regardless of incoming R0, screen becomes white).
*/
.globl RegisterRamReset
.thumb_func
RegisterRamReset:
b9C2:
		push	{r4-r7, lr}
		sub		sp, #4
		mov		r7, r0
		ldr		r5, =0x85000000			@ 0xB3C
		mov		r4, #4
		lsl		r4, r4, #24
		mov		r3, #0
		str		r3, [sp]
		mov		r1, #128
		strh	r1, [r4]
		mov		r6, #128
		tst		r6, r7
		beq		bA18
		lsr		r1, r4, #17
		add		r1, r1, r4
		mov		r2, #8
		bl		bAAC
		sub		r1, #32
		mvn		r0, r2
		strh	r0, [r1, #2]
		lsr		r1, r4, #16
		add		r1, r1, r4
		strb	r0, [r1, #16]
		add		r1, r4, #4
		mov		r2, #8
		bl		bAAC
		.hword	0x1F09
		@sub		r1, r1, #4
		mov		r2, #16
		bl		bAAC
		mov		r1, #176
		add		r1, r1, r4
		mov		r2, #24
		bl		bAAC
		str		r2, [r1, #32]
		lsr		r0, r4, #18
		strh	r0, [r4, #32]
		strh	r0, [r4, #48]
		strh	r0, [r4, #38]
		strh	r0, [r4, #54]
bA18:
		mov		r6, #32
		ldr		r1, =0x04000110			@ 0xB40
		mov		r2, #8
		bl		bAAC
		lsr		r2, r4, #11
		strh	r2, [r1, #4]
		add		r1, #16
		mov		r2, #7
		strb	r2, [r1]
		bl		bAAC
		mov		r6, #64
		tst		r6, r7
		beq		bA6A
		mov		r1, #128
		add		r1, r1, r4
		ldr		r0, =0x880E0000			@ 0xB44
		strb	r0, [r1, #4]
		strb	r1, [r1, #4]
		str		r0, [r1]
		ldrh	r0, [r1, #8]
		lsl		r0, r0, #22
		lsr		r0, r0, #22
		strh	r0, [r1, #8]
		sub		r1, #16
		strb	r1, [r1]
		add		r1, #32
		mov		r2, #8
		bl		bAAC
		sub		r1, #64
		strb	r2, [r1]
		add		r1, #32
		mov		r2, #8
		bl		bAAC
		mov		r2, #0
		mov		r1, #128
		add		r1, r1, r4
		strb	r2, [r1, #4]
bA6A:
		mov		r6, #1
		lsr		r1, r4, #1
		lsr		r2, r4, #10
		bl		bAAC
		mov		r6, #8
		mov		r1, #6
		lsl		r1, r1, #24
		lsr		r2, r1, #12
		bl		bAAC
		mov		r6, #16
		mov		r1, #7
		lsl		r1, r1, #24
		lsr		r2, r4, #18
		bl		bAAC
		mov		r6, #4
		mov		r1, #5
		lsl		r1, r1, #24
		lsr		r2, r4, #18
		bl		bAAC
		mov		r6, #2
		mov		r1, #3
		lsl		r1, r1, #24
		ldr		r2, =0x1F80				@ 0xB48
		bl		bAAC
		add		sp, #4
		pop		{r4-r7}
		pop		{r3}
		bx		r3
bAAC:
		tst		r6, r7
		bne		bAB2
		bx		lr
bAB2:
		mov		r0, sp
		orr		r2, r5
		b		bBC0
AB8:
		.word	0x2D71
		.word	0x210D
.pool

.globl CpuSet
.thumb_func
CpuSet:
bB4C:
		push	{r4, r5, lr}
		lsl		r4, r2, #11
		lsr		r4, r4, #9
		bl		bB9C
		beq		bB96
		mov		r5, #0
		lsr		r3, r2, #27
		bcc		bB78
		add		r5, r1, r4
		lsr		r3, r2, #25
		bcc		bB6E
		ldmia	r0!, {r3}
bB66:
		cmp		r1, r5
		bge		bB96
		stmia	r1!, {r3}
		b		bB66
bB6E:
		cmp		r1, r5
		bge		bB96
		ldmia	r0!, {r3}
		stmia	r1!, {r3}
		b		bB6E
bB78:
		lsr		r4, r4, #1
		lsr		r3, r2, #25
		bcc		bB8A
		ldrh	r3, [r0]
bB80:
		cmp		r5, r4
		bge		bB96
		strh	r3, [r1, r5]
		.hword	0x1CAD
		@add		r5, r5, #2
		b		bB80
bB8A:
		cmp		r5, r4
		bge		bB96
		ldrh	r3, [r0, r5]
		strh	r3, [r1, r5]
		.hword	0x1CAD
		@add		r5, r5, #2
		b		bB8A
bB96:
		pop		{r4, r5}
		pop		{r3}
		bx		r3

.globl bB9C
.thumb_func
bB9C:
		add		r3, pc, #4
		mov		r12, r4
		bx		r3
.align 2, 0
.arm
bBA4:
		cmp		r12, #0
		beq		bBBC
		bic		r12, r12, #0xFE000000
		add		r12, r0, r12
		tst		r0, #0x0E000000
		tstne	r12, #0x0E000000
bBBC:
		bx		lr

/*
SWI 0Ch (GBA/NDS7/NDS9) - CpuFastSet
Memory copy/fill in units of 32 bytes. Memcopy is implemented as repeated LDMIA/STMIA [Rb]!,r2-r9 instructions. Memfill as single LDR followed by repeated STMIA [Rb]!,r2-r9. After processing all 32-byte-blocks, the NDS additonally processes the remaining words as 4-byte blocks. BUG: The NDS uses the fast 32-byte-block processing only for the first N bytes (not for the first N words), so only the first quarter of the memory block is FAST, the remaining three quarters are SLOWLY copied word-by-word.
The length is specifed as wordcount, ie. the number of bytes divided by 4.
On the GBA, the length must be a multiple of 8 words (32 bytes). On NDS, the length may be any number of words (4 bytes).

  r0    Source address        (must be aligned by 4)
  r1    Destination address   (must be aligned by 4)
  r2    Length/Mode
          Bit 0-20  Wordcount (GBA: must be a multiple of 8 words)
          Bit 24    Fixed Source Address (0=Copy, 1=Fill by WORD[r0])

Return: No return value, Data written to destination address.
*/

.thumb
.globl CpuFastSetThumb
.thumb_func
CpuFastSetThumb:
bBC0:
		mov		r3, pc
		bx		r3
.arm
CpuFastSet:
		stmdb	sp!, {r4-r10, lr}
		mov		r10, r2, lsl #11
		movs	r12, r10, lsr #9
		bl		bBA4
		beq		bC24
		add		r10, r1, r10, lsr #9
		movs	r2, r2, lsr #25	@ Test if Bit 24 is set (fixed source address)
		bcc		bC14
		ldr		r2, [r0]		@ Copy from fixed source address
		mov		r3, r2
		mov		r4, r2
		mov		r5, r2
		mov		r6, r2
		mov		r7, r2
		mov		r8, r2
		mov		r9, r2
bC04:
		cmp		r1, r10
		stmltia	r1!, {r2-r9}
		blt		bC04
		b		bC24
bC14:
		cmp		r1, r10
		ldmltia	r0!, {r2-r9}
		stmltia	r1!, {r2-r9}
		blt		bC14
bC24:
		ldmia	sp!, {r4-r10, lr}
		bx		lr

BgAffineSet:
		stmdb	sp!, {r4-r11}
bC30:
		subs	r2, r2, #1
		blt		bCD8
		ldrh	r3, [r0, #16]
		mov		r3, r3, lsr #8
		adr		r12, SinTable				@ add		r12, pc, #0x114
		add		r8, r3, #0x40
		and		r8, r8, #0xFF
		mov		r8, r8, lsl #1
		ldrsh	r11, [r8, r12]
		mov		r8, r3, lsl #1
		ldrsh	r12, [r8, r12]
		ldrsh	r9, [r0, #12]
		ldrsh	r10, [r0, #14]
		mul		r8, r11, r9
		mov		r3, r8, asr #14
		mul		r8, r12, r9
		mov		r4, r8, asr #14
		mul		r8, r12, r10
		mov		r5, r8, asr #14
		mul		r8, r11, r10
		mov		r6, r8, asr #14
		ldmia	r0, {r9, r10, r12}
		mov		r11, r12, lsl #16
		mov		r11, r11, asr #16
		mov		r12, r12, asr #16
		rsb		r8, r11, #0
		mla		r9, r3, r8, r9
		mla		r8, r4, r12, r9
		str		r8, [r1, #8]
		rsb		r8, r11, #0
		mla		r10, r5, r8, r10
		rsb		r8, r12, #0
		mla		r8, r6, r8, r10
		str		r8, [r1, #12]
		strh	r3, [r1]
		rsb		r4, r4, #0
		strh	r4, [r1, #2]
		strh	r5, [r1, #4]
		strh	r6, [r1, #6]
		add		r0, r0, #20
		add		r1, r1, #16
		b		bC30
bCD8:
		ldmia	sp!, {r4-r11}
		bx		lr

/*
SWI 0x0F = ObjAffineSet

  r0   Source Address, pointing to data structure as such:
        s16  Scaling ratio in X direction (8bit fractional portion)
        s16  Scaling ratio in Y direction (8bit fractional portion)
        u16  Angle of rotation (8bit fractional portion) Effective Range 0-FFFF
  r1   Destination Address, pointing to data structure as such:
        s16  Difference in X coordinate along same line
        s16  Difference in X coordinate along next line
        s16  Difference in Y coordinate along same line
        s16  Difference in Y coordinate along next line
  r2   Number of calculations
  r3   Offset in bytes for parameter addresses (2=continuous, 8=OAM)
*/
ObjAffineSet:
		stmdb	sp!, {r8-r11}
StartObjAffineCalc:
		subs	r2, r2, #1
		blt		EndObjAffineCalc

		ldrh	r9, [r0, #4]				@ load angle of rotation
		mov		r9, r9, lsr #8
		adr		r12, SinTable				@ set address of table
		add		r8, r9, #0x40
		and		r8, r8, #0xFF
		mov		r8, r8, lsl #1
		ldrsh	r11, [r8, r12]				@ load sinus
		mov		r8, r9, lsl #1
		ldrsh	r12, [r8, r12]
		ldrsh	r9, [r0]
		ldrsh	r10, [r0, #2]
		mul		r8, r11, r9
		mov		r8, r8, asr #14
		strh	r8, [r1], r3
		mul		r8, r12, r9
		mov		r8, r8, asr #14
		rsb		r8, r8, #0
		strh	r8, [r1], r3
		mul		r8, r12, r10
		mov		r8, r8, asr #14
		strh	r8, [r1], r3
		mul		r8, r11, r10
		mov		r8, r8, asr #14
		strh	r8, [r1], r3
		add		r0, r0, #8
		b		StartObjAffineCalc

EndObjAffineCalc:
		ldmia	sp!, {r8-r11}
		bx		lr

SinTable:
		.hword	0x0000, 0x0192, 0x0323
		.hword	0x04B5, 0x0645, 0x07D5
		.hword	0x0964, 0x0AF1, 0x0C7C
		.hword	0x0E05, 0x0F8C, 0x1111
		.hword	0x1294, 0x1413, 0x158F
		.hword	0x1708, 0x187D, 0x19EF
		.hword	0x1B5D, 0x1CC6, 0x1E2B
		.hword	0x1F8B, 0x20E7, 0x223D
		.hword	0x238E, 0x24DA, 0x261F
		.hword	0x275F, 0x2899, 0x29CD
		.hword	0x2AFA, 0x2C21, 0x2D41
		.hword	0x2E5A, 0x2F6B, 0x3076
		.hword	0x3179, 0x3274, 0x3367
		.hword	0x3453, 0x3536, 0x3612
		.hword	0x36E5, 0x37AF, 0x3871
		.hword	0x392A, 0x39DA, 0x3A82
		.hword	0x3B20, 0x3BB6, 0x3C42
		.hword	0x3CC5, 0x3D3E, 0x3DAE
		.hword	0x3E14, 0x3E71, 0x3EC5
		.hword	0x3F0E, 0x3F4E, 0x3F84
		.hword	0x3FB1, 0x3FD3, 0x3FEC
		.hword	0x3FFB, 0x4000, 0x3FFB
		.hword	0x3FEC, 0x3FD3, 0x3FB1
		.hword	0x3F84, 0x3F4E, 0x3F0E
		.hword	0x3EC5, 0x3E71, 0x3E14
		.hword	0x3DAE, 0x3D3E, 0x3CC5
		.hword	0x3C42, 0x3BB6, 0x3B20
		.hword	0x3A82, 0x39DA, 0x392A
		.hword	0x3871, 0x37AF, 0x36E5
		.hword	0x3612, 0x3536, 0x3453
		.hword	0x3367, 0x3274, 0x3179
		.hword	0x3076, 0x2F6B, 0x2E5A
		.hword	0x2D41, 0x2C21, 0x2AFA
		.hword	0x29CD, 0x2899, 0x275F
		.hword	0x261F, 0x24DA, 0x238E
		.hword	0x223D, 0x20E7, 0x1F8B
		.hword	0x1E2B, 0x1CC6, 0x1B5D
		.hword	0x19EF, 0x187D, 0x1708
		.hword	0x158F, 0x1413, 0x1294
		.hword	0x1111, 0x0F8C, 0x0E05
		.hword	0x0C7C, 0x0AF1, 0x0964
		.hword	0x07D5, 0x0645, 0x04B5
		.hword	0x0323, 0x0192, 0x0000

		.hword	0xFE6E, 0xFCDD, 0xFB4B
		.hword	0xF9BB, 0xF82B, 0xF69C
		.hword	0xF50F, 0xF384, 0xF1FB
		.hword	0xF074, 0xEEEF, 0xED6C
		.hword	0xEBED, 0xEA71, 0xE8F8
		.hword	0xE783, 0xE611, 0xE4A3
		.hword	0xE33A, 0xE1D5, 0xE075
		.hword	0xDF19, 0xDDC3, 0xDC72
		.hword	0xDB26, 0xD9E1, 0xD8A1
		.hword	0xD767, 0xD633, 0xD506
		.hword	0xD3DF, 0xD2BF, 0xD1A6
		.hword	0xD095, 0xCF8A, 0xCE87
		.hword	0xCD8C, 0xCC99, 0xCBAD
		.hword	0xCACA, 0xC9EE, 0xC91B
		.hword	0xC851, 0xC78F, 0xC6D6
		.hword	0xC626, 0xC57E, 0xC4E0
		.hword	0xC44A, 0xC3BE, 0xC33B
		.hword	0xC2C2, 0xC252, 0xC1EC
		.hword	0xC18F, 0xC13B, 0xC0F2
		.hword	0xC0B2, 0xC07C, 0xC04F
		.hword	0xC02D, 0xC014, 0xC005
		.hword	0xC000, 0xC005, 0xC014
		.hword	0xC02D, 0xC04F, 0xC07C
		.hword	0xC0B2, 0xC0F2, 0xC13B
		.hword	0xC18F, 0xC1EC, 0xC252
		.hword	0xC2C2, 0xC33B, 0xC3BE
		.hword	0xC44A, 0xC4E0, 0xC57E
		.hword	0xC626, 0xC6D6, 0xC78F
		.hword	0xC851, 0xC91B, 0xC9EE
		.hword	0xCACA, 0xCBAD, 0xCC99
		.hword	0xCD8C, 0xCE87, 0xCF8A
		.hword	0xD095, 0xD1A6, 0xD2BF
		.hword	0xD3DF, 0xD506, 0xD633
		.hword	0xD767, 0xD8A1, 0xD9E1
		.hword	0xDB26, 0xDC72, 0xDDC3
		.hword	0xDF19, 0xE075, 0xE1D5
		.hword	0xE33A, 0xE4A3, 0xE611
		.hword	0xE783, 0xE8F8, 0xEA71
		.hword	0xEBED, 0xED6C, 0xEEEF
		.hword	0xF074, 0xF1FB, 0xF384
		.hword	0xF50F, 0xF69C, 0xF82B
		.hword	0xF9BB, 0xFB4B, 0xFCDD
		.hword	0xFE6E

.thumb
.globl BitUnPackThumb
.thumb_func
BitUnPackThumb:
bF5C:
		mov		r3, pc
		bx		r3
.arm
BitUnPack:
		stmdb	sp!, {r4-r11, lr}
		sub		sp, sp, #8
		ldrh	r7, [r2]
		movs	r12, r7
		bl		bBA4
		beq		b1004
		ldrb	r6, [r2, #2]
		rsb		r10, r6, #8
		mov		lr, #0
		ldr		r11, [r2, #4]
		mov		r8, r11, lsr #31
		ldr		r11, [r2, #4]
		mov		r11, r11, lsl #1
		mov		r11, r11, lsr #1
		str		r11, [sp, #4]
		ldrb	r2, [r2, #3]
		mov		r3, #0
bFA4:
		subs	r7, r7, #1
		blt		b1004
		mov		r11, #0xFF
		mov		r5, r11, asr r10
		ldrb	r9, [r0], #1
		mov		r4, #0
bFBC:
		cmp		r4, #8
		bge		bFA4
		and		r11, r9, r5
		movs	r12, r11, lsr r4
		cmpeq	r8, #0
		beq		bFDC
		ldr		r11, [sp, #4]
		add		r12, r12, r11
bFDC:
		orr		lr, lr, r12, lsl r3
		add		r3, r3, r2
		cmp		r3, #0x20
		blt		bFF8
		str		lr, [r1], #4
		mov		lr, #0
		mov		r3, #0
bFF8:
		mov		r5, r5, lsl r6
		add		r4, r4, r6
		b		bFBC
b1004:
		add		sp, sp, #8
		ldmia	sp!, {r4-r11, lr}
		bx		lr

.thumb
.globl HuffUnCompThumb
.thumb_func
HuffUnCompThumb:
b1010:
		mov		r3, pc
		bx		r3
.arm
HuffUnComp:
		stmdb	sp!, {r4-r11, lr}
		sub		sp, sp, #8
		movs	r12, #0x02000000
		bl		bBA4
		beq		b10EC
		add		r2, r0, #4
		add		r7, r2, #1
		ldrb	r10, [r0]
		and		r4, r10, #0xF
		mov		r3, #0
		mov		lr, #0
		and		r10, r4, #7
		add		r11, r10, #4
		str		r11, [sp, #4]
		ldr		r10, [r0]
		mov		r12, r10, lsr #8
		ldrb	r10, [r2]
		add		r10, r10, #1
		add		r0, r2, r10, lsl #1
		mov		r2, r7
b1064:
		cmp		r12, #0
		ble		b10EC
		mov		r8, #0x20
		ldr		r5, [r0], #4
b1074:
		subs	r8, r8, #1
		blt		b1064
		mov		r10, #1
		and		r9, r10, r5, lsr #31
		ldrb	r6, [r2]
		mov		r6, r6, lsl r9
		mov		r10, r2, lsr #1
		mov		r10, r10, lsl #1
		ldrb	r11, [r2]
		and		r11, r11, #0x3F
		add		r11, r11, #1
		add		r10, r10, r11, lsl #1
		add		r2, r10, r9
		tst		r6, #0x80
		beq		b10DC
		mov		r3, r3, lsr r4
		ldrb	r10, [r2]
		rsb		r11, r4, #0x20
		orr		r3, r3, r10, lsl r11
		mov		r2, r7
		add		lr, lr, #1
		ldr		r11, [sp, #4]
		cmp		lr, r11
		streq	r3, [r1], #4
		subeq	r12, r12, #4
		moveq	lr, #0
b10DC:
		cmp		r12, #0
		movgt	r5, r5, lsl #1
		bgt		b1074
		b		b1064
b10EC:
		add		sp, sp, #8
		ldmia	sp!, {r4-r11, lr}
		bx		lr

.thumb
.globl LZ77UnCompWramThumb
.thumb_func
LZ77UnCompWramThumb:
b10F8:
		mov		r3, pc
		bx		r3
.arm
LZ77UnCompWram:
		stmdb	sp!, {r4-r6, lr}
		ldr		r5, [r0], #4
		mov		r2, r5, lsr #8
		movs	r12, r2
		bl		bBA4
		beq		b118C
b1114:
		cmp		r2, #0
		ble		b118C
		ldrb	lr, [r0], #1
		mov		r4, #8
b1124:
		subs	r4, r4, #1
		blt		b1114
		tst		lr, #0x80
		bne		b1144
		ldrb	r6, [r0], #1
		strb	r6, [r1], #1
		sub		r2, r2, #1
		b		b117C
b1144:
		ldrb	r5, [r0]
		mov		r6, #3
		add		r3, r6, r5, asr #4
		ldrb	r6, [r0], #1
		and		r5, r6, #0xF
		mov		r12, r5, lsl #8
		ldrb	r6, [r0], #1
		orr		r5, r6, r12
		add		r12, r5, #1
		sub		r2, r2, r3
b116C:
		ldrb	r5, [r1, -r12]
		strb	r5, [r1], #1
		subs	r3, r3, #1
		bgt		b116C
b117C:
		cmp		r2, #0
		movgt	lr, lr, lsl #1
		bgt		b1124
		b		b1114
b118C:
		ldmia	sp!, {r4-r6, lr}
		bx		lr

LZ77UnCompVram:
		stmdb	sp!, {r4-r10, lr}
		mov		r3, #0
		ldr		r8, [r0], #4
		mov		r10, r8, lsr #8
		mov		r2, #0
		movs	r12, r10
		bl		bBA4
		beq		b1270
b11B4:
		cmp		r10, #0
		ble		b1270
		ldrb	r6, [r0], #1
		mov		r7, #8
b11C4:
		subs	r7, r7, #1
		blt		b11B4
		tst		r6, #0x80
		bne		b11F0
		ldrb	r9, [r0], #1
		orr		r3, r3, r9, lsl r2
		sub		r10, r10, #1
		eors	r2, r2, #8
		streqh	r3, [r1], #2
		moveq	r3, #0
		b		b1260
b11F0:
		ldrb	r9, [r0]
		mov		r8, #3
		add		r5, r8, r9, asr #4
		ldrb	r9, [r0], #1
		and		r8, r9, #0xF
		mov		r4, r8, lsl #8
		ldrb	r9, [r0], #1
		orr		r8, r9, r4
		add		r4, r8, #1
		rsb		r8, r2, #8
		and		r9, r4, #1
		eor		lr, r8, r9, lsl #3
		sub		r10, r10, r5
b1224:
		eor		lr, lr, #8
		rsb		r8, r2, #8
		add		r8, r4, r8, lsr #3
		mov		r8, r8, lsr #1
		mov		r8, r8, lsl #1
		ldrh	r9, [r1, -r8]
		mov		r8, #0xFF
		and		r8, r9, r8, lsl lr
		mov		r8, r8, asr lr
		orr		r3, r3, r8, lsl r2
		eors	r2, r2, #8
		streqh	r3, [r1], #2
		moveq	r3, #0
		subs	r5, r5, #1
		bgt		b1224
b1260:
		cmp		r10, #0
		movgt	r6, r6, lsl #1
		bgt		b11C4
		b		b11B4
b1270:
		ldmia	sp!, {r4-r10, lr}
		bx		lr
.thumb
RlUnCompWram:
		push	{r4-r7, lr}
		ldmia	r0!, {r3}
		lsr		r7, r3, #8
		mov		r4, r7
		bl		bB9C
		beq		b12BA
b1286:
		cmp		r7, #0
		ble		b12BA
		ldrb	r4, [r0]
		.hword	0x1C40
		@add		r0, r0, #1
		lsl		r2, r4, #25
		lsr		r2, r2, #25
		lsr		r3, r4, #8
		bcs		b12A8
		.hword	0x1C52
		@add		r2, r2, #1
		sub		r7, r7, r2
b129A:
		ldrb	r3, [r0]
		strb	r3, [r1]
		.hword	0x1C40
		@add		r0, r0, #1
		.hword	0x1C49
		@add		r1, r1, #1
		.hword	0x1E52
		@add		r2, r2, #1
		bgt		b129A
		b		b1286
b12A8:
		.hword	0x1CD2
		@add		r2, r2, #3
		sub		r7, r7, r2
		ldrb	r5, [r0]
		.hword	0x1C40
		@add		r0, r0, #1
b12B0:
		strb	r5, [r1]
		.hword	0x1C49
		@add		r1, r1, #1
		.hword	0x1E52
		@sub		r2, r2, #1
		bgt		b12B0
		b		b1286
b12BA:
		pop		{r4-r7}
		pop		{r3}
		bx		r3

RlUnCompVram:
		push	{r4-r7, lr}
		sub		sp, #0xC
		mov		r7, #0
		ldmia	r0!, {r3}
		lsr		r5, r3, #8
		mov		r4, r5
		bl		bB9C
		beq		b132A
		mov		r4, #0
b12D4:
		cmp		r5, #0
		ble		b132A
		ldrb	r3, [r0]
		str		r3, [sp, #4]
		.hword	0x1C40
		@add		r0, r0, #1
		ldr		r3, [sp, #4]
		lsl		r2, r3, #25
		lsr		r2, r2, #25
		ldr		r6, [sp, #4]
		lsr		r3, r6, #8
		bcs		b1308
		.hword	0x1C52
		@add		r2, r2, #1
		sub		r5, r5, r2
b12EE:
		ldrb	r6, [r0]
		lsl		r6, r4
		orr		r7, r6
		.hword	0x1C40
		@add		r0, r0, #1
		mov		r3, #8
		eor		r4, r3
		bne		b1302
		strh	r7, [r1]
		.hword	0x1C89
		@add		r1, r1, #2
		mov		r7, #0
b1302:
		.hword	0x1E52
		@sub		r2, r2, #1
		bgt		b12EE
		b		b12D4
b1308:
		.hword	0x1CD2
		@add		r2, r2, #3
		sub		r5, r5, r2
		ldrb	r6, [r0]
		str		r6, [sp, #8]
		.hword	0x1C40
		@add		r0, r0, #1
b1312:
		ldr		r6, [sp, #8]
		lsl		r6, r4
		orr		r7, r6
		mov		r3, #8
		eor		r4, r3
		bne		b1324
		strh	r7, [r1]
		.hword	0x1C89
		@add		r1, r1, #2
		mov		r7, #0
b1324:
		.hword	0x1E52
		@sub		r2, r2, #1
		bgt		b1312
		b		b12D4
b132A:
		add		sp, #0xC
		pop		{r4-r7}
		pop		{r3}
		bx		r3

Diff8bitUnFilterWram:
		push	{r4, lr}
		ldmia	r0!, {r4}
		lsr		r4, r4, #8
		bl		bB9C
		beq		b1356
		ldrb	r2, [r0]
		.hword	0x1C40
		@add		r0, r0, #1
		strb	r2, [r1]
		.hword	0x1C49
		@add		r1, r1, #1
b1346:
		.hword	0x1E64
		@sub		r4, r4, #1
		ble		b1356
		ldrb	r3, [r0]
		add		r2, r3, r2
		.hword	0x1C40
		@add		r0, r0, #1
		strb	r2, [r1]
		.hword	0x1C49
		@add		r1, r1, #1
		b		b1346
b1356:
		pop		{r4}
		pop		{r3}
		bx		r3

Diff8bitUnFilterVram:
		push	{r4-r7, lr}
		ldmia	r0!, {r3}
		lsr		r5, r3, #8
		mov		r4, r5
		bl		bB9C
		beq		b1392
		mov		r4, #8
		ldrb	r7, [r0]
		.hword	0x1C40
		@add		r0, r0, #1
		mov		r2, r7
b1372:
		.hword	0x1E6D
		@sub		r5, r5, #1
		ble		b1392
		ldrb	r3, [r0]
		add		r7, r3, r7
		.hword	0x1C40
		@add		r0, r0, #1
		lsl		r6, r7, #24
		lsr		r6, r6, #24
		lsl		r6, r4
		orr		r2, r6
		mov		r3, #8
		eor		r4, r3
		bne		b1372
		strh	r2, [r1]
		.hword	0x1C89
		@add		r1, r1, #2
		mov		r2, #0
		b		b1372
b1392:
		pop		{r4-r7}
		pop		{r3}
		bx		r3

Diff16bitUnFilter:
b1398:
		push	{r4, lr}
		ldmia	r0!, {r4}
		lsr		r4, r4, #8
		bl		bB9C
		beq		b13BC
		ldrh	r2, [r0]
		.hword	0x1C80
		@add		r0, r0, #2
		strh	r2, [r1]
		.hword	0x1C89
		@add		r1, r1, #2
b13AC:
		.hword	0x1EA4
		@sub		r4, r4, #2
		ble		b13BC
		ldrh	r3, [r0]
		add		r2, r3, r2
		.hword	0x1C80
		@add		r0, r0, #2
		strh	r2, [r1]
		.hword	0x1C89
		@add		r1, r1, #2
		b		b13AC
b13BC:
		pop		{r4}
		pop		{r2}
.globl b13C0
.thumb_func
b13C0:
		bx		r2
.globl b13C2
.thumb_func
b13C2:
		bx		r1

.globl SoundWhatever0
.thumb_func
SoundWhatever0:
b13C4:
		push	{r4, r5, r7, lr}
		mov		r4, r2
		mov		r5, r1
		mov		r7, r0
		cmp		r2, #1
		blt		b141E
		cmp		r4, #16
		ble		b13D6
		mov		r4, #16
b13D6:
		mov		r0, r7
		bl		b23B0
		str		r5, [r7, #44]
		ldr		r0, =0x80000000				@ 0x1424
		strb	r4, [r7, #8]
		str		r0, [r7, #4]
		mov		r0, #0
		b		b13F2
b13E8:
		sub		r1, r4, #1
		lsl		r4, r1, #24
		lsr		r4, r4, #24
		strb	r0, [r5]
		add		r5, #80
b13F2:
		cmp		r4, #0
		bgt		b13E8
		ldr		r1, =0x03007FC0				@ 0x1428
		ldr		r4, =0x68736D53				@ 0x142C
		ldr		r1, [r1, #48]
		ldr		r2, [r1]
		cmp		r2, r4
		bne		b141E
		add		r2, #1
		str		r2, [r1]
		ldr		r2, [r1, #0x20]
		cmp		r2, #0
		beq		b1414
		str		r2, [r7, #56]
		ldr		r2, [r1, #36]
		str		r2, [r7, #60]
		str		r0, [r1, #32]
b1414:
		ldr		r0, =0x2149					@ 0x1430
		str		r7, [r1, #36]
		str		r0, [r1, #32]
		str		r4, [r1]
		str		r4, [r7, #52]
b141E:
		pop		{r4, r5, r7}
		pop		{r3}
		bx		r3
b1424:
.pool

.globl SoundWhatever1
.thumb_func
SoundWhatever1:
b1434:
		push	{r4-r7, lr}
		mov		r7, r0
		ldr		r0, [r0, #52]
		ldr		r3, =0x68736D53				@ 0x14BC
		mov		r4, r1
		cmp		r0, r3
		bne		b14B4
		add		r0, #1
		str		r0, [r7, #52]
		mov		r1, #0
		str		r1, [r7, #4]
		str		r4, [r7]
		ldr		r0, [r4, #4]
		str		r0, [r7, #48]
		ldrb	r0, [r4, #2]
		strb	r0, [r7, #9]
		mov		r0, #150
		strh	r0, [r7, #28]
		strh	r0, [r7, #32]
		mov		r0, #0xFF
		add		r0, #1
		strh	r0, [r7, #30]
		strh	r1, [r7, #34]
		strh	r1, [r7, #36]
		ldr		r5, [r7, #44]
		mov		r6, #0
		b		b1482
b146A:
		mov		r0, r7
		mov		r1, r5
		bl		b23E6
		mov		r0, #192
		strb	r0, [r5]
		lsl		r0, r6, #2
		add		r0, r0, r4
		ldr		r0, [r0, #8]
		str		r0, [r5, #64]
		add		r5, #80
		add		r6, #1
b1482:
		ldrb	r0, [r4]
		cmp		r6, r0
		bge		b14A0
		ldrb	r0, [r7, #8]
		cmp		r0, r6
		bgt		b146A
		b		b14A0
b1490:
		mov		r0, r7
		mov		r1, r5
		bl		b23E6
		mov		r0, #0
		strb	r0, [r5]
		add		r5, #80
		add		r6, #1
b14A0:
		ldrb	r0, [r7, #8]
		cmp		r0, r6
		bgt		b1490
		ldrb	r0, [r4, #3]
		lsr		r1, r0, #8
		bcc		b14B0
		bl		SoundDriverMode
b14B0:
		ldr		r0, =0x68736D53				@ 0x14BC
		str		r0, [r7, #52]
b14B4:
		pop		{r4-r7}
		pop		{r3}
		bx		r3
.align 2, 0
.pool

SoundWhatever2:
		push	{r4-r7, lr}
		mov		r7, r0
		ldr		r0, [r0, #52]
		ldr		r6, =0x68736D53				@ 0x14F8
		cmp		r0, r6
		bne		b14F0
		add		r0, #1
		str		r0, [r7, #52]
		ldr		r0, [r7, #4]
		lsl		r3, r6, #31
		orr		r0, r3
		str		r0, [r7, #4]
		ldrb	r5, [r7, #8]
		ldr		r4, [r7, #44]
		b		b14EA
b14DE:
		mov		r0, r7
		mov		r1, r4
		bl		b23E6
		add		r4, #80
		sub		r5, #1
b14EA:
		cmp		r5, #0
		bgt		b14DE
		str		r6, [r7, #52]
b14F0:
		pop		{r4-r7}
		pop		{r3}
		bx		r3
.align 2, 0
.pool

SoundWhatever3:
		ldr		r2, [r0, #52]
		ldr		r1, =0x68736D53				@ 0x1510
		cmp		r2, r1
		bne		b150E
		ldr		r2, [r0, #4]
		str		r1, [r0, #52]
		lsl		r2, r2, #1
		lsr		r2, r2, #1
		str		r2, [r0, #4]
b150E:
		bx		lr
.pool

SoundWhatever4:
		push	{r7}
		ldr		r7, [r0, #52]
		ldr		r2, =0x68736D53				@ 0x1530
		cmp		r7, r2
		bne		b152A
		strh	r1, [r0, #38]
		strh	r1, [r0, #36]
		mov		r1, #0xFF
		add		r1, #1
		strh	r1, [r0, #40]
		str		r2, [r0, #52]
b152A:
		pop		{r7}
		bx		lr
.align 2, 0
.pool

.globl b1534
.thumb_func
b1534:
		push	{r4-r7, lr}
		mov		r7, r0
		ldrh	r0, [r0, #36]
		cmp		r0, #0
		beq		b1572
		ldrh	r1, [r7, #38]
		sub		r1, #1
		lsl		r1, r1, #16
		lsr		r1, r1, #16
		strh	r1, [r7, #38]
		bne		b1572
		ldrh	r1, [r7, #40]
		sub		r1, #0x10
		strh	r1, [r7, #40]
		lsl		r1, r1, #16
		asr		r1, r1, #16
		cmp		r1, #0
		bgt		b1578
		ldrb	r5, [r7, #8]
		ldr		r4, [r7, #44]
		mov		r6, #0
		b		b156E
b1560:
		mov		r0, r7
		mov		r1, r4
		bl		b23E6
		strb	r6, [r4]
		add		r4, #80
		sub		r5, #1
b156E:
		cmp		r5, #0
		bgt		b1560
b1572:
		pop		{r4-r7}
		pop		{r3}
		bx		r3
b1578:
		strh	r0, [r7, #38]
		ldrb	r1, [r7, #8]
		ldr		r0, [r7, #44]
		b		b1596
b1580:
		ldrb	r2, [r0]
		lsr		r3, r2, #8
		bcc		b1592
		ldrh	r3, [r7, #40]
		lsr		r3, r3, #2
		strb	r3, [r0, #19]
		mov		r3, #3
		orr		r2, r3
		strb	r2, [r0]
b1592:
		add		r0, #80
		sub		r1, #1
b1596:
		cmp		r1, #0
		bgt		b1580
		b		b1572

.globl b159C
.thumb_func
b159C:
		push	{r4, r5, r7, lr}
		ldrb	r5, [r1]
		mov		r7, r1
		lsr		r1, r5, #1
		bcc		b1608
		ldrb	r1, [r7, #18]
		ldrb	r2, [r7, #19]
		ldrb	r4, [r7, #24]
		mul		r1, r2
		lsr		r2, r1, #5
		cmp		r4, #1
		bne		b15BA
		mov		r3, #22
		ldrsb	r1, [r7, r3]
		add		r2, r1, r2
b15BA:
		mov		r3, #20
		ldrsb	r1, [r7, r3]
		lsl		r1, r1, #1
		mov		r3, #21
		ldrsb	r3, [r7, r3]
		add		r1, r1, r3
		cmp		r4, #2
		bne		b15D0
		mov		r3, #22
		ldrsb	r3, [r7, r3]
		add		r1, r3, r1
b15D0:
		mov		r3, #0x80
		cmn		r1, r3
		bge		b15DA
		neg		r1, r3
		b		b15E0
b15DA:
		cmp		r1, #127
		ble		b15E0
		mov		r1, #127
b15E0:
		add		r3, r1, #7
		add		r3, #121
		mul		r3, r2
		lsr		r3, r3, #8
		lsl		r3, r3, #24
		lsr		r3, r3, #24
		cmp		r3, #0xFF
		bls		b15F2
		mov		r3, #0xFF
b15F2:
		strb	r3, [r7, #16]
		mov		r3, #127
		sub		r1, r3, r1
		mul		r1, r2
		lsr		r1, r1, #8
		lsl		r1, r1, #24
		lsr		r1, r1, #24
		cmp		r1, #0xFF
		bls		b1606
		mov		r1, #0xFF
b1606:
		strb	r1, [r7, #17]
b1608:
		lsr		r1, r5, #3
		bcc		b1646
		mov		r3, #14
		ldrsb	r1, [r7, r3]
		ldrb	r2, [r7, #15]
		mul		r1, r2
		lsl		r1, r1, #2
		mov		r3, #0xC
		ldrsb	r2, [r7, r3]
		lsl		r2, r2, #2
		add		r1, r1, r2
		mov		r3, #10
		ldrsb	r2, [r7, r3]
		lsl		r2, r2, #8
		add		r1, r1, r2
		mov		r3, #11
		ldrsb	r2, [r7, r3]
		lsl		r2, r2, #8
		add		r1, r1, r2
		ldrb	r2, [r7, #13]
		add		r1, r1, r2
		ldrb	r2, [r7, #24]
		cmp		r2, #0
		bne		b1640
		mov		r3, #22
		ldrsb	r2, [r7, r3]
		lsl		r2, r2, #4
		add		r1, r2, r1
b1640:
		asr		r2, r1, #8
		strb	r2, [r7, #8]
		strb	r1, [r7, #9]
b1646:
		ldr		r2, =0x03007FC0				@ 0x1660
		mov		r1, r7
		ldr		r2, [r2, #48]
		ldr		r2, [r2, #60]
		bl		b13C0
		ldrb	r0, [r7]
		mov		r3, #5
		bic		r0, r3
		strb	r0, [r7]
		pop		{r4, r5, r7}
		pop		{r3}
		bx		r3
.pool

.globl SoundDriverInit
.thumb_func
SoundDriverInit:
b1664:
		push	{r3, r7, lr}
		mov		r7, r0
		ldr		r1, =0x040000C0				@ 0x16DC
		mov		r0, #0
		strh	r0, [r1, #6]
		strh	r0, [r1, #18]
		ldr		r0, =0x04000080				@ 0x16E0
		mov		r2, #143
		strh	r2, [r0, #4]
		ldr		r2, =0xA90E					@ 0x16E4
		strh	r2, [r0, #2]
		ldrb	r2, [r0, #9]
		lsl		r2, r2, #26
		lsr		r2, r2, #26
		mov		r3, #64
		orr		r2, r3
		mov		r3, #53
		lsl		r3, r3, #4
		strb	r2, [r0, #9]
		add		r2, r7, r3
		str		r2, [r0, #60]
		ldr		r0, =0x040000A0				@ 0x16E8
		mov		r3, #19
		lsl		r3, r3, #7
		str		r0, [r1]
		add		r0, r7, r3
		str		r0, [r1, #8]
		ldr		r0, =0x040000A4				@ 0x16EC
		ldr		r2, b16DC+0x18				@ 0x16F4
		str		r0, [r1, #12]
		ldr		r0, b16DC+0x14				@ 0x16F0
		str		r7, [r0, #48]
		mov		r0, #0
		str		r0, [sp]
		mov		r0, sp
		mov		r1, r7
		bl		CpuSet
		mov		r0, #8
		strb	r0, [r7, #6]
		mov		r0, #15
		strb	r0, [r7, #7]
		ldr		r0, b16DC+0x1C				@ 0x16F8
		str		r0, [r7, #56]
		ldr		r0, b16DC+0x20				@ 0x16FC
		str		r0, [r7, #40]
		str		r0, [r7, #44]
		str		r0, [r7, #48]
		str		r0, [r7, #60]
		ldr		r0, b16DC+0x24				@ 0x1700
		str		r0, [r7, #52]
		mov		r0, #1
		lsl		r0, r0, #18
		bl		b170A
		ldr		r0, b16DC+0x28				@ 0x1704
		str		r0, [r7]
		pop		{r3, r7}
		pop		{r3}
		bx		r3
b16DC:
.pool
b16F0:
		.word	0x03007FC0
		.word	0x050003EC
		.word	0x2425
		.word	0x1709
		.word	0x3738
		.word	0x68736D53
b1708:
		bx		lr

.globl b170A
.thumb_func
b170A:
		push	{r4, r7, lr}
		ldr		r1, =0x03007FC0				@ 0x1784
		mov		r3, #0xF
		lsl		r3, r3, #16
		and		r0, r3
		ldr		r7, [r1, #48]
		lsr		r0, r0, #16
		strb	r0, [r7, #8]
		ldr		r1, =0x31E8					@ 0x1788
		lsl		r0, r0, #1
		add		r0, r0, r1
		sub		r0, #32
		ldrh	r0, [r0, #30]
		mov		r1, #99
		lsl		r1, r1, #4
		mov		r4, r0
		str		r0, [r7, #16]
		blown	0x3720, 0x172C
		strb	r0, [r7, #11]
		ldr		r0, =0x00091D1B				@ 0x178C
		ldr		r3, =5000					@ 0x1790
		mul		r0, r4
		add		r1, r0, r3
		lsl		r0, r3, #1
		blown	0x3720, 0x173C
		mov		r1, #1
		lsl		r1, r1, #24
		str		r0, [r7, #20]
		blown	0x3720, 0x1746
		add		r0, #1
		asr		r0, r0, #1
		str		r0, [r7, #24]
		ldr		r4, =0x04000100				@ 0x1794
		mov		r0, #0
		strh	r0, [r4, #2]
		ldr		r0, [r7, #16]
		ldr		r1, =0x00044940				@ 0x1798
		blown	0x3720, 0x175A
		mov		r1, #1
		lsl		r1, r1, #16
		sub		r0, r1, r0
		strh	r0, [r4]
		bl		SoundDriverVSyncOn
		mov		r0, #1
		lsl		r0, r0, #26
b176E:
		ldrb	r1, [r0, #6]
		cmp		r1, #159
		beq		b176E
b1774:
		ldrb	r1, [r0, #6]
		cmp		r1, #159
		bne		b1774
		mov		r0, #0x80
		strh	r0, [r4, #2]
		pop		{r4, r7}
		pop		{r3}
		bx		r3
b1784:
.pool

/* GBATEK
SWI 1Bh - SoundDriverMode
Sets the sound driver operation mode.
  r0  Sound driver operation mode
       Bit    Expl.
       0-6    Direct Sound Reverb value (0-127, default=0) (ignored if Bit7=0)
       7      Direct Sound Reverb set (0=ignore, 1=apply reverb value)
       8-11   Direct Sound Simultaneously-produced (1-12 channels, default 8)
       12-15  Direct Sound Master volume (1-15, default 15)
       16-19  Direct Sound Playback Frequency (1-12 = 5734,7884,10512,13379,
              15768,18157,21024,26758,31536,36314,40137,42048, def 4=13379 Hz)
       20-23  Final number of D/A converter bits (8-11 = 9-6bits, def. 9=8bits)
       24-31  Not used.

Return: No return value.
*/
.globl SoundDriverMode
.thumb_func
SoundDriverMode:
b179C:
		push	{r4, r5, r7, lr}
		ldr		r1, =0x03007FC0				@ 0x1818
		ldr		r5, =0x68736D53				@ 0x181C
		ldr		r7, [r1, #48]
		ldr		r1, [r7]
		cmp		r1, r5
		bne		b1812
		add		r1, #1
		str		r1, [r7]
		lsl		r1, r0, #24
		lsr		r1, r1, #24
		beq		b17BA
		lsl		r1, r1, #25
		lsr		r1, r1, #25
		strb	r1, [r7, #5]
b17BA:
		mov		r1, #0xF
		lsl		r1, r1, #8
		and		r1, r0
		beq		b17D6
		lsr		r1, r1, #8
		strb	r1, [r7, #6]
		mov		r1, #12
		mov		r3, #0
		add		r2, r7, #7
		add		r2, #73
b17CE:
		strb	r3, [r2]
		add		r2, #64
		sub		r1, #1
		bne		b17CE
b17D6:
		mov		r1, #0xF
		lsl		r1, r1, #12
		and		r1, r0
		beq		b17E2
		lsr		r1, r1, #12
		strb	r1, [r7, #7]
b17E2:
		mov		r1, #11
		lsl		r1, r1, #20
		and		r1, r0
		beq		b17FE
		mov		r3, #3
		lsl		r3, r3, #20
		ldr		r2, =0x04000080				@ 0x1820
		and		r1, r3
		ldrb	r3, [r2, #9]
		lsr		r1, r1, #14
		lsl		r3, r3, #26
		lsr		r3, r3, #26
		orr		r1, r3
		strb	r1, [r2, #9]
b17FE:
		mov		r4, #0xF
		lsl		r4, r4, #16
		and		r4, r0
		beq		b1810
		bl		SoundDriverVSyncOff
		mov		r0, r4
		bl		b170A
b1810:
		str		r5, [r7]
b1812:
		pop		{r4, r5, r7}
		pop		{r3}
		bx		r3
b1818:
.pool

/* GBATEK
SWI 1Eh - SoundChannelClear
Clears all direct sound channels and stops the sound.
This function may not operate properly when the library which expands the sound driver feature is combined afterwards. In this case, do not use it.
No parameters, no return value.
*/
SoundChannelClear:
		push	{r4-r7, lr}
		ldr		r0, =0x03007FC0				@ 0x1870
		ldr		r6, =0x68736D53				@ 0x1874
		ldr		r7, [r0, #48]
		ldr		r0, [r7]
		cmp		r0, r6
		bne		b1868
		add		r0, #1
		str		r0, [r7]
		add		r0, r7, #7
		mov		r1, #12
		add		r0, #73
b183C:
		mov		r2, #0
		strb	r2, [r0]
		add		r0, #64
		sub		r1, #1
		cmp		r1, #0
		bgt		b183C
		ldr		r5, [r7, #28]
		cmp		r5, #0
		beq		b1866
		mov		r4, #1
b1850:
		lsl		r0, r4, #24
		lsr		r0, r0, #24
		ldr		r1, [r7, #44]
		bl		b13C2
		add		r4, #1
		add		r5, #64
		cmp		r4, #4
		ble		b1850
		mov		r2, #0
		strb	r2, [r5]
b1866:
		str		r6, [r7]
b1868:
		pop		{r4-r7}
		pop		{r3}
		bx		r3
.align 2, 0
b1870:
.pool

/* GBATEK
SWI 28h - SoundDriverVSyncOff
Due to problems with the main program if the V-Blank interrupts are stopped, and SoundDriverVSync cannot be called every 1/60 a second, this function must be used to stop sound DMA.
Otherwise, even if you exceed the limit of the buffer the DMA will not stop and noise will result.
No parameters, no return value.
*/
.globl SoundDriverVSyncOff
.thumb_func
SoundDriverVSyncOff:
b1878:
		push	{r3, r7, lr}
		ldr		r0, =0x03007FC0				@ 0x18B8
		ldr		r3, =0x68736D53				@ 0x18BC
		ldr		r7, [r0, #48]
		ldr		r0, [r7]
		cmp		r0, r3
		bcc		b18B0
		add		r3, #1
		cmp		r0, r3
		bhi		b18B0
		add		r0, #1
		str		r0, [r7]
		mov		r0, #0
		ldr		r1, =0x040000C0				@ 0x18C0
		mov		r3, #53
		strh	r0, [r1, #6]
		strh	r0, [r1, #18]
		strb	r0, [r7, #4]
		lsl		r3, r3, #4
		add		r1, r7, r3
		str		r0, [sp]
		mov		r0, sp
		ldr		r2, =0x05000318				@ 0x18C4
		bl		CpuSet
		ldr		r0, [r7]
		sub		r0, #1
		str		r0, [r7]
b18B0:
		pop		{r3, r7}
		pop		{r3}
		bx		r3
.align 2, 0
.pool

/* GBATEK
SWI 29h - SoundDriverVSyncOn
This function restarts the sound DMA stopped with the previously described SoundDriverVSyncOff.
After calling this function, have a V-Blank occur within 2/60 of a second and call SoundDriverVSync.
No parameters, no return value.
*/
.globl SoundDriverVSyncOn
.thumb_func
SoundDriverVSyncOn:
b18C8:
		mov		r1, #91
		ldr		r0, =0x040000C0				@ 0x18D4
		lsl		r1, r1, #9
		strh	r1, [r0, #6]
		strh	r1, [r0, #18]
		bx		lr
.pool

/* GBATEK
SWI 1Fh - MidiKey2Freq
Calculates the value of the assignment to ((SoundArea)sa).vchn[x].fr when playing the wave data, wa, with the interval (MIDI KEY) mk and the fine adjustment value (halftones=256) fp.
  r0  WaveData* wa
  r1  u8 mk
  r2  u8 fp

Return:
  r0  u32

This function is particularly popular because it allows to read from BIOS memory without copy protection range checks. The formula to read one byte (a) from address (i, 0..3FFF) is:
a = (MidiKey2Freq(i-(((i AND 3)+1)OR 3), 168, 0) * 2) SHR 24
*/
.globl MidiKey2Freq
.thumb_func
MidiKey2Freq:
b18D8:
		push	{r4-r7, lr}
		lsl		r2, r2, #24
		mov		r7, r0
		cmp		r1, #178
		ble		b18E6
		ldr		r2, =0xFF000000				@ 0x1920
		mov		r1, #178
b18E6:
		ldr		r0, =0x3104					@ 0x1924
		ldrb	r3, [r0, r1]
		lsl		r4, r3, #28
		lsr		r4, r4, #28
		lsl		r4, r4, #2
		add		r5, r0, #7
		add		r5, #173
		ldr		r4, [r5, r4]
		lsr		r6, r3, #4
		lsr		r4, r6
		add		r0, r0, r1
		ldrb	r0, [r0, #1]
		lsl		r1, r0, #28
		lsr		r1, r1, #28
		lsl		r1, r1, #2
		ldr		r1, [r5, r1]
		lsr		r0, r0, #4
		lsr		r1, r0
		sub		r0, r1, r4
		mov		r1, r2
		bl		b1DB4
		add		r1, r0, r4
		ldr		r0, [r7, #4]
		bl		b1DB4
		pop		{r4-r7}
		pop		{r3}
		bx		r3
.align
.pool

b1928:
		push	{r4-r7, lr}
		sub		sp, #52
		mov		r1, #0
		mov		r0, #0
		str		r0, [sp, #20]
		mov		r0, #16
		str		r0, [sp, #12]
		mvn		r7, r1
		mov		r0, #0xFF
		str		r1, [sp, #16]
		str		r1, [sp]
		bl		RegisterRamReset
		ldr		r0, b1D2C					@ 0x1D2C
		mov		r5, #1
		strb	r5, [r0]
		mov		r0, #1
		bl		SoundBias
		ldr		r6, b1D2C+0x4				@ 0x1D30
		mov		r0, #8
		lsl		r1, r0, #23
		strh	r5, [r6]
		strh	r0, [r1, #4]
		ldrh	r0, [r6, #4]
		lsr		r0, r0, #15
		beq		b1962
		bl		b284
b1962:
		bl		b874
		mov		r0, #239
		lsl		r0, r0, #7
		mov		r1, #1
		lsl		r1, r1, #26
		strh	r0, [r1, #14]
		mov		r0, #84
		str		r0, [sp, #8]
		mov		r0, #118
		str		r0, [sp, #4]
		mov		r0, #21
		lsl		r0, r0, #10
		str		r0, [r1, #56]
		mov		r0, #59
		lsl		r0, r0, #9
		str		r0, [r1, #60]
		ldr		r1, b1D2C+0xC				@ 0x1D38
		ldr		r0, b1D2C+0x8				@ 0x1D34
		str		r0, [r1, #16]
		bl		b726
		bl		b2D68
		ldr		r0, =0x03003B2C				@ 0x1D3C
		bl		SoundDriverInit
		ldr		r0, =0x00940A00				@ 0x1D40
		bl		SoundDriverMode
		ldr		r1, =0x0300372C				@ 0x1D44
		ldr		r0, =0x030036EC				@ 0x1D48
		mov		r2, #6
		bl		SoundWhatever0
		ldr		r1, =0x0300394C				@ 0x1D4C
		ldr		r0, =0x0300390C				@ 0x1D50
		mov		r2, #6
		bl		SoundWhatever0
		b		b1C62
b19B4:
		mov		r5, #7
		b		b1B0A
b19B8:
		mov		r0, #6
		sub		r2, r0, r5
		lsl		r0, r2, #2
		add		r0, r0, r2
		add		r0, #8
		cmp		r0, r7
		str		r2, [sp, #48]
		bgt		b19D2
		ldr		r3, =0x03003564				@ 0x1D54
		lsl		r1, r5, #2
		ldr		r2, [r3, r1]
		add		r2, #1
		str		r2, [r3, r1]
b19D2:
		ldr		r3, =0x03003564				@ 0x1D54
		lsl		r1, r5, #2
		ldr		r2, [r3, r1]
		ldr		r3, =0x03003580				@ 0x1D58
		lsl		r1, r5, #4
		add		r1, r1, r3
		lsl		r4, r5, #3
		str		r1, [sp, #24]
		str		r2, [sp, #28]
		ldr		r6, [r1, #8]
		mov		r3, #7
		lsl		r3, r3, #24
		str		r4, [sp, #44]
		add		r4, r4, r3
		cmp		r6, #0
		bge		b1AC2
		cmp		r0, r7
		bgt		b19FA
		add		r6, #2
		str		r6, [r1, #8]
b19FA:
		lsl		r0, r5, #16
		asr		r0, r0, #16
		ldr		r1, [sp, #24]
		bl		b738
		mov		r0, #20
		mul		r0, r5
		ldr		r1, =0x030035F0				@ 0x1D5C
		add		r1, r0, r1
		str		r1, [sp, #40]
		str		r1, [sp, #36]
		ldr		r0, [sp, #24]
		bl		b768
		lsl		r0, r5, #5
		ldr		r3, =0x07000026				@ 0x1D60
		mov		r2, #1
		add		r1, r0, r3
		ldr		r0, [sp, #36]
		mov		r3, #8
		add		r0, #12
		blown	0x3730, 0x1A24
		mov		r3, #96
		cmn		r6, r3
		ble		b1AAC
		ldr		r0, [r4]
		lsl		r2, r3, #3
		orr		r2, r0
		mov		r0, #63
		mvn		r0, r0
		mov		r1, r0
		cmp		r6, r0
		str		r2, [r4]
		bge		b1A44
		cmp		r5, #4
		bge		b1A84
b1A44:
		mov		r3, #75
		cmn		r6, r3
		blt		b1A84
		cmp		r6, #0
		bge		b1A76
		ldrh	r0, [r4]
		mov		r3, #1
		lsl		r3, r3, #15
		orr		r0, r3
		strh	r0, [r4]
		ldr		r0, [sp, #44]
		ldr		r2, =0x369C					@ 0x1D64
		add		r0, r0, r2
		ldrh	r2, [r4, #4]
		ldrh	r0, [r0, #4]
		lsr		r2, r2, #10
		lsl		r2, r2, #10
		add		r0, #4
		lsl		r0, r0, #22
		lsr		r0, r0, #22
		orr		r0, r2
		strh	r0, [r4, #4]
		mov		r0, #31
		mvn		r0, r0
		b		b1A84
b1A76:
		mov		r3, #3
		lsl		r3, r3, #8
		bic		r2, r3
		mov		r0, #15
		mvn		r0, r0
		lsl		r1, r0, #1
		str		r2, [r4]
b1A84:
		ldr		r2, [sp, #40]
		ldr		r3, =0xFE00FFFF				@ 0x1D68
		ldrh	r2, [r2, #8]
		add		r0, r2, r0
		ldr		r2, [r4]
		and		r2, r3
		lsl		r0, r0, #23
		lsr		r0, r0, #23
		lsl		r0, r0, #16
		orr		r0, r2
		str		r0, [r4]
		ldr		r2, [sp, #40]
		ldrh	r2, [r2, #10]
		add		r1, r2, r1
		lsr		r0, r0, #8
		lsl		r0, r0, #8
		lsl		r1, r1, #24
		lsr		r1, r1, #24
		orr		r0, r1
		str		r0, [r4]
b1AAC:
		neg		r0, r6
		lsl		r1, r0, #28
		lsr		r1, r1, #28
		lsl		r1, r1, #1
		ldr		r2, [sp, #48]
		asr		r0, r0, #4
		lsl		r2, r2, #2
		add		r0, #1
		add		r2, #1
		bl		b7BC
b1AC2:
		ldr		r2, [sp, #28]
		sub		r0, r2, #7
		sub		r0, #56
		cmp		r0, #34
		bhi		b1AE6
		ldr		r0, =0x36EC					@ 0x1D6C
		ldr		r2, [sp, #28]
		add		r0, r0, r2
		sub		r0, #64
		ldrb	r1, [r0, #1]
		ldr		r0, [r4]
		lsr		r2, r0, #8
		lsl		r2, r2, #8
		add		r0, r0, r1
		lsl		r0, r0, #24
		lsr		r0, r0, #24
		orr		r0, r2
		str		r0, [r4]
b1AE6:
		ldr		r2, [sp, #28]
		sub		r1, r2, #7
		sub		r1, #89
		cmp		r1, #80
		bhi		b1B0A
		mov		r0, #5
		blown	0x3720, 0x1AF2
		sub		r0, #8
		bl		b39C
		ldr		r2, [sp, #48]
		lsl		r1, r0, #2
		lsl		r0, r2, #2
		add		r2, r0, #1
		mov		r0, #0
		bl		b7BC
b1B0A:
		sub		r5, #1
		bmi		b1B10
		b		b19B8
b1B10:
		mov		r4, #7
		lsl		r4, r4, #24
		cmp		r7, #108
		beq		b1B1C
		cmp		r7, #180
		bne		b1B38
b1B1C:
		ldr		r0, [sp, #8]
		mov		r1, #6
		sub		r0, #56
		str		r0, [sp, #8]
		ldr		r0, [sp, #4]
		str		r1, [sp, #16]
		sub		r0, #128
		str		r0, [sp, #4]
		mov		r0, #10
		str		r0, [sp, #12]
		ldr		r0, =0x10001F5F				@ 0x1D70
		ldr		r1, b1D2C+0xC				@ 0x1D38
		str		r0, [r1, #16]
		b		b1BC4
b1B38:
		cmp		r7, #108
		ble		b1BBC
		ldr		r0, [sp, #8]
		sub		r0, #3
		str		r0, [sp, #8]
		ldr		r0, [sp, #32]
		cmp		r0, #0
		bne		b1B4C
		mov		r0, #1
		b		b1B4E
b1B4C:
		mov		r0, #2
b1B4E:
		mov		r3, #3
		ldr		r1, [r4, #72]
		lsl		r3, r3, #8
		bic		r1, r3
		lsl		r0, r0, #30
		lsr		r0, r0, #30
		lsl		r0, r0, #8
		orr		r0, r1
		ldr		r1, =0xFE00FFFF				@ 0x1D68
		and		r1, r0
		mov		r3, #1
		lsl		r3, r3, #18
		add		r0, r0, r3
		ldr		r3, =0xFE00FFFF				@ 0x1D68
		bic		r0, r3
		orr		r0, r1
		str		r0, [r4, #72]
		mov		r5, #0
b1B72:
		lsl		r0, r5, #3
		add		r0, r0, r4
		add		r1, r0, #7
		add		r1, #121
		mov		r2, #3
		mov		r6, r1
		bl		CpuSet
		ldrh	r0, [r6]
		mov		r3, #3
		lsl		r3, r3, #10
		eor		r0, r3
		add		r5, #1
		cmp		r5, #9
		strh	r0, [r6]
		blt		b1B72
		mov		r0, #6
		mov		r1, r7
		blown	0x3720, 0x1B96
		cmp		r1, #0
		bne		b1BB2
		ldr		r1, [sp, #16]
		ldr		r0, [sp, #12]
		add		r1, #1
		sub		r0, #1
		str		r0, [sp, #12]
		lsl		r0, r0, #8
		orr		r0, r1
		str		r1, [sp, #16]
		ldr		r1, b1D2C+0xC				@ 0x1D38
		strh	r0, [r1, #18]
b1BB2:
		ldr		r0, =0x3F27					@ 0x1D74
		ldr		r1, b1D2C+0xC				@ 0x1D38
		strh	r0, [r1, #10]
		ldr		r0, =0x9802					@ 0x1D78
		b		b1BBE
b1BBC:
		ldr		r0, =0x1002					@ 0x1D7C
b1BBE:
		mov		r1, #1
		lsl		r1, r1, #26
		strh	r0, [r1]
b1BC4:
		ldr		r0, [sp, #8]
		lsl		r0, r0, #8
		mov		r1, #1
		lsl		r1, r1, #26
		str		r0, [r1, #56]
		ldr		r0, [sp, #4]
		lsl		r0, r0, #8
		str		r0, [r1, #60]
		cmp		r7, #16
		blt		b1BE4
		bl		SoundDriverMain
		cmp		r7, #16
		bne		b1BE4
		ldr		r1, =0x3908					@ 0x1D80
		b		b1BEA
b1BE4:
		cmp		r7, #162
		bne		b1BF0
		ldr		r1, =0x39C0					@ 0x1D84
b1BEA:
		ldr		r0, =0x030036EC				@ 0x1D48
		bl		SoundWhatever1
b1BF0:
		sub		r0, r7, #7
		sub		r0, #58
		cmp		r0, #79
		bcs		b1C1C
		ldr		r0, [sp, #32]
		cmp		r0, #0
		bne		b1C1C
		ldr		r0, =0x03000064				@ 0x1D88
		mov		r3, #1
		ldr		r0, [r0, #36]
		cmn		r0, r3
		beq		b1C1C
		ldr		r0, =0x04000130				@ 0x1D8C
		ldrb	r0, [r0]
		cmp		r0, #243
		bne		b1C1C
		ldr		r1, =0x389C					@ 0x1D90
		ldr		r0, =0x0300390C				@ 0x1D50
		bl		SoundWhatever1
		mov		r0, #1
		str		r0, [sp, #32]
b1C1C:
		cmp		r7, #56
		ble		b1C3C
		ldr		r0, [sp, #32]
		cmp		r0, #0
		beq		b1C3C
		ldr		r1, [sp]
		cmp		r1, #32
		bge		b1C32
		ldr		r1, [sp]
		add		r1, #2
		str		r1, [sp]
b1C32:
		mov		r2, #31
		mov		r0, #6
		ldr		r1, [sp]
		bl		b7BC
b1C3C:
		bl		b2B34
		ldr		r1, b1D2C+0x4				@ 0x1D30
		mov		r0, #1
		strh	r0, [r1, #8]
		blown	0x3728, 0x1C46
		cmp		r7, #16
		bge		b1C62
		ldr		r1, [sp, #16]
		ldr		r0, [sp, #12]
		add		r1, #1
		sub		r0, #1
		str		r0, [sp, #12]
		lsl		r0, r0, #8
		orr		r0, r1
		str		r1, [sp, #16]
		ldr		r1, b1D2C+0xC				@ 0x1D38
		strh	r0, [r1, #18]
b1C62:
		add		r7, #1
		cmp		r7, #210
		bgt		b1C6A
		b		b19B4
b1C6A:
		ldr		r0, =0x03000088				@ 0x1D94
		bl		b6E8
		mov		r6, #0
		mov		r7, r0
		cmp		r0, #0
		ldr		r5, =0x03FFFFF0				@ 0x1D98
		bne		b1C80
		ldr		r0, [sp, #32]
		cmp		r0, #0
		beq		b1CD8
b1C80:
		mov		r0, #1
		strb	r0, [r5, #11]
		strb	r6, [r5, #7]
b1C86:
		bl		b2B34
		lsl		r0, r0, #24
		lsr		r0, r0, #24
		strb	r0, [r5, #10]
		bne		b1CD8
		bl		SoundDriverMain
		blown	0x3728, 0x1C96
		cmp		r7, #0
		bne		b1C86
		ldrb	r0, [r5, #7]
		cmp		r0, #0
		bne		b1C86
		ldrb	r0, [r5, #11]
		cmp		r0, #0
		beq		b1CC2
		ldr		r0, =0x04000130				@ 0x1D8C
		ldrb	r0, [r0]
		mvn		r0, r0
		mov		r3, #243
		and		r0, r3
		beq		b1C86
		ldr		r1, =0x3818					@ 0x1D9C
		ldr		r0, =0x0300390C				@ 0x1D50
		bl		SoundWhatever1
		strb	r6, [r5, #11]
		b		b1C86
b1CC2:
		ldr		r1, [sp]
		cmp		r1, #0
		ble		b1CD8
		ldr		r1, [sp]
		mov		r2, #31
		sub		r1, #1
		str		r1, [sp]
		mov		r0, #6
		bl		b7BC
		b		b1C86
b1CD8:
		ldr		r1, =0x00103FBF				@ 0x1DA0
		ldr		r0, b1D2C+0xC				@ 0x1D38
		str		r1, [r0, #16]
		str		r6, [r0, #20]
		mov		r1, #0
b1CE2:
		lsl		r2, r1, #3
		ldr		r7, [r4, r2]
		mov		r3, #3
		lsl		r3, r3, #10
		bic		r7, r3
		add		r1, #1
		cmp		r1, #9
		str		r7, [r4, r2]
		blt		b1CE2
		mov		r7, #0
		mvn		r7, r7
		mov		r4, r0
		b		b1D16
b1CFC:
		bl		SoundDriverMain
		blown	0x3728, 0x1D00
		lsr		r0, r7, #1
		bcs		b1D16
		ldr		r0, [sp, #20]
		cmp		r0, #16
		beq		b1D16
		ldr		r0, [sp, #20]
		add		r0, #1
		str		r0, [sp, #20]
		str		r0, [r4, #20]
b1D16:
		add		r7, #1
		cmp		r7, #50
		ble		b1CFC
		bl		SoundDriverVSyncOff
		ldrb	r0, [r5, #10]
		cmp		r0, #0
		beq		b1DA4
		mov		r0, #222
		b		b1DA6
		b		b1DA4
b1D2C:
		.word	0x04000300
		.word	0x04000200
		.word	0x10003F5F
		.word	0x04000040
.pool
b1DA4:
		mov		r0, #0xFF
b1DA6:
		bl		RegisterRamReset
		add		sp, #52
		pop		{r4-r7}
		pop		{r3}
		bx		r3
.align 2, 0

b1DB4:
		adr		r2, b1DB8
		bx		r2
.arm
b1DB8:
		umull	r2, r3, r0, r1
		add		r0, r3, #0
		bx		lr

.thumb
SoundDriverMain:
b1DC4:
		ldr		r0, b2138+8					@ ldr		r0, [pc, #888]				@ 0x2140
		ldr		r0, [r0]
		ldr		r2, b2138+0xC					@ ldr		r2, [pc, #888]				@ 0x2144
		ldr		r3, [r0]
		cmp		r2, r3
		beq		b1DD2
		bx		lr
b1DD2:
		.hword	0x1C5B
		@add		r3, r3, #1
		str		r3, [r0]
		push	{r4-r7, lr}
		mov		r1, r8
		mov		r2, r9
		mov		r3, r10
		mov		r4, r11
		push	{r0-r4}
		sub		sp, #20
		ldr		r3, [r0, #32]
		cmp		r3, #0
		beq		b1DF2
		ldr		r0, [r0, #36]
		bl		b2102
		ldr		r0, [sp, #20]
b1DF2:
		ldr		r3, [r0, #40]
		bl		b2102
		ldr		r0, [sp, #20]
		ldr		r3, [r0, #16]
		mov		r8, r3
		ldr		r5, =0x0350					@ 0x2104
		add		r5, r5, r0
		ldrb	r4, [r0, #4]
		sub		r7, r4, #1
		bls		b1E12
		ldrb	r1, [r0, #11]
		sub		r1, r1, r7
		mov		r2, r8
		mul		r2, r1
		add		r5, r5, r2
b1E12:
		str		r5, [sp, #8]
		ldr		r6, =0x0630					@ 0x2108
		ldrb	r3, [r0, #5]
		cmp		r3, #0
		beq		b1E74
		adr		r1, b1E20
		bx		r1
.arm
b1E20:
		cmp		r4, #2
		addeq	r7, r0, #0x350
		addne	r7, r5, r8
		mov		r4, r8
b1E30:
		ldrsb	r0, [r5, r6]
		ldrsb	r1, [r5]
		add		r0, r0, r1
		ldrsb	r1, [r7, r6]
		add		r0, r0, r1
		ldrsb	r1, [r7], #1
		add		r0, r0, r1
		mul		r1, r0, r3
		mov		r0, r1, asr #9
		tst		r0, #0x80
		addne	r0, r0, #1
		strb	r0, [r5, r6]
		strb	r0, [r5], #1
		subs	r4, r4, #1
		bgt		b1E30
		adr		r0, b1E74+0x2F				@ add		r0, pc, #0x2F				@ 0x1E74+0x2F
		bx		r0
.thumb
b1E74:
		mov		r0, #0
		mov		r1, r8
		add		r6, r6, r5
		lsr		r1, r1, #3
		bcc		b1E82
		stmia	r5!, {r0}
		stmia	r6!, {r0}
b1E82:
		lsr		r1, r1, #1
		bcc		b1E8E
		stmia	r5!, {r0}
		stmia	r6!, {r0}
		stmia	r5!, {r0}
		stmia	r6!, {r0}
b1E8E:
		stmia	r5!, {r0}
		stmia	r6!, {r0}
		stmia	r5!, {r0}
		stmia	r6!, {r0}
		stmia	r5!, {r0}
		stmia	r6!, {r0}
		stmia	r5!, {r0}
		stmia	r6!, {r0}
		.hword	0x1E49
		@sub		r1, r1, #1
		bgt		b1E8E
		ldr		r4, [sp, #20]
		ldr		r0, [r4, #20]
		mov		r9, r0
		ldr		r0, [r4, #24]
		mov		r12, r0
		ldrb	r0, [r4, #6]
		add		r4, #80
b1EB0:
		str		r0, [sp, #4]
		ldr		r3, [r4, #36]
		ldrb	r6, [r4]
		mov		r0, #199
		tst		r0, r6
		bne		b1EBE
		b		b20E4
b1EBE:
		mov		r0, #128
		tst		r0, r6
		beq		b1EEE
		mov		r0, #64
		tst		r0, r6
		bne		b1EFE
		mov		r6, #3
		strb	r6, [r4]
		mov		r0, r3
		add		r0, #16
		str		r0, [r4, #40]
		ldr		r0, [r3, #12]
		str		r0, [r4, #24]
		mov		r5, #0
		strb	r5, [r4, #9]
		str		r5, [r4, #28]
		ldrb	r2, [r3, #3]
		mov		r0, #192
		tst		r0, r2
		beq		b1F46
		mov		r0, #16
		orr		r6, r0
		strb	r6, [r4]
		b		b1F46
b1EEE:
		ldrb	r5, [r4, #9]
		mov		r0, #4
		tst		r0, r6
		beq		b1F04
		ldrb	r0, [r4, #13]
		.hword	0x1E40
		@sub		r0, r0, #1
		strb	r0, [r4, #13]
		bhi		b1F54
b1EFE:
		mov		r0, #0
		strb	r0, [r4]
		b		b20E4
b1F04:
		mov		r0, #64
		tst		r0, r6
		beq		b1F24
		ldrb	r0, [r4, #7]
		mul		r5, r0
		lsr		r5, r5, #8
		ldrb	r0, [r4, #12]
		cmp		r5, r0
		bhi		b1F54
b1F16:
		ldrb	r5, [r4, #12]
		cmp		r5, #0
		beq		b1EFE
		mov		r0, #4
		orr		r6, r0
		strb	r6, [r4]
		b		b1F54
b1F24:
		mov		r2, #3
		and		r2, r6
		cmp		r2, #2
		bne		b1F42
		ldrb	r0, [r4, #5]
		mul		r5, r0
		lsr		r5, r5, #8
		ldrb	r0, [r4, #6]
		cmp		r5, r0
		bhi		b1F54
		mov		r5, r0
		beq		b1F16
		.hword	0x1E76
		@sub		r6, r6, #1
		strb	r6, [r4]
		b		b1F54
b1F42:
		cmp		r2, #3
		bne		b1F54
b1F46:
		ldrb	r0, [r4, #4]
		add		r5, r5, r0
		cmp		r5, #0xFF
		bcc		b1F54
		mov		r5, #0xFF
		.hword	0x1E76
		@sub		r6, r6, #1
		strb	r6, [r4]
b1F54:
		strb	r5, [r4, #9]
		ldr		r0, [sp, #20]
		ldrb	r0, [r0, #7]
		.hword	0x1C40
		@add		r0, r0, #1
		mul		r0, r5
		lsr		r5, r0, #4
		ldrb	r0, [r4, #2]
		mul		r0, r5
		lsr		r0, r0, #8
		strb	r0, [r4, #10]
		ldrb	r0, [r4, #3]
		mul		r0, r5
		lsr		r0, r0, #8
		strb	r0, [r4, #11]
		mov		r0, #0x10
		and		r0, r6
		str		r0, [sp, #16]
		beq		b1F88
		mov		r0, r3
		add		r0, #16
		ldr		r1, [r3, #8]
		add		r0, r0, r1
		str		r0, [sp, #12]
		ldr		r0, [r3, #12]
		sub		r0, r0, r1
		str		r0, [sp, #16]
b1F88:
		ldr		r5, [sp, #8]
		ldr		r2, [r4, #24]
		ldr		r3, [r4, #40]
		adr		r0, b1F94
		bx		r0
.align 2, 0
.arm
b1F94:
		str		r8, [sp]
		ldrb	r10, [r4, #10]
		ldrb	r11, [r4, #11]
		ldrb	r0, [r4, #1]
		tst		r0, #8
		beq		b1FFC
b1FAC:
		ldrsb	r6, [r3], #1
		mul		r1, r6, r11
		ldrb	r0, [r5, #0x630]
		add		r0, r0, r1, asr #8
		strb	r0, [r5, #0x630]
		mul		r1, r6, r10
		ldrb	r0, [r5]
		add		r0, r0, r1, asr #8
		strb	r0, [r5], #1
		subs	r2, r2, #1
		bne		b1FF0
		ldr		r2, [sp, #16]
		cmp		r2, #0
		ldrne	r3, [sp, #12]
		bne		b1FF0
		strb	r2, [r4]
		b		b20D8
b1FF0:
		.word	0xE2588001
		@subs	r8, r8, #1
		bgt		b1FAC
		b		b20D0
b1FFC:
		ldr		r7, [r4, #28]
		ldr		lr, [r4, #32]
b2004:
		cmp		r7, r9, lsl #2
		bcc		b2028
b200C:
		cmp		r2, #4
		ble		b204C
		sub		r2, r2, #0x4
		add		r3, r3, #0x4
		sub		r7, r7, r9, lsl	#2
		cmp		r7, r9, lsl #2
		bcs		b200C
b2028:
		cmp		r7, r9, lsl	#1
		bcc		b2044
		cmp		r2, #2
		ble		b204C
		sub		r2, r2, #0x2
		add		r3, r3, #0x2
		sub		r7, r7, r9, lsl	#1
b2044:
		cmp		r7, r9
		bcc		b207C
b204C:
		subs	r2, r2, #0x1
		bne		b206C
		ldr		r2, [sp, #16]
		cmp		r2, #0
		ldrne	r3, [sp, #12]
		bne		b2070
		strb	r2, [r4]
		b		b20D8
b206C:
		add		r3, r3, #0x1
b2070:
		sub		r7, r7, r9
		cmp		r7, r9
		bcs		b204C
b207C:
		ldrsb	r0, [r3]
		ldrsb	r1, [r3, #1]
		sub		r1, r1, r0
		mul		r6, r1, r7
		mul		r1, r6, r12
		add		r6, r0, r1, asr #23
		mul		r1, r6, r11
		ldrb	r0, [r5, #0x630]
		add		r0, r0, r1, asr #8
		strb	r0, [r5, #0x630]
		mul		r1, r6, r10
		ldrb	r0, [r5]
		add		r0, r0, r1, asr #8
		strb	r0, [r5], #1
		add		r7, r7, lr
		subs	r8, r8, #0x1
		beq		b20CC
		cmp		r7, r9
		bcc		b207C
		b		b2004
b20CC:
		str		r7, [r4, #28]
b20D0:
		str		r2, [r4, #24]
		str		r3, [r4, #40]
b20D8:
		ldr		r8, [sp]
		add		r0, pc, #0x1
		bx		r0

.thumb
b20E4:
		ldr		r0, [sp, #4]
		.hword	0x1E40
		@sub		r0, r0, #1
		ble		b20EE
		add		r4, #64
		b		b1EB0
b20EE:
		ldr		r0, [sp, #20]
		ldr		r3, [pc, #80]				@ 0x2144
		str		r3, [r0]
		add		sp, #24
		pop		{r0-r7}
		mov		r8, r0
		mov		r9, r1
		mov		r10, r2
		mov		r11, r3
		pop		{r3}

.globl b2102
.thumb_func
b2102:
		bx		r3
b2104:
.pool

SoundDriverVSync:
		ldr		r0, b2138+0x8				@ 0x2140
		ldr		r0, [r0]
		ldr		r2, b2138+0xC				@ 0x2144
		ldr		r3, [r0]
		cmp		r2, r3
		bne		b2136
		ldrb	r1, [r0, #4]
		.hword	0x1E49
		@sub		r1, r1, #1
		strb	r1, [r0, #4]
		bgt		b2136
		ldrb	r1, [r0, #11]
		strb	r1, [r0, #4]
		mov		r0, #0
		mov		r1, #182
		lsl		r1, r1, #8
		ldr		r2, =0x040000C6				@ 0x2138
		ldr		r3, =0x040000D2				@ 0x213C
		strh	r0, [r2]
		strh	r0, [r3]
		strh	r1, [r2]
		strh	r1, [r3]
b2136:
		bx		lr
b2138:
.pool
		.word	0x03007FF0
		.word	0x68736D53
b2148:
		ldr		r2, b23A4+0x8				@ 0x23AC
		ldr		r3, [r0, #52]
		cmp		r2, r3
		beq		b2152
		bx		lr
b2152:
		.hword	0x1C5B
		@add		r3, r3, #1
		str		r3, [r0, #52]
		push	{r4-r7, lr}
		mov		r4, r8
		mov		r5, r9
		mov		r6, r10
		mov		r7, r11
		push	{r4-r7}
		mov		r7, r0
		ldr		r3, [r7, #56]
		cmp		r3, #0
		beq		b2170
		ldr		r0, [r7, #60]
		bl		b2102
b2170:
		ldr		r0, [r7, #4]
		cmp		r0, #0
		bge		b2178
		b		b2392
b2178:
		ldr		r0, b23A4+0x4				@ 0x23A8
		ldr		r0, [r0]
		mov		r8, r0
		mov		r0, r7
		bl		b1534
		ldrh	r0, [r7, #34]
		ldrh	r1, [r7, #32]
		add		r0, r0, r1
		b		b22D6
b218C:
		ldrb	r2, [r7, #8]
		ldr		r5, [r7, #44]
		mov		r3, #1
		mov		r4, #0
b2194:
		ldrb	r0, [r5]
		mov		r1, #128
		tst		r1, r0
		bne		b219E
		b		b22B6
b219E:
		mov		r9, r2
		mov		r10, r3
		orr		r4, r3
		mov		r11, r4
		ldr		r4, [r5, #32]
		cmp		r4, #0
		beq		b21D4
b21AC:
		ldrb	r1, [r4]
		mov		r0, #199
		tst		r0, r1
		beq		b21C8
		ldrb	r0, [r4, #16]
		cmp		r0, #0
		beq		b21CE
		.hword	0x1E40
		@sub		r0, r0, #1
		strb	r0, [r4, #16]
		bne		b21CE
		mov		r0, #64
		orr		r1, r0
		strb	r1, [r4]
		b		b21CE
b21C8:
		mov		r0, r4
		bl		b23C6
b21CE:
		ldr		r4, [r4, #52]
		cmp		r4, #0
		bne		b21AC
b21D4:
		ldrb	r3, [r5]
		mov		r0, #64
		tst		r0, r3
		beq		b2254
		mov		r0, r5
		bl		b23B0
		mov		r0, #128
		strb	r0, [r5]
		mov		r0, #2
		strb	r0, [r5, #15]
		mov		r0, #64
		strb	r0, [r5, #19]
		mov		r0, #22
		strb	r0, [r5, #25]
		mov		r0, #1
		add		r1, r5, #6
		strb	r0, [r1, #30]
		b		b2254
b21FA:
		ldr		r2, [r5, #64]
		ldrb	r1, [r2]
		cmp		r1, #128
		bcs		b2206
		ldrb	r1, [r5, #7]
		b		b2210
b2206:
		.hword	0x1C52
		@add		r2, r2, #1
		str		r2, [r5, #64]
		cmp		r1, #189
		bcc		b2210
		strb	r1, [r5, #7]
b2210:
		cmp		r1, #207
		bcc		b2226
		mov		r0, r8
		ldr		r3, [r0, #56]
		mov		r0, r1
		sub		r0, #207
		mov		r1, r7
		mov		r2, r5
		bl		b2102
		b		b2254
b2226:
		cmp		r1, #176
		bls		b224A
		mov		r0, r1
		sub		r0, #177
		strb	r0, [r7, #10]
		mov		r3, r8
		ldr		r3, [r3, #52]
		lsl		r0, r0, #2
		add		r3, r3, r0
		ldr		r3, [r3]
		mov		r0, r7
		mov		r1, r5
		bl		b2102
		ldrb	r0, [r5]
		cmp		r0, #0
		beq		b22B0
		b		b2254
b224A:
		ldr		r0, =0x30D0				@ 0x23A4
		sub		r1, #128
		add		r1, r1, r0
		ldrb	r0, [r1]
		strb	r0, [r5, #1]
b2254:
		ldrb	r0, [r5, #1]
		cmp		r0, #0
		beq		b21FA
		.hword	0x1E40
		@sub		r0, r0, #1
		strb	r0, [r5, #1]
		ldrb	r1, [r5, #25]
		cmp		r1, #0
		beq		b22B0
		ldrb	r0, [r5, #23]
		cmp		r0, #0
		beq		b22B0
		ldrb	r0, [r5, #28]
		cmp		r0, #0
		beq		b2276
		.hword	0x1E40
		@sub		r0, r0, #1
		strb	r0, [r5, #28]
		b		b22B0
b2276:
		ldrb	r0, [r5, #26]
		add		r0, r0, r1
		strb	r0, [r5, #26]
		mov		r1, r0
		sub		r0, #64
		lsl		r0, r0, #24
		bpl		b228A
		lsl		r2, r1, #24
		asr		r2, r2, #24
		b		b228E
b228A:
		mov		r0, #128
		sub		r2, r0, r1
b228E:
		ldrb	r0, [r5, #23]
		mul		r0, r2
		asr		r2, r0, #6
		ldrb	r0, [r5, #22]
		eor		r0, r2
		lsl		r0, r0, #24
		beq		b22B0
		strb	r2, [r5, #22]
		ldrb	r0, [r5]
		ldrb	r1, [r5, #24]
		cmp		r1, #0
		bne		b22AA
		mov		r1, #12
		b		b22AC
b22AA:
		mov		r1, #3
b22AC:
		orr		r0, r1
		strb	r0, [r5]
b22B0:
		mov		r2, r9
		mov		r3, r10
		mov		r4, r11
b22B6:
		.hword	0x1E52
		@sub		r2, r2, #1
		ble		b22C2
		mov		r0, #80
		add		r5, r5, r0
		lsl		r3, r3, #1
		b		b2194
b22C2:
		mov		r6, r11
		cmp		r6, #0
		bne		b22D0
		mov		r0, #0x80
		lsl		r0, r0, #24
		str		r0, [r7, #4]
		b		b2392
b22D0:
		str		r6, [r7, #4]
		ldrh	r0, [r7, #34]
		sub		r0, #150
b22D6:
		strh	r0, [r7, #34]
		cmp		r0, #150
		bcc		b22DE
		b		b218C
b22DE:
		ldrb	r2, [r7, #8]
		ldr		r5, [r7, #44]
b22E2:
		ldrb	r0, [r5]
		mov		r1, #0x80
		tst		r1, r0
		beq		b2388
		mov		r1, #15
		tst		r1, r0
		beq		b2388
		mov		r9, r2
		mov		r0, r7
		mov		r1, r5
		bl		b159C
		ldr		r4, [r5, #32]
		cmp		r4, #0
		beq		b237E
b2300:
		ldrb	r1, [r4]
		mov		r0, #199
		tst		r0, r1
		bne		b2310
		mov		r0, r4
		bl		b23C6
		b		b2378
b2310:
		ldrb	r0, [r4, #1]
		mov		r6, #7
		and		r6, r0
		ldrb	r3, [r5]
		mov		r0, #3
		tst		r0, r3
		beq		b233C
		ldrb	r1, [r4, #18]
		ldrb	r0, [r5, #16]
		mul		r0, r1
		asr		r0, r0, #7
		strb	r0, [r4, #2]
		ldrb	r0, [r5, #17]
		mul		r0, r1
		asr		r0, r0, #7
		strb	r0, [r4, #3]
		cmp		r6, #0
		beq		b233C
		ldrb	r0, [r4, #29]
		mov		r1, #1
		orr		r0, r1
		strb	r0, [r4, #29]
b233C:
		mov		r0, #12
		tst		r0, r3
		beq		b2378
		ldrb	r1, [r4, #8]
		mov		r0, #8
		ldrsb	r0, [r5, r0]
		add		r2, r1, r0
		bpl		b234E
		mov		r2, #0
b234E:
		cmp		r6, #0
		beq		b236C
		mov		r0, r8
		ldr		r3, [r0, #48]
		mov		r1, r2
		ldrb	r2, [r5, #9]
		mov		r0, r6
		bl		b2102
		str		r0, [r4, #32]
		ldrb	r0, [r4, #29]
		mov		r1, #2
		orr		r0, r1
		strb	r0, [r4, #29]
		b		b2378
b236C:
		mov		r1, r2
		ldrb	r2, [r5, #9]
		ldr		r0, [r4, #36]
		bl		MidiKey2Freq
		str		r0, [r4, #32]
b2378:
		ldr		r4, [r4, #52]
		cmp		r4, #0
		bne		b2300
b237E:
		ldrb	r0, [r5]
		mov		r1, #240
		and		r0, r1
		strb	r0, [r5]
		mov		r2, r9
b2388:
		.hword	0x1E52
		@sub		r2, r2, #1
		ble		b2392
		mov		r0, #80
		add		r5, r5, r0
		bgt		b22E2
b2392:
		ldr		r0, b23A4+0x8			@ 0x23AC
		str		r0, [r7, #52]
		pop		{r0-r7}
		mov		r8, r0
		mov		r9, r1
		mov		r10, r2
		mov		r11, r3
		pop		{r0}
		bx		r0

b23A4:
.pool
		.word	0x03007FF0
		.word	0x68736D53

b23B0:
		mov		r12, r4
		mov		r1, #0
		mov		r2, #0
		mov		r3, #0
		mov		r4, #0
		stmia	r0!,{r1-r4}
		stmia	r0!,{r1-r4}
		stmia	r0!,{r1-r4}
		stmia	r0!,{r1-r4}
		mov		r4, r12
		bx		lr

.globl b23C6
.thumb_func
b23C6:
		ldr		r3, [r0, #44]
		cmp		r3, #0
		beq		b23E4
		ldr		r1, [r0, #52]
		ldr		r2, [r0, #48]
		cmp		r2, #0
		beq		b23D8
		str		r1, [r2, #52]
		b		b23DA
b23D8:
		str		r1, [r3, #32]
b23DA:
		cmp		r1, #0
		beq		b23E0
		str		r2, [r1, #48]
b23E0:
		mov		r1, #0
		str		r1, [r0, #44]
b23E4:
		bx		lr

b23E6:
		push	{r4-r6, lr}
		mov		r5, r1
		ldrb	r1, [r5]
		mov		r0, #0x80
		tst		r0, r1
		beq		b241E
		ldr		r4, [r5, #32]
		cmp		r4, #0
		beq		b241C
		mov		r6, #0
b23FA:
		ldrb	r0, [r4]
		cmp		r0, #0
		beq		b2416
		ldrb	r0, [r4, #1]
		mov		r3, #7
		and		r0, r3
		beq		b2412
		ldr		r3, =0x03007FF0		@ p2620
		ldr		r3, [r3]
		ldr		r3, [r3, #44]
		bl		b2102
b2412:
		strb	r6, [r4]
		str		r6, [r4, #44]
b2416:
		ldr		r4, [r4, #52]
		cmp		r4, #0
		bne		b23FA
b241C:
		str		r4, [r5, #32]
b241E:
		pop		{r4-r6}
		pop		{r0}
		bx		r0

b2424:
		push	{r4-r7, lr}
		mov		r4, r8
		mov		r5, r9
		mov		r6, r10
		mov		r7, r11
b242E:
		push	{r4-r7}
		sub		sp, #20
		str		r1, [sp]
		mov		r5, r2
		ldr		r1, =0x03007FF0		@ p2620
		ldr		r1, [r1]
		str		r1, [sp, #4]
		ldr		r1, =0x30D0			@ p2620+0x4
		.hword	0x1840
		@add		r0, r0, r1
		ldrb	r0, [r0]
		strb	r0, [r5, #4]
		ldr		r3, [r5, #64]
		ldrb	r0, [r3]
		cmp		r0, #0x80
		bcs		b246A
		strb	r0, [r5, #5]
		.hword	0x1C5B
		@add		r3, r3, #1
		ldrb	r0, [r3]
		cmp		r0, #0x80
		bcs		b2468
		strb	r0, [r5, #6]
		.hword	0x1C5B
		@add		r3, r3, #1
		ldrb	r0, [r3]
		cmp		r0, #0x80
		bcs		b2468
		ldrb	r1, [r5, #4]
		add		r1, r1, r0
		strb	r1, [r5, #4]
		.hword	0x1C5B
		@add		r3, r3, #1
b2468:
		str		r3, [r5, #64]
b246A:
		mov		r4, r5
		add		r4, #36
		ldrb	r2, [r4]
		mov		r0, #192
		tst		r0, r2
		beq		b24C0
		ldrb	r3, [r5, #5]
		mov		r0, #64
		tst		r0, r2
		beq		b2486
		ldr		r1, [r5, #44]
		add		r1, r1, r3
		ldrb	r0, [r1]
		b		b2488
b2486:
		mov		r0, r3
b2488:
		lsl		r1, r0, #1
		add		r1, r1, r0
		lsl		r1, r1, #2
		ldr		r0, [r5, #40]
		add		r1, r1, r0
		mov		r9, r1
		mov		r6, r9
		ldrb	r1, [r6]
		mov		r0, #192
		tst		r0, r1
		beq		b24A0
		b		b260E
b24A0:
		mov		r0, #0x80
		tst		r0, r2
		beq		b24C4
		ldrb	r1, [r6, #3]
		mov		r0, #0x80
		tst		r0, r1
		beq		b24BC
		sub		r1, #192
		lsl		r1, r1, #1
		strb	r1, [r5, #21]
		ldrb	r0, [r5]
		mov		r1, #3
		orr		r0, r1
		strb	r0, [r5]
b24BC:
		ldrb	r3, [r6, #1]
		b		b24C4
b24C0:
		mov		r9, r4
		ldrb	r3, [r5, #5]
b24C4:
		str		r3, [sp, #8]
		ldr		r6, [sp]
		ldrb	r1, [r6, #9]
		ldrb	r0, [r5, #29]
		add		r0, r0, r1
		cmp		r0, #0xFF
		bls		b24D4
		mov		r0, #0xFF
b24D4:
		str		r0, [sp, #16]
		mov		r6, r9
		ldrb	r0, [r6]
		mov		r6, #7
		and		r6, r0
		str		r6, [sp, #12]
		beq		b2514
		ldr		r0, [sp, #4]
		ldr		r4, [r0, #28]
		cmp		r4, #0
		bne		b24EC
		b		b260E
b24EC:
		.hword	0x1E76
		@sub		r6, r6, #1
		lsl		r0, r6, #6
		add		r4, r4, r0
		ldrb	r1, [r4]
		mov		r0, #199
		tst		r0, r1
		beq		b2568
		mov		r0, #64
		tst		r0, r1
		bne		b2568
		ldrb	r1, [r4, #19]
		ldr		r0, [sp, #16]
		cmp		r1, r0
		bcc		b2568
		beq		b250C
		b		b260E
b250C:
		ldr		r0, [r4, #44]
		cmp		r0, r5
		bcs		b2568
		b		b260E
b2514:
		ldr		r6, [sp, #16]
		mov		r7, r5
		mov		r2, #0
		mov		r8, r2
		ldr		r4, [sp, #4]
		ldrb	r3, [r4, #6]
		add		r4, #80
b2522:
		ldrb	r1, [r4]
		mov		r0, #199
		tst		r0, r1
		beq		b2568
		mov		r0, #64
		tst		r0, r1
		beq		b253C
		cmp		r2, #0
		bne		b2540
		.hword	0x1C52
		@add		r2, r2, #1
		ldrb	r6, [r4, #19]
		ldr		r7, [r4, #44]
		b		b255A
b253C:
		cmp		r2, #0
		bne		b255C
b2540:
		ldrb	r0, [r4, #19]
		cmp		r0, r6
		bcs		b254C
		mov		r6, r0
		ldr		r7, [r4, #44]
		b		b255A
b254C:
		bhi		b255C
		ldr		r0, [r4, #44]
		cmp		r0, r7
		bls		b2558
		mov		r7, r0
		b		b255A
b2558:
		bcc		b255C
b255A:
		mov		r8, r4
b255C:
		add		r4, #64
		.hword	0x1E5B
		@sub		r3, r3, #1
		bgt		b2522
		mov		r4, r8
		cmp		r4, #0
		beq		b260E
b2568:
		mov		r0, r4
		bl		b23C6
		mov		r1, #0
		str		r1, [r4, #48]
		ldr		r3, [r5, #32]
		str		r3, [r4, #52]
		cmp		r3, #0
		beq		b257C
		str		r4, [r3, #48]
b257C:
		str		r4, [r5, #32]
		str		r5, [r4, #44]
		ldrb	r0, [r5, #27]
		strb	r0, [r5, #28]
		cmp		r0, r1
		beq		b258C
		strb	r1, [r5, #26]
		strb	r1, [r5, #22]
b258C:
		ldr		r0, [sp]
		mov		r1, r5
		bl		b159C
		ldr		r0, [r5, #4]
		str		r0, [r4, #16]
		ldr		r0, [sp, #16]
		strb	r0, [r4, #19]
		ldr		r0, [sp, #8]
		strb	r0, [r4, #8]
		mov		r6, r9
		ldrb	r0, [r6]
		strb	r0, [r4, #1]
		ldr		r7, [r6, #4]
		str		r7, [r4, #36]
		ldr		r0, [r6, #8]
		str		r0, [r4, #4]
		ldrh	r0, [r5, #30]
		strh	r0, [r4, #12]
		ldrb	r1, [r4, #18]
		ldrb	r0, [r5, #16]
		mul		r0, r1
		asr		r0, r0, #7
		strb	r0, [r4, #2]
		ldrb	r0, [r5, #17]
		mul		r0, r1
		asr		r0, r0, #7
		strb	r0, [r4, #3]
		ldrb	r1, [r4, #8]
		mov		r0, #8
		ldrsb	r0, [r5, r0]
		add		r3, r1, r0
		bpl		b25D0
		mov		r3, #0
b25D0:
		ldr		r6, [sp, #12]
		cmp		r6, #0
		beq		b25F6
		mov		r6, r9
		ldrb	r0, [r6, #2]
		strb	r0, [r4, #30]
		ldrb	r1, [r6, #3]
		mov		r0, #0x80
		tst		r0, r1
		bne		b25E6
		strb	r1, [r4, #31]
b25E6:
		ldrb	r2, [r5, #9]
		mov		r1, r3
		ldr		r0, [sp, #12]
		ldr		r3, [sp, #4]
		ldr		r3, [r3, #48]
		bl		b2102
		b		b2600
b25F6:
		ldrb	r2, [r5, #9]
		mov		r1, r3
		mov		r0, r7
		bl		MidiKey2Freq
b2600:
		str		r0, [r4, #32]
		mov		r0, #0x80
		strb	r0, [r4]
		ldrb	r1, [r5]
		mov		r0, #240
		and		r0, r1
		strb	r0, [r5]
b260E:
		add		sp, #20
		pop		{r0-r7}
		mov		r8, r0
		mov		r9, r1
		mov		r10, r2
		mov		r11, r3
		pop		{r0}
		bx		r0

@ p2620
.pool

b2628:
		push	{r4, lr}
		ldr		r2, [r1, #64]
		ldrb	r3, [r2]
		cmp		r3, #0x80
		bcs		b263A
		strb	r3, [r1, #5]
		.hword	0x1C52
		@add		r2, r2, #1
		str		r2, [r1, #64]
		b		b263C
b263A:
		ldrb	r3, [r1, #5]
b263C:
		ldr		r1, [r1, #32]
		cmp		r1, #0
		beq		b265E
		mov		r4, #131
b2644:
		ldrb	r2, [r1]
		tst		r2, r4
		beq		b2658
		ldrb	r0, [r1, #17]
		cmp		r0, r3
		bne		b2658
		mov		r0, #64
		orr		r2, r0
		strb	r2, [r1]
		b		b265E
b2658:
		ldr		r1, [r1, #52]
		cmp		r1, #0
		bne		b2644
b265E:
		pop		{r4}
		pop		{r0}
		bx		r0

b2664:
		push	{r4, r5, lr}
		mov		r5, r1
		ldr		r4, [r5, #32]
		cmp		r4, #0
		beq		b2688
b266E:
		ldrb	r1, [r4]
		mov		r0, #199
		tst		r0, r1
		beq		b267C
		mov		r0, #64
		orr		r1, r0
		strb	r1, [r4]
b267C:
		mov		r0, r4
		bl		b23C6
		ldr		r4, [r4, #52]
		cmp		r4, #0
		bne		b266E
b2688:
		mov		r0, #0
		strb	r0, [r5]
		pop		{r4, r5}
		pop		{r0}
		bx		r0

SoundGetJumpList:
b2692:
		mov		r12, lr
		mov		r1, #36
		ldr		r2, =0x3738			@ p26C0
b2698:
		ldr		r3, [r2]
		bl		b26AA
		stmia	r0!, {r3}
		.hword	0x1D12
		@add		r2, r2, #4
		.hword	0x1E49
		@sub		r1, r1, #1
		bgt		b2698
		bx		r12

.globl b26A8
.thumb_func
b26A8:
		ldrb	r3, [r2]
.globl b26AA
.thumb_func
b26AA:
b26AA_:
		push	{r0}
		lsr		r0, r2, #25
		bne		b26BC
		ldr		r0, =0x3738			@ p26C0
		cmp		r2, r0
		bcc		b26BA
		lsr		r0, r2, #14
		beq		b26BC
b26BA:
		mov		r3, #0
b26BC:
		pop		{r0}
		bx		lr

@ p26C0
.pool

.globl b26C4
.thumb_func
b26C4:
		ldr		r2, [r1, #64]
.globl b26C6
.thumb_func
b26C6:
		add		r3, r2, #1
		str		r3, [r1, #64]
		ldrb	r3, [r2]
		b		b26AA_

b26CE:
		push	{lr}
b26D0:
		ldr		r2, [r1, #64]
		ldrb	r0, [r2, #3]
		lsl		r0, r0, #8
		ldrb	r3, [r2, #2]
		orr		r0, r3
		lsl		r0, r0, #8
		ldrb	r3, [r2, #1]
		orr		r0, r3
		lsl		r0, r0, #8
		bl		b26A8
		orr		r0, r3
		str		r0, [r1, #64]
		pop		{r0}
		bx		r0

b26EE:
		ldrb	r2, [r1, #2]
		cmp		r2, #3
		bcs		b2706
		lsl		r2, r2, #2
		add		r3, r1, r2
		ldr		r2, [r1, #64]
		.hword	0x1D12
		@add		r2, r2, #4
		str		r2, [r3, #68]
		ldrb	r2, [r1, #2]
		.hword	0x1C52
		@add		r2, r2, #1
		strb	r2, [r1, #2]
		b		b26CE
b2706:
		b		b2664
		ldrb	r2, [r1, #2]
		cmp		r2, #0
		beq		b271A
		.hword	0x1E52
		@sub		r2, r2, #1
		strb	r2, [r1, #2]
		lsl		r2, r2, #2
		add		r3, r1, r2
		ldr		r2, [r3, #68]
		str		r2, [r1, #64]
b271A:
		bx		lr

b271C:
		push	{lr}
		ldr		r2, [r1, #64]
		ldrb	r3, [r2]
		cmp		r3, #0
		bne		b272C
		.hword	0x1C52
		@add		r2, r2, #1
		str		r2, [r1, #64]
		b		b26D0
b272C:
		ldrb	r3, [r1, #3]
		.hword	0x1C5B
		@add		r3, r3, #1
		strb	r3, [r1, #3]
		mov		r12, r3
		bl		b26C4
		cmp		r12, r3
		bcs		b273E
		b		b26D0
b273E:
		mov		r3, #0
		strb	r3, [r1, #3]
		.hword	0x1D52
		@add		r2, r2, #5
		str		r2, [r1, #64]
		pop		{r0}
		bx		r0

b274A:
		mov		r12, lr
		bl		b26C4
		strb	r3, [r1, #29]
		bx		r12

b2754:
		mov		r12, lr
		bl		b26C4
		lsl		r3, r3, #1
		strh	r3, [r0, #28]
		ldrh	r2, [r0, #30]
		mul		r3, r2
		lsr		r3, r3, #8
		strh	r3, [r0, #32]
		bx		r12

b2768:
		mov		r12, lr
		bl		b26C4
		strb	r3, [r1, #10]
		ldrb	r3, [r1]
		mov		r2, #12
		orr		r3, r2
		strb	r3, [r1]
		bx		r12

		mov		r12, lr
		ldr		r2, [r1, #64]
		ldrb	r3, [r2]
		.hword	0x1C52
		@add		r2, r2, #1
		str		r2, [r1, #64]
		lsl		r2, r3, #1
		add		r2, r2, r3
		lsl		r2, r2, #2
		ldr		r3, [r0, #48]
		add		r2, r2, r3
		ldr		r3, [r2]
		bl		b26AA
		str		r3, [r1, #36]
		ldr		r3, [r2, #4]
		bl		b26AA
		str		r3, [r1, #40]
		ldr		r3, [r2, #8]
		bl		b26AA
		str		r3, [r1, #44]
		bx		r12

		mov		r12, lr
		bl		b26C4
		strb	r3, [r1, #18]
		ldrb	r3, [r1]
		mov		r2, #3
		orr		r3, r2
		strb	r3, [r1]
		bx		r12

		mov		r12, lr
		bl		b26C4
		sub		r3, #64
		strb	r3, [r1, #20]
		ldrb	r3, [r1]
		mov		r2, #3
		orr		r3, r2
		strb	r3, [r1]
		bx		r12

		mov		r12, lr
		bl		b26C4
		sub		r3, #64
		strb	r3, [r1, #14]
		ldrb	r3, [r1]
		mov		r2, #12
		orr		r3, r2
		strb	r3, [r1]
		bx		r12

		mov		r12, lr
		bl		b26C4
		strb	r3, [r1, #15]
		ldrb	r3, [r1]
		mov		r2, #12
		orr		r3, r2
		strb	r3, [r1]
		bx		r12

		mov		r12, lr
		bl		b26C4
		strb	r3, [r1, #25]
		cmp		r3, #0
		bne		b2802
		strb	r3, [r1, #22]
b2802:
		bx		r12

		mov		r12, lr
		bl		b26C4
		strb	r3, [r1, #27]
		bx		r12

		mov		r12, lr
		bl		b26C4
		strb	r3, [r1, #23]
		cmp		r3, #0
		bne		b281C
		strb	r3, [r1, #22]
b281C:
		bx		r12

		mov		r12, lr
		bl		b26C4
		ldrb	r0, [r1, #24]
		cmp		r0, r3
		beq		b2834
		strb	r3, [r1, #24]
		ldrb	r3, [r1]
		mov		r2, #15
		orr		r3, r2
		strb	r3, [r1]
b2834:
		bx		r12

		mov		r12, lr
		bl		b26C4
		sub		r3, #64
		strb	r3, [r1, #12]
		ldrb	r3, [r1]
		mov		r2, #12
		orr		r3, r2
		strb	r3, [r1]
		bx		r12

		mov		r12, lr
		ldr		r2, [r1, #64]
		ldrb	r3, [r2]
		.hword	0x1C52
		@add		r2, r2, #1
		ldr		r0, =0x04000060		@ 0x2860
		add		r0, r0, r3
		bl		b26C6
		strb	r3, [r0]
		bx		r12
.pool

.globl b2864
.thumb_func
b2864:
		mov		r6, #32
b2866:
		mov		r1, r5
		eor		r1, r2
		lsr		r5, r5, #1
		lsr		r1, r1, #1
		bcc		b2872
		eor		r5, r0
b2872:
		lsr		r2, r2, #1
		.hword	0x1E76
		@sub		r6, r6, #1
		bne		b2866
		bx		lr

.globl b287A
.thumb_func
b287A:
		push	{r2, r4, r6, lr}
		mov		r12, r1
		str		r3, [r7, #84]
		ldr		r3, [r7, #68]
		ldr		r1, [r7, #56]
		sub		r1, r1, r3
		asr		r1, r1, #2
		ble		b28BA
		cmp		r1, #137
		ble		b2890
		mov		r1, #137
b2890:
		ldr		r4, [r7, #4]
		ldrh	r5, [r7, #32]
b2894:
		str		r1, [r7, #80]
		mov		r1, r12
		mul		r4, r1
		.hword	0x1C64
		@add		r4, r4, #1
		ldr		r2, [r3]
		eor		r2, r4
		neg		r1, r3
		eor		r2, r1
		ldr		r1, [r7, #84]
		eor		r2, r1
		stmia	r3!, {r2}
		strh	r5, [r7, #34]
		bl		b2864
		ldr		r1, [r7, #80]
		.hword	0x1E49
		@sub		r1, r1, #1
		bne		b2894
		strh	r5, [r7, #32]
		str		r4, [r7, #4]
b28BA:
		pop		{r2, r4, r6, pc}

.global b28BC
.thumb_func
b28BC:
		push	{lr}
		bl		b2AA6
		pop		{r1}
		mov		lr, r1
.globl b28C6
.thumb_func
b28C6:
b28C6_:
		ldrh	r1, [r6, #8]
		lsr		r1, r1, #8
		bcs		b28C6_
		bx		lr

MultiBoot:
b28CE:
		push	{r1, r3-r7, lr}
		mov		r3, #223
		adr		r2, b2C00
		bl		b2AA4
		mov		r7, r0
		mov		r4, #255
		bl		bB9C
		beq		b296E
		lsr		r4, r7, #20
		mov		r3, #232
		and		r3, r4
		cmp		r3, #32
		bne		b296E
		mov		r4, #0
		cmp		r1, #1
		beq		b28FA
		cmp		r1, #2
		bgt		b296E
		ldr		r4, b2C08+0x10		@ 0x2C18
		orr		r4, r1
b28FA:
		strh	r4, [r7, #58]
		ldr		r0, [r7, #32]
		str		r0, [r7, #16]
		ldr		r4, [r7, #36]
		sub		r4, r4, r0
		ldr		r3, b2C08			@ 0x2C08
		and		r4, r3
		str		r4, [r7, #12]
		bl		bB9C
		beq		b296E
		ldr		r4, b2C08+0x4		@ 0x2C0C
		ldrh	r0, [r4, #10]
		ldrh	r2, [r4, #22]
		orr		r0, r2
		ldrh	r2, [r4, #34]
		orr		r0, r2
		ldrh	r2, [r4, #46]
		orr		r0, r2
		lsr		r0, r0, #16
		bcs		b296E
		ldr		r6, b2C08+0x8		@ 0x2C10
		ldrb	r0, [r7, #30]
		lsl		r0, r0, #28
		lsr		r0, r0, #29
		ldrh	r1, [r6]
		ldrh	r2, [r7, #58]
		cmp		r2, #0
		beq		b293A
		lsl		r0, r0, #31
		lsr		r0, r0, #31
		ldrb	r1, [r7, #20]
b293A:
		strb	r0, [r7, #8]
		strb	r1, [r7, #4]
		ldr		r3, b2C08+0x10		@ 0x2C18
		lsr		r3, r3, #16
		ldr		r1, b2D2C			@ 0x2D2C
		cmp		r2, #0
		ldr		r4, b2D2C+0x8		@ 0x2D34
		bne		b2950
		ldr		r1, b2D2C+0x4		@ 0x2D30
		ldr		r3, b2C08			@ 0x2C08
		ldr		r4, b2D2C+0xC		@ 0x2D38
b2950:
		strh	r1, [r7, #62]
		strh	r3, [r7, #56]
		str		r4, [r7, #64]
		ldr		r1, [r7, #24]
		str		r1, [r7]
		ldrb	r1, [r7, #28]
		strb	r1, [r7]
		mov		r4, r6
b2960:
		lsr		r0, r0, #1
		bcc		b2970
		ldrb	r1, [r4, #3]
		cmp		r1, #115
		bne		b296E
b296A:
		.hword	0x1CA4
		@add		r4, r4, #2
		b		b2960
b296E:
		b		b2A8E
b2970:
		bne		b296A
		ldr		r5, [r7, #12]
		lsr		r0, r5, #2
		sub		r0, #52
		ldr		r1, [r7, #16]
		add		r1, r1, r5
		str		r1, [r7, #12]
		ldr		r1, b2C08			@ 0x2C08
b2980:
		.hword	0x1E49
		@sub		r1, r1, #1
		bne		b2980
		bl		b28BC
		ldrh	r1, [r6, #2]
		strb	r1, [r7, #5]
		ldrh	r1, [r6, #4]
		ldrh	r2, [r6, #6]
		ldrh	r3, [r7, #58]
		cmp		r3, #0
		beq		b299A
		mov		r1, #255
		mov		r2, #255
b299A:
		strb	r1, [r7, #6]
		strb	r2, [r7, #7]
		mov		r4, #2
		mov		r12, r4
		ldr		r3, [r7, #16]
b29A4:
		ldr		r1, [r7, #32]
		sub		r1, r1, r3
		lsr		r1, r1, #2
		ldrh	r0, [r7, #60]
		bcs		b29DE
		ldr		r2, [r3]
		ldrh	r0, [r7, #62]
		ldrh	r5, [r7, #56]
		bl		b2864
		strh	r5, [r7, #56]
		ldr		r1, [r7]
		ldr		r0, b2D2C+0x18		@ 0x2D44
		mul		r1, r0
		.hword	0x1C49
		@add		r1, r1, #1
		str		r1, [r7]
		ldr		r0, [r3]
		eor		r0, r1
		ldr		r1, [r7, #32]
		sub		r2, r3, r1
		ldr		r1, b2D94			@ 0x2D94
		add		r2, r2, r1
		neg		r1, r2
		ldr		r2, [r7, #64]
		eor		r1, r2
		eor		r0, r1
		lsr		r2, r0, #16
		strh	r2, [r7, #60]
		ldr		r6, b2C08+0x8		@ 0x2C10
b29DE:
		bl		b28C6
		ldr		r1, [r7, #32]
		cmp		r1, r3
		beq		b2A1A
		mov		lr, r4
		sub		r4, r3, r1
		.hword	0x1EA4
		@sub		r4, r4, #2
		ldrh	r1, [r7, #58]
		cmp		r1, #0
		beq		b29F6
		.hword	0x1EA4
		@sub		r4, r4, #2
b29F6:
		ldr		r1, b2D94			@ 0x2D94
		add		r4, r4, r1
b29FA:
		ldrb	r2, [r7, #8]
		mov		r5, r6
b29FE:
		lsr		r2, r2, #1
		bcc		b2A0E
		ldrh	r1, [r5, #2]
		eor		r1, r4
		lsl		r1, r1, #16
		bne		b2A8E
b2A0A:
		.hword	0x1CAD
		@add		r5, r5, #2
		b		b29FE
b2A0E:
		bne		b2A0A
		mov		r4, lr
		cmp		r2, r12
		bne		b2A1A
		mov		r0, #0
		b		b2A90
b2A1A:
		bl		b2AA6
		cmp		r4, #0
		beq		b2A3C
		.hword	0x1C9B
		@add		r3, r3, #2
		ldrh	r1, [r7, #58]
		cmp		r1, #0
		beq		b2A2C
		.hword	0x1C9B
		@add		r3, r3, #2
b2A2C:
		cmp		r4, #2
		bne		b2A36
		ldr		r1, [r7, #12]
		cmp		r1, r3
		bne		b29A4
b2A36:
		mov		r0, #101
		.hword	0x1E64
		@sub		r4, r4, #1
		b		b29DE
b2A3C:
		mov		r4, #1
		bl		b28C6
		ldrb	r2, [r7, #8]
		mov		r3, r6
b2A46:
		lsr		r2, r2, #1
		bcc		b2A5E
		ldrh	r1, [r3, #2]
		cmp		r1, #117
		beq		b2A5A
		cmp		r0, #101
		bne		b2A8E
		cmp		r1, #116
		bne		b2A8E
		mov		r4, #0
b2A5A:
		.hword	0x1C9B
		@add		r3, r3, #2
		b		b2A46
b2A5E:
		bne		b2A5A
		cmp		r0, #102
		beq		b2A70
		cmp		r4, #0
		beq		b2A6A
		mov		r0, #102
b2A6A:
		bl		b2AA6
		b		b2A3C
b2A70:
		cmp		r4, #0
		beq		b2A8E
		ldrh	r0, [r7, #62]
		ldrh	r5, [r7, #56]
		ldr		r2, [r7, #4]
		bl		b2864
		ldr		r6, b2C08+0x8		@ 0x2C10
		mov		r0, r5
		bl		b28BC
		mov		r1, #0
		mov		r12, r1
		mov		r4, r0
		b		b29FA
b2A8E:
		mov		r0, #1
b2A90:
		str		r0, [r7, #56]
		str		r0, [r7, #60]
		str		r0, [r7, #64]
		mov		r1, r7
		add		r1, #20
b2A9A:
		stmia	r7!, {r0}
		cmp		r1, r7
		bne		b2A9A
		pop		{r1, r3-r7}
		pop		{r2}
b2AA4:
		bx		r2

b2AA6:
		mov		r1, #150
b2AA8:
		.hword	0x1E49
		@sub		r1, r1, #1
		bne		b2AA8
		str		r0, [r6]
		strh	r0, [r6, #10]
		ldrh	r1, [r7, #58]
		cmp		r1, #0
		bne		b2AB8
		ldr		r1, b2D94+0x4		@ 0x2D98
b2AB8:
		strh	r1, [r6, #8]
		bx		lr

.globl b2ABC
.thumb_func
b2ABC:
		push	{lr}
		ldr		r0, b2D94+0x8		@ 0x2D9C
		ldrb	r1, [r0]
		cmp		r1, #1
		bne		b2AE4
		ldrb	r0, [r7, #10]
		lsl		r0, r0, #25
		bcc		b2AE4
		ldrb	r0, [r7, #18]
		ldrb	r1, [r7, #19]
		orr		r0, r1
		bne		b2AE8
		ldr		r0, [r7, #56]
		ldr		r1, b2D94			@ 0x2D94
		sub		r1, r1, r0
		bge		b2AE4
		mov		r0, #120
		strb	r0, [r7, #18]
		ldr		r0, b2D2C+0x1C		@ 0x2D48
		.hword	0xE2A9
		@b		0x3038
b2AE4:
		mov		r0, #0
		pop		{pc}

b2AE8:
		ldr		r2, [r7, #8]
		lsl		r1, r2, #13
		lsr		r1, r1, #30
		ldrb	r0, [r7, #20]
b2AF0:
		.hword	0x1CC0
		@add		r0, r0, #3
		.hword	0x1E49
		@sub		r1, r1, #1
		bpl		b2AF0
		strb	r0, [r7, #20]
		lsr		r0, r0, #2
		lsl		r1, r0, #26
		lsl		r2, r2, #12
		eor		r1, r2
		asr		r1, r1, #31
		eor		r0, r1
		mov		r1, #31
		and		r1, r0
		ldr		r2, [r7, #8]
		lsl		r0, r2, #9
		lsr		r0, r0, #29
		cmp		r0, #7
		blt		b2B1E
		mov		r1, #0
		lsl		r0, r2, #12
		lsr		r0, r0, #29
		cmp		r0, #7
		blt		b2B1E
		mov		r0, #0
b2B1E:
		mov		r2, #31
		bl		b7BC
		ldrb	r0, [r7, #18]
		.hword	0x1E40
		@sub		r0, r0, #1
		blt		b2B2E
		strb	r0, [r7, #18]
		bne		b2AE4
b2B2E:
		mov		r0, #5
		strb	r0, [r7, #19]
		pop		{pc}

.globl b2B34
.thumb_func
b2B34:

		push	{r4-r7}
		push	{lr}
		ldr		r7, b2D94+0xC		@ 0x2DA0
		ldr		r4, b2C08+0x8		@ 0x2C10
		ldr		r0, [r7, #76]
		ldr		r1, b2D2C+0x14		@ 0x2D40
		mul		r0, r1
		.hword	0x1C40
		@add		r0, r0, #1
		str		r0, [r7, #76]
		b		b304A
b2B48:
		ldr		r0, [r7, #76]
		mov		r1, #224
		bic		r0, r1
		mov		r1, #160
		eor		r0, r1
		mov		r3, #128
		lsl		r3, r3, #8
		bic		r0, r3
		ldr		r1, b2D94+0x8		@ 0x2D9C
		ldrb	r2, [r1]
		cmp		r2, #1
		beq		b2B68
		ldr		r1, b2C08+0x14		@ 0x2C1C
		ldr		r2, [r1, #36]
		.hword	0x1C52
		@add		r2, r2, #1
		bne		b2B6A
b2B68:
		orr		r0, r3
b2B6A:
		str		r0, [r7]
		ldrb	r5, [r7, #15]
		ldrb	r6, [r7, #14]
		ldrb	r0, [r7, #13]
		cmp		r0, #0
		bne		b2BAE
		bl		b2D5C
		ldrb	r3, [r7, #12]
		ldrh	r0, [r4, #16]
		cmp		r6, #2
		bne		b2B8C
		.hword	0x1E5B
		@sub		r3, r3, #1
		bpl		b2BA8
b2B86:
		mov		r6, #0
		mov		r3, #6
		b		b2BA2
b2B8C:
		cmp		r6, #1
		bne		b2B9A
		.hword	0x1E5B
		@sub		r3, r3, #1
		bpl		b2BA8
b2B94:
		mov		r6, #2
		mov		r3, #6
		b		b2BA2
b2B9A:
		.hword	0x1E5B
		@sub		r3, r3, #1
		bpl		b2BA8
b2B9E:
		mov		r6, #1
		mov		r3, #30
b2BA2:
		ldr		r1, b2C08+0xC		@ 0x2C14
		str		r1, [r7, #52]
		mov		r5, #0
b2BA8:
		strb	r3, [r7, #12]
		bl		b2D64
b2BAE:
		cmp		r5, #0
		bne		b2C20
		str		r5, [r7, #16]
		strb	r5, [r7, #10]
		ldr		r2, b2D2C+0x1C		@ 0x2D48
		str		r2, [r7, #56]
		ldr		r2, b2D94			@ 0x2D94
		str		r2, [r7, #60]
		str		r2, [r7, #68]
		mov		r2, #1
		strb	r2, [r7, #15]
		strb	r6, [r7, #14]
		cmp		r6, #0
		bne		b2BE2
		mov		r2, #192
		lsl		r2, r2, #8
		strh	r2, [r4, #20]
		ldr		r1, [r4, #48]
		str		r5, [r4, #52]
		strh	r5, [r4, #56]
		mov		r1, #7
		strh	r1, [r4, #32]
		mov		r1, #173
		lsl		r1, r1, #5
b2BDE:
		strh	r1, [r7, #32]
		b		b2D56
b2BE2:
		cmp		r6, #1
		bne		b2BF4
		strh	r5, [r4, #20]
		ldr		r2, b2D2C+0x20		@ 0x2D4C
		ldr		r1, b2C08			@ 0x2C08
b2BEC:
		strh	r2, [r4, #8]
		strh	r5, [r4, #10]
		str		r5, [r4]
		b		b2BDE
b2BF4:
		strh	r5, [r4, #20]
		ldr		r2, [pc, #872]		@ 0x2F60
		lsr		r2, r2, #16
		ldr		r1, b2C08+0x10		@ 0x2C18
		lsr		r1, r1, #16
		b		b2BEC

.arm
b2C00:
		msr		CPSR_fc, r3
		bx		lr

b2C08:
.pool
		.word	0x0003FFF8
		.word	0x040000B0
		.word	0x04000120
		.word	0x0000301D
		.word	0xC3871089
		.word	0x03000064

.thumb
b2C20:
		cmp		r5, #1
		bne		b2C58
		bl		b2D5C
		mov		r1, #128
		strh	r1, [r3, #2]
		ldrh	r2, [r3]
		orr		r2, r1
		strh	r2, [r3]
		strh	r5, [r3, #8]
		cmp		r6, #0
		bne		b2C44
		mov		r1, #71
		strh	r1, [r4, #32]
		add		r2, pc, #804		@ adr		r2, 0x2F64
		b		b2C50
b2C40:		
		ldr		r1, [pc, #796]		@ 0x2F60
		b		b2C4C
b2C44:
		cmp		r6, #1
		bne		b2C40
		ldr		r1, b2D2C+0x20		@ 0x2D4C
		lsr		r1, r1, #16
b2C4C:
		strh	r1, [r4, #8]
		adr		r2, b2DA4
b2C50:
		mov		r1, #2
		strb	r1, [r7, #15]
		str		r2, [r7, #52]
		b		b2D56
b2C58:
		cmp		r5, #2
		bne		b2C5E
		b		b2D56
b2C5E:
		bl		b2ABC
		cmp		r5, #3
		beq		b2C6C
		cmp		r0, #0
		beq		b2D56
		b		b2D58
b2C6C:
		bl		b2D5C
		ldr		r0, [r7, #48]
		.hword	0x1E40
		@sub		r0, r0, #1
		bpl		b2C8A
		cmp		r6, #0
		bne		b2C84
		ldrh	r1, [r4, #56]
		mov		r2, #48
		and		r1, r2
		beq		b2C8C
		b		b2B86
b2C84:
		cmp		r6, #1
		bne		b2B94
		b		b2B9E
b2C8A:
		str		r0, [r7, #48]
b2C8C:
		mov		r0, #1
		strh	r0, [r3, #8]
		ldrb	r0, [r7, #17]
		cmp		r0, #0
		bne		b2D50
		cmp		r6, #0
		bne		b2CEE
		ldr		r0, b2D94+0x4		@ 0x2D98
		lsr		r0, r0, #16
		ldr		r1, [pc, #992]		@ 0x3080
		ldr		r3, b2D2C+0x10		@ 0x2D3C
		bl		b287A
		str		r3, [r7, #68]
		ldr		r1, [r7, #60]
		cmp		r3, r1
		bne		b2D56
		ldr		r0, [r7, #56]
		eor		r0, r3
		bne		b2D56
		.hword	0x1F09
		@sub		r1, r1, #4
		ldrh	r2, [r1]
		ldrh	r3, [r7, #34]
		cmp		r3, r2
		bne		b2CE8
		str		r0, [r1]
		ldr		r1, [r7, #8]
		ldr		r2, [pc, #984]		@ 0x309C
		and		r1, r2
		cmp		r1, r2
		bne		b2CE8
		ldrb	r1, [r7, #10]
		ldrb	r2, [r7, #8]
		add		r1, r1, r2
		ldrb	r2, [r7, #9]
		add		r1, r1, r2
		ldrb	r2, [r7, #11]
		sub		r1, r1, r2
		lsl		r1, r1, #25
		bne		b2CE8
		ldr		r0, [pc, #960]		@ 0x30A0
		mov		r1, #1
		blown	0x301E, 0x2CE0
		cmp		r0, #0
		beq		b2D50
b2CE8:
		mov		r0, #0
		strb	r0, [r7, #15]
		b		b2D56
b2CEE:
		ldr		r2, [r7, #60]
		ldr		r3, [r7, #68]
		cmp		r3, r2
		beq		b2D56
		ldr		r0, b2D2C+0x4		@ 0x2D30
		ldr		r3, b2D2C+0xC		@ 0x2D38
		cmp		r6, #2
		bne		b2D02
		ldr		r0, b2D2C			@ 0x2D2C
		ldr		r3, b2D2C+0x8		@ 0x2D34
b2D02:
		ldr		r1, b2D2C+0x18		@ 0x2D44
		bl		b287A
		cmp		r2, r3
		bne		b2D26
		ldr		r2, [r7, #28]
		bl		b2864
		strh	r5, [r7, #32]
		ldr		r0, [pc, #908]		@ 0x30A4
		mov		r1, #4
		ldrb	r6, [r7, #14]
		sub		r1, r1, r6
		blown	0x301E, 0x2D1C
		cmp		r0, #0
		bne		b2CE8
		ldr		r3, [r7, #60]
b2D26:
		str		r3, [r7, #68]
		b		b2D56
.align 2, 0
b2D2C:
.pool
		.word	0x0000C37B
		.word	0x0000A517
b2D34:
		.ascii	"// Coded by Kawasedo"
		/*.word	0x43202F2F
		.word	0x6465646F
		.word	0x20796220
		.word	0x6177614B
		.word	0x6F646573*/
		.word	0x02000000
		.word	0x60032003
b2D50:
		mov		r0, #4
		strb	r0, [r7, #17]
		strb	r0, [r7, #15]
b2D56:
		mov		r0, #0
b2D58:
		pop		{r3-r7}
		bx		r3

.globl b2D5C
.thumb_func
b2D5C:
		mov		r0, #0
b2D5E:
		ldr		r3, [pc, #808]		@ 0x3088
		strh	r0, [r3, #8]
		bx		lr

b2D64:
		mov		r0, #1
		b		b2D5E

.globl b2D68
.thumb_func
b2D68:
		ldr		r3, [pc, #812]		@ 0x3098
		mov		r1, #0
		strb	r1, [r3, #15]
		bx		lr

b2D70:
		ldr		r2, [pc, #772]		@ 0x3078
		ldrh	r1, [r2]
		ldr		r3, [pc, #800]		@ 0x3098
		ldrb	r0, [r3, #14]
		cmp		r0, #1
		bne		b2D86
		ldrh	r0, [r2, #8]
		lsr		r0, r0, #7
		bcs		b2E62
b2D82:
		ldr		r0, [r3, #52]
		mov		pc, r0
b2D86:
		cmp		r0, #2
		beq		b2D82
		ldrh	r1, [r2, #32]
		strh	r1, [r2, #32]
		mov		r0, #7
		and		r1, r0
		b		b2D82

b2D94:
.pool
		.word	0x020000C0
		.word	0xA1C12083
		.word	0x03007FFB
		.word	0x0300000C
/*		lsl		r0, r0, #3
		lsl		r0, r0, #8
		mov		r0, #131
		add		r1, pc, #772		@ adr	r1, 0x30A0
		ldrb	r3, [r7, #31]
		lsl		r0, r0, #12
		lsl		r4, r1, #0
		lsl		r0, r0, #12*/

b2DA4:
		lsr		r0, r1, #8
		cmp		r0, #98
		bne		b2E84
		ldrb	r0, [r3, #14]
		cmp		r0, #2
		bne		b2DB4
		mov		r0, #1
		b		b2DBA
b2DB4:
		ldrh	r0, [r2, #8]
		lsl		r0, r0, #26
		lsr		r0, r0, #30
b2DBA:
		strb	r0, [r3, #22]
		beq		b2E62
		mov		r1, #1
		lsl		r1, r0
		strb	r1, [r3, #21]
		ldrh	r1, [r2]
		adr		r0, b2DF0
		str		r0, [r3, #52]
b2DCA:
		strb	r1, [r3, #16]
		mov		r0, #11
		strb	r0, [r3, #12]
		mov		r0, #17
		and		r0, r1
		bne		b2E62
		lsr		r0, r1, #4
		orr		r0, r1
		lsr		r2, r1, #4
		eor		r2, r1
		eor		r2, r0
		lsl		r2, r2, #28
		bne		b2E62
		mov		r0, #114
		lsl		r0, r0, #8
		ldrb	r1, [r3, #21]
		orr		r1, r0
		b		b2F3E
.align 2, 0
b2DF0:
		lsr		r0, r1, #8
b2DF2:
		cmp		r0, #98
		beq		b2DCA
		cmp		r0, #97
		bne		b2E62
		mov		r0, #3
		strb	r0, [r3, #15]
		strb	r0, [r3, #13]
		ldr		r2, [pc, #656]		@ 0x3094
		str		r2, [r3, #56]
		mov		r2, #96
		adr		r0, b2E0C
		b		b2E1C
		lsl		r0, r0, #0
b2E0C:
		ldr		r2, [r3, #56]
		strh	r1, [r2]
		.hword	0x1C92
		@add		r2, r2, #2
		str		r2, [r3, #56]
		ldr		r2, [r3, #72]
		.hword	0x1E52
		@sub		r2, r2, #1
		bne		b2E1C
		adr		r0, b2E28
b2E1C:
		str		r2, [r3, #72]
		lsl		r2, r2, #8
		ldrb	r1, [r3, #21]
		orr		r1, r2
		b		b2F3C
		lsl		r0, r0, #0
b2E28:
		lsr		r0, r1, #8
		cmp		r0, #99
		bne		b2DF2
		mov		r0, #0xFF
		strb	r0, [r3, #26]
		strb	r0, [r3, #27]
b2E34:
		strb	r1, [r3, #10]
		strb	r1, [r3, #24]
		ldrb	r0, [r3, #14]
		cmp		r0, #2
		bne		b2E44
		ldrb	r0, [r3, #23]
		strb	r0, [r3, #25]
		b		b2E50
b2E44:
		ldrh	r0, [r2, #2]
		strb	r0, [r3, #25]
		ldrh	r0, [r2, #4]
		strb	r0, [r3, #26]
		ldrh	r0, [r2, #6]
		strb	r0, [r3, #27]
b2E50:
		ldr		r0, [r3, #24]
		str		r0, [r3, #4]
		ldrb	r2, [r3, #1]
		strb	r2, [r3, #23]
		adr		r0, b2E64
b2E5A:
		mov		r1, #115
		lsl		r1, r1, #8
		orr		r1, r2
		b		b2F3C
b2E62:
		b		b2F56
b2E64:
		lsr		r0, r1, #8
		cmp		r0, #99
		beq		b2E34
		cmp		r0, #100
		bne		b2F56
		strb	r1, [r3, #28]
		ldrb	r2, [r3, #2]
		ldrb	r0, [r3, #14]
		cmp		r0, #2
		bne		b2E80
		strb	r2, [r3, #29]
		mov		r0, #0xFF
		strb	r0, [r3, #30]
		strb	r0, [r3, #31]
b2E80:
		adr		r0, b2E88
		b		b2E5A
b2E84:
		b		b2F44
		lsl		r0, r0, #0
b2E88:
		ldrb	r0, [r3, #14]
		cmp		r0, #2
		beq		b2E9A
		ldrh	r0, [r2, #2]
		strb	r0, [r3, #29]
		ldrh	r0, [r2, #4]
		strb	r0, [r3, #30]
		ldrh	r0, [r2, #6]
		strb	r0, [r3, #31]
b2E9A:
		lsl		r1, r1, #2
		add		r1, #200
		ldr		r0, [pc, #476]		@ 0x307C
		and		r0, r1
		eor		r1, r0
		bne		b2F56
		ldr		r1, [pc, #484]		@ 0x308C
		add		r2, r1, r0
		add		r2, #8
		str		r2, [r3, #60]
		adr		r0, b2EB4
		b		b2F3C
		lsl		r0, r0, #0
b2EB4:
		ldrb	r0, [r3, #14]
		cmp		r0, #2
		ldr		r0, [r3, #56]
		strh	r1, [r0]
		bne		b2EC4
		ldr		r1, [r2]
		str		r1, [r0]
		.hword	0x1C80
		@add		r0, r0, #2
b2EC4:
		add		r1, r0, #2
		str		r1, [r3, #56]
		ldr		r0, [r3, #60]
		cmp		r0, r1
		bne		b2F3E
		adr		r0, b2ED4
		b		b2F3C
		lsl		r0, r0, #0
b2ED4:
		cmp		r1, #101
		bne		b2F56
		ldr		r1, [r3, #68]
		ldr		r2, [r3, #60]
		cmp		r1, r2
		beq		b2EE4
		mov		r1, #116
		b		b2F3E
b2EE4:
		mov		r1, #117
		adr		r0, b2EEC
		b		b2F3C
		lsl		r0, r0, #0
b2EEC:
		cmp		r1, #101
		beq		b2EE4
		cmp		r1, #102
		bne		b2F56
		ldrh	r1, [r3, #32]
		adr		r0, b2EFC
		b		b2F3C
		lsl		r0, r0, #0
b2EFC:
		ldrh	r0, [r3, #32]
		cmp		r0, r1
		bne		b2F56
		ldrb	r1, [r3, #14]
		cmp		r1, #1
		bne		b2f22
		ldrb	r3, [r3, #16]
		lsl		r3, r3, #28
		lsr		r3, r3, #29
b2F0E:
		lsr		r3, r3, #1
		bcc		b2F1C
		ldrh	r1, [r2, #2]
		cmp		r0, r1
		bne		b2F1E
b2F18:
		.hword	0x1C92
		@add		r2, r2, #2
		b		b2F0E
b2F1C:
		bne		b2F18
b2F1E:
		ldr		r3, [pc, #376]		@0x3098
		bne		b2F56
b2f22:
		ldr		r0, [r3, #28]
		ldrb	r1, [r3, #25]
		sub		r0, r0, r1
		ldrb	r1, [r3, #26]
		sub		r0, r0, r1
		ldrb	r1, [r3, #27]
		sub		r0, r0, r1
		sub		r0, #17
		lsl		r0, r0, #24
		bne		b2F56
		mov		r1, #255
		strb	r1, [r3, #17]
		add		r0, pc, #224		@ adr r0, 0x301C
b2F3C:
		str		r0, [r3, #52]
b2F3E:
		ldr		r2, [pc, #312]		@ 0x3078
		strh	r1, [r2, #10]
		strh	r1, [r2, #2]
b2F44:
		ldrb	r0, [r3, #14]
		cmp		r0, #2
		bne		b2F50
		ldr		r2, [pc, #300]		@ 0x3078
		ldr		r0, [pc, #16]		@ 0x2F60
		strh	r0, [r2, #8]
b2F50:
		mov		r0, #11
		str		r0, [r3, #48]
		bx		lr

b2F56:
		mov		r1, #0
		strb	r1, [r3, #15]
		add		r0, pc, #192		@ adr r1, 0x301C
		b		b2F3C

@ maybe only pool
b2F60:
.align 2, 0
		str		r0, [r1, r2]
		.hword	0x1008
		@asr		r0, r1, #0
		cmp		r1, #1
		bne		b2F56
		ldr		r0, [r3]
		str		r0, [r3, #4]
		ldr		r1, [pc, #276]		@ 0x3084
		eor		r0, r1
		str		r0, [r2, #52]
		mov		r0, #11
		strb	r0, [r3, #12]
		mov		r1, #16
		add		r0, pc, #0			@ adr r0, 0x2F7C
		b		b3014
		cmp		r1, #4
		bne		b2F56
		mov		r0, #3
		strb	r0, [r3, #15]
		strb	r0, [r3, #13]
		adr		r0, b2F8C
		b		b3018

.align 2, 0
b2F8C:
		cmp		r1, #2
		bne		b2F56
		ldr		r0, [r2, #48]
		mov		r1, #2
		lsl		r1, r1, #8
		and		r1, r0
		lsr		r1, r1, #7
		adr		r2, b3080
		add		r2, r2, r1
		ldr		r1, [r2]
		eor		r0, r1
		str		r0, [r3, #8]
		lsr		r1, r0, #8
		mov		r2, #127
		and		r1, r2
		lsl		r0, r0, #16
		bcc		b2FB0
		add		r1, #128
b2FB0:
		lsr		r0, r0, #16
		and		r0, r2
		lsl		r1, r1, #7
		orr		r1, r0
		add		r1, #63
		lsl		r1, r1, #3
		ldr		r0, b3078+0x4		@ 0x307C
		and		r0, r1
		cmp		r0, r1
		beq		b2FD0
		ldrb	r0, [r3, #10]
		lsl		r0, r0, #25
		lsr		r0, r0, #25
		strb	r0, [r3, #10]
		mov		r0, #137
		lsl		r0, r0, #7
b2FD0:
		add		r0, #12
		ldr		r1, b3088+0xC		@ 0x3094
		add		r1, r1, r0
		str		r1, [r3, #64]
		mov		r1, #32
		adr	r0, b2FE0
		b		b3014
.align 2, 0
b2FE0:
		cmp		r1, #2
		bne		b2F56
		ldrh	r0, [r2, #56]
		mov		r1, #16
		eor		r1, r0
		ldr		r0, [r2, #48]
		strh	r1, [r2, #56]
		ldr		r1, [r3, #56]
		stmia	r1!, {r0}
		str		r1, [r3, #56]
		ldr		r0, [r3, #60]
		cmp		r0, r1
		bne		b2F44
		ldr		r1, b3088+0x4		@ 0x308C
		cmp		r0, r1
		bne		b3006
		ldr		r0, [r3, #64]
		str		r0, [r3, #60]
		b		b2F44
b3006:
		ldr		r0, b3088+0x8		@ 0x3090
		ldr		r1, [r0, #4]
		ldr		r0, [r0]
		mul		r0, r1
		str		r0, [r2, #52]
		mov		r1, #0
		adr		r0, b301C
b3014:
		ldr		r2, =b3078+0x0		@ 0x3078
		strh	r1, [r2, #56]
b3018:
		str		r0, [r3, #52]
		b		b2F44
b301C:
		bx		lr
		push	{lr}
		ldr		r2, b3088+0xC		@ 0x3094
		str		r0, [r2]
		ldr		r3, b3088+0x4		@ 0x308C
		strb	r1, [r3, #4]
		cmp		r1, #1
		beq		b3030
		ldrb	r0, [r7, #22]
		strb	r0, [r3, #5]
b3030:
		add		r0, r2, #4
		bl		b6E8
		pop		{pc}
		.hword	0x1D00
		@add		r0, r0, #4
		bl		b94A
		ldrh	r0, [r7, #36]
		cmp		r0, #0
		bne		b3048
		mov		r0, #60
		strh	r0, [r7, #36]
b3048:
		b		b2AE8
b304A:
		ldrb	r0, [r7, #18]
		cmp		r0, #119
		bne		b3056
		bl		b974
		b		b305E
b3056:
		cmp		r0, #118
		bne		b305E
		bl		b982
b305E:
		ldrh	r0, [r7, #36]
		cmp		r0, #0
		beq		b3074
		.hword	0x1E40
		@sub		r0, r0, #1
		strh	r0, [r7, #36]
		cmp		r0, #57
		bne		b3074
		ldr		r0, b3088+0x20		@ 0x30A8
		ldr		r1, b3088+0x24		@ 0x30AC
		bl		SoundWhatever1
b3074:
		b		b2B48

.align 2, 0
b3078:
		.word	0x04000120, 0x0003FFF8
b3080:
		.ascii	"Kawasedo"
b3088:
		.word	0x04000200, 0x020000C0, 0x020001F8, 0x02000000
		.word	0x0300000C, 0x80808080, 0xEA000036, 0xEA00002E
		.word	0x0300390C, 0x00003980, 0x00280022, 0x00880082
		.word	0x00E800E2, 0x01480142, 0x08020200, 0x00000000
		.word	0x080101C0, 0x0000001E, 0x03020100, 0x07060504
		.word	0x0B0A0908, 0x0F0E0D0C, 0x13121110, 0x17161514
		.word	0x201E1C18, 0x2C2A2824, 0x38363430, 0x4442403C
		.word	0x504E4C48, 0x5C5A5854
b3100:
/*
Data Adresses:


b3290:
    @ Nintendo Logo Character Data
		.byte 0x24,0xFF,0xAE,0x51,0x69,0x9A,0xA2,0x21,0x3D,0x84,0x82,0x0A
		.byte 0x84,0xE4,0x09,0xAD,0x11,0x24,0x8B,0x98,0xC0,0x81,0x7F,0x21
		.byte 0xA3,0x52,0xBE,0x19,0x93,0x09,0xCE,0x20,0x10,0x46,0x4A,0x4A
		.byte 0xF8,0x27,0x31,0xEC,0x58,0xC7,0xE8,0x33,0x82,0xE3,0xCE,0xBF
		.byte 0x85,0xF4,0xDF,0x94,0xCE,0x4B,0x09,0xC1,0x94,0x56,0x8A,0xC0
		.byte 0x13,0x72,0xA7,0xFC,0x9F,0x84,0x4D,0x73,0xA3,0xCA,0x9A,0x61
		.byte 0x58,0x97,0xA3,0x27,0xFC,0x03,0x98,0x76,0x23,0x1D,0xC7,0x61
		.byte 0x03,0x04,0xAE,0x56,0xBF,0x38,0x84,0x00,0x40,0xA7,0x0E,0xFD
		.byte 0xFF,0x52,0xFE,0x03,0x6F,0x95,0x30,0xF1,0x97,0xFB,0xC0,0x85
		.byte 0x60,0xD6,0x80,0x25,0xA9,0x63,0xBE,0x03,0x01,0x4E,0x38,0xE2
		.byte 0xF9,0xA2,0x34,0xFF,0xBB,0x3E,0x03,0x44,0x78,0x00,0x90,0xCB
		.byte 0x88,0x11,0x3A,0x94,0x65,0xC0,0x7C,0x63,0x87,0xF0,0x3C,0xAF
		.byte 0xD6,0x25,0xE4,0x8B,0x38,0x0A,0xAC,0x72,0x21,0xD4,0xF8,0x07
b332C:

b3F00:
		.word 0x00000000, 0x00000000, 0x00000000
b3F0C:
		.word 0x00000001
*/
.end
