@ Memory
.equ WRAM,				0x02000000		@ - 0x203FFFF
.equ tempData,	0x02000050+276+276*2

.equ IRAM,				0x03000000		@ - 0x3007FFF
.equ BIOSInterruptFlags,0x03007FF8
.equ BGPaletteMem,		0x05000000		@ +(0x20*Nr) Background Palette
.equ OBJPaletteMem,		0x05000200		@ +(0x20*Nr) Sprite Palette
.equ VideoBuffer,		0x06000000		@ Display Memory (the screen)
.equ OAMData,			0x06010000


.equ OAMmem,			0x07000000		@ Object Attribute Memory
@ Atribute0 stuff
.equ ROTATION_FLAG,			0x100
.equ SIZE_DOUBLE,			0x200
.equ MODE_NORMAL,     		0x0
.equ MODE_TRANSPERANT,		0x400
.equ MODE_WINDOWED,			0x800
.equ MOSAIC,				0x1000
.equ COLOR_16,				0x0000
.equ COLOR_256,				0x2000
.equ SQUARE,				0x0
.equ TALL,					0x4000
.equ WIDE,					0x8000

@ Atribute1 stuff
@.equ ROTDATA(n),			((n) << 9)
.equ HORIZONTAL_FLIP,		0x1000
.equ VERTICAL_FLIP,			0x2000
.equ SIZE_8,				0x0
.equ SIZE_16,				0x4000
.equ SIZE_32,				0x8000
.equ SIZE_64,				0xC000

@ Atribute2 stuff
@.equ PRIORITY(n),			((n) << 10)
@.equ PALETTE(n),			((n) << 12)


@ Registers
.equ REG_INTERUPT,	0x03007FFC		@ Interrupt Register
.equ REG_DISPCNT,	0x04000000		@ Display Control (Mode)
.equ REG_DISPCNT_L,	0x04000000		@ Display Control (Mode) Lo Word
.equ REG_DISPCNT_H,	0x04000002		@ Display Control (Mode) Hi Word
.equ REG_DISPSTAT,	0x04000004		@ Display Status
.equ REG_VCOUNT,	0x04000006		@ Vertical Control (Sync)
.equ REG_BG0CNT,	0x04000008		@ Background 0
.equ REG_BG1CNT,	0x0400000A		@ Background 1
.equ REG_BG2CNT,	0x0400000C		@ Background 2
.equ REG_BG3CNT,	0x0400000E		@ Background 3
.equ REG_BG2HOFS,	0x04000018		@ BG2 X-Offset
.equ REG_BG2VOFS,	0x0400001A		@ BG2 Y-Offset
.equ REG_BG3HOFS,	0x0400001C		@ BG3 X-Offset
.equ REG_BG3VOFS,	0x0400001E		@ BG3 Y-Offset
.equ REG_MOSAIC,	0x0400004C		@ Mosaic Mode
.equ REG_MOSAIC_L,	0x0400004C		@ Mosaic Mode Lo Word
.equ REG_MOSAIC_H,	0x0400004E		@ Mosaic Mode Hi Word
.equ REG_BLDCNT,	0x04000050		@ Color Special Effects Selection
.equ REG_BLDALPHA,	0x04000052		@ Alpha Blending Coefficients
.equ REG_BLDY,		0x04000054		@ Brightness (Fade-In/Out) Coefficient


@ GBA Sound Controller
@@@@@@@@@@@@@@@@@@@@@@

@ GBA Sound Channel 1 - Tone & Sweep
@ ----------------------------------
.equ REG_SOUND1CNT_L,		0x04000060		@ Channel 1 Sweep register
.equ SOUND1_NR_SWEEP_SHIFT,	0	@ Number of sweep shift      (n=0-7)
.equ SOUND1_SWEEP_DIR,		3	@ Sweep Frequency Direction  (0=Increase, 1=Decrease)
.equ SOUND1_SWEEP_INC,		0<<3
.equ SOUND1_SWEEP_DEC,		1<<3
.equ SOUND1_SWEEP_TIME,		4	@ Sweep Time; units of 7.8ms (0-7, min=7.8ms, max=54.7ms)

.equ REG_SOUND1CNT_H,		0x04000062		@ Channel 1 Duty/Len/Envelope
.equ SOUND1_LENGTH,			0	@ Sound length; units of (64-n)/256s  (0-63)
.equ SOUND1_WAVE_DUTY,		6	@ Wave Pattern Duty                   (0-3)
.equ SOUND1_ENV_TIME,		8	@ Envelope Step-Time; units of n/64s  (1-7, 0=No Envelope)
.equ SOUND1_ENV_DIR,		11	@ Envelope Direction                  (0=Decrease, 1=Increase)
.equ SOUND1_ENV_INC,		1<<11
.equ SOUND1_ENV_DEC,		0<<11
.equ SOUND1_ENV,			12	@ Initial Volume of envelope          (1-15, 0=No Sound)

.equ REG_SOUND1CNT_X,		0x04000064		@ Channel 1 Frequency/Control
.equ SOUND1_FREQ,			0	@ Frequency; 131072/(2048-n)Hz  (0-2047)
.equ SOUND1_LENGTH_FLAG,	14	@ Length Flag  (1=Stop output when length in NR11 expires)
.equ SOUND1_PLAY_LOOP,		0<<14
.equ SOUND1_PLAY_ONCE,		1<<14
.equ SOUND1_INIT,			15	@ Initial      (1=Restart Sound)
.equ SOUND1_RESTART,		1<<15

@ GBA Sound Channel 2 - Tone
@ ----------------------------------
.equ REG_SOUND2CNT_L,		0x04000068		@ Channel 2 Duty/Length/Envelope
.equ SOUND2_LENGTH,			0	@ Sound length; units of (64-n)/256s  (0-63)
.equ SOUND2_WAVE_DUTY,		6	@ Wave Pattern Duty                   (0-3)
.equ SOUND2_ENV_TIME,		8	@ Envelope Step-Time; units of n/64s  (1-7, 0=No Envelope)
.equ SOUND2_ENV_DIR,		11	@ Envelope Direction                  (0=Decrease, 1=Increase)
.equ SOUND2_ENV_INC,		1<<11
.equ SOUND2_ENV_DEC,		0<<11
.equ SOUND2_ENV,			12	@ Initial Volume of envelope          (1-15, 0=No Sound)

.equ REG_SOUND2CNT_H,		0x0400006C		@ Channel 2 Frequency/Control
.equ SOUND2_FREQ,			0	@ Frequency; 131072/(2048-n)Hz  (0-2047)
.equ SOUND2_LENGTH_FLAG,	14	@ Length Flag  (1=Stop output when length in NR11 expires)
.equ SOUND2_PLAY_LOOP,		0<<14
.equ SOUND2_PLAY_ONCE,		1<<14
.equ SOUND2_INIT,			15	@ Initial      (1=Restart Sound)
.equ SOUND2_RESTART,		1<<15

@ GBA Sound Channel 3 - Wave Output
@ ----------------------------------
.equ REG_SOUND3CNT_L,		0x04000070
.equ SOUND3_BANK32,			0x0000	@ Use two banks of 32 steps each
.equ SOUND3_BANK64,			0x0020	@ Use one bank of 64 steps
.equ SOUND3_SETBANK0,		0x0000	@ Bank to play 0 or 1 (non set bank is written to)
.equ SOUND3_SETBANK1,		0x0040
.equ SOUND3_PLAY,			0x0080	@ Output sound

.equ REG_SOUND3CNT_H,		0x04000072
.equ SOUND3_LENGTH,			0		@ Sound length; units of (256-n)/256s  (0-255)
.equ SOUND3_OUTPUT0,		0	@ Mute output
.equ SOUND3_OUTPUT1,		1	@ Output unmodified
.equ SOUND3_OUTPUT12,		2	@ Output 1/2
.equ SOUND3_OUTPUT14,		3	@ Output 1/4
.equ SOUND3_OUTPUT34,		1<<2  @ Output 3/4
.equ SOUND3_VOL,			13
.equ SOUND3_OUTPUT1000,		SOUND3_OUTPUT1						@ 1.000
.equ SOUND3_OUTPUT0750,		SOUND3_OUTPUT34						@ 0.750
.equ SOUND3_OUTPUT0500,		SOUND3_OUTPUT12						@ 0.500
.equ SOUND3_OUTPUT0250,		SOUND3_OUTPUT14						@ 0.250
.equ SOUND3_OUTPUT0000,		SOUND3_OUTPUT0						@ 0.000

.equ REG_SOUND3CNT_X,		0x04000074
.equ SOUND3_FREQ,			0	@ Sample Rate; 2097152/(2048-n) Hz   (0-2047)
.equ SOUND2_LENGTH_FLAG,	14	@ Length Flag  (1=Stop output when length in NR31 expires)
.equ SOUND3_PLAYONCE,		0x4000	@ Play sound once
.equ SOUND3_PLAYLOOP,		0x0000	@ Play sound looped
.equ SOUND3_INIT,			0x8000	@ Makes the sound restart

.equ REG_WAVE_RAM0,			0x04000090
.equ REG_WAVE_RAM0_L,		0x04000090
.equ REG_WAVE_RAM0_H,		0x04000092
.equ REG_WAVE_RAM1,			0x04000094
.equ REG_WAVE_RAM1_L,		0x04000094
.equ REG_WAVE_RAM1_H,		0x04000096
.equ REG_WAVE_RAM2,			0x04000098
.equ REG_WAVE_RAM2_L,		0x04000098
.equ REG_WAVE_RAM2_H,		0x0400009A
.equ REG_WAVE_RAM3,			0x0400009C
.equ REG_WAVE_RAM3_L,		0x0400009C
.equ REG_WAVE_RAM3_H,		0x0400009E

@ GBA Sound Channel 4 - Noise
@ ----------------------------------
.equ REG_SOUND4CNT_L,		0x04000078		@ Channel 4 Length/Envelope
.equ SOUND4_LENGTH,			0	@ Sound length; units of (64-n)/256s  (0-63)
.equ SOUND4_ENV_TIME,		8	@ Envelope Step-Time; units of n/64s  (1-7, 0=No Envelope)
.equ SOUND4_ENV_DIR,		11	@ Envelope Direction                  (0=Decrease, 1=Increase)
.equ SOUND4_ENV_INC,		1<<11
.equ SOUND4_ENV_DEC,		0<<11
.equ SOUND4_ENV,			12	@ Initial Volume of envelope          (1-15, 0=No Sound)

.equ REG_SOUND4CNT_H,		0x0400007C		@ Channel 4 Frequency/Control
.equ SOUND4_DIV_RATIO_FREQ,	0		@ Dividing Ratio of Frequencies (r)
.equ SOUND4_COUNTER_STEP,	3		@ Counter Step/Width (0=15 bits, 1=7 bits)
.equ SOUND4_COUNTER_15BITS,	0<<3
.equ SOUND4_COUNTER_7BITS,	1<<3
.equ SOUND4_SHIFT_CLK_FREQ,	4		@ R/W  Shift Clock Frequency (s)
.equ SOUND4_PLAYONCE,		0x4000	@ play sound once
.equ SOUND4_PLAYLOOP,		0x0000	@ play sound looped
.equ SOUND4_INIT,			0x8000	@ makes the sound restart

@ GBA Sound Control Registers
@ ----------------------------------
.equ REG_SOUNDCNT_X,		0x04000084		@ Sound on/off (R/W 32)
.equ SND_ENABLED,			0x80

.equ REG_SOUNDCNT_L,		0x04000080
.equ SND_MASTER_VOL_RIGHT,	0				@ Sound 1-4 Master Volume RIGHT (0-7)
.equ SND_MASTER_VOL_LEFT,	4				@ Sound 1-4 Master Volume LEFT (0-7)
.equ SOUND1_ENABLE_RIGHT,	1<<8
.equ SOUND2_ENABLE_RIGHT,	1<<9
.equ SOUND3_ENABLE_RIGHT,	1<<10
.equ SOUND4_ENABLE_RIGHT,	1<<11
.equ SOUND1_ENABLE_LEFT,	1<<12
.equ SOUND2_ENABLE_LEFT,	1<<13
.equ SOUND3_ENABLE_LEFT,	1<<14
.equ SOUND4_ENABLE_LEFT,	1<<15

.equ REG_SOUNDCNT_H,		0x04000082
.equ SND_OUTPUT_RATIO_25,	0x0000
.equ SND_OUTPUT_RATIO_50,	0x0001
.equ SND_OUTPUT_RATIO_100,	0x0002
.equ DSA_OUTPUT_RATIO_50,	0x0000
.equ DSA_OUTPUT_RATIO_100,	0x0004
.equ DSA_OUTPUT_TO_RIGHT,	0x0100
.equ DSA_OUTPUT_TO_LEFT,	0x0200
.equ DSA_OUTPUT_TO_BOTH,	0x0300
.equ DSA_TIMER0,			0x0000
.equ DSA_TIMER1,			0x0400
.equ DSA_FIFO_RESET,		0x0800
.equ DSB_OUTPUT_RATIO_50,	0x0000
.equ DSB_OUTPUT_RATIO_100,	0x0008
.equ DSB_OUTPUT_TO_RIGHT,	0x1000
.equ DSB_OUTPUT_TO_LEFT,	0x2000
.equ DSB_OUTPUT_TO_BOTH,	0x3000
.equ DSB_TIMER0,			0x0000
.equ DSB_TIMER1,			0x4000
.equ DSB_FIFO_RESET,		0x8000

.equ REG_FIFO_A,			0x040000A0
.equ REG_FIFO_B,			0x040000A4

@REG_DISPCNT defines
.equ MODE_0,			0x0
.equ MODE_1,			0x1
.equ MODE_2,			0x2
.equ MODE_3,			0x3
.equ MODE_4,			0x4
.equ MODE_5,			0x5

.equ BACKBUFFER,		0x10
.equ H_BLANK_OAM,		0x20 

.equ OBJ_MAP_2D,		0x0
.equ OBJ_MAP_1D,		0x40

.equ FORCE_BLANK,		0x80

.equ BG0_ENABLE,		0x100
.equ BG1_ENABLE,		0x200 
.equ BG2_ENABLE,		0x400
.equ BG3_ENABLE,		0x800
.equ OBJ_ENABLE,		0x1000 

.equ WIN1_ENABLE,		0x2000 
.equ WIN2_ENABLE,		0x4000
.equ WINOBJ_ENABLE,		0x8000


.equ REG_DMA0SAD,		0x040000B0		@ DMA0 Source Address
.equ REG_DMA0DAD,		0x040000B4		@ DMA0 Destination Address
.equ REG_DMA0CNT,		0x040000B8		@ DMA0 Control (Amount)
.equ REG_DMA1SAD,		0x040000BC		@ DMA1 Source Address
.equ REG_DMA1DAD,		0x040000C0		@ DMA1 Desination Address
.equ REG_DMA1CNT,		0x040000C4		@ DMA1 Control (Amount)
.equ REG_DMA2SAD,		0x040000C8		@ DMA2 Source Address
.equ REG_DMA2DAD,		0x040000CC		@ DMA2 Destination Address
.equ REG_DMA2CNT,		0x040000D0		@ DMA2 Control (Amount)
.equ REG_DMA3SAD,		0x040000D4		@ DMA3 Source Address
.equ REG_DMA3DAD,		0x040000D8		@ DMA3 Destination Address
.equ REG_DMA3CNT,		0x040000DC		@ DMA3 Control (Amount)

.equ DMA_ENABLE,					0x80000000
.equ DMA_INTERUPT_ENABLE,			0x40000000
.equ DMA_TIMING_IMMEDIATE,			0x00000000
.equ DMA_TIMING_VBLANK,				0x10000000
.equ DMA_TIMING_HBLANK,				0x20000000
.equ DMA_TIMING_SYNC_TO_DISPLAY,	0x30000000
.equ START_ON_FIFO_EMPTY,			0x30000000
.equ DMA_16,						0x00000000
.equ DMA_32,						0x04000000
.equ DMA_REPEAT,					0x02000000
.equ DMA_SOURCE_INCREMENT,			0x00000000
.equ DMA_SOURCE_DECREMENT,			0x00800000
.equ DMA_SOURCE_FIXED,				0x01000000
.equ DMA_DEST_INCREMENT,			0x00000000
.equ DMA_DEST_DECREMENT,			0x00200000
.equ DMA_DEST_FIXED,				0x00400000
.equ DMA_DEST_RELOAD,				0x00600000

.equ DMA_32NOW,					(DMA_ENABLE|DMA_TIMING_IMMEDIATE|DMA_32)
.equ DMA_16NOW,					(DMA_ENABLE|DMA_TIMING_IMMEDIATE|DMA_16)


.equ REG_TM0D,			0x04000100		@ Timer 0 Data
.equ REG_TM0CNT,		0x04000102		@ Timer 0 Control
.equ REG_TM1D,			0x04000104		@ Timer 1 Data
.equ REG_TM1CNT,		0x04000106		@ Timer 1 Control
.equ REG_TM2D,			0x04000108		@ Timer 2 Data
.equ REG_TM2CNT,		0x0400010A		@ Timer 2 Control
.equ REG_TM3D,			0x0400010C		@ Timer 3 Data
.equ REG_TM3CNT,		0x0400010E		@ Timer 3 Control

.equ FREQUENCY_0,		0x0
.equ FREQUENCY_64,		0x1
.equ FREQUENCY_256,		0x2
.equ FREQUENCY_1024,	0x3

.equ TIMER_CASCADE,		1<<2
.equ TIMER_IRQ,			1<<6
.equ TIMER_ENABLE,		1<<7

.equ REG_KEYINPUT,		0x04000130		@ Key Status
.equ REG_KEYCNT,		0x04000132		@ Key Interrupt Control

.equ REG_RCNT,			0x04000134		@ SIO Mode Select/General Purpose Data

.equ REG_IE,			0x04000200		@ Master Interrupt Enable
.equ REG_IF,			0x04000202		@ Interrupt Flags
.equ REG_WSCNT,			0x04000204		@ Waitstate Control
.equ REG_IME,			0x04000208		@ Interrupt Master Enable
.equ REG_PAUSE,			0x04000300		@ Pause

.equ INT_VBLANK,		0x0001
.equ INT_HBLANK,		0x0002 
.equ INT_VCOUNT,		0x0004
.equ INT_TIMER0,		0x0008
.equ INT_TIMER1,		0x0010
.equ INT_TIMER2,		0x0020 
.equ INT_TIMER3,		0x0040
.equ INT_COMUNICATION,	0x0080
.equ INT_DMA0,			0x0100
.equ INT_DMA1,			0x0200
.equ INT_DMA2,			0x0400
.equ INT_DMA3,			0x0800
.equ INT_KEYBOARD,		0x1000
.equ INT_CART,			0x2000
.equ INT_ALL,			0x4000

.globl PlayerVariableSize
.equ PlayerVariableSize,	0x1C

.equ EFFECT0, 0
.equ EFFECT1, 1
.equ EFFECT2, 2
.equ EFFECT3, 3
.equ EFFECT4, 4
.equ EFFECT5, 5
.equ EFFECT010, 6
.equ EFFECT011, 7
.equ EFFECT012, 8
.equ EFFECT013, 9
.equ EFFECT014, 10
.equ EFFECT017, 11
