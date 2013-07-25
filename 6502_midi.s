;
; MUSIC KEYBOARD CONTROL PROGRAM 
; THIS EXAMPLE PROGRAM SCANS A 61 NOTE VELOCITY SENSITIVE MUSIC 
; KEYBOARD AND FORMATS THE SIGNIFICANT EVENTS INTO 4 BYTE GROUPS 
; FOR USE BY A HOST MICROCOMPUTER SYSTEM. 
;
; Original version from "Musical Applications of Microprocessors"
; by Hal Chamberlin
;
; Modifications by Joe Britt, britt@shapetable.com, 7/2013
;
; Build with the cc65 toolchain:
;
;	ca65 -o midi.o -l midi.lst 6502_midi.s
;	ld65 -o midi.bin -C 6502_midi.cfg midi.o
;

	.FEATURE force_range	; needed this to avoid incorrect "range error"
				;  when assembling

; DEVICE AND MEMORY ADDRESSES

;P0RAM	= 	$0000		; first page 0 ram loc
;ROM	= 	$f800		; 1st loc in EPROM

PAREG	= 	$0080		; 6532 I/O port A data reg
PADIR	= 	$0081		; 6532 I/O port A dir reg
PBREG	= 	$0082		; 6532 I/O port B data reg
PBDIR	= 	$0083		; 6532 I/O port B dir reg
INTFLG	=	$0085		; 6532 IRQ status: b7 = timer, b6 = PA7 edge
TMWRI	= 	$009c		; 6532 timer write, /1, enable timer int
TMRD	= 	$008c		; 6532 timer read, leave int enabled
P1RAM	= 	$0100		; first page 1 RAM loc (same 128 bytes as P0RAM)

MKYBU	= 	$0400		; base addr of music kybd upper bus
MKYBL	= 	$0440		; base addr of music kybd lower bus

SERCTL	=	$0800		; 6850 acia control reg
SERSTS	=	$0800		; 6850 acia status reg
SERDTA	=	$0801		; 6850 acia data reg


; RAM allocation

	.segment "ZEROPAGE"

IRQCNT:	.res	1		; counts down for timed IRQ events (LED blink)
NEXTSCN:.res	1		; when 0, OK to do another scan
MIDICH:	.res	1		; byte for our channel
MKYBST:	.res	61		; space for cur state of 61 keys
QUIP:	.res	1		; event q input ptr
QUOP:	.res	1		; event q output ptr
EVQU:	.res	32		; space to queue up to 7 events
TIMCNT:	.res	1		; 5 counter for timers interrupts
VCJPPT:	.res	2		; indirect ptr for vector jump

	.segment "CODE"		; start at beginning of ROM


;	-----------------------------------------------------------------------
; 	INDIRECT JUMP TABLE FOR STATE PROCESSING

VCJPTB:	jmp	STAT0		; go to state 0 processing
	.byte	0		; 1 byte pad so vect addrs are div by 4
	jmp	STAT1		; go to state 1
	.byte 	0
	jmp	STAT2		; go to state 2
	.byte	0	
	jmp	STAT3		; go to state 3


;	-----------------------------------------------------------------------
; 	INITIALIZATION

INIT:	sei			; interrupts disabled

	lda	#$ff		; set data dir regs on 6532
	sta	PBDIR		; port B to all outputs
	sta	PADIR
	lda	#$00
	sta	PBREG		; port B to all 1s (LED off)
	sta	PAREG

	lda	#$17		; init 6850 ACIA
	sta	SERCTL
	lda	#$15
	sta	SERCTL

	lda	#0		; default to MIDI channel 1
	sta	MIDICH

	lda	#$ff
	sta	NEXTSCN

	ldx	#$7f		; zero RAM
	txs
	lda	#0
zram:	pha	
	dex
	bpl	zram
	
	cld			; set binary arith mode
	ldx	#$7f		; init SP to end of page 1 RAM
	txs

	lda	#(VCJPTB/256)	; init upper byte of vector jump ptr
	sta	VCJPPT+1

	lda	#5		; init timer interrupt count
	sta	TIMCNT
	lda	#255		; start timer, set for 255 us
	sta	TMWRI		; and enable timer interrupt

	cli			; enable interrupt system

;	-----------------------------------------------------------------------
; 	MAIN PROGRAM LOOP

MLOOP:	
	lda	PAREG
	ora	#$01
	sta	PAREG

;	-----------------------------------------------------------------------
; 	scan the keyboard, scan loop is expanded by 4 for greater speed

KYSCN:	
;	ldy	#60		; init key address
	ldy	#48		; init key address -- 49 keys to scan 
	bne	KYSCN4		; enter expanded loop at proper place
KYSCN1:	lda	MKYBU,Y		; get upper bus contact indication
	ora	MKYBST,Y	; combine with previous key state
	bne	KYPROC		; branch if action required
KYSCNA:	dey			; decrement key address
KYSCN2:	lda	MKYBU,Y		; repeat for next key
	ora	MKYBST,Y
	bne	KYPROC
KYSCNB:	dey
KYSCN3:	lda	MKYBU,Y		; repeat for next key
	ora	MKYBST,Y
	bne	KYPROC
KYSCNC:	dey
KYSCN4:	lda	MKYBU,Y		; repeat for next key
	ora	MKYBST,Y
	bne	KYPROC
KYSCND:	dey			; decrement key address and test if done
	bpl	KYSCN1		; go scan more keys if not finished

	lda	PAREG
	and	#$fe
	sta	PAREG
	
KYSCN5:	lda	NEXTSCN		; timer interrupt sets this to 0
	bne	KYSCN5		; wait for timer interrupt

	dec	NEXTSCN
	jmp	MLOOP

;	-----------------------------------------------------------------------
;	based on previous key state dispatch to correct key processing

KYPROC:	and	#$03		; isolate state number
	asl			; set up vector jump
	asl	
	sta	VCJPPT
	jmp	(VCJPPT)	; do the vector jump

; 	re-enter the scan loop at the proper place based on key address
;
;	1 xx11 --> A  S=1, C=1
;	2 xx10 --> B  S=0, C=1  +
;	3 xx01 --> C  S=1. C=0
;	4 xx00 --> D  S=0, C=0  +


SCNREN:	tya			; get 2 low bits of key address in carry
	ror			; and sign flags
	ror	
	bpl	SCNRE1		; remainder of (key address)/4
	bcs	KYSCNA		; S=1, C=1
	bcc	KYSCNC		; S=1, C=0
SCNRE1:	bcs	KYSCNB		; S=0, C=1
	bcc	KYSCND		; S=0, C=0

;	key state processing routines

;	-----------------------------------------------------------------------
;	STATE 0 - IDLE
;
;	Key has started going down
;

STAT0:	lda	#1		; set the state to 1 and zero the velocity
	sta	MKYBST,Y	; count
	bne	SCNREN		; re-enter scanning loop

;	-----------------------------------------------------------------------
;	STATE 1 - DOWNSTROKE
;
;	Key is in flight towards bottom bus 
;

STAT1:	lda	MKYBU,Y		; test key contact with upper bus
	bmi	STAT1A		; jump if not contacting it
	sta	MKYBST,Y	; clear key state to 0 (inactive) if
	beq	SCNREN		; contacting it and re-enter scan loop
STAT1A:	lda	MKYBL,Y		; test key contact with lower bus
	bpl	STAT1C		; jump if contacting it
	lda	MKYBST,Y	; if not, get key state and increment the
	clc			; velocity count
	adc	#4
	bcc	STAT1B		; skip if no overflow
	sbc	#4		; restore max velocity count if overflow
STAT1B:	sta	MKYBST,Y
	bne	SCNREN		; re-enter scan loop
STAT1C:	sei			; disable interrupts while storing event
	ldx	QUIP		; output an event, get queue input pointer
	lda	#$90		; key down event on midi channel MIDICH
	ora	MIDICH
	sta	EVQU,X		; store in queue
	inx			; increment queue ptr
	cpx	#$20		; wrap ptr if necessary
	bne	STAT1D
	ldx	#0
STAT1D:	tya			; get key number
	clc			; adjust to midi standard of 60 = middle C
	adc	#60-24		; this gives 2 octaves below and 3 above
	sta	EVQU,X		; store result in queue
	inx			; increment and wraparound queue ptr
	cpx	#$20
	bne	STAT1E
	ldx	#0
STAT1E:	stx	QUIP		; save queue ptr temporarily
	lda	MKYBST,Y	; get velocity cnt, range 0-63
	lsr
	lsr
	tax			; lookup appropriate midi velocity
	lda	VELTAB,X	; in a table
	ldx	QUIP		; restore queue ptr
	sta	EVQU,X		; store the velocity in the queue
	inx			; increment and wraparound ptr
	cpx	#$20
	bne	STAT1F
	ldx	#0
STAT1F:	stx	QUIP		; store the updated queue ptr
	cli			; re-enable interrupts
	lda	#2		; set key state to 2
	sta	MKYBST,Y
	jmp	SCNREN		; resume scanning

;	-----------------------------------------------------------------------
;	STATE 2 - HELD DOWN
;
;	Key is held down (against lower bus) 
;

STAT2:	lda	MKYBL,Y		; test key contact with lower bus
	bpl	SCNREN		; resume scanning if in contact
	lda	#3		; set the state to 3 and zero velocity
	sta	MKYBST,Y	;  count if no contact
	bne	GOSREN		; and resume scanning

;	-----------------------------------------------------------------------
;	STATE 3 - UPSTROKE 
;
;	Key is in flight towards upper bus
;

STAT3:	lda	MKYBL,Y		; test key contact with lower bus
	bmi	STAT3A		; jump if not contacting it
	lda	#2		; set state to 2 and clear velocity count
	sta	MKYBST,Y	; if contacting lower bus
	bne	GOSREN		; re-enter scan loop
STAT3A:	lda	MKYBU,Y		; test key contact with upper bus
	bpl	STAT3C		; jump if contacting it
	lda	MKYBST,Y	; if not, get key state and increment the
	clc			; velocity count
	adc	#4
	bcc	STAT3B		; skip if no overflow
	sbc	#4		; restore max velocity count if overflow
STAT3B:	sta	MKYBST,Y
GOSREN:	jmp	SCNREN		; re-enter scan loop
STAT3C:	sei			; disable timer interrupt while queueing
	ldx	QUIP		; output an event, get queue input ptr
	lda	#$80		; key up on midi channel MIDICH
	sta	EVQU,X		; store in queue
	inx			; increment queue ptr
	cpx	#$20		; wraparound the ptr if necessary
	bne	STAT3D
	ldx	#0
STAT3D:	tya			; get key number
	clc			; adjust to midi standard
	adc	#60-24
	sta	EVQU,X		; store result in queue
	inx			; increment and wraparound queue ptr
	cpx	#$20
	bne	STAT3E
	ldx	#0
STAT3E:	stx	QUIP		; save queue ptr temporarily
	lda	MKYBST,Y	; get velocity count, range 0-63
	lsr
	lsr
	tax			; lookup appropriate midi velocity
	lda	VELTAB,X	; in a table
	ldx	QUIP		; restore the queue ptr
	sta	EVQU,X		; store the velocity in the queue
	inx			; increment and wraparound queue ptr
	cpx	#$20
	bne	STAT3F
	ldx	#0
STAT3F:	stx	QUIP		; store the updated queue ptr
	cli			; re-enable interrupts
	lda	#0		; set key state to 0 (inactive)
	sta	MKYBST,Y
	jmp	SCNREN		; resume scanning


;	-----------------------------------------------------------------------
; 	PROCESS TIMER INTERRUPT

IRQHANDLER:	
	pha			; save A on the stack

	lda	SERSTS		; check 6850 transmitter status
	and	#$02
	beq	CHKTMR		; jump ahead if busy

	lda	QUOP		; if free, test if anything to xmit
	cmp	QUIP
	beq	CHKTMR		; jump ahead if not

	txa			; if so, save X
	pha
	ldx	QUOP		; get the queue output pointer
	lda	EVQU,X		; get next byte to xmit
	sta	SERDTA		; and xmit it

	inx			; move queue output ptr
	cpx	#$20		; and wraparound if necessary
	bne	NOWRAP	
	ldx	#0
NOWRAP:	stx	QUOP		; save updated queue output ptr
	pla			; restore X from stack
	tax			; and continue

	lda	#$ff		; blink the LED
	sta	IRQCNT		; this will count down

CHKTMR:
	lda	INTFLG
	bpl	IRQEXIT		; b7 = 1 -> timer interrupt

	lda	IRQCNT
	beq	CNTZRO
	dec	IRQCNT
	rol
	rol
	sta	PBREG		; that becomes current LED state
CNTZRO:

	lda	#255		; restart timer, set for 255 us
	sta	TMWRI		; and enable timer interrupt

	cmp	TMRD		; clear the timer int req

	dec	TIMCNT		; test if 5th timer interrupt
	bne	IRQEXIT		; if not, just return 

	lda	#5		; reset 5 interrupt counter
	sta	TIMCNT

	lda	#0
	sta	NEXTSCN		; tell the main loop to scan again

IRQEXIT:	
	pla
	rti			; if not, return from interrupt


VELTAB:	.byte	127,127,127,127,127,117,96,81	; midi velocity translate table
	.byte	69,61,53,48,43,39,35,32
	.byte	30,27,25,23,22,20,19,18		; prepared from equation:
	.byte	16,15,14,14,13,12,11,10		; TABLE=(635/N)-10
	.byte	10,9,9,8,8,7,7,6		; will probably need change
	.byte	6,5,5,5,4,4,4,4			; to best match the velocity
	.byte	3,3,3,2,2,2,2,2			; control of the synthesizer
	.byte	1,1,1,1,1,0,0,0			; being used



; MACHINE INTERRUPT AND RESET VECTORS

	.segment "VECTORS"

	.word	0		; fffa, fffb	NMI
	.word	INIT		; fffc, fffd	RESET
	.word	IRQHANDLER	; fffe, ffff	IRQ

	.end


