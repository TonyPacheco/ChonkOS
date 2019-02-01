/*
*  Video Sample 
*
*	From here (+ a few modifications):
*   https://github.com/mmuszkow/NoOsBootstrap/tree/master/arm
*
*/


@ The bootloader will print text to UART and display 
@ white "NO OS" bitmap on video output

@ I chose Raspberry Pi cause it's very popular and
@ it has a Video Controller with HDMI/Composite output
@ that can be easily programmed via mailbox interface

.section .text
.globl _video_sample
.align 2

@@@@@@@@@@@@@@ START @@@@@@@@@@@@@

_video_sample:
	push {lr}
    @ Store my params for where to put the pixel
    @ In registers that I wont be touching until
    mov r7, r0
    mov r8, r1
    mov r9, r2
    @ set Video Controller resolution to 640x480x16bit
    @ 16-bit, cause the 8-bit depth needs a palette
    @ and I'm too lazy to set it up
    ldr r1, =vc_set_res
    bl  mb0_c8_write
    bl  mb0_c8_read
    tst r0, #0x80000000
    beq .vc_init_fail

    @ get VC framebuffer address
    ldr r1, =vc_alloc_fb
    bl  mb0_c8_write
    bl  mb0_c8_read
    tst r0, #0x80000008
    beq .vc_init_fail

    @ check if the address is correct
    ldr r0, [r1, #20]
    cmp r0, #0
    beq .vc_init_fail

	@ draw my pixel
	bl vc_draw_pixel
    
	pop {pc}

.vc_init_fail:
    ldr r1, =txt_vc_fail

halt:
    wfe @ low-power mode
    b halt

@@@@@@@@@@@@@@ UART @@@@@@@@@@@@@

.equ UART0BASE,  0x3F201000 @ for raspi2 & 3, 0x20201000 for raspi1
.equ UART0_CR,   UART0BASE + 0x30
.equ UART0_ICR,  UART0BASE + 0x44
.equ UART0_IBRD, UART0BASE + 0x24
.equ UART0_FBRD, UART0BASE + 0x28
.equ UART0_LCRH, UART0BASE + 0x2C
.equ UART0_IMSC, UART0BASE + 0x38
.equ UART0_FR,   UART0BASE + 0x18
.equ UART0_DR,   UART0BASE + 0x00
.equ GPIO_BASE,  0x3F200000 @ for raspi2 & 3, 0x20200000 for raspi1
.equ GPPUD,      GPIO_BASE + 0x94
.equ GPPUDCLK0,  GPIO_BASE + 0x98

.macro wait, count, reg0 = r1
    mov  \reg0, \count
1001: 
    sub \reg0, #1
    cmp \reg0, #0
    bne 1001b
.endm

.macro mem_write, addr, val, reg0 = r1, reg1 = r2
    ldr \reg0, =\addr
    ldr \reg1, =\val
    str \reg1, [\reg0]
.endm

@ init UART0 to 115200, no parity
uart0_init:
    mem_write UART0_CR, #0               @ disable UART0
    mem_write GPPUD, #0                  @ disable pull up/down for all GPIO pins
    wait #150
    mem_write GPPUDCLK0, #0b1100000000000000
    wait #150
    mem_write GPPUDCLK0,  #0
    mem_write UART0_ICR,  #0x7FF         @ clear pending interrupts
    mem_write UART0_IBRD, #1             @ divider = 3000000 / (16 * 115200) = 1.627 = ~1
    mem_write UART0_FBRD, #40            @ fractional part register = (.627 * 64) + 0.5 = 40.6 = ~40
    mem_write UART0_LCRH, #0b1110000     @ enable FIFO & 8 bit data transmissio (1 stop bit, no parity)
    mem_write UART0_IMSC, #0b11111110010 @ mask all interrupts
    mem_write UART0_CR,   #0b1100000001  @ enable UART0, receive & transfer part of UART
    mov pc, lr

@@@@@@@@@@@@@@@ VC @@@@@@@@@@@@@@

.equ MBOX0, 0x3f00b880

@ writes to mailbox #0, channel 8
@ r1 - message
mb0_c8_write:
    message .req r1
    mailbox .req r3
    status  .req r2

    ldr mailbox, =MBOX0
.mb0_full:
    ldr status, [mailbox, #0x18]
    tst status, #0x80000000  @ mailbox full flag
    bne .mb0_full 
    add message, #8          @ channel 8
    str message, [mailbox, #0x20] @ write addr
    sub message, #8

    .unreq mailbox
    .unreq message
    .unreq status
    mov pc, lr

@ reads from mailbox #0, channel 8
@ r1 - message
@ returns status in r0
mb0_c8_read:
    message .req r1
    mailbox .req r2
    status  .req r3
    value   .req r4

    ldr mailbox, =MBOX0
.mb0_empty:
    ldr status, [mailbox, #0x18] 
    tst status, #0x40000000  @ mailbox empty flag
    bne .mb0_empty

    ldr value, [mailbox] @ check if the message channel is 8
    and r0, value, #0xf
    teq r0, #8
    bne .mb0_empty

    ldr r0, [message, #4] 
    .unreq message
    .unreq mailbox
    .unreq status
    .unreq value
    mov pc, lr

@TONY try to draw a pixel on the screen
@r0 - frame buffer address, ie the top left pixel
@Screen goes from left=0 to right=1279
@Screen goes from top=0 to bottom= ??
vc_draw_pixel:
	fb    .req r0
	width .req r2
	pixl  .req r3
	inx   .req r7
	iny   .req r8
    color .req r9
    ldr color, =#0xFFFFFFFF
	ldr width, =#1280

	mul pixl, iny, width
	add pixl, inx

	str color, [fb, pixl]

	mov pc, lr

@ raspi mailbox requests, must be padded to 16 bytes
.align 4
vc_set_res:  .word 80, 0                      @ total size, code (0=req)
             .word 0x00048003, 8, 8, 640, 480 @ set physical size (640x480)
             .word 0x00048004, 8, 8, 640, 480 @ set virtual size (640x480)
             .word 0x00048005, 4, 4, 16       @ set depth (16-bit)
             .word 0, 0, 0, 0                 @ end tag & padding

vc_alloc_fb: .word 32, 0                      @ total size, code (0=req)
             .word 0x00040001, 8, 4, 16, 0    @ allocate framebuffer
             .word 0                          @ end tag & padding

.align 2
txt_welcome: .asciz "No OS installed\r\n"
txt_vc_fail: .asciz "VC initialization failed\r\n"
