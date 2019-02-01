
/*
*	
*	The Kernel
*	
*/

#include <stddef.h>
#include <stdint.h>
#include "kernel.h"

#define BOOL short
#define BOOL_F 0
#define BOOL_T 1

#define CHAR_W 15
#define CHAR_H 17

//uart
void uart_init(void);
void uart_putc(uint8_t);
uint8_t uart_getc();

//memory
uint32_t memory_read(uint32_t);
void memory_write(uint32_t, uint32_t );

void delay(int32_t);
void _video_sample(uint32_t, uint32_t, uint32_t);

//HAL - Serial
void hal_io_serial_init(void);
void hal_io_serial_putc(uint8_t);
uint8_t hal_io_serial_getc(void);

//HAL - Video
void hal_io_video_init(void);
void hal_io_video_putpixel(int x, int y, int color);
void hal_io_video_putc(int x, int y, int color, uint8_t character);

void draw_os_name_to_video(void);

/*
 *		Kernel's entry point
 *
**/

int cursor_x = CHAR_W + CHAR_W/2;
int cursor_y = 0;

void main(uint32_t r0, uint32_t r1, uint32_t atags){

	draw_os_name_to_video();

	//Begin the one-line typewriter
	hal_io_serial_init();
	while (1){
		char c = hal_io_serial_getc();
		cursor_drawc(c);
		hal_io_serial_putc(c);
	}
}

void hal_io_video_init(){
	_video_sample(0,0,0);
}

void hal_io_video_putpixel(int x, int y, int color){
	_video_sample(x, y, color);
}

void hal_io_video_putc(int x, int y, int color, uint8_t character){
	switch(character){
		case ' ':
			break;
		case '\n':
		case '\r':
			cursor_newln();
			break;
		case 'a' : case 'A':
			draw_A(x, y, color);
			break;
		case 'b' : case 'B':
			draw_B(x, y, color);
			break;
		case 'c' : case 'C':
			draw_C(x, y, color);
			break;
		case 'd' : case 'D':
			draw_D(x, y, color);
			break;
		case 'e' : case 'E':
			draw_E(x, y, color);
			break;
		case 'f' : case 'F':
			draw_F(x, y, color);
			break;
		case 'g' : case 'G':
			draw_G(x, y, color);
			break;
		case 'h' : case 'H':
			draw_H(x, y, color);
			break;
		case 'i' : case 'I':
			draw_I(x, y, color);
			break;
		case 'j' : case 'J':
			draw_J(x, y, color);
			break;
		case 'l' : case 'L':
			draw_L(x, y, color);
			break;
		case 'm' : case 'M':
			draw_M(x, y, color);
			break;
		case 'o' : case 'O':
			draw_O(x, y, color);
			break;
		case 'n' : case 'N':
			draw_N(x, y, color);
			break;
		case 'k' : case 'K':
			draw_K(x, y, color);
			break;
		case 'p' : case 'P':
			draw_P(x, y, color);
			break;
		case 'q' : case 'Q':
			draw_Q(x, y, color);
			break;
		case 'r' : case 'R':
			draw_R(x, y, color);
			break;
		case 's' : case 'S':
			draw_S(x, y, color);
			break;
		case 't' : case 'T':
			draw_T(x, y, color);
			break;
		case 'u' : case 'U':
			draw_U(x, y, color);
			break;
		case 'v' : case 'V':
			draw_V(x, y, color);
			break;
		case 'w' : case 'W':
			draw_W(x, y, color);
			break;
		case 'x' : case 'X':
			draw_X(x, y, color);
			break;
		case 'y' : case 'Y':
			draw_Y(x, y, color);
			break;
		case 'z' : case 'Z':
			draw_Z(x, y, color);
			break;
		case '0':
			draw_0(x, y, color);
			break;
		case '1':
			draw_1(x, y, color);
			break;
		default:
			draw_non_char(x, y, color);
	}
}

void hal_io_serial_init(){
	uart_init();
}

void hal_io_serial_putc(uint8_t c){
	uart_putc(c);
}

uint8_t hal_io_serial_getc(){
	return uart_getc();
}

void uart_putc(uint8_t c){
	//wait for it to be ready
	while ( memory_read(UART0_FR) & (1 << 5) );
	
	//write
	memory_write(UART0_DR, c);
}
 
uint8_t uart_getc(void){
    //wait for it to be ready
    while ( memory_read(UART0_FR) & (1 << 4) );
	
	//write
    return memory_read(UART0_DR);
}

void memory_write(uint32_t address, uint32_t v){
	*(volatile uint32_t*)address = v;
} 

uint32_t memory_read(uint32_t address){
	return *(volatile uint32_t*)address;
}
 

/*
*	From 
*	https://wiki.osdev.org/Raspberry_Pi_Bare_Bones#Building_a_Cross-Compiler
*
*/
void delay(int32_t count) {
	asm volatile("__delay_%=: subs %[count], %[count], #1; bne __delay_%=\n"
		 : "=r"(count): [count]"0"(count) : "cc");
}
 
 
/*
*	From 
*	https://wiki.osdev.org/Raspberry_Pi_Bare_Bones#Building_a_Cross-Compiler
*
*/
void uart_init(void){
	
	// Disable UART0.
	memory_write(UART0_CR, 0x00000000);
	// Setup the GPIO pin 14 && 15.
 
	// Disable pull up/down for all GPIO pins & delay for 150 cycles.
	memory_write(GPPUD, 0x00000000);
	delay(150);
 
	// Disable pull up/down for pin 14,15 & delay for 150 cycles.
	memory_write(GPPUDCLK0, (1 << 14) | (1 << 15));
	delay(150);
 
	// Write 0 to GPPUDCLK0 to make it take effect.
	memory_write(GPPUDCLK0, 0x00000000);
 
	// Clear pending interrupts.
	memory_write(UART0_ICR, 0x7FF);
 
	// Set integer & fractional part of baud rate.
	// Divider = UART_CLOCK/(16 * Baud)
	// Fraction part register = (Fractional part * 64) + 0.5
	// UART_CLOCK = 3000000; Baud = 115200.
 
	// Divider = 3000000 / (16 * 115200) = 1.627 = ~1.
	memory_write(UART0_IBRD, 1);
	// Fractional part register = (.627 * 64) + 0.5 = 40.6 = ~40.
	memory_write(UART0_FBRD, 40);
 
	// Enable FIFO & 8 bit data transmissio (1 stop bit, no parity).
	memory_write(UART0_LCRH, (1 << 4) | (1 << 5) | (1 << 6));
 
	// Mask all interrupts.
	memory_write(UART0_IMSC, (1 << 1) | (1 << 4) | (1 << 5) | (1 << 6) |
	                       (1 << 7) | (1 << 8) | (1 << 9) | (1 << 10));
 
	// Enable UART0, receive & transfer part of UART.
	memory_write(UART0_CR, (1 << 0) | (1 << 8) | (1 << 9));
}
 
 
void draw_char_pipe_left(int x, int y, int color){
	for (int row = 0; row < CHAR_H; ++row) {
		hal_io_video_putpixel(x, y + row, color);
	}
}

void draw_char_pipe_right(int x, int y, int color){
	for (int row = 0; row < CHAR_H; ++row) {
		hal_io_video_putpixel(x + CHAR_W, y + row, color);
	}
}

void draw_char_pipe_mid(int x, int y, int color){
	for (int row = 0; row <= CHAR_H; ++row) {
		hal_io_video_putpixel(x + CHAR_W/2, y + row, color);
	}
}

void draw_char_bar_top(int x, int y, int color){
	for (int offset = 0; offset <= CHAR_W; ++offset){
		hal_io_video_putpixel(x + offset, y , color);
	}
}

void draw_char_bar_btm(int x, int y, int color){
	for (int offset = 0; offset <= CHAR_W; ++offset){
		hal_io_video_putpixel(x + offset, y + CHAR_H, color);
	}
}

void draw_char_bar_mid(int x, int y, int color){
	for (int offset = 0; offset <= CHAR_W; ++offset){
		hal_io_video_putpixel(x + offset, y + CHAR_H/2, color);
	}
}

void draw_char_left_bar(int x, int y, int color){
	for (int offset = 0; offset <= CHAR_W/2; ++offset){
		hal_io_video_putpixel(x + offset, y + CHAR_H/2, color);
	}
}

void draw_char_right_bar(int x, int y, int color){
	for (int offset = CHAR_H/2; offset <= CHAR_W; ++offset){
		hal_io_video_putpixel(x + offset, y + CHAR_H/2, color);
	}
}

void draw_char_diag(int x, int y, int color, BOOL invert){
	if(invert){
		draw_char_top_left_diag(x, y, color);
		draw_char_btm_right_diag(x, y, color);
	} else {
		draw_char_top_right_diag(x, y, color);
		draw_char_btm_left_diag(x, y, color);
	}
}

void draw_char_top_right_diag(int x, int y, int color) {
	int row = 0;
	for(int offset = CHAR_W; offset > CHAR_W/2; --offset){
		hal_io_video_putpixel(x + offset, y + ++row, color);
	}
}

void draw_char_top_left_diag(int x, int y, int color) {
	for(int offset = 0; offset <= CHAR_W/2; ++offset)
		hal_io_video_putpixel(x + offset, y + offset, 0);
}

void draw_char_btm_left_diag(int x, int y, int color) {
	int row = CHAR_H/2;
	for(int offset = CHAR_W/2; offset > 0; --offset)
		hal_io_video_putpixel(x + offset, y + ++row, 0);	
}

void draw_char_btm_right_diag(int x, int y, int color) {
	int row = CHAR_H/2;
	for(int offset = CHAR_W/2; offset <= CHAR_W; ++offset){
		hal_io_video_putpixel(x + offset, y + ++row, 0);
	}
}

void draw_non_char(int x, int y, int color){
	for(int offsetX = 0; offsetX <= CHAR_W; ++offsetX){
		for(int offsetY = 0; offsetY <= CHAR_H; ++offsetY){
			hal_io_video_putpixel(x + offsetX, y + offsetY, color);
		}
	}
}

void draw_0(int x, int y, int color) {
	draw_O(x, y, color);
	draw_char_diag(x, y, color, BOOL_F);
}

void draw_1(int x, int y, int color) {
	draw_char_pipe_mid(x, y, color);
}

void draw_A(int x, int y, int color){
	draw_char_pipe_left(x, y, color);
	draw_char_pipe_right(x, y, color);
	draw_char_bar_mid(x, y, color);
	draw_char_bar_top(x, y, color);
}

void draw_B(int x, int y, int color){
	draw_char_pipe_left(x, y, color);
	draw_char_left_bar(x, y, color);
	draw_char_bar_top(x, y, color);
	draw_char_bar_btm(x, y, color);
	draw_char_top_right_diag(x, y, color);
	draw_char_btm_right_diag(x, y, color);
}

void draw_C(int x, int y, int color){
	draw_L(x,y,color);
	draw_char_bar_top(x, y, color);
}

void draw_D(int x, int y, int color){
	draw_char_pipe_left(x,y,color);
	draw_char_btm_left_diag(x,y,color);
	draw_char_top_left_diag(x,y,color);
}

void draw_E(int x, int y, int color){
	draw_F(x,y,color);
	draw_char_bar_btm(x,y,color);
}

void draw_F(int x, int y, int color){
	draw_char_pipe_left(x,y,color);
	draw_char_bar_top(x,y,color);
	draw_char_left_bar(x,y,color);
}

void draw_G(int x, int y, int color){
	draw_C(x,y,color);
	draw_char_btm_right_diag(x,y,color);
}

void draw_H(int x, int y, int color){
	draw_char_pipe_left(x, y, color);
	draw_char_pipe_right(x, y, color);
	draw_char_bar_mid(x, y, color);
}

void draw_I(int x, int y, int color){
	draw_char_pipe_mid(x, y, color);
	draw_char_bar_top(x,y,color);
	draw_char_bar_btm(x,y,color);
}

void draw_J(int x, int y, int color){
	draw_char_pipe_right(x, y, color);
	draw_char_bar_top(x,y,color);
	draw_char_bar_btm(x,y,color);
	draw_char_btm_left_diag(x,y,color);
}

void draw_K(int x, int y, int color){
	draw_char_pipe_left(x, y, color);
	draw_char_top_right_diag(x, y, color);
	draw_char_btm_right_diag(x, y, color);
}

void draw_L(int x, int y, int color){
	draw_char_pipe_left(x, y, color);
	draw_char_bar_btm(x,y,color);
}

void draw_M(int x, int y, int color){
	draw_char_pipe_left(x,y,color);
	draw_char_pipe_right(x,y,color);
	draw_char_top_left_diag(x,y,color);
	draw_char_top_right_diag(x,y,color);
}

void draw_O(int x, int y, int color){
	draw_C(x,y,color);
	draw_char_pipe_right(x, y, color);
}

void draw_N(int x, int y, int color){
	draw_char_pipe_left(x, y, color);
	draw_char_pipe_right(x, y, color);
	draw_char_diag(x, y, color, BOOL_T);
}

void draw_P(int x, int y, int color){
	draw_F(x,y,color);
	draw_char_top_right_diag(x,y,color);
}

void draw_Q(int x, int y, int color){
	draw_O(x,y,color);
	draw_char_btm_right_diag(x,y,color);
}

void draw_R(int x, int y, int color){
	draw_P(x,y,color);
	draw_char_btm_right_diag(x,y,color);
}

void draw_S(int x, int y, int color){
	draw_char_bar_top(x, y, color);
	draw_char_bar_btm(x, y, color);
	draw_char_diag(x, y, color, BOOL_T);
}

void draw_T(int x, int y, int color){
	draw_char_pipe_mid(x,y,color);
	draw_char_bar_top(x,y,color);
}

void draw_U(int x, int y, int color){
	draw_char_pipe_left(x,y,color);
	draw_char_pipe_right(x,y,color);
	draw_char_bar_btm(x,y,color);
}

void draw_V(int x, int y, int color){
	draw_char_pipe_left(x,y,color);
	draw_char_diag(x,y,color,BOOL_F);
}

void draw_W(int x, int y, int color){
	draw_char_pipe_left(x,y,color);
	draw_char_pipe_right(x,y,color);
	draw_char_btm_left_diag(x,y,color);
	draw_char_btm_right_diag(x,y,color);
}

void draw_X(int x, int y, int color){
	draw_char_diag(x,y,color,BOOL_T);
	draw_char_diag(x,y,color,BOOL_F);
}

void draw_Y(int x, int y, int color){
	draw_char_pipe_mid(x,y,color);
	draw_char_top_left_diag(x,y,color);
	draw_char_top_right_diag(x,y,color);
}

void draw_Z(int x, int y, int color){
	draw_char_bar_top(x,y,color);
	draw_char_bar_btm(x,y,color);
	draw_char_bar_mid(x,y,color);
	draw_char_diag(x, y, color, BOOL_F);
}

void cursor_drawc(int character){
	hal_io_video_putc(cursor_x, cursor_y, 0, character);
	if(character != '\n' && character != '\r')
		cursor_forwd();
}

void cursor_forwd(){
	cursor_x += CHAR_W + (CHAR_W / 2);
}

void cursor_newln(){
	cursor_x = CHAR_W + (CHAR_W / 2);
	cursor_y += (CHAR_H + 2);
}

void draw_os_name_to_video(){
	cursor_drawc('C');
	cursor_drawc('H');
	cursor_drawc('O');
	cursor_drawc('N');
	cursor_drawc('K');
	cursor_forwd();
	cursor_drawc('O');
	cursor_drawc('S');
	cursor_newln();
}