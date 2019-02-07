
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
#define SCRN_W 1280 //not really, but theres two mem slots per each of the 640 pixels of the width
#define SRCN_H 420

#define BLK 0b0000000000000000
#define WHT 0b1111111111111111
#define BLU 0b0000000000011111
#define RED 0b1111100000000000
#define GRN 0b0000011111100000

void delay(int32_t);

//uart
void uart_init(void);
void uart_putc(uint8_t);
uint8_t uart_getc();

//memory
uint32_t memory_read(uint32_t);
void memory_write(uint32_t, uint32_t );

//assembly functions
uint32_t _video_init(void);
void _v_draw_pixel(uint32_t,uint32_t,uint32_t,uint32_t);

//HAL - Serial
void hal_io_serial_init(void);
void hal_io_serial_putc(uint8_t);
uint8_t hal_io_serial_getc(void);

//HAL - Video
uint32_t hal_io_video_init(void);
void hal_io_video_putpixel(int,int,int,int);
void hal_io_video_putc(int,int, int,int,uint8_t);

//convenience
void draw_os_name_to_video(int,int);

/*
 *		Kernel's entry point
 *
**/

int cursor_x;
int cursor_y;

void main(uint32_t r0, uint32_t r1, uint32_t atags){

	uint32_t fb = hal_io_video_init(); //Gets frame buffers location
	cursor_reset();					   //Brings the cursor to the start of the terminal screen
	draw_os_name_to_video(fb, BLU);    

	hal_io_serial_init();
	while (BOOL_T){
		char c = hal_io_serial_getc(); 
		cursor_drawc(fb, c, WHT);
		hal_io_serial_putc(c);
	}
}

uint32_t hal_io_video_init(){
	return _video_init();
}

void hal_io_video_putpixel(int fb, int x, int y, int color){
	_v_draw_pixel(fb, x, y, color);
}

void hal_io_video_putc(int fb, int x, int y, int color, uint8_t character){
	//TODO: Replace this with something more elegant
	//ie one function which draws anything given a bit mask
	switch(character){
		case'\b':
		    cursor_bkspc(fb, x, y);
		    break;
		case ' ':
			break;
		case '\n':
		case '\r':
			cursor_newln();
			break;
		case 'a' : case 'A':
			draw_A(fb, x, y, color);
			break;
		case 'b' : case 'B':
			draw_B(fb, x, y, color);
			break;
		case 'c' : case 'C':
			draw_C(fb, x, y, color);
			break;
		case 'd' : case 'D':
			draw_D(fb, x, y, color);
			break;
		case 'e' : case 'E':
			draw_E(fb, x, y, color);
			break;
		case 'f' : case 'F':
			draw_F(fb, x, y, color);
			break;
		case 'g' : case 'G':
			draw_G(fb, x, y, color);
			break;
		case 'h' : case 'H':
			draw_H(fb, x, y, color);
			break;
		case 'i' : case 'I':
			draw_I(fb, x, y, color);
			break;
		case 'j' : case 'J':
			draw_J(fb, x, y, color);
			break;
		case 'l' : case 'L':
			draw_L(fb, x, y, color);
			break;
		case 'm' : case 'M':
			draw_M(fb, x, y, color);
			break;
		case 'o' : case 'O':
			draw_O(fb, x, y, color);
			break;
		case 'n' : case 'N':
			draw_N(fb, x, y, color);
			break;
		case 'k' : case 'K':
			draw_K(fb, x, y, color);
			break;
		case 'p' : case 'P':
			draw_P(fb, x, y, color);
			break;
		case 'q' : case 'Q':
			draw_Q(fb, x, y, color);
			break;
		case 'r' : case 'R':
			draw_R(fb, x, y, color);
			break;
		case 's' : case 'S':
			draw_S(fb, x, y, color);
			break;
		case 't' : case 'T':
			draw_T(fb, x, y, color);
			break;
		case 'u' : case 'U':
			draw_U(fb, x, y, color);
			break;
		case 'v' : case 'V':
			draw_V(fb, x, y, color);
			break;
		case 'w' : case 'W':
			draw_W(fb, x, y, color);
			break;
		case 'x' : case 'X':
			draw_X(fb, x, y, color);
			break;
		case 'y' : case 'Y':
			draw_Y(fb, x, y, color);
			break;
		case 'z' : case 'Z':
			draw_Z(fb, x, y, color);
			break;
		case '0':
			draw_0(fb, x, y, color);
			break;
		case '1':
			draw_1(fb, x, y, color);
			break;
		case '`':
			draw_full_box(fb, x, y, color);
			break;
		default:
			draw_non_char(fb, x, y, color);
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
 
 
void draw_char_pipe_left(int fb, int x, int y, int color){
	for (int row = 0; row < CHAR_H; ++row) {
		hal_io_video_putpixel(fb, x, y + row, color);
	}
}

void draw_char_pipe_right(int fb, int x, int y, int color){
	x-=1;
	for (int row = 0; row < CHAR_H; ++row) {
		hal_io_video_putpixel(fb, x + CHAR_W, y + row, color);
	}
}

void draw_char_pipe_mid(int fb, int x, int y, int color){
	for (int row = 0; row <= CHAR_H; ++row) {
		hal_io_video_putpixel(fb, x + (CHAR_W/2)-1, y + row, color);
	}
}

void draw_char_pipe_half(int fb, int x, int y, int color){
	for (int row = CHAR_H; row >= CHAR_H/2; --row) {
		hal_io_video_putpixel(fb, x + (CHAR_W/2)-1, y + row, color);
	}
}

void draw_char_bar_top(int fb, int x, int y, int color){
	for (int offset = 0; offset <= CHAR_W; offset+=2){
		hal_io_video_putpixel(fb, x + offset, y , color);
	}
}

void draw_char_bar_btm(int fb, int x, int y, int color){
	for (int offset = 0; offset <= CHAR_W; offset+=2){
		hal_io_video_putpixel(fb, x + offset, y + CHAR_H, color);
	}
}

void draw_char_bar_mid(int fb, int x, int y, int color){
	for (int offset = 0; offset <= CHAR_W; offset+=2){
		hal_io_video_putpixel(fb, x + offset, y + CHAR_H/2, color);
	}
}

void draw_char_left_bar(int fb, int x, int y, int color){
	for (int offset = 0; offset <= CHAR_W/2; offset+=2){
		hal_io_video_putpixel(fb, x + offset, y + CHAR_H/2, color);
	}
}

void draw_char_right_bar(int fb, int x, int y, int color){
	for (int offset = CHAR_H/2; offset <= CHAR_W; offset+=2){
		hal_io_video_putpixel(fb, x + offset, y + CHAR_H/2, color);
	}
}

void draw_char_diag(int fb, int x, int y, int color, BOOL invert){
	if(invert){
		draw_char_top_left_diag(fb, x, y, color);
		draw_char_btm_right_diag(fb, x, y, color);
	} else {
		draw_char_top_right_diag(fb, x, y, color);
		draw_char_btm_left_diag(fb, x, y, color);
	}
}

void draw_char_top_right_diag(int fb, int x, int y, int color) {
	for(int offset = CHAR_H/2, row = (CHAR_W/2)+1; row > 0; offset+=2){
		hal_io_video_putpixel(fb, x + offset, y + --row, color);
		hal_io_video_putpixel(fb, x + offset, y + --row, color);
	}	
}

void draw_char_top_left_diag(int fb, int x, int y, int color) {
	for(int offset = 0, row = 0; row < CHAR_H/2; offset+=2){
		hal_io_video_putpixel(fb, x + offset, y + ++row, color);
		hal_io_video_putpixel(fb, x + offset, y + ++row, color);
	}		
}

void draw_char_btm_left_diag(int fb, int x, int y, int color) {
	for(int offset = 0, row = CHAR_H; row > CHAR_H/2; offset+=2){
		hal_io_video_putpixel(fb, x + offset, y + --row, color);
		hal_io_video_putpixel(fb, x + offset, y + --row, color);
	}
}

void draw_char_btm_right_diag(int fb,int x, int y, int color) {
	for(int offset = (CHAR_W/2)-1, row = (CHAR_W/2)-1; offset < CHAR_W; offset+=2){
		hal_io_video_putpixel(fb, x + offset, y + ++row, color);
		hal_io_video_putpixel(fb, x + offset, y + ++row, color);
	}
}

void draw_non_char(int fb,int x, int y, int color){
	for(int offsetX = 0; offsetX <= CHAR_W; offsetX+=2){
		for(int offsetY = 0; offsetY <= CHAR_H; ++offsetY){
			hal_io_video_putpixel(fb, x + offsetX, y + offsetY, color);
		}
	}
}

void draw_full_box(int fb,int x, int y, int color){
	for(int offsetX = -(CHAR_W/2); offsetX <= CHAR_W + (CHAR_W/2); offsetX+=2){
		for(int offsetY = -2; offsetY <= CHAR_H + 2; ++offsetY){
			hal_io_video_putpixel(fb, x + offsetX, y + offsetY, color);
		}
	}
}

void draw_0(int fb,int x, int y, int color) {
	draw_O(fb, x, y, color);
	draw_char_diag(fb, x, y, color, BOOL_F);
}

void draw_1(int fb,int x, int y, int color) {
	draw_char_pipe_mid(fb, x, y, color);
}

void draw_A(int fb,int x, int y, int color){
	draw_char_pipe_left(fb, x, y, color);
	draw_char_pipe_right(fb, x, y, color);
	draw_char_bar_mid(fb, x, y, color);
	draw_char_bar_top(fb, x, y, color);
}

void draw_B(int fb,int x, int y, int color){
	draw_char_pipe_left(fb, x, y, color);
	draw_char_left_bar(fb, x, y, color);
	draw_char_bar_top(fb, x, y, color);
	draw_char_bar_btm(fb, x, y, color);
	draw_char_top_right_diag(fb, x, y, color);
	draw_char_btm_right_diag(fb, x, y, color);
}

void draw_C(int fb, int x, int y, int color){
	draw_L(fb, x,y,color);
	draw_char_bar_top(fb, x, y, color);
}

void draw_D(int fb, int x, int y, int color){
	draw_char_pipe_left(fb,x,y,color);
	draw_char_btm_left_diag(fb,x,y,color);
	draw_char_top_left_diag(fb,x,y,color);
}

void draw_E(int fb,int x, int y, int color){
	draw_F(fb,x,y,color);
	draw_char_bar_btm(fb,x,y,color);
}

void draw_F(int fb,int x, int y, int color){
	draw_char_pipe_left(fb,x,y,color);
	draw_char_bar_top(fb,x,y,color);
	draw_char_left_bar(fb,x,y,color);
}

void draw_G(int fb, int x, int y, int color){
	draw_C(fb,x,y,color);
	draw_char_btm_right_diag(fb,x,y,color);
}

void draw_H(int fb,int x, int y, int color){
	draw_char_pipe_left(fb,x, y, color);
	draw_char_bar_mid(fb,x, y, color);
	draw_char_pipe_right(fb,x, y, color);
}

void draw_I(int fb, int x, int y, int color){
	draw_char_pipe_mid(fb, x, y, color);
	draw_char_bar_top(fb, x,y,color);
	draw_char_bar_btm(fb, x,y,color);
}

void draw_J(int fb, int x, int y, int color){
	draw_char_pipe_right(fb, x, y, color);
	draw_char_bar_top(fb,x,y,color);
	draw_char_bar_btm(fb,x,y,color);
	draw_char_btm_left_diag(fb,x,y,color);
}

void draw_K(int fb,int x, int y, int color){
	draw_char_pipe_left(fb, x, y, color);
	draw_char_top_right_diag(fb, x, y, color);
	draw_char_btm_right_diag(fb, x, y, color);
}

void draw_L(int fb,int x, int y, int color){
	draw_char_pipe_left(fb, x, y, color);
	draw_char_bar_btm(fb,x,y,color);
}

void draw_M(int fb,int x, int y, int color){
	draw_char_pipe_left(fb, x,y,color);
	draw_char_pipe_right(fb, x,y,color);
	draw_char_top_left_diag(fb,x,y,color);
	draw_char_top_right_diag(fb,x,y,color);
}

void draw_O(int fb,int x, int y, int color){
	draw_C(fb,x,y,color);
	draw_char_pipe_right(fb,x, y, color);
}

void draw_N(int fb,int x, int y, int color){
	draw_char_pipe_left(fb, x, y, color);
	draw_char_pipe_right(fb, x, y, color);
	draw_char_diag(fb, x, y, color, BOOL_T);
}

void draw_P(int fb,int x, int y, int color){
	draw_F(fb, x,y,color);
	draw_char_top_right_diag(fb, x,y,color);
}

void draw_Q(int fb,int x, int y, int color){
	draw_O(fb,x,y,color);
	draw_char_btm_right_diag(fb,x,y,color);
}

void draw_R(int fb,int x, int y, int color){
	draw_P(fb, x,y,color);
	draw_char_btm_right_diag(fb, x,y,color);
}

void draw_S(int fb,int x, int y, int color){
	draw_char_bar_top(fb, x, y, color);
	draw_char_bar_btm(fb, x, y, color);
	draw_char_diag(fb, x, y, color, BOOL_T);
}

void draw_T(int fb,int x, int y, int color){
	draw_char_pipe_mid(fb, x,y,color);
	draw_char_bar_top(fb, x,y,color);
}

void draw_U(int fb,int x, int y, int color){
	draw_char_pipe_left(fb, x,y,color);
	draw_char_pipe_right(fb, x,y,color);
	draw_char_bar_btm(fb, x,y,color);
}

void draw_V(int fb,int x, int y, int color){
	draw_char_pipe_left(fb, x,y,color);
	draw_char_diag(fb, x,y,color,BOOL_F);
}

void draw_W(int fb,int x, int y, int color){
	draw_char_pipe_left(fb, x,y,color);
	draw_char_pipe_right(fb, x,y,color);
	draw_char_btm_left_diag(fb, x,y,color);
	draw_char_btm_right_diag(fb, x,y,color);
}

void draw_X(int fb,int x, int y, int color){
	draw_char_diag(fb, x,y,color,BOOL_T);
	draw_char_diag(fb, x,y,color,BOOL_F);
}

void draw_Y(int fb,int x, int y, int color){
	draw_char_pipe_half(fb, x,y,color);
	draw_char_top_left_diag(fb, x,y,color);
	draw_char_top_right_diag(fb, x,y,color);
}

void draw_Z(int fb,int x, int y, int color){
	draw_char_bar_top(fb, x, y, color);
	draw_char_bar_btm(fb, x, y, color);
	draw_char_bar_mid(fb, x, y, color);
	draw_char_diag(fb, x, y, color, BOOL_F);
}

/*
 * Draws a character at the current postiton of the cursor, 
 * and then moves the cursor forward on spot 
*/
void cursor_drawc(int fb, int character, int color){
	if((cursor_x + CHAR_W + (CHAR_W / 2) >= SCRN_W)) { //if the next char would draw off the right edge of the screen 
		cursor_newln();
	}
	hal_io_video_putc(fb, cursor_x, cursor_y, color, character);
	if(character != '\n' && character != '\r')
		cursor_forwd();
}

void clear_char(int fb, int x, int y){
	draw_non_char(fb, x, y, BLK);
}

void cursor_bkwrd(){
	if(cursor_x < (CHAR_W + CHAR_W/2)){
		return; //TODO:Bring the cursor to the end of the prev line
	}
	cursor_x -= CHAR_W + CHAR_W/2;
}

void cursor_bkspc(int fb, int x, int y){
	cursor_bkwrd();
	clear_char(fb, cursor_x, cursor_y);
	cursor_bkwrd();
}

void cursor_forwd(){
	cursor_x += CHAR_W + (CHAR_W / 2);
	if(cursor_x >= SCRN_W)
		cursor_newln();
}

void cursor_newln(){
	cursor_x = CHAR_W + (CHAR_W / 2);
	cursor_y += (CHAR_H + 2);
}

void draw_os_name_to_video(int fb, int color){

	cursor_newln();
	cursor_drawc(fb, 'C', color);
	cursor_drawc(fb, 'H', color);
	cursor_drawc(fb, 'O', color);
	cursor_drawc(fb, 'N', color);
	cursor_drawc(fb, 'K', color);
	cursor_forwd();
	cursor_drawc(fb, 'O', color);
	cursor_drawc(fb, 'S', color);
	cursor_newln();
}

void cursor_reset(){
	cursor_x = CHAR_W + CHAR_W/2;
	cursor_y = CHAR_H/2;
}