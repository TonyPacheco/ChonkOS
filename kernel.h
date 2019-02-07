#define BOOL short
#define BOOL_F 0
#define BOOL_T 1


/*
*	From 
*	https://wiki.osdev.org/Raspberry_Pi_Bare_Bones#Building_a_Cross-Compiler
*
*/
enum
{
    // The GPIO registers base address.
    GPIO_BASE = 0x3F200000, // for raspi2 & 3, 0x20200000 for raspi1
 
    // The offsets for reach register.
 
    // Controls actuation of pull up/down to ALL GPIO pins.
    GPPUD = (GPIO_BASE + 0x94),
 
    // Controls actuation of pull up/down for specific GPIO pin.
    GPPUDCLK0 = (GPIO_BASE + 0x98),
 
    // The base address for UART.
    UART0_BASE = 0x3F201000, // for raspi2 & 3, 0x20201000 for raspi1
 
    // The offsets for reach register for the UART.
    UART0_DR     = (UART0_BASE + 0x00),
    UART0_RSRECR = (UART0_BASE + 0x04),
    UART0_FR     = (UART0_BASE + 0x18),
    UART0_ILPR   = (UART0_BASE + 0x20),
    UART0_IBRD   = (UART0_BASE + 0x24),
    UART0_FBRD   = (UART0_BASE + 0x28),
    UART0_LCRH   = (UART0_BASE + 0x2C),
    UART0_CR     = (UART0_BASE + 0x30),
    UART0_IFLS   = (UART0_BASE + 0x34),
    UART0_IMSC   = (UART0_BASE + 0x38),
    UART0_RIS    = (UART0_BASE + 0x3C),
    UART0_MIS    = (UART0_BASE + 0x40),
    UART0_ICR    = (UART0_BASE + 0x44),
    UART0_DMACR  = (UART0_BASE + 0x48),
    UART0_ITCR   = (UART0_BASE + 0x80),
    UART0_ITIP   = (UART0_BASE + 0x84),
    UART0_ITOP   = (UART0_BASE + 0x88),
    UART0_TDR    = (UART0_BASE + 0x8C),
};


//CURSOR INPUT
void cursor_drawc(int, int, int);
void cursor_forwd(void);
void cursor_bkwrd(void);
void cursor_bkspc(int,int,int);
void cursor_newln(void);
void cursor_reset(void);
void clear_char(int,int,int);

//CHARACTER PARTS
void draw_char_bar_top(int,int,int,int);
void draw_char_bar_btm(int,int,int,int);
void draw_char_bar_mid(int,int,int,int);
void draw_char_left_bar(int,int,int,int);
void draw_char_right_bar(int,int,int,int);
void draw_char_pipe_left(int,int,int,int);
void draw_char_pipe_right(int,int,int,int);
void draw_char_pipe_mid(int,int,int,int);
void draw_char_pipe_half(int,int,int,int);
void draw_char_diag(int,int,int,int,BOOL);
void draw_char_top_left_diag(int,int,int,int);
void draw_char_top_right_diag(int,int,int,int);
void draw_char_btm_left_diag(int,int,int,int);
void draw_char_btm_right_diag(int,int,int,int);

//CHARACTERS
void draw_non_char(int,int,int,int);
void draw_full_box(int,int,int,int);
void draw_0(int,int,int,int);
void draw_1(int,int,int,int);
void draw_A(int,int,int,int);
void draw_B(int,int,int,int);
void draw_C(int,int,int,int);
void draw_D(int,int,int,int);
void draw_E(int,int,int,int);
void draw_F(int,int,int,int);
void draw_G(int,int,int,int);
void draw_H(int,int,int,int);
void draw_I(int,int,int,int);
void draw_J(int,int,int,int);
void draw_K(int,int,int,int);
void draw_L(int,int,int,int);
void draw_M(int,int,int,int);
void draw_N(int,int,int,int);
void draw_O(int,int,int,int);
void draw_P(int,int,int,int);
void draw_Q(int,int,int,int);
void draw_R(int,int,int,int);
void draw_S(int,int,int,int);
void draw_T(int,int,int,int);
void draw_U(int,int,int,int);
void draw_V(int,int,int,int);
void draw_W(int,int,int,int);
void draw_X(int,int,int,int);
void draw_Y(int,int,int,int);
void draw_Z(int,int,int,int);