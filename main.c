//******************************************************************************
// MSP430FR4133 launchpad test project
//
// author: Ondrej Hejda
// date:   26.11.2015
//
// resources:
//
//   https://github.com/sgergo/project4133 .. nice project, thanks
//   TI example sources .. MSP-EXP430FR4133_Software_Examples_windows.zip
//
// hardware: MSP430FR4133 (launchpad)
//
//                MSP430FR4133
//             -----------------
//         /|\|              XIN|----  -----------
//          | |                 |     | 32.768kHz |---
//          --|RST          XOUT|----  -----------    |
//            |                 |                    ---
//            |                 |
//            |           P1.1,0|---> UART (debug output 9.6kBaud)
//            |                 |
//            |             P1.0|-X-> RED LED (active high)
//            |             P4.0|---> GREEN LED (active high)
//            |                 |
//            |             P1.2|<--- BTN1 --
//            |             P2.6|<--- BTN2 --|--
//            |                 |               |
//            |                 |              ---
//            |                 |
//            |                 |         -------------------
//            | L0..31, L36..39 |--/36/--| LCD  108seg. 4com |
//            |                 |        |                   |
//            |                 |        |    8.8:8.8:8.8    |
//             -----------------          -------------------
//
//
//******************************************************************************

// include section
#include <msp430fr4133.h>
#include <stdbool.h>

#include "board.h"
#include "lcd.h"

#define PWM_MAX 1024

int hour=0;
int minute=0;
int second=0;
bool timeset = false;

int sunrise_hour = 6;
int sunrise_minute = 0;

int dusk_hour = 20;
int dusk_minute = 0;

void rtc_init(void)
{
	P4SEL0 |= BIT1 | BIT2; // set XT1 pin as second function
	do {
        CSCTL7 &= ~(XT1OFFG | DCOFFG); // clear XT1 and DCO fault flag
        SFRIFG1 &= ~OFIFG;
	} while (SFRIFG1 & OFIFG); // test oscillator fault flag
	RTCMOD = 16-1;
	RTCCTL = RTCSS__XT1CLK | RTCSR | RTCPS__1024 | RTCIE;
}

void uart_init(void)
{
    // Configure UART pins
    P1SEL0 |= BIT0 | BIT1;                    // set 2-UART pin as second function

    // Configure UART
    UCA0CTLW0 |= UCSWRST;                      // Put eUSCI in reset
    UCA0CTLW0 |= UCSSEL__SMCLK;
    // Baud Rate calculation
    UCA0BR0 = 52;                              // 8000000/16/115200 = 4.34
    UCA0BR1 = 0;
    UCA0MCTLW = 0x4900 | UCOS16 |  UCBRF_1;   // 8000000/16/115200 - INT(8000000/16/115200)=0.34
                                              // UCBRSx value = 0x55 (See UG)
    UCA0CTLW0 &= ~UCSWRST;                    // Initialize eUSCI
    UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
}

int c2d(uint8_t c)
{
    if ((c>='0') && (c<='9'))
        return (c-'0');
    return -1;
}

void uart_buffer(uint8_t c)
{
    static int s = 0;
    static int d=0,sec=0,hr=0,min=0;
    static uint8_t cmd = 0;

    d = c2d(c);
    if (d<0) s=0;

    switch (s) {
    case 0: // wait for start letter
        if ((c=='t') || (c=='s') || (c=='d')){
            cmd = c;
            s++;
        }
        break;
    case 1:
        hr = d*10;
        s++;
        break;
    case 2:
        hr += d;
        s++;
        break;
    case 3:
        min = d*10;
        s++;
        break;
    case 4:
        min += d;
        if (cmd=='s') {
            sunrise_hour = hr;
            sunrise_minute = min;
            s = 0;
        }
        else if (cmd=='d') {
            dusk_hour = hr;
            dusk_minute = min;
        }
        else s++;
        break;
    case 5:
        sec = d*10;
        s++;
        break;
    case 6:
        sec += d;
        hour = hr;
        minute = min;
        second = sec;
        timeset = true;
        s=0;
        break;
    }
}

void pwm_init(void)
{
    P1DIR  |= BIT7;         // P1.7 output
    P1SEL0 |= BIT7;         // P1.7 options select

    TA0CCR0 = PWM_MAX;      // PWM Period/2 (approx. 500Hz)
    TA0CCTL1 = OUTMOD_6;    // TACCR1 toggle/set
    TA0CCR1 = PWM_MAX;      // TACCR1 PWM duty cycle

    TA0CTL = TASSEL__SMCLK | ID__8 | MC__UPDOWN;  // SMCLK, fclk/8, up-down mode
}

void pwm_set(uint16_t val)
{
    TA0CCR1 = val;
}

// main program body
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// Stop WDT

	board_init(); // init dco and leds
	rtc_init();
	pwm_init();
	uart_init();
	lcd_init();

	static uint16_t pwm = PWM_MAX;
	static uint16_t desired_pwm = PWM_MAX;

    PM5CTL0 &= ~LOCKLPM5;

	while(1)
	{
        showChar('0'+hour/10,pos1);
        showChar('0'+hour%10,pos2);
        showChar('0'+minute/10,pos3);
        showChar('0'+minute%10,pos4);
        showChar('0'+second/10,pos5);
        showChar('0'+second%10,pos6);

	    dispWordOR(LCD_COLON,pos2);
	    dispWordOR(LCD_COLON,pos4);

	    __bis_SR_register(LPM3_bits | GIE);

        if (timeset) {
            showChar('0'+hour%10,pos2);
            showChar('0'+minute%10,pos4);

            second++;
            if (second>=60) {
                second=0;
                minute++;
                if (minute>=60) {
                    minute=0;
                    hour++;
                    if (hour>=24) {
                        hour=0;
                    }
                }
            }

            if ((hour==sunrise_hour) && (minute==sunrise_minute) && (second==0))
                desired_pwm = 0;

            if ((hour==dusk_hour) && (minute==dusk_minute) && (second==0))
                desired_pwm = PWM_MAX;

            if (pwm>desired_pwm) pwm--;
            if (pwm<desired_pwm) pwm++;
            pwm_set(pwm);
        }
        else {
            showChar(' ',pos1);
            showChar(' ',pos2);
            showChar(' ',pos3);
            showChar(' ',pos4);
            showChar(' ',pos5);
            showChar(' ',pos6);
        }

	    __bis_SR_register(LPM3_bits | GIE);
	}

	return -1;
}

void __attribute__ ((interrupt(RTC_VECTOR))) RTC_ISR (void)
{
    //switch(__even_in_range(RTCIV,RTCIV_RTCIF))
    switch(RTCIV)
    {
        case  RTCIV_NONE:   break;          // No interrupt
        case  RTCIV_RTCIF:                  // RTC Overflow
            __bic_SR_register_on_exit(LPM3_bits);
            break;
        default: break;
    }
}

void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
{
    //switch(__even_in_range(UCA0IV,USCI_UART_UCTXCPTIFG))
    switch(UCA0IV)
    {
        case USCI_NONE: break;
        case USCI_UART_UCRXIFG: // receive interrupt
            UCA0IFG &=~ UCRXIFG;    // Clear interrupt
            uint8_t c = UCA0RXBUF;  // Clear buffer
            UCA0TXBUF = c;
            uart_buffer(c);
            break;
        case USCI_UART_UCTXIFG:     // transmit interrupt
            UCA0IFG &=~ UCTXIFG;    // Clear interrupt
            break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
    }
}
