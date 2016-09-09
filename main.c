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

#include "board.h"
#include "lcd.h"

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

void pwm_init(void)
{
    P1DIR |= BIT6 | BIT7;                      // P1.7 output
    P1SEL0 |= BIT6 | BIT7;                     // P1.7 options select

    TA0CCR0 = 128;                             // PWM Period/2
    TA0CCTL1 = OUTMOD_6;                       // TACCR1 toggle/set
    TA0CCR1 = 32;                              // TACCR1 PWM duty cycle
    TA0CCTL2 = OUTMOD_6;                       // TACCR2 toggle/set
    TA0CCR2 = 96;                              // TACCR2 PWM duty cycle

    TA0CTL = TASSEL_1 | MC_3;                  // ACLK, up-down mode
}

// main program body
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// Stop WDT

	board_init(); // init dco and leds
	rtc_init();
	//pwm_init();
	uart_init();
	lcd_init();

    PM5CTL0 &= ~LOCKLPM5;

    int hour=0;
    int minute=0;
    int second=0;

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

        LED_GREEN_SWAP();

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
            while(!(UCA0IFG&UCTXIFG));
            UCA0IFG &=~ UCRXIFG;            // Clear interrupt
            UCA0TXBUF = UCA0RXBUF;     // Clear buffer
            break;
        case USCI_UART_UCTXIFG: // transmit interrupt
            UCA0IFG &=~ UCTXIFG;            // Clear interrupt
            break;
        case USCI_UART_UCSTTIFG: break;
        case USCI_UART_UCTXCPTIFG: break;
    }
}
