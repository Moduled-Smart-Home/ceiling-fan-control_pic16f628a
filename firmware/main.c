/*
 * File:   newmain.c
 * Author: conrado
 *
 * Created on December 12, 2021, 12:20 AM
 */

// PIC16F628A Configuration Bit Settings

// 'C' source line config statements

// CONFIG
#pragma config FOSC = HS    // Oscillator Selection bits (HS oscillator: High-speed crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = OFF      // RA5/MCLR/VPP Pin Function Select bit (RA5/MCLR/VPP pin function is digital input, MCLR internally tied to VDD)
#pragma config BOREN = OFF      // Brown-out Detect Enable bit (BOD disabled)
#pragma config LVP = OFF         // Low-Voltage Programming Enable bit (RB4/PGM pin has PGM function, low-voltage programming enabled)
#pragma config CPD = OFF        // Data EE Memory Code Protection bit (Data memory code protection off)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h> 
#define _XTAL_FREQ 20000000

#include "i2c.h"

PORTAbits_t LATCHA;
PORTBbits_t LATCHB, LATCHB_prev;

#define LED LATCHA.RA3
#define TRIAC_1 LATCHA.RA0

//#define TIME_TRIGG_us 6500
#define TIME_HOLD_us 1000
#define TIME_TRIGG_MAX_us 7333
#define TIME_TRIGG_POT1_us 6500
#define TIME_TRIGG_POT2_us 4000
#define TIME_HOLD_us 1000

#define TIMER_SETUP_POT1 65536 - (TIME_TRIGG_POT1_us/1.6)
#define TIMER_SETUP_POT2 65536 - (TIME_TRIGG_POT2_us/1.6)
#define TIMER_SETUP_HOLD 65536 - (TIME_HOLD_us/1.6)
//#define TIMER_SETUP_POT1 65536 -(unsigned short) (TIME_TRIGG_POT1_us / ((long)(1000000 * 8) * (double)(1/(_XTAL_FREQ/4))))
//#define TIMER_SETUP_POT2 65536 - (unsigned short) TIME_TRIGG_POT2_us / ((1/(_XTAL_FREQ/4)) * 1000000 * 8) 
//#define TIMER_SETUP_HOLD 65536 - (unsigned short) TIME_HOLD_us / ((1/(_XTAL_FREQ/4)) * 1000000 * 8) 

typedef enum {ZERO_CROSSED, TRIGGED, READY} triac_state_t;
triac_state_t triac_1;

signed char POT_LEVEL = -1;

void toggleLED(void) {
    LED = !LED;
    PORTAbits = LATCHA;
}

void setLED(unsigned char value) {
    LED = value;
    PORTAbits = LATCHA;
}

void setTriac1(unsigned char value) {
    TRIAC_1 = value;
    PORTAbits = LATCHA;
}

void  __interrupt(high_priority) myHighIsr(void) {
    if (RBIE && RBIF) {
        INTE = 0;
        LATCHB = PORTBbits;
        i2cInterruptHandle(LATCHB.RB4, LATCHB.RB5);
        RBIF=0;
        INTE = 1;
    }
    
    if (INTE && INTF) {
        INTF=0;
        
        if (POT_LEVEL > 0) {
            if (POT_LEVEL == 1) {
            TMR1 = TIMER_SETUP_POT1;
            }
            else if (POT_LEVEL == 2) {
                TMR1 = TIMER_SETUP_POT2;
            }
        
            triac_1 = ZERO_CROSSED;
            TMR1IE = 1; //enable TMR1 interrupt
            TMR1ON = 1; //start TMR1 count
        }
    }
    
    if (TMR1IE && TMR1IF) {
        TMR1IF = 0;
        
        if (triac_1 == ZERO_CROSSED) {
            setTriac1(1);
            TMR1 = TIMER_SETUP_HOLD;
            triac_1 = TRIGGED;
        }
        else if (triac_1 == TRIGGED) {
            setTriac1(0);
            triac_1 = READY;
            TMR1ON = 0; //stop TMR1
        }
    }
}



void main(void) {
     // XTAL Oscilator
    TRISAbits.TRISA6 = 1;
    TRISAbits.TRISA7 = 1;
    
    // Configuration LED e MOC
    PCONbits.OSCF=1;
    TRISAbits.TRISA3 = 0; //Output - LED
    TRISAbits.TRISA0 = 0; //Output - MOC 
    
    // Configuration Zero Crossing Interrupt
    TRISBbits.TRISB0 = 1;
    INTCONbits.GIE=1;
    INTCONbits.INTE=1;
    OPTION_REGbits.INTEDG=1;
    
    LATCHA = PORTAbits; // Initialize Latch with actual port values
    
    //TIMER1 config
    TMR1ON = 0; //ensure tmr1 is stopped
    TMR1CS = 0; //timer mode
    PEIE = 1; //enable peripheral interrupt
    
    T1CKPS0 = 1; //prescale 1:8
    T1CKPS1 = 1;
    
    setSlaveAddress8bits(0xAA);
    
    DATA_RECEIVED = 0;
    
    triac_1 = READY;
    
    int ret;
    ret = startI2C();
    
    setLED(0);
    
    while(1) {      

//        if (i2c_state == REC_DATA) {
//            toggleLED();
//        }
        
        if (DATA_RECEIVED == 1) {
            
            
            if (cmd == LED_OFF) {
                setLED(0);
            }
            else if (cmd == LED_ON) {
                setLED(1);
            }
            else if (cmd == POT1) {
                POT_LEVEL = 1;
            }
            else if (cmd == POT2) {
                POT_LEVEL = 2;
            }
            
            DATA_RECEIVED = 0;
        }
        
    }
    
    
    return;
}
