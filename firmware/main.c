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
#define Q LATCHA.RA0

#define TIME_TRIGG_us 6500
#define TIME_HOLD_us 1000


int count=0;

void toggleLED(void) {
    LED = !LED;
    PORTAbits = LATCHA;
}

void setLED(unsigned char value) {
    LED = value;
    PORTAbits = LATCHA;
}

void setQ(unsigned char value) {
    Q = value;
    PORTAbits = LATCHA;
}

void  __interrupt(high_priority) myHighIsr(void) {
    if (RBIE && RBIF) {
        
        LATCHB = PORTBbits;
                
        i2cInterruptHandle(LATCHB.RB4, LATCHB.RB5);
        
        RBIF=0;
        
    }
    
    if (INTE && INTF) {
        INTF=0;
        

        __delay_us(TIME_TRIGG_us);
        setQ(1);
        __delay_us(TIME_HOLD_us);
        setQ(0);
        
        count++;
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
    
    setSlaveAddress8bits(0xAA);
    
    //TESTE I2C
    int ret;
    ret = startI2C();
    
    setLED(0);
    
    while(1) {      

        if (i2c_state == REC_ADDR || i2c_state == WAIT_FOR_IDLE) {
            toggleLED();
        }
        
        if (i2c_state == WAIT_FOR_IDLE) {
            waitForIdle();
        }
        
    }
    
    
    return;
}
