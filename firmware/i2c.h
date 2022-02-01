/* 
 * File:   i2c.h
 * Author: conrado
 *
 * Created on 26 de Dezembro de 2021, 12:39
 */

#ifndef I2C_H
#define	I2C_H

#ifdef	__cplusplus
extern "C" {
#endif
    
#include <xc.h>
#define _XTAL_FREQ 20000000

    
// Define I2C ports. SDA and SCL must be on of 4:7 PORTB pins, that are featured
    // with change state interrrupt
    
#define T_SCL TRISBbits.TRISB4
#define T_SDA TRISBbits.TRISB5
#define P_SCL PORTBbits.RB4
#define P_SDA PORTBbits.RB5
    
typedef enum {UNDEFINED, IDLE, REC_ADDR, ACK_DOWN, ACK_UP, REC_DATA, SEND_DATA, ACK_DOWN1, ACK_UP1, WAIT_FOR_NAK, WAIT_START_STOP} I2C_STATE;
I2C_STATE i2c_state = UNDEFINED;



typedef enum  {LED_ON, LED_OFF, FAN_OFF, FAN_ON_DIR1, FAN_ON_DIR2, POT1, POT2} messages_t;
messages_t cmd;

unsigned char SLAVE_ADDRESS_8bits;
unsigned char SCL_actual, SDA_actual, SCL_prev, SDA_prev, idleCheck;
unsigned char address_rec, counter_rec, counter_send, data_rec, data_send, data;

unsigned char DATA_RECEIVED;
enum {W=0, R=1} wr;
    
void i2cPinsConfig(void);

void setSlaveAddress8bits(unsigned char address);

void i2cInterruptHandle(unsigned char SCL, unsigned char SDA);
void waitForIdle(void);

int startI2C(void);

#ifdef	__cplusplus
}
#endif

#endif	/* I2C_H */

