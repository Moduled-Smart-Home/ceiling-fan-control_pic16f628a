
#include "i2c.h"

void i2cPinsConfig(void) {
    T_SCL = 1; // as input - high impedance
    T_SDA = 1; // as input - high impedance
    P_SCL = 0; // pull to down when set to output
    P_SDA = 0; // pull to down when set to output
    
    INTCONbits.RBIF=0; //Clear interrupt flag
    INTCONbits.RBIE=1; //Enable change state PORTB<7:4> interrupt
}

void i2cInterruptHandle(unsigned char SCL, unsigned char SDA) {
    SCL_actual = SCL;
    SDA_actual = SDA;
    
    if (SCL_prev != SCL_actual || SDA_prev != SDA_actual) {
        
        switch(i2c_state) {
            
            case UNDEFINED:
                if (idleCheck == 1 && !(SCL_actual == 1 && SDA_actual == 1)) {
                    idleCheck = 0;
                }
                break;
                
            case IDLE:
                if (SCL_actual == 1 && SDA_actual == 0) {
                    i2c_state = REC_ADDR;
                    address_rec = 0;
                    counter_rec = 0;
                }
                break;
                
            case REC_ADDR:
                if (SCL_actual == 1 && SCL_prev == 0) {
                    if (counter_rec < 7) {
                        address_rec <<= 1;
                        address_rec |= SDA_actual;
                        counter_rec++;
                    } else if(counter_rec == 7) {
                        counter_rec=0;
                        wr = SDA_actual;
                        
                        if (address_rec == SLAVE_ADDRESS_8bits>>1) {
                            i2c_state = ACK_DOWN;
                        }
                        else {
                            //i2c_state = WAIT_FOR_IDLE;
                            i2c_state = IDLE;
                        }
                        
                    }
                }
                break;
                
            case ACK_DOWN:
                if (SCL_actual == 0 && SCL_prev == 1) {
                    T_SDA = 0;
                }
                else if (SCL_actual == 1 && SCL_prev == 0) {
                    i2c_state = ACK_UP;
                }
                break;
                
            case ACK_UP:
                if (SCL_actual == 0 && SCL_prev == 1) {
                    if (wr == 0) {
                        T_SDA = 1;
                        i2c_state = REC_DATA;
                        data_rec=0;
                        counter_rec=0;
                    }
                    else {
                        i2c_state = SEND_DATA;
                        T_SDA = 0;
                        data_send = data;
                        counter_send=1;
                        if ((data_send & 0b10000000) == 0) {
                            P_SDA = 0;
                        } else {
                            P_SDA = 1;
                        }
                        data_send <<= 1;
                    }
                }
                break;
                
            case REC_DATA:
                if (SCL_actual == 1 && SCL_prev == 0) {
                    counter_rec++;
                    if (counter_rec <= 8) {
                        data_rec <<= 1;
                        data_rec |= SDA_actual;
                        
                    } 
                    
                    if (counter_rec == 8) {
                        counter_rec=0;
                        data = data_rec;
                        cmd = data_rec;
                        DATA_RECEIVED = 1;
                        i2c_state = ACK_DOWN1;
                    }
                }
                break;
                
            case SEND_DATA:
                if (SCL_actual == 0 && SCL_prev == 1) {
                    counter_send++;
                    if (counter_send <= 8) {
                        if ((data_send & 0b10000000) == 0) {
                            P_SDA = 0;
                        } else {
                            P_SDA = 1;
                        }
                        data_send <<= 1;
                    }
                    
                    if (counter_send > 8) {
                        counter_send=0;
                        T_SDA = 1;
                        i2c_state = WAIT_FOR_NAK;
                    }
                }
                break;
                
            case ACK_DOWN1:
                if (SCL_actual == 0 && SCL_prev == 1) {
                    T_SDA = 0;
                }
                else if (SCL_actual == 1 && SCL_prev == 0) {
                    
                    i2c_state = ACK_UP1;
                }
                break;
                
            case ACK_UP1:
                if (SCL_actual == 0 && SCL_prev == 1) {
                    T_SDA = 1;
                    i2c_state = WAIT_START_STOP;
                }
                break;
                
            case WAIT_FOR_NAK:
                if (SCL_actual == 1 && SCL_prev == 0 && SDA_actual == 1) {
                    i2c_state = WAIT_START_STOP;
                }
                break;
                
            case WAIT_START_STOP:
                if (SCL_actual == 1 && SCL_prev == 1) {
                    if (SDA_actual == 0 && SDA_prev == 1) {
                        // got STARTED signal
                        i2c_state = REC_ADDR;
                    }
                    else if (SDA_actual == 1 && SDA_prev == 0) {
                        // got STOP signal
                        //i2c_state = WAIT_FOR_IDLE;
                        i2c_state = IDLE;
                    }
                }
                break;
        }
        
        SCL_prev = SCL_actual;
        SDA_prev = SDA_actual;
    }
}

void waitForIdle(void) {
    i2c_state = UNDEFINED;
    idleCheck = 0;
    
    P_SCL = 0; // pull to down when set to output
    P_SDA = 0; // pull to down when set to output
    
    
    while(1) {
        if (P_SDA == 1 && P_SCL == 1) {
            idleCheck = 1;
            __delay_us(1000); //If states change, interrupt will changes idleCheck
            if (idleCheck == 1){
                i2c_state = IDLE;
                break;
            } else {
                continue;
            }
        }
    }
    return;
}

void setSlaveAddress8bits(unsigned char address) {
    SLAVE_ADDRESS_8bits = address;
}

int startI2C(void) {

    SCL_prev = P_SCL;
    SDA_prev = P_SDA;
    idleCheck = 0;
    
    i2c_state = UNDEFINED;
    
    i2cPinsConfig();
    
    waitForIdle();
    
    return 0;
}