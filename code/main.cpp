//! \file main.cpp
//! \brief Main file for the MotorBoard
//! \author Remy Guyonneau, Baptiste Hamard, Franck Mercier
//! \date 2017 05 29



// for ATMEL studio, not needed with the raspberry pi
#define FOSC            16000       //!< The ATMEGA clock speed
#define F_CPU           16000000UL  //!< The ATMEGA clock speed

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/interrupt.h>

#include "led.h"
#include "pin.h"
#include "m32m1_pwm.h"
#include "motor_dc.h"
#include "spi.h"
#include "counter.h"
#include "pid.h"

#include <string.h> //POUR LES TESTS

#define PI                      3.14159     //!< The PI constant, to handle mrad/s

#define RIGHT_MOTOR             (1)         //!< The right motor value
#define LEFT_MOTOR              (-1)        //!< The left motor value

#define SIDE_MOTOR              RIGHT_MOTOR  //!< To handle left and right moteur
                                            //!  choose between LEFT_MOTOR or RIGHT_MOTOR

#define LED_RED_PORT            PORTB       //!< The port for the red LED
#define LED_RED_PIN             3           //!< The pin for the red LED
#define LED_RED_POL             0           //!< The polarity of the red LED

#define LED_YELLOW_PORT         PORTB       //!< The port for the yellow LED
#define LED_YELLOW_PIN          2           //!< The pin for the yellow LED
#define LED_YELLOW_POL          0           //!< The polarity of the yellow LED

#define ID_MOTORBOARD_DATASPEED 0x35        //!< The polarity of the yellow LED
#define ID_MOTORBOARD_DATAPID   0x37        //!< The polarity of the yellow LED
#define ID_MOTORBOARD_DATACONF  0x36        //!< The polarity of the yellow LED

#define NB_STEPS                1920        //!< Number of tics for a complete wheel turn

#define MAX_WATCH_DOG           100         //!< Time after the motor will stop
                                            //!  if not receiving speed command* 10ms
#define MAX_NB_FLAT             100         //!< Number of 0 value from the sensor before
                                            //!  shutting down the robot (avoid motion after
                                            //!  an emmergency stop for instance)

#define DEFAULT_KP              0.1         //!< default KP for the PID 
#define DEFAULT_KI              0           //!< default KI for the PID
#define DEFAULT_KD              0           //!< default KD for the PID

#define F_MOTOR_TIC2PWM(tic) (SIDE_MOTOR*35*tic) //!< To convert counter value to PWM,
                                                 //!  the values are extracted from experimental tests

//! \fn void initCANBus()
//! \brief Intialize the CAN Bus.
//!
//! Function that initializes the CANBus according to the wiring and the needed speed (500kb/s).
//! It also enable the CAN module interruptions
void initCANBus(){
    // Enable of CAN transmit
    DDRC|=0x80; // Configuration of the standby pin as an output
                // (for a transceiver MCP2562 with the stby pin wired to the uC PC7)
    // Enable the MCP2562 (STANDBY to 0)
    PORTC &= 0x7F; // Activation of the MCP2562
                   // (for a transceiver MCP2562 with the stby pin wired to the uC PC7)

    // Initialization of CAN module
    CANGCON  = (1 << SWRES); // Reset CAN module
    CANGCON  = 0x02; // Enable CAN module

    CANBT1 = 0x06; // Speed Config (500kb/s)
    CANBT2 = 0x04;
    CANBT3 = 0x13;

    CANHPMOB = 0x00; // No priority between the CAN MOB

    // Enable CAN interruptions and especially reception interruptions
    CANGIE |= 0xA0;
}

// POUR LES TESTS
void sendCANdata(uint8_t id, uint8_t dlc, uint8_t* data){
    CANPAGE  = 0x00; //Selection of MOB 0

    CANIDT4 = 0x00; // Config as data (rtr = 0)
    CANIDT3 = 0x00;
    CANIDT2 = (uint8_t)( (id & 0x00F)<< 5 ); // set the identifier
    CANIDT1 = (uint8_t)( id >> 3 );

    for(uint8_t i=0; i< dlc; i++){
        CANMSG = data[dlc-1-i];
        CANPAGE = 0x00 + 1 + i;
    }

    CANCDMOB = 0x40 + (0x0F & dlc);// send the message using the MOB 0

    while ( !(CANSTMOB & (1 << TXOK)));
}


//! \fn void initCANMOB()
//! \brief Intialize the CAN MOB.
//!
//! Intialize the CAN MOB. A MOB can be seen as a "CAN socket".
//! In this case the MOB1 is used to receive data request (speed value)
//! In this case the MOB2 is used to receive data request (pid value)
void initCANMOB(){
    //--- MOB 1 : set motor speed (receive data) ---
    CANPAGE = 0x10; // selection of MOB1 (receive data speed)

    CANIDT4 = 0x00; // Config as reception data (rtr = 0)
    CANIDT3 = 0x00;
    CANIDT2 = (uint8_t)( (ID_MOTORBOARD_DATASPEED & 0x00F)<< 5 ); // using the constant to configure the remote request identifier
    CANIDT1 = (uint8_t)( ID_MOTORBOARD_DATASPEED >> 3 );

    CANIDM4 = 0x04; // mask over the rtr value
    CANIDM3 = 0xFF; // mask over the identifier (we only want an interruption if the received CAN message is a remote request with the corresponding identifier)
    CANIDM2 = 0xFF;
    CANIDM1 = 0xFF;

    CANCDMOB = 0x80; // Config MOB as reception

    //--- MOB 2 : set PID value (receive data) ---
    CANPAGE = 0x20; // selection of MOB2 (receive PID data)

    CANIDT4 = 0x00; // Config as reception data (rtr = 0)
    CANIDT3 = 0x00;
    CANIDT2 = (uint8_t)( (ID_MOTORBOARD_DATAPID & 0x00F)<< 5 ); // using the constant to configure the remote request identifier
    CANIDT1 = (uint8_t)( ID_MOTORBOARD_DATAPID >> 3 );

    CANIDM4 = 0x04; // mask over the rtr value
    CANIDM3 = 0xFF; // mask over the identifier (we only want an interruption if the received CAN message is a remote request with the corresponding identifier)
    CANIDM2 = 0xFF;
    CANIDM1 = 0xFF;

    CANCDMOB = 0x80; // Config MOB as reception

    //--- MOB 3 : CONFIGURATION - enable PID - (receive data) ---
    CANPAGE = 0x30; // selection of MOB3 (receive Configuration word)

    CANIDT4 = 0x00; // Config as reception data (rtr = 0)
    CANIDT3 = 0x00;
    CANIDT2 = (uint8_t)( (ID_MOTORBOARD_DATACONF & 0x00F)<< 5 ); // using the constant to configure the remote request identifier
    CANIDT1 = (uint8_t)( ID_MOTORBOARD_DATACONF >> 3 );

    CANIDM4 = 0x04; // mask over the rtr value
    CANIDM3 = 0xFF; // mask over the identifier (we only want an interruption if the received CAN message is a remote request with the corresponding identifier)
    CANIDM2 = 0xFF;
    CANIDM1 = 0xFF;

    CANCDMOB = 0x80; // Config MOB as reception
    
    CANIE2 = 0x0E; // enable the interruptions over the MOB 1, MOB2 and MOB3 (0b 0000 1110)
}

// POUR LES TESTS
void printCAN(void* variable, size_t size, uint8_t id=0xFF){
    uint8_t data[8];
    memcpy(data, variable, size);
    sendCANdata(id, size, data);
}

Led redLed(&LED_RED_PORT, LED_RED_PIN, LED_RED_POL);             //!< the red LED
Led yellowLed(&LED_YELLOW_PORT, LED_YELLOW_PIN, LED_YELLOW_POL); //!< the yellow LED
M32m1_pwm pwm;                                                   //!< the PWM for the motor
Motor_dc motor(&pwm, 0);                                         //!< the DC motor
Spi spi;                                                         //!< the SPI communication
Counter counter(&spi,&PORTC,&DDRC,PORTC1);                       //!< the counter (motor speed sensor)
Pid pid(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);                     //!< the PID

volatile uint32_t watch_dog;     //!< To stop the motor if no speed command reveiced after a delay
volatile int16_t nb_tics_cmd;    //!< The counter value command
volatile int16_t nb_tics_target; //!< The target counter value
volatile uint8_t enablePID;      //!< To enable/disable the PID
volatile uint8_t nbFlat;         //!< To stop the motor when not turning (after emmergency stop)

//! \fn int main(void)
//! \brief The main function of the MotorBoard
//!
int main(void)
{
    cli(); // clear all interruptions

    // initialization of the flags and other global variables
    watch_dog = 0;
    enablePID = 1;
    nb_tics_cmd = 0;
    nb_tics_target = 0;
    nbFlat = 0;

    // make the LED blink to show that the board is alive
    for (uint8_t i=0; i<5; i++) {
        redLed.blink(50);
        yellowLed.blink(50);
    }

    // to enable the LM2575
    Output enable_LM2575(&PORTC,PORTC7);
    enable_LM2575.setLow();

    // Initialization of the Timer
    TIFR1  |= 0x04;
    TCCR1B |= 0x0D;
    TCCR1A |= 0;
    OCR1A   = 78;
    // Value for the interruption (OCR1A)
    // 7812 = 1   sec
    // 1562 = 200 ms
    // 781  = 100 ms
    // 390  = 50  ms
    // 195  = 25  ms
    // 78   = 10  ms
    TIMSK1 |= (1<<OCIE1A);

    // initialization of the SPI communication
    spi.spi_init_master(true, SPI_FALLING_EDGE);

    counter.write_mode_register_0(0x03); // FILTER_1 | DISABLE_INDX | FREE_RUN | QUADRX4
    counter.write_mode_register_1(0x00); // NO_FLAGS | EN_CNTR | BYTE_4
    counter.clear_counter(); // reset the counter value
    counter.clear_status_register(); // clear the counter register

    initCANBus(); // intialization of the CAN Bus
    initCANMOB(); // intialization of the CAN MOB

    motor.enableMotor(); // enable the motor


    sei(); // set enable interruption

    while(1) {
        // do nothing, everything is handled with the interruption
        // (timer and CAN interruptions)
    }
}


//! \fn ISR(TIMER1_COMPA_vect)
//! \brief TIMER 1 interruption.
//! This function is called when a TIMER1 interruption is raised.
ISR(TIMER1_COMPA_vect){
    cli(); // clear all interruption
    TIFR1 |= 0; // reset the timer for the next interruption

    int16_t val = counter.read_counter()*SIDE_MOTOR; // read the counter value
    counter.clear_counter(); //reset the counter (for the next interruption)
    
    if(val == 0){ // if the motor did not turned
        nbFlat ++; // increments the flat flag
    }else{
        nbFlat = 0; // reset the flat flag
    }

    if(watch_dog > MAX_WATCH_DOG || nb_tics_target == 0 || nbFlat > MAX_NB_FLAT){ 
        // the motor is stopped if:
        //      - the time of the received last command is over the watch dog delay
        //      - the speed command is 0
        //      - the number of 0 counter value is over the max value (possible emmergency stop)
        motor.setSpeed(0);
        pid.reset(); // reset the PID
        nb_tics_target = 0; // reset the speed target
    }else{
        watch_dog ++; // increments the watch dog (reseted when receiving new speed command)
        if(enablePID==0x01){ // if the PID is enabled
            // compute the corrected command with the PID
            int16_t cmd = nb_tics_cmd + pid.update(nb_tics_target, val);
            // set the motor speed 
            motor.setSpeed(F_MOTOR_TIC2PWM(cmd));
        }else{
            // if the PID is desactivated, set directly the motor with the estimated transfer function
            motor.setSpeed(F_MOTOR_TIC2PWM(nb_tics_cmd));
        }
    }
    sei(); // enable the interruptions
}

//! \fn ISR(CAN_INT_vect)
//! \brief CAN interruption.
//! This function is called when an CAN interruption is raised.
ISR(CAN_INT_vect){
    cli(); // disable the interruption (no to be disturbed when dealing with one)

    if ( (CANSIT2 & 0x02 )!=0x00){ // MOB1 interruption - SET MOTOR SPEED
        CANPAGE = 0x10; // Selection of MOB 1

        // get the data from the mob 1:
        uint8_t dlc = CANCDMOB & 0x0F; // get the DLC of the CAN frame
        if(dlc == 0x03){ // rotationCW | speed(MSB) | speed(LSB)
            // get the first data byte (rotationCW)
            CANPAGE = 0x10; 
            uint8_t rotationCW = CANMSG;
            // get the second data byte (speed MSB)
            CANPAGE = 0x11; 
            uint8_t speedH = CANMSG;
            // get the third and last data byte (speed LSB)
            CANPAGE = 0x12; 
            uint8_t speedL = CANMSG;
            // get the target speed (integer mrad/s)
            uint16_t mrads = (uint16_t)(speedH << 8) + speedL;
            int16_t nb_tics_new_target = 0; // to update the speed target (counter value)
            watch_dog = 0; // reset the watch dog (a new command has been received)
            if(mrads == 0){ // if the speed is 0, special case
                nb_tics_new_target = 0; // the counter target must be O
            }else if(!rotationCW){ // convert the mrad/s speed to counter/10ms speed, according to the rotation direction
                nb_tics_new_target = (int16_t) (mrads*(NB_STEPS/100.0)/(2.0*PI*1000.0)); // convert mrad/s to tics/10ms
            }else{
                nb_tics_new_target = -(int16_t) (mrads*(NB_STEPS/100.0)/(2.0*PI*1000.0));
            }

            if(nb_tics_new_target != nb_tics_target){
                // if the target speed has been changed
                nb_tics_target = nb_tics_new_target; // update the target
                nb_tics_cmd = nb_tics_target; // update the command according to the target
                pid.reset(); // reset the PID
                nbFlat = 0; // reset the nbFlat flag
            } // otherwise, nothing to change
        }
        // else: the data is not correct

        // reset the MOB1 configuration for next CAN message
        CANPAGE   = 0x10; // select MOB1
        CANSTMOB  = 0x00; // Reset the status of the MOB1
        CANCDMOB  = 0x80; // Config as reception MOB1
        CANIE2   |= 0x02; // Enable the interuption over MOB 1 (for the next one)
        CANSIT2  &= 0xFD; // remove the MOB1 raised flag

    }else if((CANSIT2 & 0x04 )!=0x00){ //MOB2 interruption - Set PID coefficients
        // 1 -> set KP
        // 2 -> set KI
        // 3 -> set KD
        CANPAGE = 0x20; // select MOB2
        // get the data from the mob 2:
        uint8_t dlc = CANCDMOB & 0x0F; // get the DLC of the CAN frame
        if(dlc == 0x05){ // Kid | float value MSB | float value | float value | float value (LSB)
            // get the first data byte (K identifier (P, I or D))
            CANPAGE = 0x20; 
            uint8_t id_K = CANMSG;

            // get the float value
            uint8_t data[4];
            for(uint8_t i=1; i<5; i++){ // get all the data of the CAN message
                CANPAGE = (0x20+i);
                data[5-i-1] = (uint8_t)CANMSG; // warning : MSB and LSB
            }
            float value;
            memcpy(&value, data, sizeof(float)); // put the data in a float variable
            // update the corresponding coefficent value of the PID
            switch(id_K){
                case 1:
                    pid.setKp(value);
                    yellowLed.blink(10);
                    break;
                case 2:
                    pid.setKi(value);
                    yellowLed.blink(10);
                    break;
                case 3:
                    pid.setKd(value);
                    yellowLed.blink(10);
                    break;
            }
            // reset the PID
            pid.reset();
        }
        // else: the data is not correct

        // reset the MOB2 configuration for next CAN message
        CANPAGE   = 0x20; // Selection of MOB 2
        CANSTMOB  = 0x00; // Reset the status of the MOB2
        CANCDMOB  = 0x80; // Config as reception MOB2
        CANIE2   |= 0x04; // Enable the interuption over MOB 2 (for the next one)
        CANSIT2  &= 0xFB; // remove the MOB2 raised flag

    }else if((CANSIT2 & 0x08 )!=0x00){ //MOB3 interruption : CONFIGURATION WORD
        CANPAGE = 0x30; // select the MOB 3
        //get the data from the mob 3:
        uint8_t dlc = CANCDMOB & 0x0F; // get the dlc of the CAN frame
        if(dlc == 0x01){ // configuration word, 1 byte
            //get the first data byte (configuration byte)
            CANPAGE = 0x30;
            uint8_t configuration = CANMSG;
            // enable PID is the LSB of the configuration word
            enablePID = configuration & 0x01;

            // blink the led to show the PID status
            if(enablePID==0x01) yellowLed.blink(50);
            else redLed.blink(50);

            pid.reset(); // reset the PID
        }
        // else: the data is not correct

        // reset the MOB3 configuration for next CAN message
        CANPAGE   = 0x30; // select MOB3
        CANSTMOB  = 0x00; // Reset the status of the MOB3
        CANCDMOB  = 0x80; // Config as reception MOB3
        CANIE2   |= 0x08; // Enable the interuption over MOB 3 (for the next one)
        CANSIT2  &= 0xF7; // remove the MOB3 raised flag
    }

    sei(); // enable the interruptions
}
