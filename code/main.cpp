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
#include "CanISR.h"

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

#define ID_MOTORBOARD_DATASPEED 0x040       //!< The polarity of the yellow LED

#define NB_STEPS                1920        //!< Number of tics for a complete wheel turn

#define MAX_WATCH_DOG           100         //!< Time after the motor will stop
                                            //!  if not receiving speed command* 10ms
#define MAX_NB_FLAT             100         //!< Number of 0 value from the sensor before
                                            //!  shutting down the robot (avoid motion after
                                            //!  an emmergency stop for instance)

#define DEFAULT_KP              0.07         //!< default KP for the PID  0.05
#define DEFAULT_KI              0.001           //!< default KI for the PID  0.0009
#define DEFAULT_KD              0.008           //!< default KD for the PID  0.8

#define F_MOTOR_TIC2PWM(tic) (SIDE_MOTOR*35*tic) //!< To convert counter value to PWM,
                                                 //!  the values are extracted from experimental tests

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
    OCR1A   = 390;
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

    initCANBus(); // initialization of the CAN Bus
    initCANMOBasReceiver (1, ID_MOTORBOARD_DATASPEED, 0); // initialization of the CAN MOB

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

    if(watch_dog > MAX_WATCH_DOG || nb_tics_target == 0 || nbFlat > MAX_NB_FLAT){	//  || nb_tics_target == 0
        // the motor is stopped if:
        //      - the time of the received last command is over the watch dog delay
        //      - the speed command is 0
        //      - the number of 0 counter value is over the max value (possible emergency stop)
        if (enablePID) {pid.reset(); } // reset the PID
        motor.setSpeed(0);
        nb_tics_target = 0; // reset the speed target
    }else{
        watch_dog ++; // increments the watch dog (reseted when receiving new speed command)
        if(enablePID){ // if the PID is enabled
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
			//uint16_t mrads = 0x8000;
			//rotationCW = 01;
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
                //nb_tics_cmd = nb_tics_target; // update the command according to the target
                //if (enablePID) {pid.reset(); } // reset the PID
                nbFlat = 0; // reset the nbFlat flag
            } // otherwise, nothing to change
        }

        // reset the MOB1 configuration for next CAN message
        CANPAGE   = 0x10; // select MOB1
        CANSTMOB  = 0x00; // Reset the status of the MOB1
        CANCDMOB  = 0x80; // Config as reception MOB1
        CANIE2   |= 0x02; // Enable the interruption over MOB 1 (for the next one)
        CANSIT2  &= 0xFD; // remove the MOB1 raised flag

    }

    sei(); // enable the interruptions
}
