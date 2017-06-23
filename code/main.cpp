//! \file main.cpp
//! \brief Main file for the MotorBoard
//! \author Remy Guyonneau, Baptiste Hamard, Franck Mercier
//! \date 2017 05 29



// for ATMEL studio, not needed with the raspberry pi
#define FOSC            16000       //!< The ATMEGA clock speed
#define F_CPU           16000000UL  //!< The ATMEGA clock speed

#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "led.h"
#include "pin.h"
#include "m32m1_pwm.h"
#include "motor_dc.h"
#include "spi.h"
#include "counter.h"
#include "pid.h"
#include "libcan.h"

#include <string.h> //POUR LES TESTS

#define PI                      3.14159     //!< The PI constant, to handle mrad/s

#define RIGHT_MOTOR             (1)         //!< The right motor value
#define LEFT_MOTOR              (-1)        //!< The left motor value

#define LED_RED_PORT            PORTB       //!< The port for the red LED
#define LED_RED_PIN             3           //!< The pin for the red LED
#define LED_RED_POL             0           //!< The polarity of the red LED

#define LED_YELLOW_PORT         PORTB       //!< The port for the yellow LED
#define LED_YELLOW_PIN          2           //!< The pin for the yellow LED
#define LED_YELLOW_POL          0           //!< The polarity of the yellow LED


#define NB_STEPS                1920        //!< Number of tics for a complete wheel turn

#define MAX_WATCH_DOG           100         //!< Time after the motor will stop
                                            //!  if not receiving speed command* 10ms
#define MAX_NB_FLAT             100         //!< Number of 0 value from the sensor before
                                            //!  shutting down the robot (avoid motion after
                                            //!  an emmergency stop for instance)

#define DEFAULT_KP              0.1         //!< default KP for the PID 
#define DEFAULT_KI              0           //!< default KI for the PID
#define DEFAULT_KD              0           //!< default KD for the PID



#define MOB_SENDDATA            0
#define MOB_SETMOTORSPEED       1
#define MOB_SETPIDVALUE         2
#define MOB_SETENPID            3


//constants to change for the motors:
#define SIDE_MOTOR              LEFT_MOTOR //!< To handle left and right moteur
                                            //!  choose between LEFT_MOTOR or RIGHT_MOTOR
// #define ID_MOTOR                0x00        //!<   0x00 : LEFT_MOTOR, 0x01 : RIGHT_MOTOR
#define ID_MOTORBOARD_DATASPEED 0x30        //!< ID of CAN Frame to set the motor speed
                                            //   |0x30|direction|Hspeed0|Lspeed0|Hspeed1|Lspeed1
                                            //   direction: xxxx xxLR with R the direction of right motor and L direction of left motor
                                            //   0: forward, 1: backward
#define ID_MOTORBOARD_DATAPID   0x31        //!< ID of CAN Frame to set the PID values
#define ID_MOTORBOARD_DATACONF  0x32        //!< ID of CAN Frame to set the motor configuration

int16_t f_motor_tic2pwm(int16_t tics){
    return SIDE_MOTOR*15*tics;
}

#define F_MOTOR_TIC2PWM(tic) (SIDE_MOTOR*15*tic) //!< To convert counter value to PWM,
                                                 //!  the values are extracted from experimental tests

// POUR LES TESTS
void sendCANdata(uint8_t id, uint8_t dlc, uint8_t* data){
    CAN_lib::sendData(MOB_SENDDATA, (uint32_t)id, dlc, data);
}


//! \fn void initCANMOB()
//! \brief Intialize the CAN MOB.
//!
//! Intialize the CAN MOB. A MOB can be seen as a "CAN socket".
//! In this case the MOB1 is used to receive data request (speed value)
//! In this case the MOB2 is used to receive data request (pid value)
void initCANMOB(){
    //--- MOB 1 : set motor speed (receive data) ---
    CAN_lib::initCANMOBasReceiver(MOB_SETMOTORSPEED, ID_MOTORBOARD_DATASPEED, 0);

    //--- MOB 2 : set PID value (receive data) ---
    CAN_lib::initCANMOBasReceiver(MOB_SETPIDVALUE, ID_MOTORBOARD_DATAPID, 0);

    //--- MOB 3 : CONFIGURATION - enable PID - (receive data) ---
    CAN_lib::initCANMOBasReceiver(MOB_SETENPID, ID_MOTORBOARD_DATACONF, 0);
    
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
    enablePID = 0;
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

    CAN_lib::initCANBus(); // intialization of the CAN Bus
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

    cli();

    if(watch_dog > MAX_WATCH_DOG){ 
        motor.setSpeed(0);
    }else{
        watch_dog ++;
    }

    sei();
}

//! \fn ISR(CAN_INT_vect)
//! \brief CAN interruption.
//! This function is called when an CAN interruption is raised.
ISR(CAN_INT_vect){
    cli(); // disable the interruption (no to be disturbed when dealing with one)
    if ( CAN_lib::isMOBInterflagRaised(MOB_SETMOTORSPEED) != 0x00 ){ // MOB1 interruption - SET MOTOR SPEED

        uint8_t data[8];
        uint8_t dlc;

        int16_t speed;

        dlc = CAN_lib::getData(MOB_SETMOTORSPEED, data); // get the data and the dlc from the CAN Message

    

        if(dlc == 4){ // check if the data message has a correct dlc (4 bytes: 2 motor0, 2 motor1)
            yellowLed.blink(500);
            if( SIDE_MOTOR == RIGHT_MOTOR){
                memcpy((int8_t*)(&speed), &data[3], sizeof(int8_t));
                memcpy((int8_t*)(&speed)+1, &data[2], sizeof(int8_t));
            }else{
                memcpy((int8_t*)(&speed), &data[1], sizeof(int8_t));
                memcpy((int8_t*)(&speed)+1, &data[0], sizeof(int8_t));
                speed = -speed;
            }
            printCAN(data, dlc);
            motor.setSpeed(speed);
            watch_dog = 0;
        }

        CAN_lib::resetReceptionMOB(MOB_SETMOTORSPEED);
    }
    sei(); // enable the interruptions
}
