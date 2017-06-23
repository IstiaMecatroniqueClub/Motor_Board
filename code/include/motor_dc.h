#ifndef MOTOR_DC_H
#define MOTOR_DC_H

//! \file motor_dc.h
//! \brief Motor_dc class
//! \author Franck Mercier, Remy Guyonneau
//! \date 2017 05 23

#include "m32m1_pwm.h"

//! \class Motor_dc
//! \brief Motor_dc class. 
//!
//! Motor_dc class. This class provide a low level control over a DC motor
class Motor_dc
{
public:
    //! \brief Motor_dc constructor
    //!
    //! Motor_dc constructor. Initilizes cmdc motor (initialize PWM, and hall effect sensors)
    //! The PWM frequency is given by the following parameters
    //!
    //! \param ppwm : Pointer over the M21m1_pwm object (to handle the pwm signal)
    //!
    //! \param[in] prescaler             Value for setting the clock prescaler. Possible values are:
    //!                                  PWM_PRESCALER_NONE
    //!                                  PWM_PRESCALER_4
    //!                                  PWM_PRESCALER_32
    //!                                  PWM_PRESCALER_256
    //!
    //! \param[in] sourceClock           Set the source clock for the PWM module. Possible values are:
    //!                                  PWM_SOURCE_CLK_CPU_CLK
    //!                                  PWM_SOURCE_CLK_PLL_32MHZ
    //!                                  PWM_SOURCE_CLK_PLL_64MHZ
    //!
    //! \param[in] deadTimeNumberCycles   Set the number of cycles for the dead-time. The duration of one cycle is given
    //!                               by the period of the sourceClock divied by the prescaler. (One eleemntary cycle)
    //!
    Motor_dc(M32m1_pwm* ppwm,
             uint8_t defaultRotation = 0x00,
             unsigned char prescaler=PWM_PRESCALER_NONE,
             unsigned char sourceClock=PWM_SOURCE_CLK_PLL_64MHZ,
             unsigned char deadTimeNumberCycles=PWM_DEADTIME_DEFAULT_NB_CYCLES);

    //!
    //! \brief enableMotor  Enable motor (allows commutation)
    //!                     Start the commutation process for controlling the motor
    void enableMotor();


    //!
    //! \brief disableMotor Stop the commutation process of the motor
    //!                     and open the six transistors of the H-Bridge
    //!
    void disableMotor();

    //!
    //! \brief disableMotor Stop the commutation process of the motor and brakes the motor
    //!                     braking is done by connecting the three phase to the ground
    //!                     The three transistors connected to the ground are closed
    //!
    void brakeMotor();

    //!
    //! \briefsetSpeed Set the speed of the motor
    //! \param speed         Speed of the motor (PWM duty cycle)
    //!                      speed is included between -PWM_COUNTER_MAX and +PWM_COUNTER_MAX
    //!                      Values outside of this range are bounded
    void setSpeed(int16_t speed);


    //!
    //! \brief cmdc_commutation  This function performs the righ commutation
    //!
    void commutation();

private:

    M32m1_pwm* _ppwm;            //!< Pointer over the PWM
    uint16_t   _duty_cycle;      //!< The PWM duty cycle (speed of the motor)
    uint8_t    _rotationCW;      //!< rotation clock wise
    uint8_t    _defaultRotation; //!< To differentiate left wheels and right wheels
};

#endif // MOTOR_DC_H
