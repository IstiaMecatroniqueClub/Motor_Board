#include "motor_dc.h"
#include "output.h"

Motor_dc::Motor_dc(M32m1_pwm* ppwm,
                   uint8_t defaultRotation,
                   unsigned char prescaler,
                   unsigned char sourceClock,
                   unsigned char deadTimeNumberCycles){
    _ppwm = ppwm;
    _duty_cycle = 0;
    _rotationCW = 0x00;
    _defaultRotation = defaultRotation;
    
    // Configure ports as output and set low for MOSFET Drivers
    // (disactivate all transistors)
    Output pscout0A (&PSCOUT0A_PORT,PSCOUT0A_PIN); pscout0A.setLow();
    Output pscout0B (&PSCOUT0B_PORT,PSCOUT0B_PIN); pscout0B.setLow();
    Output pscout1A (&PSCOUT1A_PORT,PSCOUT1A_PIN); pscout1A.setLow();
    Output pscout1B (&PSCOUT1B_PORT,PSCOUT1B_PIN); pscout1B.setLow();

    _ppwm->init(prescaler,sourceClock,deadTimeNumberCycles);
}


// Enable or re-enable the motor
void Motor_dc::enableMotor()
{
    // Lock PSC module
    _ppwm->lock();
    // Open the bottom transistors in the H-Bridge
    PORTB &= ~((1<<PB1) | (1<<PB6) | (1<<PB7));
    //timer1_attachInterrupt(onInterruptTimer1);
    // unlock PSC module
    _ppwm->unlock();
}


// Disable the motor (phase are disconnected)
void Motor_dc::disableMotor()
{
    // Lock PSC module
    _ppwm->lock();
    // Open the bottom transistors in the H-Bridge
    PORTB &= ~((1<<PB1) | (1<<PB6) | (1<<PB7));
    // Disable PWM from pins (outputs are standard ports)
    _ppwm->setOutputConfiguration(PWM_CONFIG_DISABLE_ALL);
    // Unlock PSC module
    _ppwm->unlock();
}

// Brakes the motor, each phase is connected to the ground
void Motor_dc::brakeMotor()
{
    // Lock PSC module
    _ppwm->lock();
    // Disable PWM from pins (outputs are standard ports)
    _ppwm->setOutputConfiguration(PWM_CONFIG_DISABLE_ALL);
    // Close the bottom transistors in the H-Bridge
    // The three phases are connected to the ground
    PORTB |= (1<<PB1) | (1<<PB6) | (1<<PB7);
    // Unlock PSC module
    _ppwm->unlock();
}




// Set the speed of the motor
void Motor_dc::setSpeed(int speed)
{
    // Set rotation direction
    _rotationCW = ((speed<0)&&_defaultRotation) || ((speed>0)&&(!_defaultRotation));
    // Update PWM_cmdc_duty_cycle
    if(speed < 0){
        _duty_cycle = (-speed);
    }else{
        _duty_cycle = speed;
    }

    this->commutation();
}

// This function manage phase commutation
void Motor_dc::commutation()
{
    // Lock PWM to avoid transtory unexpected changes
    _ppwm->lock();
    _ppwm->setOutputConfiguration(0b001111);
    // Commute according to the requested rotation direction,
    if (_rotationCW)
    {
        // ClockWise
        _ppwm->setDutyCycle0(_duty_cycle);
        _ppwm->setDutyCycle1(0);
    }
    else
    {
        // Counter ClockWise
        _ppwm->setDutyCycle0(0);
        _ppwm->setDutyCycle1(_duty_cycle);
    }
    // Unlock PWM all the updated parameters are set simultaneously
    _ppwm->unlock();
}
