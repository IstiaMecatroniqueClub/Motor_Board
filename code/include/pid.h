#ifndef PID_H
#define PID_H

//! \file pid.h
//! \brief Pid class
//! \author Baptiste Hamard, Remy Guyonneau
//! \date 2017 05 25

#include <stdint.h>

//! \class Pid
//! \brief Pid class. 
//!
//! PID class
class Pid
{
public:

    //! \brief Pid : Constructor to represent an Pid
    //!
    //! Constructor to represent an Pid
    //!
    //! \param[in] p : the P coefficient
    //! \param[in] i : the I coefficient
    //! \param[in] d : the D coefficient
    Pid(float p, float i, float d);

    //! \brief setKp : Set P coeffcient
    //!
    //! \param[in] kp : the P coefficient
    void setKp(float kp);

    //! \brief setKi : Set I coeffcient
    //!
    //! \param[in] ki : the I coefficient
    void setKi(float ki);

    //! \brief setKd : Set D coeffcient
    //!
    //! \param[in] kd : the D coefficient
    void setKd(float kd);

    //! \brief setKd : update the PID value (error and correction)
    //!
    //! \param[in] target : the target state
    //! \param[in] state : the measured state
    //! \return : the new correction value to apply to the command
    int16_t update(int16_t target, int16_t state);

    //! \brief reset : reset the PID value (error and correction)
    void reset();

private:
    float _kp;              //!< The PID P coefficient
    float _ki;              //!< The PID I coefficient
    float _kd;              //!< The PID D coefficient
    float _sum_errors;      //!< The PID error sum
    float _previous_error;  //!< The previous error
    float _correction;      //!< The correction value

};

#endif // PID_H
