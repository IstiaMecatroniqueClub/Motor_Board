#ifndef PLL_H
#define PLL_H

//! \file m32m1_pll.h
//! \brief M32m1_pll class
//! \author Remy Guyonneau, Philippe Lucidarme
//! \date 2017 05 29

#include <avr/io.h>

#define PLL_FREQUENCY_64MHZ true  //!< TODO
#define PLL_FREQUENCY_32MHZ false //!< TODO


//! \class M32m1_pll
//! \brief M32m1_pll class   Manage internal PLL
//! Manage the internal PLL of the atmega32m1. This class must be a singleton
class M32m1_pll
{
public:



    //! \brief Constructor
    //! M32m1_pll Constructor, do nothing
    M32m1_pll();

    //!
    //!\brief pll_setFrequency  Set the PLL factor (32MHz or 64MHz)
    //!\param Frequency_64MHz   true, the PLL output is 64MHz
    //!                          false, the PLL output is 32MHz
    //!
    void setFrequency(bool PLL_Factor=PLL_FREQUENCY_64MHZ);

    //! \brief start Start PLL
    //! The PLL needs about 100ms to start
    //! Check pll_isReady before use
    void start();


    //!
    //! \brief stop  Stop the PLL (disable PLLE=PLL enable)
    //!
    void stop();


    //!
    //! \brief pll_isReady   check if the PLL is locked
    //! \return              true if the PLL is ready
    //!                      false if the PLL is not yet locked
    //!
    bool isReady();

};




#endif // PLL_H
