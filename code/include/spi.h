#ifndef SPI_H
#define SPI_H

//! \file spi.h
//! \brief SPI class. 
//! \author Franck Mercier, Remy Guyonneau 
//! \date 2017 05 12

#include "pin.h"

#define STD_DIRECTION 0 //!< TODO
#define A_DIRECTION   1 //!< TODO


#define SPI_RISING_EDGE  0x5D // for the IMU
#define SPI_FALLING_EDGE 0x51 // for the counter


class Spi
{
public:
    //! \brief Spi constructor
    Spi();

    //! \brief Spi destructor
    ~Spi();

    //! \brief Configuration of the SPI as master
    //! \param[in] redirection : Use std SPI port (portb) or _A port (portd)
    void spi_init_master(bool redirection, uint8_t edge = SPI_RISING_EDGE);

    //! \brief Configuration of the SPI as slave
    //! \param[in] redirection : Use std SPI port (portb) or _A port (portd)
    void spi_init_slave(bool redirection);

    //! \brief Set the CS to 0
    void spi_begin_transceive ();

    //! \brief Set the CS to 1
    void spi_stop_transceive ();

    //! \brief Function to send or receive a data
    unsigned char spi_tranceiver (unsigned char data);

private:

    bool _spiOut; //!< TODO
};
#endif // SPI_H
