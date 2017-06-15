#ifndef COUNTERS_H
#define COUNTERS_H

//! \file counters.h
//! \brief Counter class
//! \author Baptiste Hamard, Remy Guyonneau
//! \date 2017 05 23

#include <avr/io.h>        // for the ATMEGA registers definition
#include <avr/interrupt.h> // for the interruptions
#include "spi.h"

//Count modes 
#define NQUAD   0x00          //!< non-quadrature mode
#define QUADRX1 0x01          //!< X1 quadrature mode
#define QUADRX2 0x02          //!< X2 quadrature mode
#define QUADRX4 0x03          //!< X4 quadrature mode

//Running modes 
#define FREE_RUN    0x00      //!< TODO
#define SINGE_CYCLE 0x04      //!< TODO
#define RANGE_LIMIT 0x08      //!< TODO
#define MODULO_N    0x0C      //!< TODO

//Index modes 
#define DISABLE_INDX 0x00     //!< index_disabled
#define INDX_LOADC   0x10     //!< TODO

//index_load_CNTR 
#define INDX_RESETC 0x20      //!< index_rest_CNTR
#define INDX_LOADO  0x30      //!< index_load_OL
#define ASYNCH_INDX 0x00      //!< asynchronous index
#define SYNCH_INDX  0x80      //!< synchronous index

//Clock filter modes 
#define FILTER_1 0x00         //!< filter clock frequncy division factor 1 
#define FILTER_2 0x80         //!< filter clock frequncy division factor 2

// MDR1 configuration data; any of these
// data segments can be ORed together

//F lag modes 
#define NO_FLAGS 0x00 //!< all flags disabled
#define IDX_FLAG 0x10 //!< IDX flag 
#define CMP_FLAG 0x20 //!< CMP flag 
#define BW_FLAG  0x40 //!< BW flag 
#define CY_FLAG 0x80  //!< CY flag 

//1 to 4 bytes data-width 
#define BYTE_4 0x00   //!< four byte mode
#define BYTE_3 0x01   //!< three byte mode
#define BYTE_2 0x02   //!< two byte mode
#define BYTE_1 0x03   //!< one byte mode
 
//Enable/disable counter
#define EN_CNTR  0x00 //!<counting enabled 
#define DIS_CNTR 0x04 //!< counting disabled 

// LS7366R op-code list
#define CLR_MDR0   0x08  //!< TODO
#define CLR_MDR1   0x10  //!< TODO
#define CLR_CNTR   0x20  //!< TODO
#define CLR_STR    0x30  //!< TODO
#define READ_MDR0  0x48  //!< TODO
#define READ_MDR1  0x50  //!< TODO
#define READ_CNTR  0x60  //!< TODO
#define READ_OTR   0x68  //!< TODO
#define READ_STR   0x70  //!< TODO
#define WRITE_MDR1 0x90  //!< TODO
#define WRITE_MDR0 0x88  //!< TODO
#define WRITE_DTR  0x98  //!< TODO
#define LOAD_CNTR  0xE0  //!< TODO
#define LOAD_OTR   0xE4  //!< TODO


class Counter{
    public:
        //! \brief Counter constructor
        //!
        //! Counter constructor
        //!
        //! \param spi : pointer over the SPI interface
        //! \param port : pointer over the counter port
        //! \param ddr : TODO
        //! \param[in] pin : the counter pin
        Counter(Spi *spi, volatile uint8_t *port, volatile uint8_t *ddr, uint8_t pin);

        //! \brief Counter destructor
        //!
        //! Counter destructor
        ~Counter();

        //! \brief clear_mode_register_0 TODO
        //!
        //! TODO
        void clear_mode_register_0();

        //! \brief clear_mode_register_1 TODO
        //!
        //! TODO
        void clear_mode_register_1();

        //! \brief clear_status_register TODO
        //!
        //! TODO
        void clear_status_register();

        //! \brief clear_counter clear counter
        //!
        //! Clear the counter value (to 0)
        void clear_counter();


        //! \brief write_mode_register_0 TODO
        //!
        //!TODO
        //!
        //! param[in] data : the mode to write in the register
        void write_mode_register_0(uint8_t data);

        //! \brief write_mode_register_1 TODO
        //!
        //!TODO
        //!
        //! param[in] data : the mode to write in the register
        void write_mode_register_1(uint8_t data);

        //! \brief write_data_register TODO
        //!
        //!TODO
        //!
        //! param[in] data : the data to write in the register
        void write_data_register(int32_t data);

        //! \brief read_counter Read the counter value (int32_t)
        //!
        //!Read the counter value (int32_t)
        //!
        //! return : The value of the counter
        int32_t read_counter();

        //! \brief read_status_register TODO
        //!
        //!TODO
        //!
        //! return : The value of the status register
        uint8_t read_status_register();

        //! \brief read_OTR TODO
        //!
        //!TODO
        //!
        //! return : The value of the OTR
        long read_OTR();

    protected:
        Spi* _spi;                     //!< The SPI interface pointer, to communicate with the counter
        volatile uint8_t *_port, _pin; //!< The pointer of the port and pi of the counter

};

#endif
