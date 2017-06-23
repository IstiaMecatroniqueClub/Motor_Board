/*
 * CPPFile1.cpp
 *
 * Created: 24/05/2017 12:28:09
 * Modified: 23/06/2017
 *  Author: f.mercier & r.guyonneau
 */ 

#ifndef LIBCAN_H
#define LIBCAN_H

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>

namespace CAN_lib {
    //! \fn void initCANBus()
    //! \brief Initialize the CAN Bus.
    //!
    //! Function that initializes the CANBus according to the wiring and the needed speed (500kb/s).
    //! It also enable the CAN module interruptions
    void initCANBus ();

    //! \fn void initCANMOBasReceiver()
    //! \brief Initialize the CAN MOB.
    //!
    //! Initialize a CAN MOB between 0 to 6 as a receiver. A MOB can be seen as a "CAN socket".
    void initCANMOBasReceiver (uint8_t mobNumber, uint32_t ID, uint8_t rtr);

    //! \fn void initCANMOBasIDBandReceiver()
    //! \brief Initialize the CAN MOB.
    //!
    //! Initialize a CAN MOB between 0 to 6 as ID band receiver. A MOB can be seen as a "CAN socket" on different successive ID.
    void initCANMOBasIDBandReceiver (uint8_t mobNumber, uint32_t BeginingID, uint32_t AreaSize, uint8_t rtr);

    //! \fn void disableCANMOB()
    //! \brief Resetting the CAN MOB.
    //!
    //! Reset a CAN MOB between 0 to 6.
    void disableCANMOB (uint8_t mobNumber);

    //! \fn void sendData()
    //! \brief Sending data over a CAN MOB.
    //!
    //! Sending data through a CAN MOB between 0 to 6.
    void sendData (uint8_t mobNumber, uint32_t ID, uint8_t dlc, uint8_t* buffer);

    //!TODO
    uint8_t isMOBInterflagRaised(uint8_t mobNumber);
    uint8_t getData(uint8_t mobNumber, uint8_t * pdata);
    void resetReceptionMOB(uint8_t mobNumber);
}
#endif
