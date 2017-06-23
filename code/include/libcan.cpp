#include "libcan.h"

//! \fn void initCANBus()
//! \brief Initialize the CAN Bus.
//!
//! Function that initializes the CANBus according to the wiring and the needed speed (500kb/s).
//! It also enable the CAN module interruptions
void CAN_lib::initCANBus () 
{
    // Enable of CAN transmit
    DDRC |= 0x80; // Configuration of the standby pin as an output
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

//! \fn void initCANMOBasReceiver()
//! \brief Initialize the CAN MOB.
//!
//! Initialize a CAN MOB between 0 to 6 as a receiver. A MOB can be seen as a "CAN socket".
void CAN_lib::initCANMOBasReceiver (uint8_t mobNumber, uint32_t ID, uint8_t rtr) 
{
    CANPAGE = (mobNumber << 4) & 0xF0;
    //CANPAGE = 0x10; // selection of MOB1 (reveice request)

    if (rtr) { CANIDT4 = 0x04;}
    else { CANIDT4 = 0x00;}
    //CANIDT4 = 0x04; // Config as reception remote (rtr = 1)
    CANIDT3 = 0x00;
    CANIDT2 = (uint8_t)( (ID & 0x00F)<< 5 ); // using the constant to configure the remote request identifier
    CANIDT1 = (uint8_t)( ID >> 3 );

    if (rtr) { CANIDM4 = 0x00;}
    else { CANIDM4 = 0x04;}
    //CANIDM4 = 0x04; // mask over the rtr value
    CANIDM3 = 0xFF; // mask over the identifier (we only want an interruption if the received CAN message is a remote request with the corresponding identifier)
    CANIDM2 = 0xFF;
    CANIDM1 = 0xFF;

    CANCDMOB = 0x80; // Config MOB as reception

    switch (mobNumber) {
        case 0 : CANIE2 |= 0x01; break;
        case 1 : CANIE2 |= 0x02; break;
        case 2 : CANIE2 |= 0x04; break;
        case 3 : CANIE2 |= 0x08; break;
        case 4 : CANIE2 |= 0x10; break;
        case 5 : CANIE2 |= 0x20; break;
        default : break;
    }
    //CANIE2 = 0x02; // enable the interruptions over the MOB 1

    sei(); // set enable interruption
}

//! \fn void initCANMOBasIDBandReceiver()
//! \brief Initialize the CAN MOB.
//!
//! Initialize a CAN MOB between 0 to 6 as ID band receiver. A MOB can be seen as a "CAN socket" on different successive ID.
void CAN_lib::initCANMOBasIDBandReceiver (uint8_t mobNumber, uint32_t BeginingID, uint32_t AreaSize, uint8_t rtr)
{
    CANPAGE = (mobNumber << 4) & 0xF0;

    if (rtr) { CANIDT4 = 0x04;}
    else { CANIDT4 = 0x00;}
    CANIDT3 = 0x00;
    CANIDT2 = (uint8_t)( (BeginingID & 0x000F)<< 5 ); // using the constant to configure the remote request identifier
    CANIDT1 = (uint8_t)( BeginingID >> 3 );

    AreaSize = 0x00000000 - AreaSize;
    if (rtr) { CANIDM4 = 0x00;}
    else { CANIDM4 = 0x04;}
    CANIDM3 = 0xFF; // mask over the identifier (we only want an interruption if the received CAN message is a remote request with the corresponding identifier)
    CANIDM2 = (uint8_t)( (AreaSize & 0x000F)<< 5 );
    CANIDM1 = (uint8_t)( AreaSize >> 3 );

    CANCDMOB = 0x80; // Config MOB as reception

    switch (mobNumber) {
        case 0 : CANIE2 |= 0x01; break;
        case 1 : CANIE2 |= 0x02; break;
        case 2 : CANIE2 |= 0x04; break;
        case 3 : CANIE2 |= 0x08; break;
        case 4 : CANIE2 |= 0x10; break;
        case 5 : CANIE2 |= 0x20; break;
        default : break;
    }
    sei(); // set enable interruption
}

//! \fn void disableCANMOB()
//! \brief Resetting the CAN MOB.
//!
//! Reset a CAN MOB between 0 to 6.
void CAN_lib::disableCANMOB (uint8_t mobNumber)
{
    CANPAGE = (mobNumber << 4) & 0xF0;      // selection of correct MOB

    CANCDMOB = 0x00;                        // Config MOB as disable

    switch (mobNumber) {                    // Disable the interruptions over the proper MOB
        case 0 : CANIE2 &= 0xFE; break;
        case 1 : CANIE2 &= 0xFD; break;
        case 2 : CANIE2 &= 0xFB; break;
        case 3 : CANIE2 &= 0xF7; break;
        case 4 : CANIE2 &= 0xEF; break;
        case 5 : CANIE2 &= 0xDF; break;
        default : break;
    }
}

//! \fn void sendData()
//! \brief Sending data over a CAN MOB.
//!
//! Sending data through a CAN MOB between 0 to 6.
void CAN_lib::sendData (uint8_t mobNumber, uint32_t ID, uint8_t dlc, uint8_t* buffer)
{
    CANPAGE = (mobNumber << 4) & 0xF0;          // Mob selection

    CANIDT4 = 0x00;                             // Config as data (rtr = 0)
    CANIDT3 = 0x00;
    CANIDT2 = (uint8_t)( (ID & 0x00F)<< 5 );    // identifier implementation
    CANIDT1 = (uint8_t)( ID >> 3 );

    for (int i=0; i < dlc; i++)
    {
        CANPAGE &= 0xF0;
        CANPAGE |= i;
        CANMSG = buffer[i];
    }
    CANCDMOB = 0x40 | dlc;  // send the message using the proper MOB
}


uint8_t CAN_lib::isMOBInterflagRaised(uint8_t mobNumber){
    return  (CANSIT2 & (0x01 << mobNumber) ) ;
}

uint8_t CAN_lib::getData(uint8_t mobNumber, uint8_t * pdata){
    uint8_t mob_select = (mobNumber <<4);
    CANPAGE = mob_select; // select MOB mobNumber
    uint8_t dlc = CANCDMOB & 0x0F; // get the DLC of the CAN frame

    for(uint8_t i=0; i<dlc; i++){
        CANPAGE = mob_select + i;
        pdata[i] = CANMSG;
    }

    return dlc;
}

void CAN_lib::resetReceptionMOB(uint8_t mobNumber){
    CANPAGE = (mobNumber <<4); // select MOB mobNumber
  
    CANSTMOB  = 0x00; // Reset the status of the MOB
    CANCDMOB  = 0x80; // Config as reception MOB
    CANIE2   |= (1 << mobNumber); // Enable the interuption over MOB  (for the next one)
    CANSIT2  &= (~(1 << mobNumber)); // remove the MOB raised flag
}

