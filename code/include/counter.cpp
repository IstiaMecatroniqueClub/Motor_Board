#include "counter.h"

Counter::Counter(Spi *spi,volatile uint8_t *port,volatile uint8_t *ddr, uint8_t pin){
    *ddr  |= (1<<pin);
    *port |= (1<<pin);

    _port = port;
    _pin  = pin;

    _spi  = spi;

};

Counter::~Counter(){};

void Counter::clear_mode_register_0(){
    *_port &= ~(1<<_pin);

    _spi->spi_tranceiver(CLR_MDR0);

    *_port |= (1<<_pin);
}

void Counter::clear_mode_register_1(){
    *_port &= ~(1<<_pin);

    _spi->spi_tranceiver(CLR_MDR1);

    *_port |= (1<<_pin);
}

void Counter::clear_status_register(){
    *_port &= ~(1<<_pin);

    _spi->spi_tranceiver(CLR_STR);

    *_port |= (1<<_pin);
}


void Counter::clear_counter(){
    *_port &= ~(1<<_pin);

    _spi->spi_tranceiver(CLR_CNTR);

    *_port |= (1<<_pin);
}


int32_t Counter::read_counter(){

    int32_t data=0;
    int8_t  i=4;

    *_port &= ~(1<<_pin);


    _spi->spi_tranceiver(READ_CNTR);

    while (i>0)
    {
        data = (data <<8) | (_spi->spi_tranceiver(0x00));
        i--;
    }

    *_port |= (1<<_pin);
    return data;
}

void Counter::write_mode_register_0(uint8_t data){
    *_port &= ~(1<<_pin);

    _spi->spi_tranceiver(WRITE_MDR0);
    _spi->spi_tranceiver(data);

    *_port |= (1<<_pin);
}

void Counter::write_mode_register_1(uint8_t data){
    *_port &= ~(1<<_pin);

    _spi->spi_tranceiver(WRITE_MDR1);
    _spi->spi_tranceiver(data);


    *_port |= (1<<_pin);
}

void Counter::write_data_register(int32_t data){
    int32_t val = data;

    *_port &= ~(1<<_pin);

    _spi->spi_tranceiver(WRITE_DTR);
    for (uint8_t i=0;i<4;i++)
    {
        val = (uint8_t)(data >> (8*(3 - i)));
        _spi->spi_tranceiver(val);
    }

    *_port |= (1<<_pin);
}

uint8_t Counter::read_status_register(){
    *_port &= ~(1<<_pin);

    _spi->spi_tranceiver(READ_STR);
    uint8_t data = _spi->spi_tranceiver(0x00);

    *_port |= (1<<_pin);
    return data;

}
