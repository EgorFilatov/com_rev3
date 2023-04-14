#include "main.hpp"

#ifndef SRC_SPIPORT_HPP_
#define SRC_SPIPORT_HPP_

class SpiPort {
private:
	uint8_t spiRx [6];
	uint8_t spiTx [6];
	uint8_t spiRxSaved [6];
	uint8_t spiTxSaved [6];
	GPIO_TypeDef* port;
	uint8_t pin;
	uint8_t type;

public:
	SpiPort();
};

#endif
