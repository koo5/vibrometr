#include <SPI.h>
#include "SRAM.h"

SRAM::SRAM(uint8_t cs_pin) {
  this->cs_pin = cs_pin;
  this->mode = SRAM_SEQN_MODE;
  pinMode(this->cs_pin, OUTPUT);
}

void SRAM::begin() {
}

