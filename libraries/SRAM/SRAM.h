#include <SPI.h>

#define SRAM_READ  0b011  // Read data from memory array beginning at selected address
#define SRAM_WRITE 0b010  // Write data to memory array beginning at selected address
#define SRAM_RDSR  0b101  // Read STATUS register
#define SRAM_WRSR  0b001  // Write STATUS register

// operation modes (status register)

#define SRAM_BYTE_MODE 0b00000000  // Byte mode (default operation) 
#define SRAM_PAGE_MODE 0b10000000  // Page mode
#define SRAM_SEQN_MODE 0b01000000  // Sequential mode
#define SRAM_RSVD_MODE 0b11000000  // Reserved
#define SRAM_HOLD_MODE 0b00000001  // Set this bit to DISABLE hold mode

#define SRAM_256  0b0
#define SRAM_1024 0b1

class SRAM
{
  
public:
  uint8_t mode;
  uint8_t cs_pin;

  SRAM(uint8_t);
  void begin();  
};
