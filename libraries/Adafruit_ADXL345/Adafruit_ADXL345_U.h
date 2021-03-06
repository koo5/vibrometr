/**************************************************************************/
/*!
    @file     Adafruit_ADS1015.h
    @author   K. Townsend (Adafruit Industries)
    @license  BSD (see license.txt)

    This is a library for the Adafruit ADS1015 breakout board
    ----> https://www.adafruit.com/products/???

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

 #include "Arduino.h"






#define SENSORS_GRAVITY_EARTH             (9.80665F)              /**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_MOON              (1.6F)                  /**< The moon's gravity in m/s^2 */
#define SENSORS_GRAVITY_SUN               (275.0F)                /**< The sun's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD          (SENSORS_GRAVITY_EARTH)
#define SENSORS_MAGFIELD_EARTH_MAX        (60.0F)                 /**< Maximum magnetic field on Earth's surface */
#define SENSORS_MAGFIELD_EARTH_MIN        (30.0F)                 /**< Minimum magnetic field on Earth's surface */
#define SENSORS_PRESSURE_SEALEVELHPA      (1013.25F)              /**< Average sea level pressure is 1013.25 hPa */
#define SENSORS_DPS_TO_RADS               (0.017453293F)          /**< Degrees/s to rad/s multiplier */
#define SENSORS_GAUSS_TO_MICROTESLA       (100)                   /**< Gauss to micro-Tesla multiplier */







#include <Wire.h>

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define ADXL345_ADDRESS                 (0x53)    // Assumes ALT address pin low
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    #define ADXL345_REG_DEVID               (0x00)    // Device ID
    #define ADXL345_REG_THRESH_TAP          (0x1D)    // Tap threshold
    #define ADXL345_REG_OFSX                (0x1E)    // X-axis offset
    #define ADXL345_REG_OFSY                (0x1F)    // Y-axis offset
    #define ADXL345_REG_OFSZ                (0x20)    // Z-axis offset
    #define ADXL345_REG_DUR                 (0x21)    // Tap duration
    #define ADXL345_REG_LATENT              (0x22)    // Tap latency
    #define ADXL345_REG_WINDOW              (0x23)    // Tap window
    #define ADXL345_REG_THRESH_ACT          (0x24)    // Activity threshold
    #define ADXL345_REG_THRESH_INACT        (0x25)    // Inactivity threshold
    #define ADXL345_REG_TIME_INACT          (0x26)    // Inactivity time
    #define ADXL345_REG_ACT_INACT_CTL       (0x27)    // Axis enable control for activity and inactivity detection
    #define ADXL345_REG_THRESH_FF           (0x28)    // Free-fall threshold
    #define ADXL345_REG_TIME_FF             (0x29)    // Free-fall time
    #define ADXL345_REG_TAP_AXES            (0x2A)    // Axis control for single/double tap
    #define ADXL345_REG_ACT_TAP_STATUS      (0x2B)    // Source for single/double tap
    #define ADXL345_REG_BW_RATE             (0x2C)    // Data rate and power mode control
    #define ADXL345_REG_POWER_CTL           (0x2D)    // Power-saving features control
    #define ADXL345_REG_INT_ENABLE          (0x2E)    // Interrupt enable control
    #define ADXL345_REG_INT_MAP             (0x2F)    // Interrupt mapping control
    #define ADXL345_REG_INT_SOURCE          (0x30)    // Source of interrupts
    #define ADXL345_REG_DATA_FORMAT         (0x31)    // Data format control
    #define ADXL345_REG_DATAX0              (0x32)    // X-axis data 0
    #define ADXL345_REG_DATAX1              (0x33)    // X-axis data 1
    #define ADXL345_REG_DATAY0              (0x34)    // Y-axis data 0
    #define ADXL345_REG_DATAY1              (0x35)    // Y-axis data 1
    #define ADXL345_REG_DATAZ0              (0x36)    // Z-axis data 0
    #define ADXL345_REG_DATAZ1              (0x37)    // Z-axis data 1
    #define ADXL345_REG_FIFO_CTL            (0x38)    // FIFO control
    #define ADXL345_REG_FIFO_STATUS         (0x39)    // FIFO status
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    #define ADXL345_MG2G_MULTIPLIER (0.004)  // 4mg per lsb
/*=========================================================================*/

/* Used with register 0x2C (ADXL345_REG_BW_RATE) to set bandwidth */
const byte
  ADXL345_DATARATE_3200_HZ    = 0b1111; // 1600Hz Bandwidth   140�A IDD
const byte 
  ADXL345_DATARATE_1600_HZ    = 0b1110; //  800Hz Bandwidth    90�A IDD
const byte 
  ADXL345_DATARATE_800_HZ     = 0b1101; //  400Hz Bandwidth   140�A IDD
const byte 
  ADXL345_DATARATE_400_HZ     = 0b1100; //  200Hz Bandwidth   140�A IDD
const byte 
  ADXL345_DATARATE_200_HZ     = 0b1011; //  100Hz Bandwidth   140�A IDD
const byte 
  ADXL345_DATARATE_100_HZ     = 0b1010; //   50Hz Bandwidth   140�A IDD
const byte 
  ADXL345_DATARATE_50_HZ      = 0b1001; //   25Hz Bandwidth    90�A IDD
const byte 
  ADXL345_DATARATE_25_HZ      = 0b1000; // 12.5Hz Bandwidth    60�A IDD
const byte 
  ADXL345_DATARATE_12_5_HZ    = 0b0111; // 6.25Hz Bandwidth    50�A IDD
const byte 
  ADXL345_DATARATE_6_25HZ     = 0b0110; // 3.13Hz Bandwidth    45�A IDD
const byte 
  ADXL345_DATARATE_3_13_HZ    = 0b0101; // 1.56Hz Bandwidth    40�A IDD
const byte 
  ADXL345_DATARATE_1_56_HZ    = 0b0100; // 0.78Hz Bandwidth    34�A IDD
const byte 
  ADXL345_DATARATE_0_78_HZ    = 0b0011; // 0.39Hz Bandwidth    23�A IDD
const byte 
  ADXL345_DATARATE_0_39_HZ    = 0b0010; // 0.20Hz Bandwidth    23�A IDD
const byte 
  ADXL345_DATARATE_0_20_HZ    = 0b0001; // 0.10Hz Bandwidth    23�A IDD
const byte 
  ADXL345_DATARATE_0_10_HZ    = 0b0000; // 0.05Hz Bandwidth    23�A IDD (default value)

typedef byte dataRate_t;

/* Used with register 0x31 (ADXL345_REG_DATA_FORMAT) to set g range */
typedef enum
{
  ADXL345_RANGE_16_G          = 0b11,   // +/- 16g
  ADXL345_RANGE_8_G           = 0b10,   // +/- 8g
  ADXL345_RANGE_4_G           = 0b01,   // +/- 4g
  ADXL345_RANGE_2_G           = 0b00    // +/- 2g (default value)
} range_t;

class Acc {
 public:
  Acc();
  Acc(uint8_t cs);

  bool       begin(void);
  void       setRange(range_t range);
  range_t    getRange(void);
  void       setDataRate(dataRate_t dataRate);
  dataRate_t getDataRate(void);

  uint8_t    getDeviceID(void);
  void       writeRegister(uint8_t reg, uint8_t value);
  uint8_t    readRegister(uint8_t reg);


  bool    _i2c; //ro


 private:

  uint8_t _cs;
};



void displayAccDetails(Acc &acc);
void displayDataRate(Acc &acc);
void displayRange(Acc &acc);


typedef unsigned long HZ;


HZ dataRate2hz(dataRate_t dataRate);
String hz_str(HZ hz);
String range_str(range_t r);



