/*todo reset i2c po resetu arduina?*/



#include <Wire.h>
#include <Adafruit_ADXL345_U.h>


#include "SRAM.h"

#include <SPI.h>


const byte cssd = 10;
const byte csacc = 9;
const byte csram = 8;

//rtc i2c




SRAM sram(csram);
const uint32_t ramsz = 1024L*1024L;
















void init_ram(void)
{
  
  
  SPI.begin();//Transaction(sramspi);
  digitalWrite(sram.cs_pin, LOW);
  SPI.transfer(SRAM_WRSR);
  SPI.transfer(sram.mode);
  digitalWrite(sram.cs_pin, HIGH);
  SPI.end();

  
}





















bool init_acc_twi(Acc &acc)
{
  if(!acc.begin())
  {
    Serial.println(F("not detected"));
    return false;
  }
  Wire.setClock(400000L);
  acc.setDataRate(ADXL345_DATARATE_800_HZ);
  
  acc.setRange(ADXL345_RANGE_16_G);
  acc.writeRegister(ADXL345_REG_FIFO_CTL, 0b10000000);//stream mode
  acc.writeRegister(ADXL345_REG_INT_ENABLE, 0b10000000);//DATA_READY (na INT1)
  return true;
}

bool init_acc_spi(Acc &acc)
{
  if(!acc.begin())
  {
    Serial.println(F("not detected"));
    return false;
  }
  SPI.setClockDivider(SPI_CLOCK_DIV4); // The maximum SPI clock speed is 5 MHz with 100 pF
maximum loading
  acc.setDataRate(ADXL345_DATARATE_3200_HZ);
  
  acc.setRange(ADXL345_RANGE_16_G);
  acc.writeRegister(ADXL345_REG_FIFO_CTL, 0b10000000);//stream mode
  acc.writeRegister(ADXL345_REG_INT_ENABLE, 0b10000000);//DATA_READY (na INT1)
  return true;
}


void acc_flush(unsigned int count = 50)
{
  for (int i = 0; i < count; i++)
  {
    Wire.beginTransmission(ADXL345_ADDRESS);
    Wire.write(ADXL345_REG_DATAX0);
    Wire.endTransmission();
    Wire.requestFrom(ADXL345_ADDRESS, 1);
    Wire.read();
  }
}


void acc_spi_flush(unsigned int count = 50)
{
  for (int i = 0; i < count; i++)
  {
    Wire.beginTransmission(ADXL345_ADDRESS);
    Wire.write(ADXL345_REG_DATAX0);
    Wire.endTransmission();
    Wire.requestFrom(ADXL345_ADDRESS, 1);
    Wire.read();
  }
}


void acc_spi_wait(void)
{
  while(true)
  {
    byte status;

    Wire.beginTransmission(ADXL345_ADDRESS);
    //Wire.write(ADXL345_REG_FIFO_STATUS);
    Wire.write(ADXL345_REG_INT_SOURCE);
    Wire.endTransmission();
    Wire.requestFrom(ADXL345_ADDRESS, 1);

    status = Wire.read();

    /*byte entries;
    entries = status & 0b111111;
    if (entries != 0) 
      break;
      */
    if (status & (1<<7)) break;
    //Serial.println(F("waiting for samples.."));
  }
}


void test_acc(Acc &acc)
{
  if (!init_acc(acc))
    return;


  const uint32_t nsamples = /*128*10*/(ramsz/6L);
    
  displayAccDetails(acc);  
  displayDataRate(acc);
  displayRange(acc);
  Serial.println("");


  SPI.begin();




  acc_flush();
  // Enable measurements
  acc.writeRegister(ADXL345_REG_POWER_CTL, 0b1000);
  acc_wait();
  acc_flush();

  digitalWrite(sram.cs_pin, LOW);
  SPI.transfer(SRAM_WRITE);
  //adresa
  SPI.transfer(0);
  SPI.transfer(0);
  SPI.transfer(0);


  attachInterrupt(digitalPinToInterrupt(2), rd, FALLING);  
}


void test_acc_spi(Acc &acc)
{
  if (!init_acc_spi(acc))
    return;

  const uint32_t nsamples = /*128*10*/(ramsz/6L);
    
  displayAccDetails(acc);  
  displayDataRate(acc);
  displayRange(acc);
  Serial.println("");


  SPI.begin();


  digitalWrite(sram.cs_pin, HIGH);

  digitalWrite(csacc, LOW);
  acc_spi_flush();
  digitalWrite(csacc, HIGH);
  
  acc.writeRegister(ADXL345_REG_POWER_CTL, 0b1000);   // Enable measurements
  acc_spi_wait();
  
  digitalWrite(csacc, LOW);
  acc_spi_flush();

  digitalWrite(sram.cs_pin, LOW);
  SPI.transfer(SRAM_WRITE);
  //adresa
  SPI.transfer(0);
  SPI.transfer(0);
  SPI.transfer(0);


  attachInterrupt(digitalPinToInterrupt(3), rd, FALLING);  
}

volatile bool do_read = false;

void rd(void)
{
  do_read = true;    
}

const unsigned int bufsiz = 128;
unsigned int bufpos = 0;
char[bufsiz] buf;

const byte RD = 1 << 7;
const byte MB = 1 << 6;

byte sample_size = 6;

void spi_loop(void) 
{
  if (do_read)
  {
    do_read = false;
    digitalWrite(csacc, LOW);
    SPI.transfer(ADXL345_REG_DATAX0 | RD | MB);

    for (int a = 0; a < 6; a++)
    {
      buf[bufpos++] = SPI.transfer(0);
    }

    SPI.transfer(0); // FIFO_CTL zahodime
   
    byte status = SPI.transfer(0);
    byte entries;
    entries = status & 0b111111;
    //Serial.println(entries);
    if (entries > 30) 
      Serial.println(F("overflow"));
  }
}

void setup(void) 
{
  Serial.begin(115200);
  
  digitalWrite(csram, 1);
  digitalWrite(cssd, 1);
  digitalWrite(csacc, 1);  
  
  pinMode(csram, 1);
  pinMode(cssd, 1);
  pinMode(csacc, 1);

  init_ram();

  divider(); 
  
  Acc acctwi = Acc(12345);
  Serial.println(F("twi acc.."));
  test_acc(acctwi);

  Acc accspi = Acc(13, 12, 11, csacc);


  
}

void divider(void)
{

  Serial.println("");
    Serial.println(F("------------------------------------"));
  Serial.println("");

}




/*
 * > - uvidime jak rychle to pujde a pak jak dlouhy maximalni casovy usek s co nejrychlejsim vzorkovanim se vejde do pameti
> - zapsat data z pameti na kartu a pak pokracovat v dalsim nabirani
>
> Data na kartu zapisujte rekneme ve formatu, prvni radek hlavicka
> HH:MM:SS DD-MM-YYYY
> (vase ostatni udaje)
> X, Y, Z
> a pak uz jenom data na kazdy radek tri cisla oddelena carkou znamenajici X,Y,Z


> Tedy ted hlavne udelat tu zakladni merici smycku popsanou vyse, aby behala spolehlive a naber dat bylo co nejpravidelnejsi. Ukladani na kartu pak, to uz samozrejme nijak rovnomerne byt nemusi, to uz jenom co nejrychleji, ale to vzorkovani ze senzoru musi byt pravidelne, protoze jinak by se z toho nedaly vydolovat zadne uzitecne udaje.



*/
 
