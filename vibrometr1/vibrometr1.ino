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
  
  digitalWrite(sram.cs_pin, LOW);
  SPI.transfer(SRAM_WRSR);
  SPI.transfer(sram.mode);
  digitalWrite(sram.cs_pin, HIGH);
  
}


void test_ram() {
  uint32_t address = 0;
  const uint32_t bs = 1024L*256L;
  Serial.println(F("sram test:"));
  for(address = 0; address < ramsz; address+=bs)
  {
     Serial.print(address);
     Serial.print(":");
/*
Mode  Clock Polarity (CPOL) Clock Phase (CPHA)
SPI_MODE0 0 0
SPI_MODE1 0 1
*/
    digitalWrite(sram.cs_pin, LOW);
    SPI.transfer(SRAM_WRITE);
    SPI.transfer((address >> 16) & 0xFF);
    SPI.transfer((address >> 8)  & 0xFF);
    SPI.transfer(address & 0xFF);
    uint32_t d;
    for(d = 0; d < bs; d++)
    {
      SPI.transfer(d);
    }
    digitalWrite(sram.cs_pin, HIGH);

    Serial.print("..");

    digitalWrite(sram.cs_pin, LOW);
    SPI.transfer(SRAM_READ);
    SPI.transfer((address >> 16) & 0xFF);
    SPI.transfer((address >> 8)  & 0xFF);
    SPI.transfer(address & 0xFF);
    for(d = 0; d < bs; d++)
    {
      byte data = SPI.transfer(0);
      byte exptd = (byte)d;
      if (data != exptd)
      {
        Serial.print(data);
        Serial.print("!=");
        Serial.println(exptd);
      }
    }
    digitalWrite(sram.cs_pin, HIGH);
    Serial.println(F("done"));
  }
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

  acc.setDataRate(ADXL345_DATARATE_800_HZ);
  
  acc.setRange(ADXL345_RANGE_16_G);
  acc.writeRegister(ADXL345_REG_FIFO_CTL, 0b10000000);//stream mode
  acc.writeRegister(ADXL345_REG_INT_ENABLE, 0b10000000);//DATA_READY (na INT1)
  return true;
}


void acc_twi_flush(unsigned int count = 50)
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





bool done;
long unsigned int address = 0;
long unsigned int toread;


const unsigned int bufsiz = 6;
unsigned int bufpos = 0;
char buf[bufsiz];

const byte RD = 1 << 7;
const byte MB = 1 << 6;

byte sample_size = 6;

volatile bool do_read;






void acc_spi_flush(unsigned int count = 50)
{
  for (int i = 0; i < count; i++)
  {
    do_read = true;
    spi_read();
  }
}


void test_acc_spi(Acc &acc)
{
  if (!init_acc_spi(acc))
    return;

  const uint32_t nsamples = /*128*10*/(ramsz/6L);
  toread = nsamples * 6;
    
  displayAccDetails(acc);  
  displayDataRate(acc);
  displayRange(acc);
  Serial.println("");

  
  digitalWrite(sram.cs_pin, HIGH);
  digitalWrite(csacc, HIGH);


  acc.writeRegister(ADXL345_REG_POWER_CTL, 0b0000);
  Serial.println("flush..");

  acc_spi_flush();
  
  do_read = false;
  done = false;
  address = 0;
  Serial.println((long)digitalRead(3));  
  attachInterrupt(digitalPinToInterrupt(3), spi_int, FALLING);  
  Serial.println("sampling..");
  acc.writeRegister(ADXL345_REG_POWER_CTL, 0b1000);   // Enable measurements
}

volatile unsigned int ticks;
volatile unsigned int old_ticks = 65536;
volatile unsigned long periods = 0;

void spi_int(void) 
{
  do_read = true;
  ticks = TCNT1;
  if (ticks < old_ticks)
  {
    periods++;
  }
  old_ticks = ticks;
}

void setup(void) 
{
  TIMSK0 = 0;
  TIMSK1 = 0;
  TCCR1A = 0;
  TCCR1B = 1;
  
    
  Serial.begin(115200);
  
  digitalWrite(csram, 1);
  digitalWrite(cssd, 1);
  digitalWrite(csacc, 1);  
  
  pinMode(csram, 1);
  pinMode(cssd, 1);
  pinMode(csacc, 1);
 
  SPI.setClockDivider(SPI_CLOCK_DIV4); // The maximum SPI clock speed is 5 MHz with 100 pF
  SPI.begin();//Transaction(sramspi);

  init_ram();
  test_ram();
  
  SPI.setDataMode(SPI_MODE3);

  divider(); 
  
  Acc accspi = Acc(13, 12, 11, csacc);
 
  test_acc_spi(accspi);

  Serial.println(F("setup done"));  
  
}

void divider(void)
{

  Serial.println("");
    Serial.println(F("------------------------------------"));
  Serial.println("");

}

void loop(void)
{
    //Serial.println((long)digitalRead(3));  
    spi_read();
}
 
inline void spi_read(void)
{    
  if(do_read)
  {
    do_read = false;
    /*if (!(address & 0b11111111))
    {
       Serial.println(address );
    }*/
    Serial.println(ticks + periods * 65536);
    address += 6;
    if (address == toread)
    {
     detachInterrupt(digitalPinToInterrupt(3));
     done = true;
     Serial.println(F("done"));
    }
    
  
    for(int i = 0; i < 10; i++)
    {
    char buf[6];
    digitalWrite(csacc, LOW);
    SPI.transfer(ADXL345_REG_DATAX0 | RD | MB);
    for (int a = 0; a < 6; a++)
    {
      buf[a] = SPI.transfer(0);
    }
    SPI.transfer(0); // FIFO_CTL zahodime
    byte status = SPI.transfer(0);
    byte entries;
    entries = status & 0b111111;
    //Serial.println(entries);
    if (entries > 30) 
      Serial.println(F("overflow"));
    }

    digitalWrite(csacc, HIGH);
    
    SPI.setDataMode(SPI_MODE0);  
    
    digitalWrite(sram.cs_pin, LOW);
    SPI.transfer(SRAM_WRITE);
    SPI.transfer((address >> 16) & 0xFF);
    SPI.transfer((address >> 8)  & 0xFF);
    SPI.transfer(address & 0xFF);
    for (int a = 0; a < 6; a++)
    {
      SPI.transfer(buf[a]);
    } 
    digitalWrite(sram.cs_pin, HIGH); 
    
    SPI.setDataMode(SPI_MODE3);
  }
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
 
