#include <Wire.h>
#include <Adafruit_ADXL345_U.h>
#include <SPI.h>
#include <SD.h>
#include <Time.h>
#include <DS1307RTC.h>
#include "SRAM.h"


const bool ticks_check = true;
const bool serial_print = true;


const byte cssd = 10;
const byte csacc = 9;
const byte csram = 8;


const uint32_t ramsz = 1024L*1024L;

SRAM sram(csram);

tmElements_t tm;




void rtc_auto_set(void)
{


  // get the date and time the compiler was run
  if (getDate(__DATE__) && getTime(__TIME__))
  {
    if (RTC.write(tm))
    {
      Serial.print(F("DS1307 configured Time="));
      Serial.print(__TIME__);
      Serial.print(F(", Date="));
      Serial.println(F(__DATE__));
    }
    else
    {
      Serial.println(F("DS1307 Communication Error :-{"));
      Serial.println(F("Please check your circuitry"));
    }
  }
  else
  {
    Serial.print(F("Could not parse info from the compiler, __TIME__=\""));
    Serial.print(F(__TIME__));
    Serial.print(F("\", __DATE__=\""));
    Serial.print(F(__DATE__));
    Serial.println("\"");
  }
}

bool getTime(const char *str)
{
  int Hour, Min, Sec;

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3)
  {
    Serial.println(F("error parsing __TIME__"));
    return false;
  }
  tm.Hour = Hour;
  tm.Minute = Min;
  tm.Second = Sec;
  return true;
}

bool getDate(const char *str)
{
  String monthName = F("JanFebMarAprMayJunJulAugSepOctNovDec");
  char Month[12];
  int Day, Year;
  uint8_t monthIndex;

  if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3)
  {
    Serial.println(F("error parsing __DATE__"));
    return false;
  }

  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (
      Month[0] == monthName[monthIndex*3+0] &&
      Month[1] == monthName[monthIndex*3+1] &&
      Month[2] == monthName[monthIndex*3+2]
    ) break;
  }
  if (monthIndex >= 12)
  {
    Serial.println(F("error"));
    return false;
  }
  tm.Day = Day;
  tm.Month = monthIndex + 1;
  tm.Year = CalendarYrToTm(Year);
  return true;
}


String now_string(void)
{
  tmElements_t tm;

  if (!RTC.read(tm))
  {
    Serial.println(F("couldnt read rtc."));
    return String("");
  }

  return time_string(tm);
}

String time_string(tmElements_t &tm)
{
  char t[40];
  sprintf(t, "%04d/%02d/%02d %02d:%02d:%02d",
    tmYearToCalendar(tm.Year),
    tm.Month,
    tm.Day,
    tm.Hour,
    tm.Minute,
    tm.Second);
  return String(t);
}

String fn_time_string(tmElements_t &tm)
{
  char t[40];
  sprintf(t, "%02d%02d%02d%02d",
    tm.Day,
    tm.Hour,
    tm.Minute,
    tm.Second);
  return String(t);
}


void test_rtc() {
  Serial.println(F("DS1307RTC Read Test"));
  String t = now_string();
  if (t == "")
  {
    if (RTC.chipPresent())
    {
      Serial.println(F("The DS1307 is stopped.  setting time..."));
      rtc_auto_set();
     }
     else
     {
      Serial.println(F("DS1307 read error!  Please check the circuitry."));
      Serial.println();
    }
  }
  Serial.println(t);
}









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

  acc.setDataRate(ADXL345_DATARATE_3200_HZ);
  
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


/*const unsigned int bufsiz = 6;
unsigned int bufpos = 0;
char buf[bufsiz];
*/
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


byte samplesize ;
uint32_t nsamples ;

void test_acc_spi(Acc &acc)
{
  if (!init_acc_spi(acc))
    return;

  samplesize = 3*2;
  if (ticks_check) samplesize += 4;
  nsamples = 10;//(ramsz/samplesize);
  toread = nsamples * samplesize;

    
  displayAccDetails(acc);  
  displayDataRate(acc);
  displayRange(acc);
  Serial.println("");

  
  digitalWrite(sram.cs_pin, HIGH);
  digitalWrite(csacc, HIGH);


  acc.writeRegister(ADXL345_REG_POWER_CTL, 0b0000);
  Serial.println(F("flush.."));

  acc_spi_flush();
  
  do_read = false;
  done = false;
  address = 0;
  //Serial.println((long)digitalRead(3));  
  attachInterrupt(digitalPinToInterrupt(3), spi_int, FALLING);  
  Serial.println(F("sampling.."));
  acc.writeRegister(ADXL345_REG_POWER_CTL, 0b1000);   // Enable measurements

  address = 1;
  while(address < 100) 
    spi_read();
  
  address = 0;
  while(!done) 
    spi_read();

  detachInterrupt(digitalPinToInterrupt(3));

  SPI.setDataMode(SPI_MODE0);  
  SPI.setClockDivider(SPI_CLOCK_DIV2); 

  
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
  noInterrupts();
  TIMSK0 = 0;
  TIMSK1 = 0;
  TCCR1A = 0;
  TCCR1B = 1;
  interrupts();
    
  Serial.begin(115200);
  
  digitalWrite(csram, 1);
  digitalWrite(cssd, 1);
  digitalWrite(csacc, 1);  
  
  pinMode(csram, 1);
  pinMode(cssd, 1);
  pinMode(csacc, 1);
 
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2); // The maximum SPI clock speed is 5 MHz with 100 pF

  init_ram();
  //test_ram();
  
  SPI.setClockDivider(SPI_CLOCK_DIV4); 
  SPI.setDataMode(SPI_MODE3);

  divider(); 

  {
  Acc accspi = Acc(13, 12, 11, csacc);
 
  test_acc_spi(accspi);
  }

  sd_out();
  
  Serial.println(F("end"));  
  
}

void divider(void)
{

  Serial.println("");
    Serial.println(F("------------------------------------"));
  Serial.println("");

}

void loop(void)
{
    
}

long dead = 0;
void spi_read(void)
{    
  if(!do_read)
  {
    dead++;
    if (dead > 1000)
      Serial.println(F("dead"));
    return;
  }
    

    dead = 0;
    do_read = false;

/*
    if (!(address & 0b11111111))
    {
     Serial.println(address );
    }
*/


      
      char buf[6];
      digitalWrite(csacc, LOW);
      SPI.setClockDivider(SPI_CLOCK_DIV4); 
      SPI.transfer(ADXL345_REG_DATAX0 | RD | MB);
      for (int a = 0; a < 6; a++)
      {
        buf[a] = SPI.transfer(0);
      }

      /*
      SPI.transfer(0); // FIFO_CTL zahodime
      byte status = SPI.transfer(0);
      byte entries;
      entries = status & 0b111111;
      //Serial.println(entries);
      if (entries > 0) 
      {
        Serial.println(F("overflow"));
      }
      */

    digitalWrite(csacc, HIGH);
    
    SPI.setDataMode(SPI_MODE0);  
    SPI.setClockDivider(SPI_CLOCK_DIV2); 
    
    digitalWrite(sram.cs_pin, LOW);
    SPI.transfer(SRAM_WRITE);
    SPI.transfer((address >> 16) & 0xFF);
    SPI.transfer((address >> 8)  & 0xFF);
    SPI.transfer(address & 0xFF);
    if (ticks_check)
    {
      address += 4;
      unsigned long cycles = (ticks + periods * 65536);
      for (int a = 0; a < 4; a++)
      {
        SPI.transfer(((byte*)&cycles)[a]);
      }
    }
    for (int a = 0; a < 6; a++)
    {
      SPI.transfer(buf[a]);
    } 
    digitalWrite(sram.cs_pin, HIGH); 
    
    SPI.setDataMode(SPI_MODE3);

    
    address += 6;
    if (address == toread)
    {
     done = true;
     Serial.println(F("done"));
    }
    
  
}


void sd_out()
{

  if(!RTC.read(tm))
  {
    Serial.println(F("couldnt read rtc."));
    return;
  }


  SDClass sd;
  if (!sd.begin()) {
    Serial.println(F("SD err"));
    return;
  }


  String fn = fn_time_string(tm) + ".csv";
   
  File file = sd.open(fn.c_str(), FILE_WRITE);
  if (!file) 
  {
    Serial.print(F("error opening "));
    Serial.println(fn);
    return;
  }

  Serial.print(F("writing "));
   Serial.println(fn);
 
  file.print(F("#"));
  file.print(time_string(tm));
  file.println("");
  if (ticks_check)
    file.print(F("ticks,"));
  file.println(F("X,Y,Z"));
  
  const unsigned int bufsiz = 12;
  unsigned int sib = bufsiz/samplesize;
  unsigned long address;

  for(address = 0; address < toread; address += sib * samplesize)
  {
  
    digitalWrite(sram.cs_pin, LOW);
    SPI.transfer(SRAM_READ);
    SPI.transfer((address >> 16) & 0xFF);
    SPI.transfer((address >> 8)  & 0xFF);
    SPI.transfer(address & 0xFF);

    char buf[bufsiz];
    unsigned int b;
    for (b = 0; b < sib * samplesize; b++)
    {
      buf[b] = SPI.transfer(0);
    }

    digitalWrite(sram.cs_pin, HIGH);


    if(!serial_print)
    {
      Serial.print(F("address:"));
      Serial.print(address);
      Serial.print(F(" toread:"));
      Serial.print(toread);
/*      Serial.print(F("sib * samplesize:"));
      Serial.print(sib * samplesize);
  */    
      Serial.println(F(".."));
    }

    unsigned int bufpos = 0;

    unsigned int s;
    for (s = 0; s < sib; s++)
    {
/*
      Serial.print(F("sib:"));
      Serial.print(sib);
      Serial.print(F("s:"));
      Serial.print(s);
      Serial.print(F("bufpos:"));
      Serial.print(bufpos);
      Serial.println(F(".."));
*/
      
     if (ticks_check)
     {
      unsigned long cycles;
      for (int a = 0; a < 4; a++)
      {
        ((byte*)&cycles)[a] = buf[bufpos++];
      }
      file.print(cycles);
      file.print(",");
      if(serial_print)
      {
        Serial.print(cycles);
        Serial.print(",");
      }
     }

     int16_t xyz[3];
     char i0,i1;
     int16_t v;
      
     for (int a = 0; a < 3; a++)
     {
       i0 = buf[bufpos++];
       i1 = buf[bufpos++];
       v = i0 | (i1 << 8);
       xyz[a] = v;
     }

     if(bufpos >= bufsiz)
        Serial.print("err");

     file.print(xyz[0]);
     file.print(",");
     file.print(xyz[1]);
     file.print(",");
     file.println(xyz[2]);
     if(serial_print)
     {
      Serial.print(xyz[0]);
      Serial.print(",");
      Serial.print(xyz[1]);
      Serial.print(",");
      Serial.println(xyz[2]);
     }
    }
  }
  
  sd.end();
  Serial.println(F("done writing."));
  
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
 /*todo reset i2c po resetu arduina?*/
