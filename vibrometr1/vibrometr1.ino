#include <Wire.h>
#include <Adafruit_ADXL345_U.h>


#include "SRAM.h"

#include <SPI.h>
#include <SD.h>

#include <Time.h>
#include <DS1307RTC.h>


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
      Serial.print(", Date=");
      Serial.println(__DATE__);
    } 
    else 
    {
      Serial.println(F("DS1307 Communication Error :-{"));
      Serial.println(F("Please check your circuitry"));
    }
  }
  else
  {
    Serial.print(F("Could not parse info from the compiler, Time=\""));
    Serial.print(__TIME__);
    Serial.print("\", Date=\"");
    Serial.print(__DATE__);
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




const byte cssd = 10;
const byte csacc = 9;
const byte csram = 8;

//rtc i2c




SRAM sram(csram);
const uint32_t ramsz = 1024L*1024L;















void test_ram() {
  uint32_t address = 0;
  const uint32_t bs = 1024L*256L;
  Serial.println(F("sram test:"));
  SPI.begin();
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
  SPI.end();
}


void init_ram(void)
{
  
  
  SPI.begin();//Transaction(sramspi);
  digitalWrite(sram.cs_pin, LOW);
  SPI.transfer(SRAM_WRSR);
  SPI.transfer(sram.mode);
  digitalWrite(sram.cs_pin, HIGH);
  SPI.end();

  
}












bool init_acc(Acc &acc)
{
  if(!acc.begin())
  {
    Serial.println(F("not detected"));
    return false;
  }
  Wire.setClock(400000L);
  //SPI.setClockDivider(SPI_CLOCK_DIV2);
  acc.setDataRate(ADXL345_DATARATE_800_HZ);
  //400 kHz I2C is 800 Hz
  
  acc.setRange(ADXL345_RANGE_16_G);

  acc.writeRegister(ADXL345_REG_FIFO_CTL, 0b10000000);//stream


  return true;
}

void test_acc(Acc &acc)
{
  if (!init_acc(acc))
    return;


  int nsamples = ramsz/2/3;
    
  displayAccDetails(acc);  
  displayDataRate(acc);
  displayRange(acc);
  Serial.println("");


  SPI.begin();
  digitalWrite(sram.cs_pin, LOW);
  SPI.transfer(SRAM_WRITE);
  //adresa
  SPI.transfer(0);
  SPI.transfer(0);
  SPI.transfer(0);


  
  tmElements_t start, end;
  RTC.read(start);
  RTC.read(end);
  time_t st = makeTime(start);
  time_t et = makeTime(end);
  
  
  
  // Enable measurements
  acc.writeRegister(ADXL345_REG_POWER_CTL, 0b1000);

  long s;
  byte status;
  int16_t xyz[3];
  for(s = 0; s < nsamples; s++)
  {

    Wire.beginTransmission(ADXL345_ADDRESS);
    i2cwrite(ADXL345_REG_DATAX0);
    Wire.endTransmission();
    Wire.requestFrom(ADXL345_ADDRESS, 8);

  //  for (int a = 0; a< 3; a++)
  //	xyz[a] = (uint16_t)(i2cread() | (i2cread() << 8));  
    

    for (int a = 0; a < 6; a++)
        SPI.transfer(i2cread());

    i2cread(); // FIFO_CTL
    status = i2cread();

    entries = status & 0b111111;
    Serial.println(entries);
    if (entries > 30) 
    	Serial.println(F("overflow"));

  }

  acc.writeRegister(ADXL345_REG_POWER_CTL, 0);
  digitalWrite(sram.cs_pin, HIGH);



  RTC.read(end);



  //flush
  for (int i = 0; i < 50; i++)
  {
    Wire.requestFrom(ADXL345_ADDRESS, 1);
    i2cread();
  }







  digitalWrite(sram.cs_pin, LOW);
  SPI.transfer(SRAM_READ);
  //adresa
  SPI.transfer(0);
  SPI.transfer(0);
  SPI.transfer(0);

  
  for(s = 0; s < nsamples; s++)
  {
    for (int a = 0; a< 3; a++)
      xyz[a] = (uint16_t)(SPI.transfer(0) | (SPI.transfer(0) << 8));
     
    if(s < 3)
    {
      Serial.print(F("X: ")); Serial.print(xyz[0]);
      Serial.print(F("  Y: ")); Serial.print(xyz[1]);
      Serial.print(F("  Z: ")); Serial.println(xyz[2]); 
    }
    if(s == 3)
      Serial.println(F("...\n"));
    
    
     
      
  }


  digitalWrite(sram.cs_pin, HIGH);
  Serial.println(F("done"));

  SPI.end();

}







void loop(void) 
{
  
  test_rtc();
divider();  
  test_ram();
divider();

Acc accspi = Acc(13, 12, 11, csacc);
Acc acctwi = Acc(12345);
Serial.println(F("spi acc.."));
  test_acc(accspi);
Serial.println(F("twi acc.."));
  test_acc(acctwi);

divider();
  test_sd();
divider();
  write_time();
divider();
  delay(500);
}

void setup(void) 
{
  Serial.begin(9600);
  
  digitalWrite(csram, 1);
  digitalWrite(cssd, 1);
  digitalWrite(csacc, 1);  
  
  pinMode(csram, 1);
  pinMode(cssd, 1);
  pinMode(csacc, 1);

  init_ram();

  divider(); 
}

void divider(void)
{

  Serial.println("");
    Serial.println(F("------------------------------------"));
  Serial.println("");

}









void test_sd(void)
{
  Sd2Card card;
  
  
  Serial.print(F("\nInitializing SD card..."));

  if (!card.init()) {
    Serial.println(F("initialization failed. Things to check:"));
    Serial.println(F("* is a card inserted?"));
    Serial.println(F("* is your wiring correct?"));
    Serial.println(F("* did you change the chipSelect pin to match your shield or module?"));
    return;
  } else {
    Serial.println(F("Wiring is correct and a card is present."));
  }

  // print the type of card
  Serial.print(F("\nCard type: "));
  switch (card.type()) {
    case SD_CARD_TYPE_SD1:
      Serial.println(F("SD1"));
      break;
    case SD_CARD_TYPE_SD2:
      Serial.println(F("SD2"));
      break;
    case SD_CARD_TYPE_SDHC:
      Serial.println(F("SDHC"));
      break;
    default:
      Serial.println(F("Unknown"));
  }





  SdVolume volume;
  
  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
    Serial.println(F("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card"));
    return;
  }


  // print the type and size of the first FAT-type volume
  uint32_t volumesize;
  Serial.print(F("\nVolume type is FAT"));
  Serial.println(volume.fatType(), DEC);
  Serial.println();

  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize *= 512;                            // SD card blocks are always 512 bytes
  Serial.print(F("Volume size (bytes): "));
  Serial.println(volumesize);
  Serial.print(F("Volume size (Kbytes): "));
  volumesize /= 1024;
  Serial.println(volumesize);
  Serial.print(F("Volume size (Mbytes): "));
  volumesize /= 1024;
  Serial.println(volumesize);



  {
  Serial.println(F("\nFiles found on the card (name, date and size in bytes): "));
  
  SdFile root;

  
  root.openRoot(volume);

  // list all files in the card with date and size
  root.ls(LS_DATE | LS_SIZE);

  }

SPI.end();
}




void write_time(void)
{
  SDClass sd;
  Serial.println(F("write time.txt..."));
  if (sd.begin()) {
    File file = sd.open("time.txt", FILE_WRITE);
    if (! file) 
      Serial.println(F("error opening time.txt"));
    file.println(now_string());
    file.flush();
    sd.end();
  }
  else
    Serial.println(F("err"));
    
  SPI.end();
}



