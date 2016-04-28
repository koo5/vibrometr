#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

typedef Adafruit_ADXL345_Unified Acc;

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








void displayAccDetails(Acc &acc)
{
  sensor_t sensor;
  acc.getSensor(&sensor);
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); 
  Serial.print(sensor.resolution); Serial.println(F(" m/s^2"));  
}

void divider(void)
{

  Serial.println("");
    Serial.println(F("------------------------------------"));
  Serial.println("");

}

void displayDataRate(Acc &acc)
{
  Serial.print  (F("Data Rate:    ")); 
  
  switch(acc.getDataRate())
  {
    case ADXL345_DATARATE_3200_HZ:
      Serial.print  ("3200 "); 
      break;
    case ADXL345_DATARATE_1600_HZ:
      Serial.print  ("1600 "); 
      break;
    case ADXL345_DATARATE_800_HZ:
      Serial.print  ("800 "); 
      break;
    case ADXL345_DATARATE_400_HZ:
      Serial.print  ("400 "); 
      break;
    case ADXL345_DATARATE_200_HZ:
      Serial.print  ("200 "); 
      break;
    case ADXL345_DATARATE_100_HZ:
      Serial.print  ("100 "); 
      break;
    case ADXL345_DATARATE_50_HZ:
      Serial.print  ("50 "); 
      break;
    case ADXL345_DATARATE_25_HZ:
      Serial.print  ("25 "); 
      break;
    case ADXL345_DATARATE_12_5_HZ:
      Serial.print  ("12.5 "); 
      break;
    case ADXL345_DATARATE_6_25HZ:
      Serial.print  ("6.25 "); 
      break;
    case ADXL345_DATARATE_3_13_HZ:
      Serial.print  ("3.13 "); 
      break;
    case ADXL345_DATARATE_1_56_HZ:
      Serial.print  ("1.56 "); 
      break;
    case ADXL345_DATARATE_0_78_HZ:
      Serial.print  ("0.78 "); 
      break;
    case ADXL345_DATARATE_0_39_HZ:
      Serial.print  ("0.39 "); 
      break;
    case ADXL345_DATARATE_0_20_HZ:
      Serial.print  ("0.20 "); 
      break;
    case ADXL345_DATARATE_0_10_HZ:
      Serial.print  ("0.10 "); 
      break;
    default:
      Serial.print  ("???? "); 
      break;
  }  
  Serial.println(" Hz");  
}

void displayRange(Acc &acc)
{
  Serial.print  (F("Range:         +/- ")); 
  
  switch(acc.getRange())
  {
    case ADXL345_RANGE_16_G:
      Serial.print  ("16 "); 
      break;
    case ADXL345_RANGE_8_G:
      Serial.print  ("8 "); 
      break;
    case ADXL345_RANGE_4_G:
      Serial.print  ("4 "); 
      break;
    case ADXL345_RANGE_2_G:
      Serial.print  ("2 "); 
      break;
    default:
      Serial.print  ("?? "); 
      break;
  }  
  Serial.println(" g");  
}





















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







bool init_acc(Acc &acc)
{
  Serial.println(F("init acc.."));
  
  /* Initialise the sensor */
  if(!acc.begin())
  {
    Serial.println(F("not detected"));
    return false;
  }
  acc.setDataRate(ADXL345_DATARATE_3200_HZ);
  /* Set the range to whatever is appropriate for your project */
  acc.setRange(ADXL345_RANGE_16_G);
  // displaySetRange(ADXL345_RANGE_8_G);
  // displaySetRange(ADXL345_RANGE_4_G);
  // displaySetRange(ADXL345_RANGE_2_G);
  return true;
}

void test_acc(Acc &acc)
{
  if (!init_acc(acc))
    return;
    
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  /* Display some basic information on this sensor */
  displayAccDetails(acc);  
  /* Display additional settings (outside the scope of sensor_t) */
  displayDataRate(acc);
  displayRange(acc);
  Serial.println("");
  
  long x;
  for(x = 0; x < 100; x++)
  {
    sensors_event_t event; 
    acc.getEvent(&event);
 
    Serial.print(F("X: ")); Serial.print(event.acceleration.x);
    Serial.print(F("  Y: ")); Serial.print(event.acceleration.y);
    Serial.print(F("  Z: ")); Serial.println(event.acceleration.z); 
  }
  
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


