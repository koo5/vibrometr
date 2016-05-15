#include <Wire.h>
#include <Adafruit_ADXL345_U.h>
#include <SPI.h>
#include <SD.h>
#include <Time.h>
#include <DS1307RTC.h>
#include "SRAM.h"

/*true/false*/
const bool ticks_check = false;
const bool serial_print = 0;

const dataRate_t datarate = ADXL345_DATARATE_3200_HZ;

uint32_t nsamples = 0;

const byte cssd = 10;
const byte csacc = 9;
const byte csram = 8;

const uint32_t ramsz = 1024L*1024L;





SRAM sram(csram);
tmElements_t end_tm;
byte samplesize ;










int I2C_ClearBus() {
/**
 * This routine turns off the I2C bus and clears it
 * on return SCA and SCL pins are tri-state inputs.
 * You need to call Wire.begin() after this to re-enable I2C
 * This routine does NOT use the Wire library at all.
 *
 * returns 0 if bus cleared
 *         1 if SCL held low.
 *         2 if SDA held low by slave clock stretch for > 2sec
 *         3 if SDA held low after 20 clocks.
 */
 
#if defined(TWCR) && defined(TWEN)
  TWCR &= ~(_BV(TWEN)); //Disable the Atmel 2-Wire interface so we can control the SDA and SCL pins directly
#endif

  pinMode(SDA, INPUT_PULLUP); // Make SDA (data) and SCL (clock) pins Inputs with pullup.
  pinMode(SCL, INPUT_PULLUP);

//  delay(2500);  // Wait 2.5 secs. This is strictly only necessary on the first power
  // up of the DS3231 module to allow it to initialize properly,
  // but is also assists in reliable programming of FioV3 boards as it gives the
  // IDE a chance to start uploaded the program
  // before existing sketch confuses the IDE by sending Serial data.

  boolean SCL_LOW = (digitalRead(SCL) == LOW); // Check is SCL is Low.
  if (SCL_LOW) { //If it is held low Arduno cannot become the I2C master. 
    return 1; //I2C bus error. Could not clear SCL clock line held low
  }

  boolean SDA_LOW = (digitalRead(SDA) == LOW);  // vi. Check SDA input.
  int clockCount = 20; // > 2x9 clock

  while (SDA_LOW && (clockCount > 0)) { //  vii. If SDA is Low,
    clockCount--;
  // Note: I2C bus is open collector so do NOT drive SCL or SDA high.
    pinMode(SCL, INPUT); // release SCL pullup so that when made output it will be LOW
    pinMode(SCL, OUTPUT); // then clock SCL Low
    delayMicroseconds(10); //  for >5uS
    pinMode(SCL, INPUT); // release SCL LOW
    pinMode(SCL, INPUT_PULLUP); // turn on pullup resistors again
    // do not force high as slave may be holding it low for clock stretching.
    delayMicroseconds(10); //  for >5uS
    // The >5uS is so that even the slowest I2C devices are handled.
    SCL_LOW = (digitalRead(SCL) == LOW); // Check if SCL is Low.
    int counter = 20;
    while (SCL_LOW && (counter > 0)) {  //  loop waiting for SCL to become High only wait 2sec.
      counter--;
      delay(100);
      SCL_LOW = (digitalRead(SCL) == LOW);
    }
    if (SCL_LOW) { // still low after 2 sec error
      return 2; // I2C bus error. Could not clear. SCL clock line held low by slave clock stretch for >2sec
    }
    SDA_LOW = (digitalRead(SDA) == LOW); //   and check SDA input again and loop
  }
  if (SDA_LOW) { // still low
    return 3; // I2C bus error. Could not clear. SDA data line held low
  }

  // else pull SDA line low for Start or Repeated Start
  pinMode(SDA, INPUT); // remove pullup.
  pinMode(SDA, OUTPUT);  // and then make it LOW i.e. send an I2C Start or Repeated start control.
  // When there is only one I2C master a Start or Repeat Start has the same function as a Stop and clears the bus.
  /// A Repeat Start is a Start occurring after a Start with no intervening Stop.
  delayMicroseconds(10); // wait >5uS
  pinMode(SDA, INPUT); // remove output low
  pinMode(SDA, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  delayMicroseconds(10); // x. wait >5uS
  pinMode(SDA, INPUT); // and reset pins as tri-state inputs which is the default state on reset
  pinMode(SCL, INPUT);
  return 0; // all ok
}


void clear_i2c() {
  
  int rtn = I2C_ClearBus(); // clear the I2C bus first before calling Wire.begin()
  if (rtn != 0) {
    Serial.println(F("I2C bus error. Could not clear"));
    if (rtn == 1) {
      Serial.println(F("SCL clock line held low"));
    } else if (rtn == 2) {
      Serial.println(F("SCL clock line held low by slave clock stretch"));
    } else if (rtn == 3) {
      Serial.println(F("SDA data line held low"));
    }
  } 
  else
  { 
    Serial.println(F("i2c bus clear"));
    // re-enable Wire
    // now can start Wire Arduino master
    Wire.begin();
  }
}




void rtc_auto_set(void)
{
  tmElements_t tm;

  // get the date and time the compiler was run
  if (getDate(tm, __DATE__) && getTime(tm, __TIME__))
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

bool getTime(tmElements_t &tm, const char *str)
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

bool getDate(tmElements_t & tm, const char *str)
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




























/**************************************************************************/









bool init_acc_spi(Acc &acc)
{
  if(!acc.begin())
  {
    Serial.println(F("not detected"));
    return false;
  }

  acc.setDataRate(datarate);
  
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



void test_acc_spi(Acc &acc)
{
  if (!init_acc_spi(acc))
    return;

    
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
  Serial.print(F("start: "));
  Serial.println(now_string());
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
  Serial.begin(115200);
 
  clear_i2c();
  test_rtc();
  
  if(ticks_check)
  {
    noInterrupts();
    TIMSK0 = 0;
    TIMSK1 = 0;
    TCCR1A = 0;
    TCCR1B = 1;
    interrupts();
  }
      
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


  samplesize = 3*2;
  if (ticks_check) samplesize += 4;
  nsamples = nsamples ? nsamples : (ramsz/samplesize);
  toread = nsamples * samplesize;


  Acc accspi = Acc(13, 12, 11, csacc);
  test_acc_spi(accspi);
  if (!RTC.read(end_tm))   {    Serial.println(F("couldnt read rtc."));   }
  Serial.print(F("end: "));   Serial.println(time_string(end_tm));


  //test_sd_speed();
  sd_out();
  test_sd();
  

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
      /*nezvladali jsme dost rychle cist a clearovat interrupt flag, takze 
       * uz nedostavame interrupt pri novem samplu
       */
    return;
  }
    

    dead = 0;
    do_read = false;


    if (!(address & 0b11111111))
    {
     Serial.print('.');
    }

      
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
    address += 6;
    for (int a = 0; a < 6; a++)
    {
      SPI.transfer(buf[a]);
    } 
    
    
    digitalWrite(sram.cs_pin, HIGH); 
    
    SPI.setDataMode(SPI_MODE3);

    
    
    if (address == toread)
    {
     done = true;
     Serial.println(F("done"));
    }
    
  
}


void w_timing_end(time_t w_start_time)
{
  tmElements_t w_end_tm;
  RTC.read(w_end_tm);
  Serial.print(F("done writing in "));
  Serial.print(makeTime(w_end_tm) - w_start_time);
  //http://pastebin.com/sfEjA94n
  Serial.println(F(" seconds."));  
}



#define ifsp(x)
//#define ifsp(x) if(serial_print) {x}
#define nl  {ifsp(Serial.println("");) file.print('\n');}
#define out(x) {ifsp(Serial.print(x);) file.print(x);}

void sd_out()
{


  tmElements_t w_start_tm, w_end_tm;
  RTC.read(w_start_tm);
  time_t w_start_time = makeTime(w_start_tm);

  
  Serial.print(F("ok..")); 
  SDClass sd;
  Serial.print(F("ok..")); 
  if (!sd.begin()) {
    Serial.println(F("SD err"));
    return;
  }
  Serial.print(F("ok..")); 

  
  String fn = fn_time_string(end_tm) + ".csv";
  Serial.print(F("filename:")); 
  Serial.println(fn);
   
  File file = sd.open(fn.c_str(), FILE_WRITE);
  if (!file) 
  {
    Serial.print(F("error opening "));
    Serial.println(fn);
    return;
  }

  Serial.print(F("toread bytes:"));
  Serial.println(toread);
  Serial.println(F("writing...")); 
 
  out(time_string(end_tm));
  nl;
  out(F("nsamples:")); out(nsamples);
  out(F(" datarate:")); out(datarate);
  nl;
  
  const unsigned int bufsiz = 156;
  unsigned int sib = (bufsiz/samplesize)&~1;
  unsigned long address;
  const unsigned int bufused = sib * samplesize;

  for(address = 0; address + bufused < toread; address += bufused)
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


    //if(!serial_print)
    {
      static byte x = 0;
      if (!(x++ & 0b1111))
      {
        Serial.print(F("address:"));
        Serial.print(address);
        /*Serial.print(F("sib * samplesize:"));
        Serial.print(sib * samplesize);*/    
        Serial.print(F(".."));
  
        RTC.read(w_end_tm);
        Serial.print(address / (makeTime(w_end_tm) - w_start_time));
        Serial.println(F(" samplebytes/second"));  
      }
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

      /*
     if (ticks_check)
     {
       
       unsigned long cycles;
       for (int a = 0; a < 4; a++)
       {
         ((byte*)&cycles)[a] = buf[bufpos++];
       }
     
       out(cycles);
       out(",");
     
     }*/

     int16_t xyz[3];
     char i0,i1;
     int16_t v;

     /*if(bufpos >= bufsiz)
        Serial.print("err");*/
      
     /*for (int a = 0; a < 3; a++)
     {
       i0 = buf[bufpos++];
       i1 = buf[bufpos++];
       v = i0 | (i1 << 8);
       xyz[a] = v;
     }*/

     for (int a = 0; a < 6; a++)
     {
       ((byte*) xyz)[a] = buf[bufpos++];
     }

    /* out(xyz[0]);
     out(",");
     out(xyz[1]);
     out(",");
     out(xyz[2]);*/
     /*out("xyz[0]");
     out(',');
     out("xyz[1]");
     out(',');
     out("xyz[2]" );
     nl;*/
     char buffer[50];
     sprintf(buffer, "%d,%d,%d\n", xyz[0],xyz[1],xyz[2]);
     out(buffer);
    }
  }
  file.flush();
  sd.end();


  w_timing_end(w_start_time);

  
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

}


void test_sd_speed()
{

  tmElements_t w_start_tm, w_end_tm;
  RTC.read(w_start_tm);
  time_t w_start_time = makeTime(w_start_tm);
  
  Serial.print(F("test sd speed..")); 
  SDClass sd;
  Serial.print(F("ok..")); 
  if (!sd.begin()) {
    Serial.println(F("SD err"));
    return;
  }
  Serial.print(F("ok..")); 
  
  String fn = "test";
  Serial.print(F("filename:")); 
  Serial.println(fn);
   
  File file = sd.open(fn.c_str(), FILE_WRITE);
  if (!file) 
  {
    Serial.print(F("error opening "));
    Serial.println(fn);
    return;
  }

  Serial.println(F("writing...")); 

  for(unsigned long x = 0; x < 2100000 / 3; x++)
    file.print("tes");

  file.flush();
  sd.end();

  w_timing_end(w_start_time);
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
