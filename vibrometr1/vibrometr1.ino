#include <Wire.h>
#include <Adafruit_ADXL345_U.h>
#include <SPI.h>
#include <SD.h>
#include <Time.h>
#include <DS1307RTC.h>
#include "SRAM.h"

#define SPRINTF sprintf

/*konfigurace*/

/*true/false*/
const bool ticks_check = false;
const bool serial_print = 1;

dataRate_t conf_datarate = ADXL345_DATARATE_3200_HZ;//1600,800,400..


/*
In [5]: (1024**2)/8/6/3200
Out[5]: 6.826666666666666*/
unsigned int conf_duration = 6;

//uint32_t conf_nsamples = 0;//0=10s

const byte cssd = 10;
const byte csacc = 9;
const byte csram = 8;

const uint32_t ramsz = 1024L*1024L/8L;

/*interni*/

SRAM sram(csram);
tmElements_t end_tm;
byte samplesize ;
long dead;
long unsigned toread; //bytes
dataRate_t datarate = ADXL345_DATARATE_3200_HZ;//1600,800,400..
uint32_t nsamples = 6;


void reconf()
{
  datarate = conf_datarate;
  
  samplesize = 3*2;
  if (ticks_check) samplesize += 4;
  //nsamples = (nsamples ? nsamples : (ramsz/samplesize));


  unsigned int hz = 3200/(16-datarate);
  
  nsamples = conf_duration * hz;
 
  if (ramsz/samplesize < nsamples)
  {
    Serial.println(F("error: not enough ram"));
    nsamples = ramsz/samplesize;
  }

  nsamples &= ~1;
  
  toread = nsamples * samplesize;
}





void divider(void)
{
   Serial.println(F("\n-------------\n"));
}

void loop(void)
{
   
  menu();

}




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
  
  if (I2C_ClearBus() != 0) {
    Serial.println(F("I2C err"));
    /*if (rtn == 1) {
      Serial.println(F("SCL clock line held low"));
    } else if (rtn == 2) {
      Serial.println(F("SCL clock line held low by slave clock stretch"));
    } else if (rtn == 3) {
      Serial.println(F("SDA data line held low"));
    }*/
  } 
  else
  { 
    //Serial.println(F("i2c bus clear"));
    // re-enable Wire
    // now can start Wire Arduino master
    Wire.begin();
  }
}














bool rtc_auto_set(void)
{
  tmElements_t tm;
  tm.Second = tm.Minute = tm.Hour = tm.Day = tm.Month = tm.Year = 1;

    if (RTC.write(tm))
    {
      Serial.println(F("time set."));
      /*Serial.print(F("DS1307 configured Time="));
      Serial.print(__TIME__);
      Serial.print(F(", Date="));
      Serial.println(F(__DATE__));*/
      return true;
    }
    else
    {
      Serial.println(F("rtc err"));
      return false;
    }
}



String now_string(void)
{
  tmElements_t tm;

  if (RTC.read(tm))
    return time_string(tm);
  else
  {
    if (RTC.chipPresent())
    {
      Serial.println(F("RTC stopped. setting time..."));
      if(rtc_auto_set())
        return now_string();
    }
    else
    {
      String r = String(F("RTCerror"));
      Serial.println(r);//!  Please check the circuitry.
    }
  }
  
}

String time_string(tmElements_t &tm)
{
  char t[40];
  SPRINTF(t, "%04d/%02d/%02d %02d:%02d:%02d",
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
  SPRINTF(t, "%02d%02d%02d%02d",
    tm.Day,
    tm.Hour,
    tm.Minute,
    tm.Second);
  return String(t);
}


void test_rtc() {
  Serial.println(F("DS1307RTC Test"));
  for (byte i = 0; i < 10; i++)
  {
    Serial.println(now_string());
    delay(1000);
  }
}















void init_ram(void)
{
//  SPI.setClockDivider(SPI_CLOCK_DIV2); 
  digitalWrite(sram.cs_pin, LOW);
  SPI.transfer(SRAM_WRSR);
  SPI.transfer(sram.mode);
  digitalWrite(sram.cs_pin, HIGH);
}

void sram_start(byte mode, unsigned long address)
{
    digitalWrite(sram.cs_pin, LOW);
    SPI.transfer(mode);
    SPI.transfer((address >> 16) & 0xFF);
    SPI.transfer((address >> 8)  & 0xFF);
    SPI.transfer(address & 0xFF);
}

void test_ram() 
{
  SPI.setClockDivider(SPI_CLOCK_DIV2); 
  Serial.println(F("sram test:"));
 
  uint32_t address = 0;
  uint32_t fails = 0;
 
  sram_start(SRAM_WRITE, address);

  uint32_t d;
  for(d = 0; d < ramsz/4; d++)
  {
        uint32_t b = d;


        byte d1 = b;
        byte d2 = b >> 8;
        byte d3 = b >> 16;
        byte d4 = b >> 24;
/*

        Serial.print("in:");
        Serial.print((long)d1);
        Serial.print("  ");
        Serial.print((long)d2);
        Serial.print("  ");
        Serial.print((long)d3);
        Serial.print("  ");
        Serial.println((long)d4);
*/
        
        SPI.transfer(d1);
        SPI.transfer(d2);
        SPI.transfer(d3);
        SPI.transfer(d4);
  }
  digitalWrite(sram.cs_pin, HIGH);

  Serial.print("..");

  sram_start(SRAM_READ, address);
  for(d = 0; d < ramsz/4; d++)
  {
        uint32_t data;
        uint32_t expt = d;


        byte d1 = SPI.transfer(0);
        byte d2 = SPI.transfer(0);
        byte d3 = SPI.transfer(0);
        byte d4 = SPI.transfer(0);
  /*      
        Serial.print("out:");
        Serial.print((long)d1);
        Serial.print("  ");
        Serial.print((long)d2);
        Serial.print("  ");
        Serial.print((long)d3);
        Serial.print("  ");
        Serial.println((long)d4);
  */    
          data = (uint32_t)d1
          | ((uint32_t)d2 << 8)
          | ((uint32_t)d3 << 16)
          | ((uint32_t)d4 << 24);
        
        if (data != expt)
        {
          Serial.print(data);
          Serial.print("!=");
          Serial.println(expt);
          if (fails++ > 10) break;
        }
  }
  digitalWrite(sram.cs_pin, HIGH);
  Serial.println(F("done"));

}


/*
void test_ram() 
{
  uint32_t address = 0;
  const uint32_t bs = 1024L*256L;
  Serial.println(F("sram test:"));
  for(address = 0; address < ramsz; address+=bs)
  {
      Serial.print(address);
      Serial.print(":");


      sram_start(SRAM_WRITE, address);
      uint32_t d;
      for(d = 0; d < bs/4; d++)
      {
        uint32_t b = address + d;
        //Serial.println(b);
        //Serial.println("b");
        for (int a = 0; a < 4; a++)
        {
          //byte tb = *(((byte*)&b)+a);
          //Serial.println((long)tb);
           //SPI.transfer(tb);
           SPI.transfer(b);
           
        }
        
        //for (int a = 0; a < 4; a++)
          //SPI.transfer(a);
      }
      digitalWrite(sram.cs_pin, HIGH);

      Serial.print("..");

      sram_start(SRAM_READ, address);
      for(d = 0; d < bs/4; d++)
      {
        uint32_t data;
        byte expt = address + d;
        for (int a = 0; a < 4; a++)
        {
          data = SPI.transfer(0);
          //byte r = SPI.transfer(0);
          //Serial.println((long)r);
          //*((((byte*)&data)+a)) = r;
          //if (SPI.transfer(0) != a) Serial.println("err");
        }
        //data = data & ~(32768);
        
        if (data != expt)
        {
          Serial.print(data);
          Serial.print("!=");
          Serial.println(expt);
        }
      }
      digitalWrite(sram.cs_pin, HIGH);
      Serial.println(F("done"));
  }
}
*/













/*
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
*/


/*
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
*/


























/*testovaci jednoduzsi pomale cteni*/


void simple_read_loop(Acc &acc, unsigned long count, bool p=true)
{
  long x;
  for(x = 0; x < nsamples; x++)
  {
    sensors_event_t event; 
    acc.getEvent(&event);
  }
}

bool init_acc(Acc &acc)
{

  if (!acc._i2c)
  {
    SPI.setClockDivider(SPI_CLOCK_DIV4);
    SPI.setDataMode(SPI_MODE3);
    datarate = ADXL345_DATARATE_3200_HZ;
  }
  else
  {
    Wire.setClock(400000L);
    datarate = ADXL345_DATARATE_800_HZ;
  }
  
  if(!acc.begin())
  {
    return false;
  }

  acc.setRange(ADXL345_RANGE_16_G); //16 8 4 2
  acc.setDataRate(datarate);

  acc.writeRegister(ADXL345_REG_FIFO_CTL, 0b10000000);//stream mode
  acc.writeRegister(ADXL345_REG_INT_ENABLE, 0b10000000);//DATA_READY (na INT1)

  return true;
}

void test_acc_spi()
{
  Acc accspi = Acc(13, 12, 11, csacc);
  Serial.println(F("spi acc.."));
  test_acc(accspi);
}

void test_acc_twi()
{
  Acc acctwi = Acc(12345);
  Serial.println(F("\ntwi acc.."));
  test_acc(acctwi);
}

void test_accs()
{
  test_acc_spi();  
  test_acc_twi();
}

void test_acc(Acc &acc)
{
  reconf();
  
  if (!init_acc(acc))
    return;
  
  acc.writeRegister(ADXL345_REG_POWER_CTL, 0b1000);   // Enable measurements

  simple_read_loop(acc, 10 * datarate);

  displayAccDetails(acc); 
}





/**************************************************************************/


















/*cteni ze spi s interrupty*/



  /*
  Serial.println(acc.getRange());
  acc.setRange(ADXL345_RANGE_2_G);
  Serial.println(acc.getRange());
  acc.setRange(ADXL345_RANGE_4_G);
  Serial.println(acc.getRange());
  acc.setRange(ADXL345_RANGE_8_G);
  Serial.println(acc.getRange());
  acc.setRange(ADXL345_RANGE_16_G);*/
  
  


bool done;
long unsigned address = 0;


/*const unsigned int bufsiz = 6;
unsigned int bufpos = 0;
char buf[bufsiz];
*/
const byte RD = 1 << 7;
const byte MB = 1 << 6;

byte sample_size = 6;

volatile bool do_read;






void acc_spi_flush(unsigned int count = 40)
{
    
  /*zda se ze prilis rychle cteni zpusobi zablokovani adxl*/
}



void acquire_acc_spi()
{

  Serial.println(F("spi acquire\n"));  
  Acc acc = Acc(13, 12, 11, csacc);

  reconf();

  if(ticks_check)
  {
    noInterrupts();
    TIMSK0 = 0;
    TIMSK1 = 0;
    TCCR1A = 0;
    TCCR1B = 1;
    interrupts();
  }
  
  if (!init_acc(acc))
    return;

  acc.writeRegister(ADXL345_REG_POWER_CTL, 0b1000);   // Enable measurements
    
  displayAccDetails(acc);  

//simple_read_loop(acc);  

  acc.writeRegister(ADXL345_REG_POWER_CTL, 0b0000);
  Serial.println(F("flush.."));

  simple_read_loop(acc, 35, false);
  
  Serial.print(F("start: "));
  Serial.println(now_string());
  Serial.println(F("sampling.."));

  do_read = false;
  attachInterrupt(digitalPinToInterrupt(3), spi_int, FALLING);  
  acc.writeRegister(ADXL345_REG_POWER_CTL, 0b1000);   // Enable measurements

  dead = 0;
  address = 1;
  while(address < 35*samplesize) 
    spi_read();

  address = 0;
  done = false;
  while(!done) 
    spi_read();

  detachInterrupt(digitalPinToInterrupt(3));

  SPI.setDataMode(SPI_MODE0);  
  SPI.setClockDivider(SPI_CLOCK_DIV2); 

  RTC.read(end_tm);
  Serial.print(F("end: "));   Serial.println(time_string(end_tm));
  
}
/*z acc to leze ok, z pameti spatne*/
volatile unsigned int ticks;
volatile unsigned int old_ticks = 65536;
volatile unsigned long periods = 0;

void spi_int(void) 
{
  do_read = true;
  
  if (ticks_check )
  {
  ticks = TCNT1;
  if (ticks < old_ticks)
  {
    periods++;
  }
  old_ticks = ticks;
  }
}


void spi_read(void)
{    
  if(!do_read)
  {
    dead++;
    if (dead > 100000)
    {
      Serial.print(F("int pin dead at "));
      Serial.println((long)digitalRead(3)); 
      //nezvladali jsme dost rychle cist a clearovat interrupt flag, takze 
       // uz nedostavame interrupt pri novem samplu
       
      dead = 0;
    }
    return;
  }
  
  do_read = false;

  dead = 0;
    


    if (!(address & 0b11111111))
    {
     Serial.print('.');//address);//);
    }


      SPI.setClockDivider(SPI_CLOCK_DIV4); 
      SPI.setDataMode(SPI_MODE3);

      digitalWrite(csacc, LOW);

      SPI.transfer(ADXL345_REG_DATAX0 | RD | MB);
      
      char buf[6];
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



    if(just_print)
    {
      const int16_t *xyz = (int16_t *) buf;
/*
     char buffer[1+6*(1+5+1)];
     SPRINTF(buffer, "%d,%d,%d", xyz[0],xyz[1],xyz[2]);
     Serial.println(buffer);
*/
      Serial.print(F("X: ")); Serial.print(event.acceleration.x);
      Serial.print(F("  Y: ")); Serial.print(event.acceleration.y);
      Serial.print(F("  Z: ")); Serial.println(event.acceleration.z); 
    }
    else
    {
    
    SPI.setDataMode(SPI_MODE0);  
    SPI.setClockDivider(SPI_CLOCK_DIV2); 
    
    sram_start(SRAM_WRITE, address);
    
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
    
  
    
    if (address == toread)
    {
     done = true;
     Serial.println(F("done"));
    }
    
  
}













/*SD*/









void w_timing_end(time_t w_start_time)
{
  tmElements_t w_end_tm;
  RTC.read(w_end_tm);
  Serial.print(F("done in "));
  Serial.print(makeTime(w_end_tm) - w_start_time);
  //http://pastebin.com/sfEjA94n
  Serial.println(F("s."));  
}



#define ifsp(x)
//#define ifsp(x) if(serial_print) {x}

#define nl  {ifsp(Serial.println("");) file.print('\n');}
#define out(x) {ifsp(Serial.print(x);) file.print(x);}





void sd_out()
{
  now_string();//init

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
  out(F(" datarate:")); out(hz_str(dataRate2hz(datarate)));
  nl;
  
  const unsigned int bufsiz = 200;
  unsigned int sib = (bufsiz/samplesize)&~1;
  unsigned long address;
  const unsigned int bufused = sib * samplesize;

  for(address = 0; address + bufused < toread; address += bufused)
  {
  
    sram_start(SRAM_READ, address);

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
    for (s = 0; s < sib; s+=2)
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

     int16_t xyz[6];
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

     for (int a = 0; a < 12; a++)
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
     char buffer[1+6*(1+5+1)];
     SPRINTF(buffer, "%d,%d,%d\n%d,%d,%d\n", xyz[0],xyz[1],xyz[2],xyz[3],xyz[4],xyz[5]);
     /*zapisovani po dvou usetri jen asi 5s*/
     out(buffer);
    }
  }
  file.flush();
  sd.end();


  w_timing_end(w_start_time);

  
}









/*
void dbg_simple_ram_read()
{
  Serial.print(F("nsamples:")); Serial.println(nsamples);

  unsigned long address = 0;
  sram_start(SRAM_READ, address);

  for(address = 0; address < toread / 6; address += 6)
  {
  
    char buf[6];
    unsigned int b;

    for (b = 0; b < 6; b++)
    {
      buf[b] = SPI.transfer(0);
    }

    int16_t* xyz = (int16_t*)buf;
    char buffer[1+6*(1+5+1)];
    SPRINTF(buffer, "%d,%d,%d\n", xyz[0],xyz[1],xyz[2]);
    Serial.println(buffer);
   
  }
  
  digitalWrite(sram.cs_pin, HIGH);
}
*/


void test_sd(void)
{
  Sd2Card card;


  Serial.print(F("\nSD..."));

  if (!card.init()) {
    Serial.println(F("failed. is a card inserted?"));
    return;
  }
  
  // print the type of card
  Serial.print(F("\ntype: "));
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
  Serial.print(F("\nFAT"));
  Serial.println(volume.fatType(), DEC);
  Serial.println();

  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
  volumesize *= 512;                            // SD card blocks are always 512 bytes

  volumesize /= 1024;
  Serial.print(F("Volume size (Mbytes): "));
  volumesize /= 1024;
  Serial.println(volumesize);



  {
  Serial.println(F("\nFiles (name, date and size in bytes): "));

  SdFile root;


  root.openRoot(volume);

  // list all files in the card with date and size
  root.ls(LS_DATE | LS_SIZE);

  }

}




void list_files(void)
{
  test_sd();
}



/*
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
*/



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


/*
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
*/

















void setup(void) 
{
  digitalWrite(csram, 1);
  digitalWrite(cssd, 1);
  digitalWrite(csacc, 1);  
  
  pinMode(csram, 1);
  pinMode(cssd, 1);
  pinMode(csacc, 1);

  Serial.begin(115200);
  divider();
  Serial.println(F("setup..."));
  divider();
   
  clear_i2c();
 
/*  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);  */

  SPI.begin();
  init_ram();
}



void m_spi_acquire()
{
  divider();
  acquire_acc_spi();
  sd_out();
  list_files();
}






void m_self_test()
{
divider();  

  test_rtc();
divider();  

  test_ram();
divider();  

  test_sd();
divider();  

/*  write_time();
divider();  

  test_sd_speed();
divider();  */

  test_accs();
divider();  

Serial.println(F("selftest done"));
}


void menu()
{
  char i = 'h';
  
while(true)
{
  

  switch (i)
  {
    case 'h': Serial.println(F("\n"
"h - help\n"
"d - detect HW\n"
"f - SD card test\n"
"m - memory test\n"
"c - real time clock test\n"
"s - SPI accelerometer test\n"
"i - I2C accelerometer test\n"
"r - run (SPI)\n"));
    break;
    case 'd':
      m_self_test();
    break;
    case 'f':
      test_sd(); //az bude misto: zapis nejakeho jednoducheho souboru a jeho zpetne precteni, proste neco jednoducheho na otetsovani SD karty
    break;
    case 'm':
      test_ram();
    break;
    case 'c':
      test_rtc();
    break;
    case 's':
      test_acc_spi();
    break;
    case 'i':
      test_acc_twi();
    break;
    case 'r':
      m_spi_acquire();
/*     
> r - run, tedy jeden kompletni cyklus - detekce akcelerometru, pokud bude pripojen I2C, sber dat z nej, 
pokud nebude, pak sber dat z SPI, pokud nebude ani jeden ukonci se to s chybou. 
Nabirejte 10s dat (s tim ze v kodu mejte konstantu, kde to pujde "rucne" nastavit na delsi), 
data zapisujte do SRAM a pak do souboru. Behem tohoto cyklu do terminalu vypisujte jen stavova hlaseni co se deje
. Jaky akcelerometr detekovan, naber dat do SRAM, zapis na SD kartu, atd. Nikoli uz X Y Z data. 
A na konci vysledek, OK, error, kde a jaky byl error apod.
>*/

    break;
  }


Myslel jsem tim jen aby to nebezelo samo porad dokola. Cim vice to rozkouskujete, tim take lepe, abych mohl otestovat jednotlive casti samostatne. To cele byla idea nad menu. Kdyz menu nejak pozmenite, aby lepe pasovalo k tomu jak mate usporadany jednotlive funkce v SW, tim lepe. Dulezite je menu, vyber fuknce, provedeni akce a zase ukonceni v menu a cekani na dalsi pokyn.


Serial.println('>');  
while(!Serial.available()) {};
i = Serial.read();
}
  
}


