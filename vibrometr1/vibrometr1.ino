/*todo reset i2c po resetu arduina?*/


#include "fix_fft.h"


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
    Serial.print(F("Could not parse info from the compiler, __TIME__=\""));
    Serial.print(__TIME__);
    Serial.print("\", __DATE__=\"");
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
  for(address = 0; address < /*ramsz*/3; address+=bs)
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


void acc_wait(void)
{
  while(true)
  {
    byte status;

    Wire.beginTransmission(ADXL345_ADDRESS);
    Wire.write(ADXL345_REG_FIFO_STATUS);
    Wire.endTransmission();
    Wire.requestFrom(ADXL345_ADDRESS, 1);

    status = Wire.read();

    byte entries;
    entries = status & 0b111111;
    if (entries != 0) 
      break;
  //  Serial.println(F("waiting for samples.."));
  }
}


void test_acc(Acc &acc)
{
  if (!init_acc(acc))
    return;


  const uint32_t nsamples = 128*10;//(ramsz/6L);
    
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


while(true)
{

  acc_flush();

  
  digitalWrite(sram.cs_pin, LOW);
  SPI.transfer(SRAM_WRITE);
  //adresa
  SPI.transfer(0);
  SPI.transfer(0);
  SPI.transfer(0);


  
  tmElements_t start, end;
  RTC.read(start);
  RTC.read(end);
  //time_t st = makeTime(start);
  //time_t et = makeTime(end);
  

  unsigned long s;
  byte status;
  int16_t xyz[3];
  for(s = 0; s < nsamples; s++)
  {

    Wire.beginTransmission(ADXL345_ADDRESS);
    Wire.write(ADXL345_REG_DATAX0);
    Wire.endTransmission();
    Wire.requestFrom(ADXL345_ADDRESS, 8);

    for (int a = 0; a < 6; a++)
        SPI.transfer(Wire.read());

    Wire.read(); // FIFO_CTL
    status = Wire.read();

    {
    byte entries;
    entries = status & 0b111111;
    //Serial.println(entries);
    if (entries > 30) 
    	Serial.println(F("overflow"));
    if(entries == 0)
      acc_wait();
    }

  }

  digitalWrite(sram.cs_pin, HIGH);


  Serial.println(F("sampling done"));

  RTC.read(end);


  fft_out(nsamples);

  digitalWrite(sram.cs_pin, HIGH);
  Serial.println(F("done"));
  
}
acc.writeRegister(ADXL345_REG_POWER_CTL, 0);

SPI.end();
}



void fft_out(const uint32_t nsamples )
{
  
  
  digitalWrite(sram.cs_pin, LOW);
  SPI.transfer(SRAM_READ);
  SPI.transfer(0);  SPI.transfer(0);  SPI.transfer(0);


  const uint16_t F_SAMPLE = 800;
  

  unsigned long long int block;
  for(block = 0; block < nsamples; block+=128)
  {

    char re[128];
    char im[128];

    int16_t off[3];
    int16_t mins[3] = { 32767, 32767, 32767};
    int16_t maxs[3] = {-32768,-32768,-32768};
    
    byte s;
    for (s = 0; s < 128; s++)
    {
      int16_t xyz[3];
      
      char i0,i1;
      int16_t v;
      
      for (int a = 0; a < 3; a++)
      {
        
        i0 = SPI.transfer(0);
        i1 = SPI.transfer(0);
        v = i0 | (i1 << 8);
        xyz[a] = v;// - off[a];
      }
      if (s == 0)
        for (int a = 0; a < 3; a++)
          off[a] = xyz[a];
        
      for (int a = 0; a < 3; a++)
        xyz[a] -= off[a];
         

      for (int a = 0; a < 3; a++)
      {
        if(mins[a] > xyz[a])
          mins[a] = xyz[a];
        if(maxs[a] < xyz[a])
          maxs[a] = xyz[a];
      }
      
      int16_t l = xyz[2];
                  
      if (l > 127 || l < -126)
      {
        Serial.println(""); 
        Serial.println(F("out of char range:)"));
      }
/*

      Serial.print(F("X: ")); Serial.print(xyz[0]);
      Serial.print(F("  Y: ")); Serial.print(xyz[1]);
      Serial.print(F("  Z: ")); Serial.println(xyz[2]);

  */    
      char vv = l;//sqrt(xyz[0]*xyz[0] + xyz[1]*xyz[1] + xyz[2]*xyz[2]);
      //Serial.print(F(" v: ")); Serial.print( vv, DEC);

      float fs = s;

      re[s] = vv;//50*sin(fs / 5.0f) + 50*sin(fs*3);
      im[s] = 0;
    }

    Serial.print(F("noise:"));
    for (char a = 0; a < 3; a++)
    {
      Serial.print((char)('X' + a)); Serial.print(maxs[a] - mins[a]); Serial.print("   ");
    }

    fix_fft(re,im,7,0);
    print_fft(re,im);

  }
}

void print_fft(char *re, char*im)
{
  int i,j,largest;
  char str[65];
  char linfo[6];

  str[64] = 0;  
  largest = 0;
  // Find the largest entry which will determine how many lines
  // are needed to print the whole histogram
  for (i=0; i< 64;i++){
    re[i] = sqrt(re[i] * re[i] + im[i] * im[i]);
    if(re[i] > largest)largest = re[i];
  }
  // print a blank line just in case there's
  // garbage when the Serial monitor starts up
  Serial.println("");
  // and the title
  // print the histogram starting with the highest amplitude
  // and working our way back down to zero.
  for(j=largest;j >= 0;j--) {
    for(i=0;i<64;i++) {
      // If the magnitude of this bin is at least as large as
      // the current magnitude we print an asterisk
      if(re[i] >= j)str[i] = '*';
      // otherwise print a space
      else str[i] = ' ';
    }
    sprintf(linfo,"%3d ",j);
    Serial.print(linfo);
    Serial.println(str);
  }
  // print the bin numbers along the bottom
/*  Serial.print("    ");
  for(i=0;i<64;i++) {
    Serial.print(i%10);
  }
  Serial.println("");
  Serial.print("    ");
  for(i=0;i<64;i++) {
    if(i < 10)Serial.print(" ");
    else Serial.print((i/10)%10);
  }
  Serial.println("");*/
}





void loop(void) 
{
  
  test_rtc();
divider();  
  //test_ram();
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
  Serial.begin(115200);
  
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




void sd_out(const uint32_t nsamples )
{
/*
  Sd2Card card;
  
  if (!card.init()) {
    Serial.println(F("SD initialization failed."));
   return;
  }

  SdVolume volume;
  
  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
    Serial.println(F("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card"));
    return;
  }

  SdVolume volume;
  
  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
  if (!volume.init(card)) {
    Serial.println(F("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card"));
    return;
  }
*/


  SDClass sd;
  if (!sd.begin()) {
    Serial.println(F("SD err"));
    return;
  }
   
  File file = sd.open("mereni.csv", FILE_WRITE);
  if (! file) 
    Serial.println(F("error opening mereni.csv"));
  
  file.print(now_string());
  file.println(",,,");
  
  sd.end();
  
  }
  
  
  digitalWrite(sram.cs_pin, LOW);
  SPI.transfer(SRAM_READ);
  SPI.transfer(0);  SPI.transfer(0);  SPI.transfer(0);

  uint32_t s;
  for (s = 0; s < nsamples; s++)
  {
      int16_t xyz[3];
      
      char i0,i1;
      int16_t v;
      
      for (int a = 0; a < 3; a++)
      {
        
        i0 = SPI.transfer(0);
        i1 = SPI.transfer(0);
        v = i0 | (i1 << 8);
        xyz[a] = v;
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


      /*
SPI.transfer(0); SPI.transfer(0); xyz[2]=0;
      Serial.print((byte)i1, BIN);
      Serial.print(F(" ")); 
      Serial.print((byte)i0, BIN);
      Serial.print(F(" "));
      Serial.println(v); 
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
 */
