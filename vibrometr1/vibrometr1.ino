#include <Wire.h>


/* prekopano */
#include <Adafruit_ADXL345_U.h>



#include <SPI.h>



/*SD knihovna: odnekud z githubu, ale vypada jako identicka s oficialni arduino. dve upravy:
 * zrusen globalni SD objekt, misto toho se vytvari jen pro zapsani dat
 * kvuli tomu bylo potreba opravit https://github.com/arduino/Arduino/issues/3607 */
#include <SD.h>





#include <Time.h>
#include <DS1307RTC.h>
#include "SRAM.h"




/*nejake upravy, hlavne vylepsen kod detekce tlacitek, nejake nastaveni je tam prasacky nacpano - doporucoval bych celou knihovnu prekopirovat sem, pak by se daly odebrat callbacky a podobne */
#include "OMMenuMgr.h"





#include <LCD4884.h>













const byte dispW = OM_MENU_COLS;


/*tusim ze s menu byl na poslednim radku displaye nejaky problem*/
const byte dispH = OM_MENU_ROWS+1;




/*tahle trida pise zaroven na display a na seriak*/
class Disp : public Print
{
  char buf[dispH][dispW];//cirkularni buffer
  byte nextX = 0;/*kam si zapise dalsi pismenko*/
  byte nextY = 0;

  public:
  Disp()
  {
    cls();
  }

  void cls()
  {
    for(byte x = 0; x < dispW * dispH; x++)
      add(' ');
    nextX = nextY = 0;
  }

  void nl()
  {
    nextX = 0;
    nextY++;
    if (nextY == dispH)
      nextY = 0;
    for(byte i=0;i<dispW;i++)
      buf[nextY][i] = ' ';
  }
  
  bool add(uint8_t ch)
  { 
    bool redraw = false;
    if (ch == 13)
    {}
    else if (ch == '\n')
    {
      nl();
      redraw = true;
    }
    else
      buf[nextY][nextX++] = ch;
    if (nextX == dispW)
    {
      nl();
      redraw = true;
    }
    return redraw;
  }
  
  size_t write(uint8_t ch)
  {
    Serial.write(ch);

    bool redraw = add(ch);
    byte bufY = nextY;
    for(byte y = 0; y < dispH; y++)
    {
      if (++bufY == dispH)
        bufY = 0;
      if(redraw)
      {
        for(byte x = 0; x < dispW; x++)
        {
          lcd.LCD_set_XY(x*6, y);
          lcd.LCD_write_char(buf[bufY][x], MENU_NORMAL);
        }
      }
      else if (y == dispH - 1)
      {
        lcd.LCD_set_XY((nextX-1)*6, y);
        lcd.LCD_write_char(buf[bufY][nextX-1], MENU_NORMAL);
      }
     
    }
  }
};


Disp disp;





#define SPRINTF sprintf

/*konfigurace*/

//vypisovani verbatim toho co se zapisuje do souboru na seriak
const bool serial_print = 1;

//#define SLOW
#ifdef SLOW
dataRate_t conf_datarate = ADXL345_DATARATE_100_HZ;//1600,800,400..
const byte SPI_CLOCK_ACC = SPI_CLOCK_DIV16;
const byte SPI_CLOCK_RAM = SPI_CLOCK_DIV16;
const byte SPI_CLOCK_SD = SPI_CLOCK_DIV16;//probably not effective
const unsigned long WIRE_ACC_FREQ = 100000;//400000
const unsigned long WIRE_RTC_FREQ = 100000;
#else
dataRate_t conf_datarate = ADXL345_DATARATE_3200_HZ;//1600,800,400..
const byte SPI_CLOCK_ACC = SPI_CLOCK_DIV4;
const byte SPI_CLOCK_RAM = SPI_CLOCK_DIV2;
const byte SPI_CLOCK_SD = SPI_CLOCK_DIV2;
const unsigned long WIRE_ACC_FREQ = 400000;//400000
const unsigned long WIRE_RTC_FREQ = 100000;
#endif

/*
In [5]: (1024**2)/8/6/3200
Out[5]: 6.826666666666666*/
int conf_duration = 6;//sekund

const byte LCD_BACKLIGHT_PIN  = 7;
const byte BUT_PIN = 14;
const byte cssd = 10;
const byte csacc = 9;
const byte csram = 8;

const uint32_t ramsz = 1024L*1024L/8L;

/*interni*/

SRAM sram(csram);
tmElements_t end_tm;
byte samplesize ;
long unsigned toread; //kolik bajtu samplu precist z acc a zapsat do ram
dataRate_t datarate;
uint32_t nsamples;//kolik samplu


void reconf(bool spi)
{
	datarate = conf_datarate;

	if (!spi)
		datarate = min(ADXL345_DATARATE_800_HZ, datarate);

	samplesize = 3*2;

	unsigned int hz = 3200>>(15-datarate);

	nsamples = conf_duration * hz;

	if (ramsz/samplesize < nsamples)
	{
		disp.println(F("not enough ram"));
		nsamples = ramsz/samplesize;
	}

	nsamples &= ~1;

	toread = nsamples * samplesize;
  disp.print(F("nsamples :"));
  disp.println(nsamples);
   
}



//#define newspi
#ifdef newspi
void spi_ram_start()
{
	SPI.beginTransaction(SPISettings(SPI_CLOCK_RAM, MSBFIRST, SPI_MODE0));
	digitalWrite(sram.cs_pin, LOW);
}
void spi_sd_start()
{
	SPI.beginTransaction(SPISettings(SPI_CLOCK_SD, MSBFIRST, SPI_MODE0));
}
void spi_acc_start()
{
	SPI.beginTransaction(SPISettings(SPI_CLOCK_ACC, MSBFIRST, SPI_MODE3));
	digitalWrite(csacc, LOW);
}
void spi_end()
{
	digitalWrite(sram.cs_pin, HIGH);
	digitalWrite(csacc, HIGH);
	SPI.endTransaction();
}

#else
void spi_ram_start()
{
	SPI.setClockDivider(SPI_CLOCK_RAM);
	SPI.setDataMode(SPI_MODE0);
	SPI.begin();
	digitalWrite(sram.cs_pin, LOW);
}
void spi_sd_start()
{
	SPI.setClockDivider(SPI_CLOCK_SD);
	SPI.setDataMode(SPI_MODE0);
	SPI.begin();
}
void spi_acc_start()
{
	SPI.setClockDivider(SPI_CLOCK_ACC);
	SPI.setDataMode(SPI_MODE3);
	SPI.begin();
	digitalWrite(csacc, LOW);
}
void spi_end()
{
	digitalWrite(sram.cs_pin, HIGH);
	digitalWrite(csacc, HIGH);
	SPI.end();
}
#endif


void divider(void)
{
	Serial.println(F("\n-------------\n"));
}

void loop(void)
{

	menu();

}



/*i2c se musi resetovat po resetu arduina jinak se nekdy zasekne*/
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
		disp.println(F("I2C err"));
		/*if (rtn == 1) {
		  disp.println(F("SCL clock line held low"));
		} else if (rtn == 2) {
		  disp.println(F("SCL clock line held low by slave clock stretch"));
		} else if (rtn == 3) {
		  disp.println(F("SDA data line held low"));
		}*/
	}
	else
	{
		//disp.println(F("i2c bus clear"));
		// re-enable Wire
		// now can start Wire Arduino master
		Wire.begin();
	}
}













String now_string(void)
{
	tmElements_t tm;

	if (rtc_read(tm))
		return time_string(tm);
	else
	{
    String r = String(F("RTCerror"));
		if (RTC.chipPresent())
		{
			disp.println(F("RTC stopped. please set time."));
		}
		else
		{
		  disp.println(r);//!  Please check the circuitry.
		}
    return r;
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
	disp.println(F("DS1307RTC Test"));
	for (byte i = 0; i < 3; i++)
	{
		disp.println(now_string());
		delay(1000);
	}
}


bool rtc_read(tmElements_t &tm)
{
	Wire.setClock(WIRE_RTC_FREQ );
	return RTC.read(tm);
}












void init_ram(void)
{
	spi_ram_start();
	SPI.transfer(SRAM_WRSR);
	SPI.transfer(sram.mode);
	digitalWrite(sram.cs_pin, HIGH);
	spi_end();
}

void sram_start(byte mode, unsigned long address)
{
	spi_ram_start();
	SPI.transfer(mode);
	SPI.transfer((address >> 16) & 0xFF);
	SPI.transfer((address >> 8)  & 0xFF);
	SPI.transfer(address & 0xFF);
}

void test_ram()
{
	disp.println(F("sram test:"));

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

		        disp.print("in:");
		        disp.print((long)d1);
		        disp.print("  ");
		        disp.print((long)d2);
		        disp.print("  ");
		        disp.print((long)d3);
		        disp.print("  ");
		        disp.println((long)d4);
		*/

		SPI.transfer(d1);
		SPI.transfer(d2);
		SPI.transfer(d3);
		SPI.transfer(d4);
	}
	spi_end();

	disp.print("..");

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
		      disp.print("out:");
		      disp.print((long)d1);
		      disp.print("  ");
		      disp.print((long)d2);
		      disp.print("  ");
		      disp.print((long)d3);
		      disp.print("  ");
		      disp.println((long)d4);
		*/
		data = (uint32_t)d1
		       | ((uint32_t)d2 << 8)
		       | ((uint32_t)d3 << 16)
		       | ((uint32_t)d4 << 24);

		if (data != expt)
		{
			disp.print(data);
			disp.print("!=");
			disp.println(expt);
			if (fails++ > 10) break;
		}
	}
	digitalWrite(sram.cs_pin, HIGH);
	disp.println(F("done"));
	spi_end();
}






bool just_print = false;
byte sample_size = 6;
long unsigned address = 0;
bool use_spi;
bool do_wait = true;

bool init_acc(Acc &acc)
{

	if(acc._i2c)
		Wire.setClock(WIRE_ACC_FREQ);
	else
		spi_acc_start();

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

void test_acc(bool spi)
{
	just_print = true;
	acquire_acc(spi);
	just_print = false;
}



const byte RD = 1 << 7;
const byte MB = 1 << 6;



bool acquire_acc(bool spi)
{
	if (spi)
	{
		disp.println(F("spi acquire\n"));
		Acc acc = Acc(csacc);
		return acquire(acc);

	}
	else
	{
		disp.println(F("i2c acquire\n"));
		Acc acc = Acc();
		return acquire(acc);
	}
}

byte entries;

bool acquire(Acc &acc)
{
	
	  reconf(!acc._i2c);


	  if (!init_acc(acc))
	    return false;

	  use_spi = !acc._i2c;

	  displayAccDetails(acc);

	  disp.println(F("flush.."));
	  acc.writeRegister(ADXL345_REG_POWER_CTL, 0b0000);
	  spi_end();
	  do_wait = false;
    entries = 0;
	  for(byte i = 0; i < 50; i++)
	    acc_read();
	  do_wait = true;
	  disp.print(F("start: "));
	  disp.println(now_string());
	  if(use_spi)
	    spi_acc_start();
	  acc.writeRegister(ADXL345_REG_POWER_CTL, 0b1000);
	  spi_end();
	  address = 0;
    entries = 0;
	  while(address < nsamples * samplesize)
	    acc_read();

	  rtc_read(end_tm);
	  disp.print(F("\nend: "));   disp.println(time_string(end_tm));
	  return true;
}


void acc_wait(void)
{	
	 do
	 {
	   byte status;
	   //disp.println(F("waiting for samples."));

	   if(use_spi)
	   {
	       spi_acc_start();
	       SPI.transfer(ADXL345_REG_FIFO_STATUS | RD);
	       status = SPI.transfer(0);
	       spi_end();
	   }
	   else
	   {
	       Wire.beginTransmission(ADXL345_ADDRESS);
	       Wire.write(ADXL345_REG_FIFO_STATUS);
	       Wire.endTransmission();
	       Wire.requestFrom(ADXL345_ADDRESS, 1);
	       status = Wire.read();
	   }

	   entries = status & 0b111111;
	 }
	 while(!entries);
	 
}


void acc_read(void)
{	
	  char buf[6];
	  
	  byte status ;


	  if(use_spi)
	  {
	    spi_acc_start();
	    SPI.transfer(ADXL345_REG_DATAX0 | RD | MB);
	    for (int a = 0; a < 6; a++)
	      buf[a] = SPI.transfer(0);
      if(!entries)
      {
	      SPI.transfer(0); // FIFO_CTL zahodime
	      status = SPI.transfer(0);

        entries = status & 0b111111;
        if (entries > 30 && ! just_print)
            disp.println(F("overflow"));
    
      }
      else entries--;
      
	    spi_end();
	  }
	  else
	  {
	    Wire.beginTransmission(ADXL345_ADDRESS);
	    Wire.write(ADXL345_REG_DATAX0 | MB);
	    Wire.endTransmission();
	    Wire.requestFrom(ADXL345_ADDRESS, entries ? 6 : 8);//count seems ignored
	    for (int a = 0; a < 6; a++)
	        buf[a] = Wire.read();

      if(!entries)
      {
        Wire.read();
	      status = Wire.read();

        entries = status & 0b111111;
        if (entries > 30 && ! just_print)
            disp.println(F("overflow"));
      }
      else entries--;
       
	  }


	  if(entries == 0 && do_wait)
	  {
	    acc_wait();
	  }

	  if(just_print)
	  {
	    const int16_t *xyz = (int16_t *) buf;
	    char buffer[1+6*(1+5+1)];
	    SPRINTF(buffer, "%d,%d,%d", xyz[0],xyz[1],xyz[2]);
	    disp.println(buffer);
	  }
	  else
	  {
	    sram_start(SRAM_WRITE, address);
	    for (int a = 0; a < 6; a++)
	      SPI.transfer(buf[a]);
	    spi_end();
	  }

	  if (!(address & 0b11111111))
	  {
	   disp.print('.');//address);//);
	  }

	  address += 6;
	
}













/*SD*/









void w_timing_end(time_t w_start_time)
{
	tmElements_t w_end_tm;
	rtc_read(w_end_tm);
	disp.print(F("done in "));
	disp.print(makeTime(w_end_tm) - w_start_time);
	//http://pastebin.com/sfEjA94n
	disp.println(F("s."));
}



#define ifsp(x)
//#define ifsp(x) if(serial_print) {x}

#define nl  {ifsp(disp.println("");) file.print('\n');}
#define out(x) {ifsp(disp.print(x);) file.print(x);}





void sd_out()
{


	now_string();//init

	tmElements_t w_start_tm, w_end_tm;


	disp.print(F("ok.."));
	SDClass sd;
	disp.print(F("ok.."));
	if (!sd.begin()) {
		disp.println(F("SD err"));
		return;
	}
	disp.print(F("ok.."));


	String fn = fn_time_string(end_tm) + ".csv";
	disp.print(F("filename:"));
	disp.println(fn);

	File file = sd.open(fn.c_str(), FILE_WRITE);
	if (!file)
	{
		disp.print(F("error opening "));
		disp.println(fn);
		return;
	}

	disp.print(F("toread bytes:"));
	disp.println(toread);

	disp.println(F("writing..."));

	out(time_string(end_tm));
	nl;
	out(F("nsamples:"));
	out(nsamples);
	out(F(" datarate:"));
	out(hz_str(dataRate2hz(datarate)));
	nl;

	const unsigned int bufsiz = 200;
	unsigned int sib = (bufsiz/samplesize)&~1;
	unsigned long address;
	const unsigned int bufused = sib * samplesize;

	spi_end();


  rtc_read(w_start_tm);
  time_t w_start_time = makeTime(w_start_tm);
  

	for(address = 0; address + bufused < toread; address += bufused)
	{

		sram_start(SRAM_READ, address);

		char buf[bufsiz];
		unsigned int b;

		for (b = 0; b < sib * samplesize; b++)
		{
			buf[b] = SPI.transfer(0);
		}

		spi_end();
		spi_sd_start();


		//if(!serial_print)
		{
			static byte x = 0;
			if (!(x++ & 0b1111))
			{
				disp.print(F("address:"));
				disp.print(address);
				//disp.print(F("sib * samplesize:"));
				//disp.print(sib * samplesize);
				disp.print(F(".."));

				rtc_read(w_end_tm);
				disp.print(address / (makeTime(w_end_tm) - w_start_time));
				disp.println(F(" samplebytes/second"));
			}
		}

		unsigned int bufpos = 0;
		unsigned int s;
		for (s = 0; s < sib; s+=2)
		{


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



			/*if(bufpos >= bufsiz)
			   disp.print("err");*/

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
	file.close();
	sd.end();

	spi_end();

	w_timing_end(w_start_time);


}









/*
void dbg_simple_ram_read()
{
  disp.print(F("nsamples:")); disp.println(nsamples);

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
    disp.println(buffer);

  }

  digitalWrite(sram.cs_pin, HIGH);
}
*/


void test_sd(void)
{
/*	

	//  spi_sd_start();

	  Sd2Card card;


	  disp.print(F("\nSD..."));

	  if (!card.init()) {
	    disp.println(F("failed. is a card inserted?"));
	    return;
	  }

	  // print the type of card
	  disp.print(F("\ntype: "));
	  switch (card.type()) {
	    case SD_CARD_TYPE_SD1:
	      disp.println(F("SD1"));
	      break;
	    case SD_CARD_TYPE_SD2:
	      disp.println(F("SD2"));
	      break;
	    case SD_CARD_TYPE_SDHC:
	      disp.println(F("SDHC"));
	      break;
	    default:
	      disp.println(F("Unknown"));
	  }





	  SdVolume volume;

	  // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
	  if (!volume.init(card)) {
	    disp.println(F("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card"));
	    return;
	  }


	  // print the type and size of the first FAT-type volume
	  uint32_t volumesize;
	  disp.print(F("\nFAT"));
	  disp.println(volume.fatType(), DEC);
	  disp.println();

	  volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
	  volumesize *= volume.clusterCount();       // we'll have a lot of clusters
	  volumesize *= 512;                            // SD card blocks are always 512 bytes

	  volumesize /= 1024;
	  disp.print(F("Volume size (Mbytes): "));
	  volumesize /= 1024;
	  disp.println(volumesize);



	  {
	  disp.println(F("\nFiles (name, date and size in bytes): "));

	  SdFile root;


	  root.openRoot(volume);

	  // list all files in the card with date and size
	  root.ls(LS_DATE | LS_SIZE);
	  root.close();
	  }
	  spi_end();

	*/  
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

  disp.print(F("test sd speed.."));
  SDClass sd;
  disp.print(F("ok.."));
  if (!sd.begin()) {
    disp.println(F("SD err"));
    return;
  }
  disp.print(F("ok.."));

  String fn = "test";
  disp.print(F("filename:"));
  disp.println(fn);

  File file = sd.open(fn.c_str(), FILE_WRITE);
  if (!file)
  {
    disp.print(F("error opening "));
    disp.println(fn);
    return;
  }

  disp.println(F("writing..."));

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
  disp.println(F("write time.txt..."));
  if (sd.begin()) {
    File file = sd.open("time.txt", FILE_WRITE);
    if (! file)
      disp.println(F("error opening time.txt"));
    file.println(now_string());
    file.flush();
    sd.end();
  }
  else
    disp.println(F("err"));

  SPI.end();
}
*/








/*zda se ze prilis rychle cteni zpusobi zablokovani adxl*/








void setup(void)
{
	digitalWrite(csram, 1);
	digitalWrite(cssd, 1);
	digitalWrite(csacc, 1);
	digitalWrite(LCD_BACKLIGHT_PIN, 0);
  digitalWrite(BUT_PIN, 1);
  
	pinMode(csram, 1);
	pinMode(cssd, 1);
	pinMode(csacc, 1);
	pinMode(LCD_BACKLIGHT_PIN, 1);
  
	Serial.begin(115200);
  lcd.LCD_init();
  
	divider();
	disp.println(F("setup..."));
	divider();

	clear_i2c();
	init_ram();
	
}



void m_acquire(bool spi)
{
  disp.cls();
	divider();
	if(acquire_acc(spi))
  {
	  sd_out();
	  list_files();
  }
  else wait();
}





void m_self_test()
{
  disp.cls();
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

	test_acc(true);
	test_acc(false);

	divider();

	disp.println(F("selftest done"));
}














// Create a list of states and values for a select input
MENU_SELECT_ITEM  sel_adxlrate_3200 = { 15, {"3200HZ"} };
MENU_SELECT_ITEM  sel_adxlrate_1600 = { 14, {"1600HZ"} };
MENU_SELECT_ITEM  sel_adxlrate_800 = { 13, {"800HZ"} };
MENU_SELECT_ITEM  sel_adxlrate_400 = { 12, {"400HZ"} };
MENU_SELECT_ITEM  sel_adxlrate_200 = { 11, {"200HZ"} };
MENU_SELECT_ITEM  sel_adxlrate_100 = { 10, {"100HZ"} };
MENU_SELECT_ITEM  sel_adxlrate_50 = { 9, {"50HZ"} };

MENU_SELECT_LIST adxlrate_list [] = { 
  &sel_adxlrate_50,
  &sel_adxlrate_100, 
  &sel_adxlrate_200, 
  &sel_adxlrate_400, 
  &sel_adxlrate_800, 
  &sel_adxlrate_1600, 
  &sel_adxlrate_3200, 
};



// TARGET VAR   LENGTH                          TARGET SELECT LIST
MENU_SELECT state_select = { &conf_datarate,           MENU_SELECT_SIZE(adxlrate_list),   MENU_TARGET(&adxlrate_list) };

//    TYPE            MAX    MIN    TARGET
MENU_VALUE menu_value_duration = { TYPE_INT,       10000, 1,   MENU_TARGET(&conf_duration) };

MENU_VALUE menu_value_rate = { TYPE_SELECT,     15,     9,     MENU_TARGET(&state_select) };

//        LABEL           TYPE        LENGTH    TARGET

MENU_ITEM menu_a1 = { {"SPI acquire"}, ITEM_ACTION, 0, MENU_TARGET(m_spi_acquire) };
MENU_ITEM menu_a2 = { {"I2C acquire"}, ITEM_ACTION, 0, MENU_TARGET(m_i2c_acquire) };
MENU_ITEM menu_duration    = { {"duration[s]"},     ITEM_VALUE,  0,        MENU_TARGET(&menu_value_duration) };
MENU_ITEM menu_datarate    = { {"datarate[HZ]"}, ITEM_VALUE,  0,        MENU_TARGET(&menu_value_rate ) };
MENU_ITEM menu_a3 = { {"self-test"},  ITEM_ACTION, 0,        MENU_TARGET(m_self_test) };
MENU_ITEM menu_a4 = { {"RAM test"}, ITEM_ACTION, 0, MENU_TARGET(test_ram) };
MENU_ITEM menu_a5 = { {"RTC test"}, ITEM_ACTION, 0, MENU_TARGET(test_rtc) };
MENU_ITEM menu_a6 = { {"SPI acc test"}, ITEM_ACTION, 0, MENU_TARGET(m_spi_test) };
MENU_ITEM menu_a7 = { {"I2C acc test"}, ITEM_ACTION, 0, MENU_TARGET(m_i2c_test) };
MENU_ITEM menu_a8 = { {"clock"}, ITEM_ACTION, 0, MENU_TARGET(m_clock) };

void m_spi_acquire()
{
  m_acquire(true);
}
void m_i2c_acquire()
{
  m_acquire(false);
}

void m_spi_test()
{
  test_acc(true);
}
void m_i2c_test()
{
  test_acc(false);
}


//                       "f - SD card test\n"
//                       "o - loop spi acquire\n"


//        List of items in menu level
MENU_LIST root_list[]   = { &menu_a1, &menu_a2, &menu_duration, &menu_datarate, &menu_a3, 
&menu_a4, &menu_a5, &menu_a6, &menu_a7, &menu_a8};

// Root item is always created last, so we can add all other items to it
MENU_ITEM menu_root     = { {"Root"},        ITEM_MENU,   MENU_SIZE(root_list),    MENU_TARGET(&root_list) };


OMMenuMgr Menu(&menu_root);



void uiDraw(char* text, int row, int col, int len=0) {
	/*disp.print(row);
	disp.print(' ');
	disp.print(col);
	disp.print(':');*/
	lcd.LCD_set_XY(col*6, row);

	int i = 0;
	while(true)
	{
		char ch = text[i];


		if ((!len && !ch)
		        ||
		        ( len&&(i >= len)))
			break;
		i++;
		if( ch < '!' || ch > '~' ) ch = ' ';
		lcd.LCD_write_char(ch, MENU_NORMAL);
//		disp.print(ch);
	}
	//disp.println('.');
}


void wait()
{
      Menu.enable(false);
  while(Menu.checkInput() == BUTTON_NONE){};
      Menu.enable(true);
}

void uiClear() {

	lcd.LCD_clear();
	uiDraw("Push"/* Select for Menu"*/,0,0);
}


void menu()
{

	lcd.LCD_clear();
	digitalWrite(LCD_BACKLIGHT_PIN, 1);


	Menu.setDrawHandler(uiDraw);
	Menu.setExitHandler(uiClear);
	//Menu.setAnalogButtonPin(BUT_PIN, BUT_MAP, BUT_THRESH);
	Menu.enable(true);

	Menu.m_menuActive = true;
	Menu._handleButton(BUTTON_SELECT);


//m_clock() ;


	while(true)
	{
		byte b = Menu.checkInput();
		/*if (b)
			disp.println(b);*/
	}
}





void m_clock() 
{
    bool doupdate = true;
    bool blink_flag;
    tmElements_t old,tm;
    rtc_read(old);
    rtc_read(tm);
    unsigned long m, oldm;
    unsigned int counter;
    
    Menu.enable(false);

    lcd.LCD_clear();

    char pos = -1;

    while( pos != -2 ) {
      
      m = millis();
      if (m - oldm > 1000)
      {
        oldm = m;
        rtc_read(tm);
      }   
      
      int b = Menu.checkInput();
      char inc = 0;
      switch(b)
      {
        case BUTTON_BACK:
        pos -= 1;
        doupdate = true;
        break;
        case BUTTON_FORWARD:
        pos += 1;
        if (pos > 4) pos = 4;
        doupdate = true;
        break;
        case BUTTON_INCREASE:
        inc = 1;
        break;
        case BUTTON_DECREASE:
        inc = -1;
        break;
      }

      if(inc)
      {

        switch(pos)
        {
          case 0:
            tm.Hour += inc;
            break;
          case 1:
            tm.Minute += inc;
            break;
          case 2:
            tm.Second += inc;
            break;
          case 3:
            tm.Day += inc;
            break;
          case 4:
            tm.Month += inc;
            break;
        }
        RTC.write(tm);
        rtc_read(tm);
      }
      


      
      if (
        old.Second != tm.Second ||
        old.Minute != tm.Minute ||
        old.Hour != tm.Hour ||
        old.Day != tm.Day||
        old.Month!= tm.Month||
        old.Year != tm.Year)
        {
          doupdate = true;
          old = tm;
        }
         
         
      
      if (((m>>8)&1)==blink_flag)
      {
        blink_flag = !blink_flag;
        doupdate = true;
      }
      
      
      if(doupdate)
      {
      doupdate = false;

      char buf[5][5];

      byte * data[5];
      data [0] = &tm.Hour;
      data [1] = &tm.Minute;
      data [2] = &tm.Second;
      data [3] = &tm.Day;
      data [4] = &tm.Month;
      
      const byte loc[5][2] = {{0,0},{12*2+4,0},{12*2+4+12*2+4,0},{12,3},{12+12*2+4,3}};

      const char ends[6] = "::\0.\0";
      
      for (byte x = 0; x<5; x++)
      {
        char fmt[6] = "%02d?";
        fmt[4] = ends[x];
        SPRINTF(buf[x], fmt, (int)(*data[x]));

      }

      for (byte x = 0; x<5; x++)
      {
        byte mode;
        if ((pos == x) && blink_flag)
          mode = MENU_HIGHLIGHT;
        else mode = MENU_NORMAL;
          //continue;//blink

        lcd.LCD_write_string_big(loc[x][0],loc[x][1],buf[x],mode);

        
      }
      
      
    }
      
    }
    
    Menu.enable(true);
    lcd.LCD_clear();
}





/*
> r - run, tedy jeden kompletni cyklus - detekce akcelerometru, pokud bude pripojen I2C, sber dat z nej,
pokud nebude, pak sber dat z SPI, pokud nebude ani jeden ukonci se to s chybou.
Nabirejte 10s dat (s tim ze v kodu mejte konstantu, kde to pujde "rucne" nastavit na delsi),
data zapisujte do SRAM a pak do souboru. Behem tohoto cyklu do terminalu vypisujte jen stavova hlaseni co se deje
. Jaky akcelerometr detekovan, naber dat do SRAM, zapis na SD kartu, atd. Nikoli uz X Y Z data.
A na konci vysledek, OK, error, kde a jaky byl error apod.
>*/
//Myslel jsem tim jen aby to nebezelo samo porad dokola. Cim vice to rozkouskujete, tim take lepe, abych mohl otestovat jednotlive casti samostatne. To cele byla idea nad menu. Kdyz menu nejak pozmenite, aby lepe pasovalo k tomu jak mate usporadany jednotlive funkce v SW, tim lepe. Dulezite je menu, vyber fuknce, provedeni akce a zase ukonceni v menu a cekani na dalsi pokyn.
/*
(2) Dokoncete I2C pristup od ADXL - nastaveni, cteni dat

(3) Oprava a pripadne zmenseni kodu okolo SD karty, aby to dobre cetlo i zapisovalo soubory

Tady udelejete prosim alespon nastavovani realneho casu, 
nastavovani parametru naberu dat z ADXL (dobu sberu dat, vzorkovaci frekvenci) 
a pak sem prevedte testovaci menu.
 Do menu pridejte jeste moznost zaobrazovani realneho casu. Aby to cele vypadalo jako hodiny s datumem.




*/


