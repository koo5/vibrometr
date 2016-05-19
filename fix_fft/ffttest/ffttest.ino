// Play with the fix_fft library from:
// http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1286718155
// but note that I made two fixes to the code. They aren't bug
// fixes but they do reduce the amount of SRAM used.
// line 49 in fix_fft.cpp should be:
// const prog_char Sinewave[N_WAVE-N_WAVE/4] PROGMEM = {
// NOT prog_int8_t
// line 198 in fix_fft.cpp should be:
// 		wr =  pgm_read_byte_near(Sinewave + j+N_WAVE/4);
// NOT pgm_read_word_near
#include "fix_fft.h"

char im[128];
char re[128];

// sampling frequency rate - a frequency of 5120Hz was chosen because
// it is an exact multiple of 128. This means that there will be
// 5120/128 = 40Hz per FFT "bin"
#define F_SAMPLE 5120

// amplitude of each "carrier" wave
#define AMP_CARRIER 68

void setup()
{
  int i;
  // frequency of tone1
  double f_tone1;
  // phase of tone1
  double phase1;
  // frequency of tone2
  double f_tone2;
  // phase of tone2
  double phase2;
  double deviation;

  Serial.begin(115200);

//>> 1
  // Create 128 samples of a single 1280Hz tone and then generate
  // and draw the FFT.  
  f_tone1 = 1280.0;
  phase1 = 0.0;
  for(i=0;i<128;i++) {
    // The imaginary part is always zero
    im[i] = 0;
    // a sample of wave 1
    re[i] = (char)(AMP_CARRIER*sin(phase1*2.0*PI));
    // update the phase of wave 1
    phase1 += f_tone1/F_SAMPLE;
    if(phase1 >= 1)phase1 -= 1;
  }
  // compute the FFT of the samples
  fix_fft(re,im,7,0);
  // and send it to the serial monitor
  print_fft((char *)"1. Single 1280Hz tone sampled at 5120Hz");

//>> 2
  //This time generate two separate tones at 1040Hz and 1280Hz
  // The amplitude of the second tone is reduced to half that of
  // the other one so that together they don't overflow 8-bit
  // arithmetic. The first wave is +/- 68 and the second is
  // +/- 32 which when added together produces +/- 100 which
  // will fit within the 8-bit limitation of -128 to +127
  
#define AMP2 34
  f_tone1 = 1280.0;
  f_tone2 = 1040.0;
  phase1 = 0.0;
  phase2 = 0.0;
  for(i=0;i<128;i++) {
    // The imaginary part is always zero
    im[i] = 0;
    // a sample of wave 1
    re[i] = (char)(AMP_CARRIER*sin(phase1*2.0*PI));
    // add the sample of wave 2
    re[i] += (char)(AMP2*sin(phase2*2.0*PI));
    // update the phase of wave 1
    phase1 += f_tone1/F_SAMPLE;
    if(phase1 >= 1)phase1 -= 1;
    // update the phase of wave 2
    phase2 += f_tone2/F_SAMPLE;
    if(phase2 >= 1)phase2 -= 1;
  }
  // compute the FFT of the samples
  fix_fft(re,im,7,0);
  print_fft((char *)"2. Two tones of 1040Hz and 1280Hz sampled at 5120Hz");


//>> 3
  // This time generate a "carrier" frequency of 1280Hz which is
  // amplitude modulated by a 240Hz tone

#define AMP_AUDIO 32
  f_tone1 = 240;
  f_tone2 = 1280;
  phase1 = 0.0;
  phase2 = 0.0;
  for(i=0;i<128;i++) {
    // The imaginary part is always zero
    im[i] = 0;
    // Modulate a "carrier" wave, having a frequency of f_tone2=1280Hz,
    // with an "audio" wave having a frequency of f_tone1=240Hz
    // Note that if AMP_AUDIO is set to zero then this reduces to
    // AMP_CARRIER*sin(phase2*2*PI) which is just the carrier wave. 
    re[i] = (char)((AMP_CARRIER + AMP_AUDIO*sin(phase1*2*PI))*sin(phase2*2.0*PI));
    // update the phase of wave 1
    phase1 += f_tone1/F_SAMPLE;
    if(phase1 >= 1)phase1 -= 1;
    // update the phase of wave 2
    phase2 += f_tone2/F_SAMPLE;
    if(phase2 >= 1)phase2 -= 1;
  }
  // compute the FFT of the samples
  fix_fft(re,im,7,0);
  print_fft((char *)"3. AM modulation - 1280Hz carrier modulated by 240Hz audio tone");


//>> 4
  // Just for fun let's try FM as well!

  // deviation of 1:1
  // (actually this is the peak deviation)
  // In this case a signal (audio) frequency of 240Hz produces a
  // change of 240Hz in the carrier frequency
  deviation = 240;
  f_tone1 = 240;
  f_tone2 = 1280;
  phase1 = 0.0;
  phase2 = 0.0;
  for(i=0;i<128;i++) {
    // The imaginary part is always zero
    im[i] = 0;
    // FM modulate a "carrier" wave, having a frequency of f_tone2=1280Hz,
    // with an "audio" wave having a frequency of f_tone1=240Hz
    re[i] = (char)(AMP_CARRIER*sin(2*PI*phase2 + (deviation/f_tone1)*sin(phase1*2*PI)));
    // update the phase of wave 1
    phase1 += f_tone1/F_SAMPLE;
    if(phase1 >= 1)phase1 -= 1;
    // update the phase of wave 2
    phase2 += f_tone2/F_SAMPLE;
    if(phase2 >= 1)phase2 -= 1;
  }
  // compute the FFT of the samples
  fix_fft(re,im,7,0);
  print_fft((char *)"4. FM modulation - 1280Hz carrier modulated by 240Hz audio tone with deviation of 240Hz");


//>> 5
  // deviation of 2:1
  // Here a 240Hz signal frequency causes the carrier to change by 480Hz.
  deviation = 480;
  f_tone1 = 240;
  f_tone2 = 1280;
  phase1 = 0.0;
  phase2 = 0.0;
  for(i=0;i<128;i++) {
    // The imaginary part is always zero
    im[i] = 0;
    // Modulate a "carrier" wave, having a frequency of f_tone2=1280Hz,
    // with an "audio" wave having a frequency of f_tone1=240Hz
    re[i] = (char)(AMP_CARRIER*sin(2*PI*phase2 + (deviation/f_tone1)*sin(phase1*2*PI)));
    // update the phase of wave 1
    phase1 += f_tone1/F_SAMPLE;
    if(phase1 >= 1)phase1 -= 1;
    // update the phase of wave 2
    phase2 += f_tone2/F_SAMPLE;
    if(phase2 >= 1)phase2 -= 1;
  }
  // compute the FFT of the samples
  fix_fft(re,im,7,0);
  print_fft((char *)"5. FM modulation - 1280Hz carrier modulated by 240Hz audio tone with deviation of 480Hz");


//>> 6
  // deviation of 1:2
  // The signal of 240Hz causes the carrier frequency to change by 120Hz.
  deviation = 120;
  f_tone1 = 240;
  f_tone2 = 1280;
  phase1 = 0.0;
  phase2 = 0.0;
  for(i=0;i<128;i++) {
    // The imaginary part is always zero
    im[i] = 0;
    // Modulate a "carrier" wave, having a frequency of f_tone2=1280Hz,
    // with an "audio" wave having a frequency of f_tone1=240Hz
    re[i] = (char)(AMP_CARRIER*sin(2*PI*phase2 + (deviation/f_tone1)*sin(phase1*2*PI)));
    // update the phase of wave 1
    phase1 += f_tone1/F_SAMPLE;
    if(phase1 >= 1)phase1 -= 1;
    // update the phase of wave 2
    phase2 += f_tone2/F_SAMPLE;
    if(phase2 >= 1)phase2 -= 1;
  }
  // compute the FFT of the samples
  fix_fft(re,im,7,0);
  print_fft((char *)"6. FM modulation - 1280Hz carrier modulated by 240Hz audio tone with deviation of 120Hz");
}

// Generate the magnitude of each bin and print it on the serial monitor
void print_fft(char *title)
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
  Serial.println(title);
  Serial.println("");
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
  Serial.print("    ");
  for(i=0;i<64;i++) {
    Serial.print(i%10);
  }
  Serial.println("");
  Serial.print("    ");
  for(i=0;i<64;i++) {
    if(i < 10)Serial.print(" ");
    else Serial.print((i/10)%10);
  }
  Serial.println("");
}

void loop()
{
}

