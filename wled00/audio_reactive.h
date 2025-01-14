/*
 * This file allows you to add own functionality to WLED more easily
 * See: https://github.com/Aircoookie/WLED/wiki/Add-own-functionality
 * EEPROM bytes 2750+ are reserved for your custom use case. (if you extend #define EEPSIZE in const.h)
 * bytes 2400+ are currently ununsed, but might be used for future wled features
 */

// WARNING Sound reactive variables that are used by the animations or other asynchronous routines must NOT
// have interim values, but only updated in a single calculation. These are:
//
// sample     sampleAvg     sampleAgc       samplePeak    myVals[]
//
// fftBin[]   fftResult[]   FFT_MajorPeak   FFT_Magnitude
//
// Otherwise, the animations may asynchronously read interim values of these variables.
//

#include "wled.h"
#include <driver/i2s.h>
#include "audio_source.h"

AudioSource *audioSource;

// ALL AUDIO INPUT PINS DEFINED IN wled.h AND CONFIGURABLE VIA UI

// Comment/Uncomment to toggle usb serial debugging
// #define SR_DEBUG

#ifdef SR_DEBUG
  #define DEBUGSR_PRINT(x) Serial.print(x)
  #define DEBUGSR_PRINTLN(x) Serial.println(x)
  #define DEBUGSR_PRINTF(x...) Serial.printf(x)
#else
  #define DEBUGSR_PRINT(x)
  #define DEBUGSR_PRINTLN(x)
  #define DEBUGSR_PRINTF(x...)
#endif

// #define MIC_LOGGER
// #define MIC_SAMPLING_LOG
// #define FFT_SAMPLING_LOG

//#define MAJORPEAK_SUPPRESS_NOISE      // define to activate a dirty hack that ignores the lowest + hightest FFT bins

const i2s_port_t I2S_PORT = I2S_NUM_0;
const int BLOCK_SIZE = 128;
const int SAMPLE_RATE = 10240;                  // Base sample rate in Hz

//Use userVar0 and userVar1 (API calls &U0=,&U1=, uint16_t)

#ifndef LED_BUILTIN     // Set LED_BUILTIN if it is not defined by Arduino framework
  #define LED_BUILTIN 3
#endif

#define UDP_SYNC_HEADER "00001"

uint8_t maxVol = 10;                            // Reasonable value for constant volume for 'peak detector', as it won't always trigger
uint8_t binNum;                                 // Used to select the bin for FFT based beat detection.
uint8_t targetAgc = 60;                         // This is our setPoint at 20% of max for the adjusted output
uint8_t myVals[32];                             // Used to store a pile of samples because WLED frame rate and WLED sample rate are not synchronized. Frame rate is too low.
bool samplePeak = 0;                            // Boolean flag for peak. Responding routine must reset this flag
bool udpSamplePeak = 0;                         // Boolean flag for peak. Set at the same tiem as samplePeak, but reset by transmitAudioData
int delayMs = 10;                               // I don't want to sample too often and overload WLED
int micIn;                                      // Current sample starts with negative values and large values, which is why it's 16 bit signed
int sample;                                     // Current sample. Must only be updated ONCE!!!
int tmpSample;                                  // An interim sample variable used for calculatioins.
int sampleAdj;                                  // Gain adjusted sample value
int sampleAgc;                                  // Our AGC sample
uint16_t micData;                               // Analog input for FFT
uint16_t micDataSm;                             // Smoothed mic data, as it's a bit twitchy
long timeOfPeak = 0;
long lastTime = 0;
float micLev = 0;                               // Used to convert returned value to have '0' as minimum. A leveller
float multAgc;                                  // sample * multAgc = sampleAgc. Our multiplier
float sampleAvg = 0;                            // Smoothed Average
double beat = 0;                                // beat Detection

float expAdjF;                                  // Used for exponential filter.
float weighting = 0.2;                          // Exponential filter weighting. Will be adjustable in a future release.


// FFT Variables
const uint16_t samples = 512;                   // This value MUST ALWAYS be a power of 2
unsigned int sampling_period_us;
unsigned long microseconds;

double FFT_MajorPeak = 0;
double FFT_Magnitude = 0;
uint16_t mAvg = 0;

// These are the input and output vectors.  Input vectors receive computed results from FFT.
double vReal[samples];
double vImag[samples];
double fftBin[samples];

// Try and normalize fftBin values to a max of 4096, so that 4096/16 = 256.
// Oh, and bins 0,1,2 are no good, so we'll zero them out.
double fftCalc[16];
int fftResult[16];                              // Our calculated result table, which we feed to the animations.
double fftResultMax[16];                        // A table used for testing to determine how our post-processing is working.
float fftAvg[16];

// Table of linearNoise results to be multiplied by soundSquelch in order to reduce squelch across fftResult bins.
int linearNoise[16] = { 34, 28, 26, 25, 20, 12, 9, 6, 4, 4, 3, 2, 2, 2, 2, 2 };

// Table of multiplication factors so that we can even out the frequency response.
double fftResultPink[16] = {1.70,1.71,1.73,1.78,1.68,1.56,1.55,1.63,1.79,1.62,1.80,2.06,2.47,3.35,6.83,9.55};


struct audioSyncPacket {
  char header[6] = UDP_SYNC_HEADER;
  uint8_t myVals[32];     //  32 Bytes
  int sampleAgc;          //  04 Bytes
  int sample;             //  04 Bytes
  float sampleAvg;        //  04 Bytes
  bool samplePeak;        //  01 Bytes
  uint8_t fftResult[16];  //  16 Bytes
  double FFT_Magnitude;   //  08 Bytes
  double FFT_MajorPeak;   //  08 Bytes
};

double mapf(double x, double in_min, double in_max, double out_min, double out_max);

bool isValidUdpSyncVersion(char header[6]) {
  if (strncmp(header, UDP_SYNC_HEADER, 6) == 0) {
    return true;
  } else {
    return false;
  }
}

void getSample() {
  static long peakTime;
  //extern double FFT_Magnitude;                    // Optional inclusion for our volume routines // COMMENTED OUT - UNUSED VARIABLE COMPILER WARNINGS
  //extern double FFT_MajorPeak;                    // Same here. Not currently used though       // COMMENTED OUT - UNUSED VARIABLE COMPILER WARNINGS

  #ifdef WLED_DISABLE_SOUND
    micIn = inoise8(millis(), millis());          // Simulated analog read
  #else
    micIn = micDataSm;      // micDataSm = ((micData * 3) + micData)/4;
/*---------DEBUG---------*/
    DEBUGSR_PRINT("micIn:\tmicData:\tmicIn>>2:\tmic_In_abs:\tsample:\tsampleAdj:\tsampleAvg:\n");
    DEBUGSR_PRINT(micIn); DEBUGSR_PRINT("\t"); DEBUGSR_PRINT(micData);
/*-------END DEBUG-------*/
// We're still using 10 bit, but changing the analog read resolution in usermod.cpp
//    if (digitalMic == false) micIn = micIn >> 2;  // ESP32 has 2 more bits of A/D than ESP8266, so we need to normalize to 10 bit.
/*---------DEBUG---------*/
    DEBUGSR_PRINT("\t\t"); DEBUGSR_PRINT(micIn);
/*-------END DEBUG-------*/
  #endif
  micLev = ((micLev * 31) + micIn) / 32;          // Smooth it out over the last 32 samples for automatic centering
  micIn -= micLev;                                // Let's center it to 0 now
/*---------DEBUG---------*/
  DEBUGSR_PRINT("\t\t"); DEBUGSR_PRINT(micIn);
/*-------END DEBUG-------*/

// Using an exponential filter to smooth out the signal. We'll add controls for this in a future release.
  expAdjF = (weighting * micIn + (1.0-weighting) * expAdjF);
  expAdjF = (expAdjF <= soundSquelch) ? 0: expAdjF;

  expAdjF = fabs(expAdjF);                          // Now (!) take the absolute value
  tmpSample = (int)expAdjF;

/*---------DEBUG---------*/
  DEBUGSR_PRINT("\t\t"); DEBUGSR_PRINT(sample);
/*-------END DEBUG-------*/
  micIn = abs(micIn);                             // And get the absolute value of each sample

  sampleAdj = tmpSample * sampleGain / 40 + tmpSample / 16; // Adjust the gain.
  sampleAdj = min(sampleAdj, 255);                // Question: why are we limiting the value to 8 bits ???
  sample = sampleAdj;                             // ONLY update sample ONCE!!!!

  sampleAvg = ((sampleAvg * 15) + sample) / 16;   // Smooth it out over the last 16 samples.

/*---------DEBUG---------*/
  DEBUGSR_PRINT("\t"); DEBUGSR_PRINT(sample);
  DEBUGSR_PRINT("\t\t"); DEBUGSR_PRINT(sampleAvg); DEBUGSR_PRINT("\n\n");
/*-------END DEBUG-------*/

  // Fixes private class variable compiler error. Unsure if this is the correct way of fixing the root problem. -THATDONFC
  uint16_t MinShowDelay = strip.getMinShowDelay();

  if (millis() - timeOfPeak > MinShowDelay) {   // Auto-reset of samplePeak after a complete frame has passed.
    samplePeak = 0;
    udpSamplePeak = 0;
    }

  if (userVar1 == 0) samplePeak = 0;
  // Poor man's beat detection by seeing if sample > Average + some value.
  //  Serial.print(binNum); Serial.print("\t"); Serial.print(fftBin[binNum]); Serial.print("\t"); Serial.print(fftAvg[binNum/16]); Serial.print("\t"); Serial.print(maxVol); Serial.print("\t"); Serial.println(samplePeak);
  if ((fftBin[binNum] > maxVol) && (millis() > (peakTime + 100))) {                     // This goe through ALL of the 255 bins
  //  if (sample > (sampleAvg + maxVol) && millis() > (peakTime + 200)) {
  // Then we got a peak, else we don't. The peak has to time out on its own in order to support UDP sound sync.
    samplePeak = 1;
    timeOfPeak = millis();
    udpSamplePeak = 1;
    userVar1 = samplePeak;
    peakTime=millis();
  }
} // getSample()

/*
 * A simple averaging multiplier to automatically adjust sound sensitivity.
 */
/*
 * A simple, but hairy, averaging multiplier to automatically adjust sound sensitivity.
 *    not sure if not sure "sample" or "sampleAvg" is the correct input signal for AGC
 */
void agcAvg() {

  float lastMultAgc = multAgc;
  float tmpAgc;
  if(fabs(sampleAvg) < 2.0) {
    tmpAgc = sampleAvg;                           // signal below squelch -> deliver silence
    multAgc = multAgc * 0.95;                     // slightly decrease gain multiplier
  } else {
    multAgc = (sampleAvg < 1) ? targetAgc : targetAgc / sampleAvg;  // Make the multiplier so that sampleAvg * multiplier = setpoint
  }

  if (multAgc < 0.5) multAgc = 0.5;               // signal higher than 2*setpoint -> don't reduce it further
  multAgc = (lastMultAgc*127.0 +multAgc) / 128.0; //apply some filtering to gain multiplier -> smoother transitions
  tmpAgc = (float)sample * multAgc;               // apply gain to signal
  if (tmpAgc <= (soundSquelch*1.2)) tmpAgc = sample;  // check against squelch threshold - increased by 20% to avoid artefacts (ripples)

  if (tmpAgc > 255) tmpAgc = 255;
  sampleAgc = tmpAgc;                             // ONLY update sampleAgc ONCE because it's used elsewhere asynchronously!!!!
  userVar0 = sampleAvg * 4;
  if (userVar0 > 255) userVar0 = 255;
} // agcAvg()



////////////////////
// Begin FFT Code //
////////////////////

#include "arduinoFFT.h"

void transmitAudioData() {
  if (!udpSyncConnected) return;
  extern uint8_t myVals[];
  extern int sampleAgc;
  extern int sample;
  extern float sampleAvg;
  extern bool udpSamplePeak;
  extern int fftResult[];
  extern double FFT_Magnitude;
  extern double FFT_MajorPeak;

  audioSyncPacket transmitData;

  for (int i = 0; i < 32; i++) {
    transmitData.myVals[i] = myVals[i];
  }

  transmitData.sampleAgc = sampleAgc;
  transmitData.sample = sample;
  transmitData.sampleAvg = sampleAvg;
  transmitData.samplePeak = udpSamplePeak;
  udpSamplePeak = 0;                              // Reset udpSamplePeak after we've transmitted it

  for (int i = 0; i < 16; i++) {
    transmitData.fftResult[i] = (uint8_t)constrain(fftResult[i], 0, 254);
  }

  transmitData.FFT_Magnitude = FFT_Magnitude;
  transmitData.FFT_MajorPeak = FFT_MajorPeak;

  fftUdp.beginMulticastPacket();
  fftUdp.write(reinterpret_cast<uint8_t *>(&transmitData), sizeof(transmitData));
  fftUdp.endPacket();
  return;
} // transmitAudioData()




// Create FFT object
arduinoFFT FFT = arduinoFFT( vReal, vImag, samples, SAMPLE_RATE );

double fftAdd( int from, int to) {
  int i = from;
  double result = 0;
  while ( i <= to) {
    result += fftBin[i++];
  }
  return result;
}

// FFT main code
void FFTcode( void * parameter) {
  DEBUG_PRINT("FFT running on core: "); DEBUG_PRINTLN(xPortGetCoreID());
#ifdef MAJORPEAK_SUPPRESS_NOISE
  static double xtemp[24] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
#endif

  for(;;) {
    delay(1);           // DO NOT DELETE THIS LINE! It is needed to give the IDLE(0) task enough time and to keep the watchdog happy.
                        // taskYIELD(), yield(), vTaskDelay() and esp_task_wdt_feed() didn't seem to work.

    // Only run the FFT computing code if we're not in Receive mode
    if (audioSyncEnabled & (1 << 1))
      continue;
    audioSource->getSamples(vReal, samples);

    // old code - Last sample in vReal is our current mic sample
    //micDataSm = (uint16_t)vReal[samples - 1]; // will do a this a bit later

    // micDataSm = ((micData * 3) + micData)/4;

    double maxSample = 0.0;
    for (int i=0; i < samples; i++)
    {
	    // set imaginary parts to 0
      vImag[i] = 0;
	    // pick our  our current mic sample - we take the max value from all samples that go into FFT
	    if ((vReal[i] <= (INT16_MAX - 1024)) && (vReal[i] >= (INT16_MIN + 1024)))  //skip extreme values - normally these are artefacts
	    {
		    if (fabs(vReal[i]) > maxSample) maxSample = fabs(vReal[i]);
	    }
    }
	  micDataSm = (uint16_t)maxSample;

    FFT.DCRemoval(); // let FFT lib remove DC component, so we don't need to care about this in getSamples()

    //FFT.Windowing( FFT_WIN_TYP_HAMMING, FFT_FORWARD );        // Weigh data - standard Hamming window
    //FFT.Windowing( FFT_WIN_TYP_BLACKMAN, FFT_FORWARD );       // Blackman window - better side freq rejection
    //FFT.Windowing( FFT_WIN_TYP_BLACKMAN_HARRIS, FFT_FORWARD );// Blackman-Harris - excellent sideband rejection
    FFT.Windowing( FFT_WIN_TYP_FLT_TOP, FFT_FORWARD );         // Flat Top Window - better amplitude accuracy
    FFT.Compute( FFT_FORWARD );                             // Compute FFT
    FFT.ComplexToMagnitude();                               // Compute magnitudes

    //
    // vReal[3 .. 255] contain useful data, each a 20Hz interval (60Hz - 5120Hz).
    // There could be interesting data at bins 0 to 2, but there are too many artifacts.
    //
#ifdef MAJORPEAK_SUPPRESS_NOISE
    // teporarily reduce signal strength in the highest + lowest bins
    xtemp[0] = vReal[0]; vReal[0] *= 0.005;
    xtemp[1] = vReal[1]; vReal[1] *= 0.005;
    xtemp[2] = vReal[2]; vReal[2] *= 0.005;
    xtemp[3] = vReal[3]; vReal[3] *= 0.02;
    xtemp[4] = vReal[4]; vReal[4] *= 0.02;
    xtemp[5] = vReal[5]; vReal[5] *= 0.02;
    xtemp[6] = vReal[6]; vReal[6] *= 0.05;
    xtemp[7] = vReal[7]; vReal[7] *= 0.08;
    xtemp[8] = vReal[8]; vReal[8] *= 0.1;
    xtemp[9] = vReal[9]; vReal[9] *= 0.2;
    xtemp[10] = vReal[10]; vReal[10] *= 0.2;
    xtemp[11] = vReal[11]; vReal[11] *= 0.25;
    xtemp[12] = vReal[12]; vReal[12] *= 0.3;
    xtemp[13] = vReal[13]; vReal[13] *= 0.3;
    xtemp[14] = vReal[14]; vReal[14] *= 0.4;
    xtemp[15] = vReal[15]; vReal[15] *= 0.4;
    xtemp[16] = vReal[16]; vReal[16] *= 0.4;
    xtemp[17] = vReal[17]; vReal[17] *= 0.5;
    xtemp[18] = vReal[18]; vReal[18] *= 0.5;
    xtemp[19] = vReal[19]; vReal[19] *= 0.6;
    xtemp[20] = vReal[20]; vReal[20] *= 0.7;
    xtemp[21] = vReal[21]; vReal[21] *= 0.8;

    xtemp[22] = vReal[samples-2]; vReal[samples-2] =0.0;
    xtemp[23] = vReal[samples-1]; vReal[samples-1] =0.0;
#endif

    FFT.MajorPeak(&FFT_MajorPeak, &FFT_Magnitude);          // let the effects know which freq was most dominant

#ifdef MAJORPEAK_SUPPRESS_NOISE
	// dirty hack: limit suppressed channel intensities to FFT_Magnitude
	for (int k=0; k < 24; k++) if(xtemp[k] > FFT_Magnitude) xtemp[k] = FFT_Magnitude;
    // restore bins
    vReal[0] = xtemp[0];
    vReal[1] = xtemp[1];
    vReal[2] = xtemp[2];
    vReal[3] = xtemp[3];
    vReal[4] = xtemp[4];
    vReal[5] = xtemp[5];
    vReal[6] = xtemp[6];
    vReal[7] = xtemp[7];
    vReal[8] = xtemp[8];
    vReal[9] = xtemp[9];
    vReal[10] = xtemp[10];
    vReal[11] = xtemp[11];
    vReal[12] = xtemp[12];
    vReal[13] = xtemp[13];
    vReal[14] = xtemp[14];
    vReal[15] = xtemp[15];
    vReal[16] = xtemp[16];
    vReal[17] = xtemp[17];
    vReal[18] = xtemp[18];
    vReal[19] = xtemp[19];
    vReal[20] = xtemp[20];
    vReal[21] = xtemp[21];
    vReal[samples-2] = xtemp[22];
    vReal[samples-1] = xtemp[23];
#endif

    for (int i = 0; i < samples; i++) {                     // Values for bins 0 and 1 are WAY too large. Might as well start at 3.
      double t = 0.0;
      t = fabs(vReal[i]);                                   // just to be sure - values in fft bins should be positive any way
      t = t / 16.0;                                         // Reduce magnitude. Want end result to be linear and ~4096 max.
      fftBin[i] = t;
    } // for()


/* This FFT post processing is a DIY endeavour. What we really need is someone with sound engineering expertise to do a great job here AND most importantly, that the animations look GREAT as a result.
 *
 *
 * Andrew's updated mapping of 256 bins down to the 16 result bins with Sample Freq = 10240, samples = 512 and some overlap.
 * Based on testing, the lowest/Start frequency is 60 Hz (with bin 3) and a highest/End frequency of 5120 Hz in bin 255.
 * Now, Take the 60Hz and multiply by 1.320367784 to get the next frequency and so on until the end. Then detetermine the bins.
 * End frequency = Start frequency * multiplier ^ 16
 * Multiplier = (End frequency/ Start frequency) ^ 1/16
 * Multiplier = 1.320367784
 */

//                                               Range
      fftCalc[0] = (fftAdd(3,4)) /2;        // 60 - 100
      fftCalc[1] = (fftAdd(4,5)) /2;        // 80 - 120
      fftCalc[2] = (fftAdd(5,7)) /3;        // 100 - 160
      fftCalc[3] = (fftAdd(7,9)) /3;        // 140 - 200
      fftCalc[4] = (fftAdd(9,12)) /4;       // 180 - 260
      fftCalc[5] = (fftAdd(12,16)) /5;      // 240 - 340
      fftCalc[6] = (fftAdd(16,21)) /6;      // 320 - 440
      fftCalc[7] = (fftAdd(21,28)) /8;      // 420 - 600
      fftCalc[8] = (fftAdd(29,37)) /10;     // 580 - 760
      fftCalc[9] = (fftAdd(37,48)) /12;     // 740 - 980
      fftCalc[10] = (fftAdd(48,64)) /17;    // 960 - 1300
      fftCalc[11] = (fftAdd(64,84)) /21;    // 1280 - 1700
      fftCalc[12] = (fftAdd(84,111)) /28;   // 1680 - 2240
      fftCalc[13] = (fftAdd(111,147)) /37;  // 2220 - 2960
      fftCalc[14] = (fftAdd(147,194)) /48;  // 2940 - 3900
      fftCalc[15] = (fftAdd(194, 255)) /62; // 3880 - 5120


//   Noise supression of fftCalc bins using soundSquelch adjustment for different input types.
    for (int i=0; i < 16; i++) {
        fftCalc[i] = fftCalc[i]-(float)soundSquelch*(float)linearNoise[i]/4.0 <= 0? 0 : fftCalc[i];
    }

// Adjustment for frequency curves.
  for (int i=0; i < 16; i++) {
    fftCalc[i] = fftCalc[i] * fftResultPink[i];
  }

// Manual linear adjustment of gain using sampleGain adjustment for different input types.
    for (int i=0; i < 16; i++) {
        fftCalc[i] = fftCalc[i] * sampleGain / 40 + fftCalc[i]/16.0;
    }


// Now, let's dump it all into fftResult. Need to do this, otherwise other routines might grab fftResult values prematurely.
    for (int i=0; i < 16; i++) {
        // fftResult[i] = (int)fftCalc[i];
        fftResult[i] = constrain((int)fftCalc[i],0,254);         // question: why do we constrain values to 8bit here ???
        fftAvg[i] = (float)fftResult[i]*.05 + (1-.05)*fftAvg[i];
    }


// Looking for fftResultMax for each bin using Pink Noise
//      for (int i=0; i<16; i++) {
//          fftResultMax[i] = ((fftResultMax[i] * 63.0) + fftResult[i]) / 64.0;
//         Serial.print(fftResultMax[i]*fftResultPink[i]); Serial.print("\t");
//        }
//      Serial.println(" ");

  } // for(;;)
} // FFTcode()


void logAudio() {
#ifdef MIC_LOGGER


  //Serial.print("micData:");    Serial.print(micData);   Serial.print("\t");
  //Serial.print("micDataSm:");  Serial.print(micDataSm); Serial.print("\t");
  //Serial.print("micIn:");      Serial.print(micIn);     Serial.print("\t");
  //Serial.print("micLev:");     Serial.print(micLev);      Serial.print("\t");
  Serial.print("sample:");     Serial.print(sample);      Serial.print("\t");
  //Serial.print("sampleAvg:");  Serial.print(sampleAvg);   Serial.print("\t");
  //Serial.print("multAgc:");    Serial.print(multAgc);   Serial.print("\t");
  Serial.print("sampleAgc:");  Serial.print(sampleAgc);   Serial.print("\t");
  Serial.println(" ");

#endif

#ifdef MIC_SAMPLING_LOG
  //------------ Oscilloscope output ---------------------------
  Serial.print(targetAgc); Serial.print(" ");
  Serial.print(multAgc); Serial.print(" ");
  Serial.print(sampleAgc); Serial.print(" ");

  Serial.print(sample); Serial.print(" ");
  Serial.print(sampleAvg); Serial.print(" ");
  Serial.print(micLev); Serial.print(" ");
  Serial.print(samplePeak); Serial.print(" ");    //samplePeak = 0;
  Serial.print(micIn); Serial.print(" ");
  Serial.print(100); Serial.print(" ");
  Serial.print(0); Serial.print(" ");
  Serial.println(" ");
#endif

#ifdef FFT_SAMPLING_LOG
  #if 0
    for(int i=0; i<16; i++) {
      Serial.print(fftResult[i]);
      Serial.print("\t");
    }
    Serial.println("");
  #endif

  // OPTIONS are in the following format: Description \n Option
  //
  // Set true if wanting to see all the bands in their own vertical space on the Serial Plotter, false if wanting to see values in Serial Monitor
  const bool mapValuesToPlotterSpace = false;
  // Set true to apply an auto-gain like setting to to the data (this hasn't been tested recently)
  const bool scaleValuesFromCurrentMaxVal = false;
  // prints the max value seen in the current data
  const bool printMaxVal = false;
  // prints the min value seen in the current data
  const bool printMinVal = false;
  // if !scaleValuesFromCurrentMaxVal, we scale values from [0..defaultScalingFromHighValue] to [0..scalingToHighValue], lower this if you want to see smaller values easier
  const int defaultScalingFromHighValue = 256;
  // Print values to terminal in range of [0..scalingToHighValue] if !mapValuesToPlotterSpace, or [(i)*scalingToHighValue..(i+1)*scalingToHighValue] if mapValuesToPlotterSpace
  const int scalingToHighValue = 256;
  // set higher if using scaleValuesFromCurrentMaxVal and you want a small value that's also the current maxVal to look small on the plotter (can't be 0 to avoid divide by zero error)
  const int minimumMaxVal = 1;

  int maxVal = minimumMaxVal;
  int minVal = 0;
  for(int i = 0; i < 16; i++) {
    if(fftResult[i] > maxVal) maxVal = fftResult[i];
    if(fftResult[i] < minVal) minVal = fftResult[i];
  }
  for(int i = 0; i < 16; i++) {
    Serial.print(i); Serial.print(":");
    Serial.printf("%04d ", map(fftResult[i], 0, (scaleValuesFromCurrentMaxVal ? maxVal : defaultScalingFromHighValue), (mapValuesToPlotterSpace*i*scalingToHighValue)+0, (mapValuesToPlotterSpace*i*scalingToHighValue)+scalingToHighValue-1));
  }
  if(printMaxVal) {
    Serial.printf("maxVal:%04d ", maxVal + (mapValuesToPlotterSpace ? 16*256 : 0));
  }
  if(printMinVal) {
    Serial.printf("%04d:minVal ", minVal);  // printed with value first, then label, so negative values can be seen in Serial Monitor but don't throw off y axis in Serial Plotter
  }
  if(mapValuesToPlotterSpace)
    Serial.printf("max:%04d ", (printMaxVal ? 17 : 16)*256); // print line above the maximum value we expect to see on the plotter to avoid autoscaling y axis
  else
    Serial.printf("max:%04d ", 256);
  Serial.println();
#endif // FFT_SAMPLING_LOG
} // logAudio()
