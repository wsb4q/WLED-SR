#ifndef SOUND_REACTIVE_H
#define SOUND_REACTIVE_H

#include <stdint.h>
#include "wled.h"

/******************* COMMON OPTION DEFINITIONS **************/

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

#ifndef LED_BUILTIN     // Set LED_BUILTIN if it is not defined by Arduino framework
  #define LED_BUILTIN 3
#endif

/******************* SHARED AUDIO VARIABLES **************/

// TODO: put comments in one place only so they don't get out of sync as they change

// WARNING Sound reactive variables that are used by the animations or other asynchronous routines must NOT
// have interim values, but only updated in a single calculation. These are:
//
// sample     sampleAvg     sampleAgc       samplePeak    myVals[]
//
// fftBin[]   fftResult[]   FFT_MajorPeak   FFT_Magnitude
//
// Otherwise, the animations may asynchronously read interim values of these variables.
//

const uint16_t samples = 512;                   // This value MUST ALWAYS be a power of 2

extern uint8_t myVals[32];                      // Used to store a pile of samples because WLED frame rate and WLED sample rate are not synchronized. Frame rate is too low.
extern int sample;                              // Current sample. Must only be updated ONCE!!!
extern bool samplePeak;                         // Boolean flag for peak. Responding routine must reset this flag
extern int sampleAgc;                           // Our AGC sample
extern float sampleAvg;                         // Smoothed Average
extern double beat;                             // beat Detection
extern double FFT_MajorPeak;                    // Optional inclusion for our volume routines
extern double FFT_Magnitude;                    // Same here. Not currently used though

// Try and normalize fftBin values to a max of 4096, so that 4096/16 = 256.
// Oh, and bins 0,1,2 are no good, so we'll zero them out.
extern double fftBin[samples];                  // raw FFT data
extern int fftResult[16];                       // summary of bins array. 16 summary bins.


class SoundreactiveUsermod : public Usermod {
  public:
    void setup();

    void loop();

  private:
};

#endif
