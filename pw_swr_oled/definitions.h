/*
* Setup definitions
*
*/
#define VERSION       "V 0.1"

#define SCREEN_BT_PIN PB12 

//-----------------------------------------------------------------------------
// Internal ADC definitions
#define ADC_FWD       A0
#define ADC_REV       A1
// Measure using DVM to correct value from Vref
#define V_REF         3.251
#define ADC_VREF      3.251
#define ADC_BITS      4096
#define ADC_RES       12
// Smoothing factor (0 < alpha < 1)
#define ALPHA         0.6
// ((R1+R2) / R2) * Vo = Vi       
// ADC scale -> ((R1+R2) / R2)       
#define ADC_SCALE     4.09

// Bridge definitions (1N5711)
#define DIODE_DROP_F  0.3
#define DIODE_DROP_R  0.3
#define TURNS_RATIO   10
