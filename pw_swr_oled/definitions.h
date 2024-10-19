/*
* Setup definitions
*
*/
#define VERSION       "V 0.1"

#define SCREEN_BT_PIN PB12 
// Enable serial plot (decrease update speed from screen)
#define SERIAL        0
//-----------------------------------------------------------------------------
// Internal ADC definitions
#define ADC_FWD       A0
#define ADC_REV       A1
// Measure using DVM to correct value from Vref
#define V_REF         3.251
#define ADC_VREF      3.251
#define ADC_BITS      4095
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

// Calculate gradient and intercept to increase precision
// m = (y2 - y1) / (x2 - x1)
// c = y1 - (m * x1)
// y1, y2 measured values (use oscilloscope or RF Probe)
// x1, x2 reported values from fwdVp and revVp 
#define M_FWD_VP      1
#define C_FWD_VP      0
#define M_REV_VP      1
#define C_REV_VP      0
