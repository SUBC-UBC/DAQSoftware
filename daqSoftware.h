// DAQ 2020
// GPL v3
// SUBC: the UBC Submarine Design Team

// Define which parts are on
//#define TACHO
#define SDON
//#define DEPTHSENSOR
//#define BATTSENSE

// Change when plugged in to Arduino
#define tachoPin 2
#define BUTTONPIN PC14
#define INDICATORPIN PB12
#define SDAPIN PB9
#define SCLPIN PB8
#define IMUINTPIN PB11

//#define BEFORECALC 4
#define MEASUREDELAY 500
#define CHANGEPERREV 2
#define DELAYTIME 5
#define DEBOUNCETIME 250

// Function prototypes
void buttonPress(void);
void tachoChange(void);
void init_DMP(void);
void init_SD(void);
void init_IMU(void);
void init_Depth(void);
void getDMPData(void);
void print_data_to_file(void);
void print_data_to_serial(void);