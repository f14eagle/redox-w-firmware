
#define HAND_SENSE 12
#define RIGHT_HAND false
#define LEFT_HAND true

#define ALPHA_SENSE 20
#define ALPABETICAL false

// left hand pins

#define L_C01 10
#define L_C02 9
#define L_C03 7
#define L_C04 6
#define L_C05 5
#define L_C06 4
#define L_C07 3
#define L_C08 1
#define L_C09 0
#define L_C10 28
#define L_C11 25 
#define L_C12 24
#define L_C13 23
#define L_C14 22
#define L_R01 16
#define L_R02 15
#define L_R03 14
#define L_R04 13
#define L_R05 12

// right hand pins

#define R_C01 6
#define R_C02 5
#define R_C03 4
#define R_C04 3
#define R_C05 2
#define R_C06 0
#define R_C07 30
#define R_R01 21
#define R_R02 22
#define R_R03 23
#define R_R04 28
#define R_R05 29

#ifdef COMPILE_LEFT

#define PIPE_NUMBER 0

#define C01 L_C01
#define C02 L_C02
#define C03 L_C03
#define C04 L_C04
#define C05 L_C05
#define C06 L_C06
#define C07 L_C07
#define C08 L_C08
#define C09 L_C09
#define C10 L_C10
#define C11 L_C11
#define C12 L_C12
#define C13 L_C13
#define C14 L_C14
#define R01 L_R01
#define R02 L_R02
#define R03 L_R03
#define R04 L_R04
#define R05 L_R05

#define COLUMNS 14
#define DROWS 10 // ROWS * 2
#endif

#ifdef COMPILE_RIGHT

#define PIPE_NUMBER 2

#define C01 L_C01
#define C02 L_C02
#define C03 L_C03
#define C04 L_C04
#define C05 L_C05
#define R01 L_R01
#define R02 L_R02
#define R03 L_R03
#define R04 L_R04
#define R05 L_R05

#define COLUMNS 5 
#define DROWS 5 // ROWS
#endif

#define ROWS 5

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}
