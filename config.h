//Hardware configuration
#ifndef __CONFIG_INCLUDED__   
#define __CONFIG_INCLUDED__

#define BOARD_ID 18
#define BOARD_TYPE BOARDTYPE_ARDUINOUNO
#define PRINT_DEBUG_LINES 1
#define BYPASS_SHIELDCONFIG 0
#define SHIELD1_TYPE SHIELDTYPE_NONE
#define SHIELD2_TYPE SHIELDTYPE_NONE
#define SHIELD3_TYPE SHIELDTYPE_NONE
#define SHIELD4_TYPE SHIELDTYPE_NONE

const int AnalogInputPort1[]={A0,A1,-1,-1,-1,-1}; //6 pins.  If not used, set value to -1

#endif
