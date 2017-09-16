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
const int DigitalPort1_Pins[]={0,1,2,3,4,5,-1,-1}; //8 pins. If not used, set value to -1
const int DigitalPort1_DefaultValue[]={0,0,0,0,0,0,0,0}; //8 pins. If not used, set value to 0
const int DigitalPort1_Mode[]={PINMODE_DIGITAL_INPUT,PINMODE_DIGITAL_INPUT,PINMODE_DIGITAL_INPUT,PINMODE_DIGITAL_INPUT,PINMODE_DIGITAL_INPUT,PINMODE_DIGITAL_INPUT,PINMODE_NOTAVAILABLE,PINMODE_NOTAVAILABLE}; //8 pins. If not used, set value to PINMODE_NOTAVAILABLE
#endif
