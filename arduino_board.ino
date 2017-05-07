//Board Configuration Defines
#define BOARD_ID 17
#define BOARD_TYPE BOARDTYPE_ARDUINOMEGA
#define SHIELD1_TYPE SHIELDTYPE_TERMINALSHIELD
#define SHIELD1_ID 0 //Hex
#define SHIELD2_TYPE SHIELDTYPE_SERVOSHIELD
#define SHIELD2_ID 40 //Hex
#define SHIELD3_TYPE SHIELDTYPE_NONE
#define SHIELD3_ID 0 //Hex
#define SHIELD4_TYPE SHIELDTYPE_NONE
#define SHIELD4_ID 0 //Hex


//Useful Debug Defines
#define PRINT_DEBUG_LINES 0

#define FIRMWARE_MAJOR_VERSION 0
#define FIRMWARE_MINOR_VERSION 5
#define FIRMWARE_BUILD_NUMBER 0

#include "Arduino.h"
#include <SoftwareSerial.h>
#include "serialmessage.h"
#include "Definitions.h"

#include <Wire.h>
#include <avr/wdt.h>

#if BOARD_TYPE == BOARDTYPE_ARDUINOMEGA
    #define MAXNUMBER_SHIELDS 4
    #define MAXNUMBER_PORTS_PERSHIELD 4
#elif BOARD_TYPE == BOARDTYPE_ARDUINOUNO
    #define MAXNUMBER_SHIELDS 2
    #define MAXNUMBER_PORTS_PERSHIELD 2
#endif

#define DIOPORT_SIZE        8
#define ANAPORT_SIZE        4
#define SERIAL_MESSAGE_SIZE 16 //Start Delimiter thru Checksum

//Configuration defines for individual Shield Types

#if ((SHIELD1_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD2_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_LCDSHIELD))
  #include <LiquidCrystal.h>
  #include <LCDKeypad.h>
  #define BUTTON_RIGHT 0
  #define BUTTON_UP 1
  #define BUTTON_DOWN 2
  #define BUTTON_LEFT 3
  #define BUTTON_SELECT 4
  #define BUTTON_NONE 5
  
#endif

#if ((SHIELD1_TYPE == SHIELDTYPE_SERVOSHIELD) || (SHIELD2_TYPE == SHIELDTYPE_SERVOSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_SERVOSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_SERVOSHIELD))
  #include <Adafruit_PWMServoDriver.h>

  #define SERVOSHIELD_UPDATE_RATE 60
  //#define SERVOSHIELD_SERVO_MIN 224
  //#define SERVOSHIELD_SERVO_MAX 480
#endif

//Debugging Defines
#define PRINT_RECV_BUFFER 1

typedef struct
{
  unsigned char System;
  unsigned char SubSystem;
  unsigned char Component;
  unsigned char Diagnostic_Type;
  unsigned char Level;
  unsigned char Diagnostic_Message;
} diagnostic;

typedef struct
{
  int id;
  int Pin_Number[DIOPORT_SIZE];
  int Pin_Mode[DIOPORT_SIZE];
  int Pin_Value[DIOPORT_SIZE];
  int Pin_DefaultValue[DIOPORT_SIZE];
} dio_port;

typedef struct
{
  int id;
  int Pin_Number[ANAPORT_SIZE];
  int Pin_Mode[ANAPORT_SIZE];
  int Pin_Value[ANAPORT_SIZE];
  int Pin_DefaultValue[ANAPORT_SIZE];
} ana_port;
typedef struct
{
  int id;
  int type;
  boolean isconfigured;
  boolean dioports_configured;
  boolean anaports_configured;
  int dio_portcount;
  int ana_portcount;
  dio_port dio_ports[MAXNUMBER_PORTS_PERSHIELD];
  ana_port ana_ports[MAXNUMBER_PORTS_PERSHIELD];
} shield;

diagnostic diagnostic_status;
shield shields[MAXNUMBER_SHIELDS];
boolean dioportconfigure_messages_received[MAXNUMBER_SHIELDS*MAXNUMBER_PORTS_PERSHIELD];
boolean anaportconfigure_messages_received[MAXNUMBER_SHIELDS*MAXNUMBER_PORTS_PERSHIELD];
boolean dioportdefault_messages_received[MAXNUMBER_SHIELDS*MAXNUMBER_PORTS_PERSHIELD];
int dioportconfig_messages_expected = -1;
int dioportdefault_messages_expected = -1;
int anaportconfig_messages_expected = -1;
void run_veryfastrate_code(); //1000 Hz
void run_fastrate_code(); //100 Hz
void run_mediumrate_code(); //10 Hz
void run_slowrate_code(); //1 Hz
void run_veryslowrate_code(); //0.1 Hz

#if(SHIELD1_TYPE == SHIELDTYPE_SERVOSHIELD) 
  Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(SHIELD1_ID);
#endif

#if(SHIELD2_TYPE == SHIELDTYPE_SERVOSHIELD) 
  Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(SHIELD2_ID);
#endif

#if(SHIEL3_TYPE == SHIELDTYPE_SERVOSHIELD) 
  Adafruit_PWMServoDriver pwm3 = Adafruit_PWMServoDriver(SHIELD3_ID);
#endif

#if(SHIELD3_TYPE == SHIELDTYPE_SERVOSHIELD) 
  Adafruit_PWMServoDriver pwm4 = Adafruit_PWMServoDriver(SHIELD4_ID);
#endif

int board_mode = BOARDMODE_BOOTING;
int node_mode = BOARDMODE_UNDEFINED;
unsigned int armed_command = ROVERCOMMAND_DISARM;
unsigned int armed_state = ARMEDSTATUS_DISARMED_CANNOTARM;
unsigned char recv_buffer[32];
int current_message_buffer_length = 0;
int message_length = 0;
int message_type = 0;
int message_checksum = 0;
int passed_checksum_counter = 0;
int failed_checksum_counter = 0;
boolean message_started = false;
boolean message_complete = false;  // whether the string is complete
bool new_message = false;
SerialMessageHandler serialmessagehandler;

//Board Definitions
#if(BOARD_TYPE == BOARDTYPE_ARDUINOUNO)
  #define LED 13
  SoftwareSerial softSerial(2, 3); // RX, TX
#endif

#if(BOARD_TYPE == BOARDTYPE_ARDUINOMEGA)
  #define LED 13
#endif

int veryfastrate_counter = 0;
int fastrate_counter = 0;
int mediumrate_counter = 0;
int slowrate_counter = 0;
int veryslowrate_counter = 0;

unsigned long loop_counter = 0;
unsigned long send_mode_counter = 0;
unsigned long recv_mode_counter = 0;
unsigned long send_configure_dioport_counter = 0;
unsigned long recv_configure_dioport_counter = 0;
unsigned long send_configure_anaport_counter = 0;
unsigned long recv_configure_anaport_counter = 0;
unsigned long send_set_dio_counter = 0;
unsigned long recv_set_dio_counter = 0;
unsigned long send_set_dio_defaultvalue_counter = 0;
unsigned long recv_set_dio_defaultvalue_counter = 0;
unsigned long send_command_counter = 0;
unsigned long recv_command_counter = 0;
unsigned long send_ana_counter = 0;
unsigned long recv_ana_counter = 0;
unsigned long send_pps_counter = 0;
unsigned long recv_pps_counter = 0;
unsigned long send_diagnostic_counter = 0;
unsigned long recv_diagnostic_counter = 0;
unsigned long time_since_last_rx = 0;
unsigned long level_debug_counter = 0;
unsigned long level_info_counter = 0;
unsigned long level_notice_counter = 0;
unsigned long level_warn_counter = 0;
unsigned long level_error_counter = 0;
unsigned long level_fatal_counter = 0;
int comm_established_once = 0;



int temp_counter = 1000;
bool reverse = false;
int shield_count = -1;
int new_pps = 0;
int pps_received = 0;

//Function Prototypes for individual shields
//SERVOSHIELD
void SERVOSHIELD_setServoPulse(uint8_t pin_number, uint16_t pulse_us);  //No need for include guards, already taken care of in function.

//LCDSHIELD
#if ((SHIELD1_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD2_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_LCDSHIELD))
  LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
  void print_to_lcd(unsigned char System,unsigned char Subsystem,unsigned char Component,unsigned char DiagnosticType,unsigned char Level,unsigned char Message);
  void update_lcd();
  int read_lcd_buttons();
  int lcd_button_pressed;
  int level_selected = NOTICE;
  #define MENUPAGE_DEFAULT 0
  #define MENUPAGE_LEVELCOUNT 1
  int current_menupage = MENUPAGE_DEFAULT;
#endif

void init_shields();
String map_level_tostring(int v);
String map_component_tostring(int v);
String map_diagnostictype_tostring(int v);
String map_message_tostring(int v);
String map_mode_tostring(int v);

void(*resetFunc)(void) = 0;
void init_shields()
{
    for(int i = 0; i < MAXNUMBER_SHIELDS*MAXNUMBER_PORTS_PERSHIELD; i++)
    {
        dioportconfigure_messages_received[i] = false;
        dioportdefault_messages_received[i] = false;
        anaportconfigure_messages_received[i] = false;
    }
    int shield_index = 0;
    int shield_available = 1;
    while(shield_available == 1)
    {
        int shield_type = 0;
        int shield_id = 0;
        switch(shield_index)
        {
            case 0:
                shield_type = SHIELD1_TYPE;
                shield_id = SHIELD1_ID;
                break;
            case 1:
                shield_type = SHIELD2_TYPE;
                shield_id = SHIELD2_ID;
                break;
            case 2:
                shield_type = SHIELD3_TYPE;
                shield_id = SHIELD3_ID;
                break;
            case 3:
                shield_type = SHIELD4_TYPE;
                shield_id = SHIELD4_ID;
                break;
            default:
                shield_available = 0;
                break;
        }
        if(shield_available == 1)
        {
          shields[shield_index].id = shield_id;
          shields[shield_index].type = shield_type;
          shields[shield_index].isconfigured = false;
          switch(shield_type)
          {
              case SHIELDTYPE_UNDEFINED:
                  //Do Nothing
                  shields[shield_index].dio_portcount = 0;
                  shields[shield_index].ana_portcount = 0;
                  break;
              case SHIELDTYPE_NONE:
                  //Do Nothing
                  shields[shield_index].dio_portcount = 0;
                  shields[shield_index].ana_portcount = 0;
                  break;
              case SHIELDTYPE_SERVOSHIELD:
                  shields[shield_index].ana_portcount = 0;
                  shields[shield_index].dio_portcount = 2;
                  for(int i = 0; i < shields[shield_index].dio_portcount; i++)
                  {
                      shields[shield_index].dio_ports[i].id = i;
                      for(int j = 0; j < DIOPORT_SIZE; j++)
                      {
                          shields[shield_index].dio_ports[i].Pin_Number[j] = (DIOPORT_SIZE*i)+j;
                          shields[shield_index].dio_ports[i].Pin_Mode[j] = PINMODE_PWM_OUTPUT;
                          shields[shield_index].dio_ports[i].Pin_Value[j] = 127;
                          shields[shield_index].dio_ports[i].Pin_DefaultValue[j] = 127;
                      }
                  }
                  break;
              case SHIELDTYPE_LCDSHIELD:
                  break;
              case SHIELDTYPE_TERMINALSHIELD: //This is the default pinout for whatever arduino board 
                  if(BOARD_TYPE == BOARDTYPE_ARDUINOMEGA)
                  {
                      shields[shield_index].dio_portcount = 4;
                      shields[shield_index].ana_portcount = 4;
                      for(int i = 0; i < shields[shield_index].dio_portcount; i++)
                      {
                          shields[shield_index].dio_ports[i].id = i;
                          for(int j = 0; j < DIOPORT_SIZE; j++)
                          {
                              shields[shield_index].dio_ports[i].Pin_Number[j] = (DIOPORT_SIZE*i)+j;
                              shields[shield_index].dio_ports[i].Pin_Mode[j] = PINMODE_DIGITAL_INPUT;
                              shields[shield_index].dio_ports[i].Pin_Value[j] = 127;
                              shields[shield_index].dio_ports[i].Pin_DefaultValue[j] = 127;
                          }
                      }
                      for(int i = 0; i < shields[shield_index].ana_portcount; i++)
                      {
                          shields[shield_index].ana_ports[i].id = i;
                          for(int j = 0; j < DIOPORT_SIZE; j++)
                          {
                              shields[shield_index].ana_ports[i].Pin_Number[j] = (ANAPORT_SIZE*i)+j;
                              shields[shield_index].ana_ports[i].Pin_Mode[j] = PINMODE_ANALOG_INPUT;
                              shields[shield_index].ana_ports[i].Pin_Value[j] = 0;
                              shields[shield_index].ana_ports[i].Pin_DefaultValue[j] = 0;
                          }
                      }
                  }
                  else if(BOARD_TYPE == BOARDTYPE_ARDUINOUNO)
                  {
                      shields[shield_index].dio_portcount = 2;
                      shields[shield_index].ana_portcount = 2;
                      for(int i = 0; i < shields[shield_index].dio_portcount; i++)
                      {
                          shields[shield_index].dio_ports[i].id = i;
                          for(int j = 0; j < DIOPORT_SIZE; j++)
                          {
                              shields[shield_index].dio_ports[i].Pin_Number[j] = (DIOPORT_SIZE*i)+j;
                              if(shields[shield_index].dio_ports[i].Pin_Number[j] > 13)
                              {
                                  shields[shield_index].dio_ports[i].Pin_Mode[j] = PINMODE_NOTAVAILABLE;
                              }
                              else
                              {
                                  shields[shield_index].dio_ports[i].Pin_Mode[j] = PINMODE_DIGITAL_INPUT;
                              }
                              shields[shield_index].dio_ports[i].Pin_Value[j] = 127;
                              shields[shield_index].dio_ports[i].Pin_DefaultValue[j] = 127;
                          }
                      }
                      for(int i = 0; i < shields[shield_index].ana_portcount; i++)
                      {
                          shields[shield_index].ana_ports[i].id = i;
                          for(int j = 0; j < ANAPORT_SIZE; j++)
                          {
                              shields[shield_index].ana_ports[i].Pin_Number[j] = (ANAPORT_SIZE*i)+j;
                              if(shields[shield_index].ana_ports[i].Pin_Number[j] > 5)
                              {
                                  shields[shield_index].ana_ports[i].Pin_Mode[j] = PINMODE_NOTAVAILABLE;
                              }
                              else
                              {
                                  shields[shield_index].ana_ports[i].Pin_Mode[j] = PINMODE_ANALOG_INPUT;
                              }
                              shields[shield_index].ana_ports[i].Pin_Value[j] = 0;
                              shields[shield_index].ana_ports[i].Pin_DefaultValue[j] = 0;
                          }
                      }
                  }
                  break;
              case SHIELDTYPE_RELAYSHIELD:
                  shields[shield_index].dio_portcount = 0;
                  shields[shield_index].ana_portcount = 0;
                  break;
          }
          
          shield_index++;
      }
    }
}
void setup() {
  //Boot Procedures
  board_mode = BOARDMODE_BOOTING;
  diagnostic_status.System = ROVER;
  diagnostic_status.SubSystem = ROBOT_CONTROLLER;
  diagnostic_status.Component = GPIO_NODE;
  diagnostic_status.Diagnostic_Type = SOFTWARE;
  diagnostic_status.Level = INFO;
  diagnostic_status.Diagnostic_Message = INITIALIZING;
  wdt_disable();
  wdt_enable(WDTO_1S);
  memset(recv_buffer,0,sizeof(recv_buffer));
  pinMode(LED,OUTPUT);
  Serial.begin(115200);
  //Serial.setTimeout(1000);
  while(Serial.read() >= 0);
  Serial.flush();
  #if(BOARD_TYPE == BOARDTYPE_ARDUINOMEGA)
  {
    Serial1.begin(115200);
    Serial1.setTimeout(1000);
    while(Serial1.read() >= 0);
    Serial1.flush();
    Serial1.println("ArduinoBoard Booting");
    Serial1.print("FW Major Version: ");
    Serial1.println(FIRMWARE_MAJOR_VERSION,DEC);
    Serial1.print("FW Minor Version: ");
    Serial1.println(FIRMWARE_MINOR_VERSION,DEC);
    Serial1.print("FW Build Number: ");
    Serial1.println(FIRMWARE_BUILD_NUMBER,DEC);
    
    
  }
  #elif(BOARD_TYPE == BOARDTYPE_ARDUINOUNO)
  {
    #if ((SHIELD1_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD2_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_LCDSHIELD))
      lcd.begin(16, 2); 
      lcd.clear();
    #endif
    softSerial.begin(115200);
    softSerial.println("ArduinoBoard Booting");
    softSerial.print("FW Major Version: ");
    softSerial.println(FIRMWARE_MAJOR_VERSION,DEC);
    softSerial.print("FW Minor Version: ");
    softSerial.println(FIRMWARE_MINOR_VERSION,DEC);
    softSerial.print("FW Build Number: ");
    softSerial.println(FIRMWARE_BUILD_NUMBER,DEC);

    #if ((SHIELD1_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD2_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_LCDSHIELD))
    {
      lcd.print("Booted");
    }
    #endif
    #if (SHIELD1_TYPE == SHIELDTYPE_SERVOSHIELD)
    {
      pwm1.begin();
      pwm1.setPWMFreq(SERVOSHIELD_UPDATE_RATE);
    }
    #endif
    #if (SHIELD2_TYPE == SHIELDTYPE_SERVOSHIELD)
    {
      pwm2.begin();
      pwm2.setPWMFreq(SERVOSHIELD_UPDATE_RATE);
    }
    #endif
    #if (SHIELD3_TYPE == SHIELDTYPE_SERVOSHIELD)
    {
      pwm3.begin();
      pwm3.setPWMFreq(SERVOSHIELD_UPDATE_RATE);
    }
    #endif
    #if (SHIELD4_TYPE == SHIELDTYPE_SERVOSHIELD)
    {
      pwm4.begin();
      pwm4.setPWMFreq(SERVOSHIELD_UPDATE_RATE);
    }
    #endif
  }
  #endif
  delay(500);
  wdt_reset();
  board_mode = BOARDMODE_INITIALIZING;
  init_shields();
  
  wdt_reset();
  
  #if(BOARD_TYPE == BOARDTYPE_ARDUINOMEGA)
  {
    Serial1.println("Board Executing");
  }
  #elif(BOARD_TYPE == BOARDTYPE_ARDUINOUNO)
  {
    softSerial.println("Board Executing");
  }
 #endif
}

void loop() 
{ 
  wdt_reset();

  //SERVOSHIELD_setServoPulse(0, 2000);
  delay(1); //Delay 1 millisecond
  loop_counter++;
  veryfastrate_counter++;
  fastrate_counter++;
  mediumrate_counter++;
  slowrate_counter++;
  veryslowrate_counter++;
  if(veryfastrate_counter >= 1)
  {
    veryfastrate_counter = 0;
    run_veryfastrate_code();
  }   
  if(fastrate_counter >= 10)
  {
    fastrate_counter = 0;
    run_fastrate_code();
  }
  if(mediumrate_counter >= 100)
  {
    mediumrate_counter = 0;
    run_mediumrate_code();
  }
  if(slowrate_counter >= 1000)
  {
    slowrate_counter = 0;
    run_slowrate_code();
  }
  if(veryslowrate_counter >= 10000)
  {
    veryslowrate_counter = 0;
    run_veryslowrate_code();
  }
  if(loop_counter > 2147483648) //Half of unsigned long (max)
  {
    loop_counter = 0;
    send_mode_counter = 0;
    recv_mode_counter = 0;
    send_configure_dioport_counter = 0;
    recv_configure_dioport_counter = 0;
    send_configure_anaport_counter = 0;
    recv_configure_anaport_counter = 0;
    send_set_dio_counter = 0;
    recv_set_dio_counter = 0;
    send_set_dio_defaultvalue_counter = 0;
    recv_set_dio_defaultvalue_counter = 0;
    send_command_counter = 0;
    recv_command_counter = 0;
    send_pps_counter = 0;
    recv_pps_counter = 0;
    send_ana_counter = 0;
    recv_ana_counter = 0;
    send_diagnostic_counter = 0;
    recv_diagnostic_counter = 0;
  }
}

void run_veryfastrate_code() //1000 Hz
{
  //Serial1.println(time_since_last_rx,DEC);
  /*if(time_since_last_rx > 1000)
  {
    Serial1.println("No data from Node in a long time.  Rebooting");
    delay(50);
    resetFunc();
  }
  */
  if(time_since_last_rx > 200)
  {
    armed_state = ARMEDSTATUS_DISARMED;
  }
  
  else if(board_mode != BOARDMODE_RUNNING)
  {
    armed_state = ARMEDSTATUS_DISARMED_CANNOTARM;
  }
  else if(armed_state == ARMEDSTATUS_DISARMED) //Board Mode: RUNNING
  {
    if(armed_command == ROVERCOMMAND_ARM)
    {
      armed_state = ARMEDSTATUS_ARMED;
    }
    else
    {
      armed_state = ARMEDSTATUS_DISARMED;
    }
  }
  else if(armed_state == ARMEDSTATUS_ARMED)
  {
    if(armed_command == ROVERCOMMAND_DISARM)
    {
      armed_state = ARMEDSTATUS_DISARMED;
    }
  }
  else
  {
    armed_state = ARMEDSTATUS_DISARMED;
  }
  
}
/*
 typedef struct
{
  int id;
  int Pin_Number[ANAPORT_SIZE];
  int Pin_Mode[ANAPORT_SIZE];
  int Pin_Value[ANAPORT_SIZE];
  int Pin_DefaultValue[ANAPORT_SIZE];
} ana_port;
typedef struct
{
  int id;
  int type;
  boolean isconfigured;
  boolean dioports_configured;
  boolean anaports_configured;
  int dio_portcount;
  int ana_portcount;
  dio_port dio_ports[MAXNUMBER_PORTS_PERSHIELD];
  ana_port ana_ports[MAXNUMBER_PORTS_PERSHIELD];
} shield;
 */
void run_fastrate_code() //100 Hz
{
  time_since_last_rx++;
  if((board_mode == BOARDMODE_RUNNING) and (node_mode == BOARDMODE_RUNNING))
  {
    for(int s = 0; s < MAXNUMBER_SHIELDS; s++)
    {
      //DIO Port 
      for(int p = 0; p < shields[s].dio_portcount; p++)
      {
        for(int j = 0; j < DIOPORT_SIZE; j++)
        {
          unsigned int pulse;
          switch(shields[s].dio_ports[p].Pin_Mode[j])
          {
            case PINMODE_DIGITAL_OUTPUT:
              if(armed_state == ARMEDSTATUS_ARMED)
              {
                if(shields[s].dio_ports[p].Pin_Value[j] == 1)
                {
                   digitalWrite(shields[s].dio_ports[p].Pin_Number[j],HIGH);
                }
                else
                {
                  digitalWrite(shields[s].dio_ports[p].Pin_Number[j],LOW);
                }
              }
              break;
            case PINMODE_DIGITAL_INPUT:
              shields[s].dio_ports[p].Pin_Value[j] = digitalRead(shields[s].dio_ports[p].Pin_Number[j]);
              break;
            case PINMODE_PWM_OUTPUT:
              if(armed_state == ARMEDSTATUS_ARMED)
              {
                pulse = 1000 + 3.90625*(double)(shields[s].dio_ports[p].Pin_Value[j]);
                if(pulse > 2000) { pulse = 2000;}
                else if(pulse < 1000) { pulse = 1000; }
                SERVOSHIELD_setServoPulse(shields[s].dio_ports[p].Pin_Number[j], pulse,s);
              }
              break;
            case PINMODE_DIGITAL_OUTPUT_NON_ACTUATOR:
              if(shields[s].dio_ports[p].Pin_Value[j] == 1)
              {
                 digitalWrite(shields[s].dio_ports[p].Pin_Number[j],HIGH);
              }
              else
              {
                digitalWrite(shields[s].dio_ports[p].Pin_Number[j],LOW);
              }
              break;
            case PINMODE_PWM_OUTPUT_NON_ACTUATOR:
              pulse = 1000 + 3.90625*(double)(shields[s].dio_ports[p].Pin_Value[j]);
              if(pulse > 2000) { pulse = 2000;}
              else if(pulse < 1000) { pulse = 1000; }
              SERVOSHIELD_setServoPulse(shields[s].dio_ports[p].Pin_Number[j], pulse,s);
              break;
            case PINMODE_QUADRATUREENCODER_INPUT:
              break;
            case PINMODE_ULTRASONIC_INPUT:
              break;
            default:
              break;
          }
        }
      }
      //ANA Port 
      for(int p = 0; p < shields[s].ana_portcount; p++)
      {
        for(int j = 0; j < ANAPORT_SIZE; j++)
        {
          switch(shields[s].ana_ports[p].Pin_Mode[j])
          {
            case PINMODE_ANALOG_INPUT:
              shields[s].ana_ports[p].Pin_Value[j] = analogRead(shields[s].ana_ports[p].Pin_Number[j]);
            case PINMODE_FORCESENSOR_INPUT:
              break;
            default:
              break;
          }
        }
      }
    }
  }
  if(message_complete == true)
  {
    int computed_checksum = 0;
    int message_checksum = recv_buffer[SERIAL_MESSAGE_SIZE-1];
    for(int i = 3; i < 15;i++)
    {
      computed_checksum ^= recv_buffer[i];
    }
    if(message_checksum == computed_checksum)
    {
      comm_established_once = 1;
      time_since_last_rx = 0;
      message_type = recv_buffer[1];
      passed_checksum_counter++;
      unsigned char packet[SERIAL_MESSAGE_SIZE-4];
      for(int i = 0; i < (SERIAL_MESSAGE_SIZE-4); i++)
      {
        packet[i] = recv_buffer[i+3]; 
      }
      if(message_type == SERIAL_Mode_ID)
      {
        unsigned char node_id,node_status,node_type;
        int status = serialmessagehandler.decode_ModeSerial(packet,&node_type,&node_id,&node_status);
        if(status == 1)
        {
          node_mode = node_status;
          recv_mode_counter++;
        }
      }
      else if(message_type == SERIAL_PPS_ID)
      {
        unsigned char counter;
        int status = serialmessagehandler.decode_PPSSerial(packet,&counter);
        if(status == 1)
        {
          new_pps = 1;
          pps_received = counter;
          recv_pps_counter++;
        }
      }
      else if(message_type == SERIAL_Command_ID)
      {
        unsigned char Command,Option1,Option2,Option3;
        int status = serialmessagehandler.decode_CommandSerial(packet,&Command,&Option1,&Option2,&Option3);
        if(status == 1)
        {
          if(Command == ROVERCOMMAND_CONFIGURE)  //Expecting new config for Shields
          {
            dioportconfig_messages_expected = -1;
            dioportdefault_messages_expected = -1;
            anaportconfig_messages_expected = -1;
            for(int j = 0; j < (MAXNUMBER_SHIELDS*MAXNUMBER_PORTS_PERSHIELD); j++)
            {
              dioportconfigure_messages_received[j] = false;
              dioportdefault_messages_received[j] = false;
              anaportconfigure_messages_received[j] = false;
            }
            board_mode = BOARDMODE_CONFIGURING;
         }
         else if(Command == ROVERCOMMAND_ARM) { armed_command = Command; }
         else if(Command == ROVERCOMMAND_DISARM) { armed_command = Command; }
         else
         {
            diagnostic_status.Diagnostic_Type = COMMUNICATIONS;
            diagnostic_status.Level = WARN;
            diagnostic_status.Diagnostic_Message = DROPPING_PACKETS;
         }
         recv_command_counter++;
        }
      }
      else if(message_type == SERIAL_Set_DIO_Port_ID)
      {
        unsigned char ShieldID,PortID;
        unsigned char v1,v2,v3,v4,v5,v6,v7,v8;
        int status = serialmessagehandler.decode_Set_DIO_PortSerial(packet,&ShieldID,&PortID,&v1,&v2,&v3,&v4,&v5,&v6,&v7,&v8);
        if(status == 1)
        {
          recv_set_dio_counter++;
          int pinvalues[DIOPORT_SIZE] = {v1,v2,v3,v4,v5,v6,v7,v8};
          bool found_port = false;
          for(int s = 0; s < MAXNUMBER_SHIELDS; s++)
          {
            if(shields[s].id == ShieldID)
            {
              for(int p = 0; p < shields[s].dio_portcount; p++)
              {
                if(shields[s].dio_ports[p].id == PortID)
                {
                  found_port = true;
                  
                  for(int j = 0; j < DIOPORT_SIZE; j++)
                  {
                    
                    shields[s].dio_ports[p].Pin_Value[j] = pinvalues[j];
                  }
  
                }
              }
            }
          }
          if(found_port == false)
          {
            diagnostic_status.Diagnostic_Type = COMMUNICATIONS;
            diagnostic_status.Level = ERROR;
            diagnostic_status.Diagnostic_Message = DROPPING_PACKETS;
          }
        }
        
      }
      else if(message_type == SERIAL_Configure_DIO_Port_ID)
      {
        unsigned char ShieldID,PortID,messageindex,messagecount;
        unsigned char v1,v2,v3,v4,v5,v6,v7,v8;
        int status = serialmessagehandler.decode_Configure_DIO_PortSerial(packet,&ShieldID,&PortID,&messageindex,&messagecount,
            &v1,&v2,&v3,&v4,&v5,&v6,&v7,&v8);
        if(status == 1)
        {
          recv_configure_dioport_counter++;
          dioportconfig_messages_expected = messagecount;
          int pinmodes[DIOPORT_SIZE] = {v1,v2,v3,v4,v5,v6,v7,v8};
          if(dioportconfigure_messages_received[messageindex] == false) //Dont reconfigure port with the same value
          {
            bool found_port = false;
            for(int s = 0; s < MAXNUMBER_SHIELDS; s++)
            {
              if(shields[s].id == ShieldID)
              {
                for(int p = 0; p < shields[s].dio_portcount; p++)
                {
                  if(shields[s].dio_ports[p].id == PortID)
                  {
 
                    found_port = true;
                    for(int j = 0; j < DIOPORT_SIZE; j++)
                    {
                      if(pinmodes[j] == PINMODE_DIGITAL_INPUT) { pinMode(shields[s].dio_ports[p].Pin_Number[j],INPUT); }
                      else if(pinmodes[j] == PINMODE_ARMCOMMAND_INPUT) { pinMode(shields[s].dio_ports[p].Pin_Number[j],INPUT); }
                      else if(pinmodes[j] == PINMODE_DIGITAL_OUTPUT) { pinMode(shields[s].dio_ports[p].Pin_Number[j],OUTPUT); }
                      else if(pinmodes[j] == PINMODE_DIGITAL_OUTPUT_NON_ACTUATOR) { pinMode(shields[s].dio_ports[p].Pin_Number[j],OUTPUT); }
                      shields[s].dio_ports[p].Pin_Mode[j] = pinmodes[j];
                    }
                  }
                }
              }
            }
            if(found_port == true) 
            { 
              dioportconfigure_messages_received[messageindex] = true; 
            }
            else
            {
              diagnostic_status.Diagnostic_Type = COMMUNICATIONS;
              diagnostic_status.Level = ERROR;
              diagnostic_status.Diagnostic_Message = DROPPING_PACKETS;
            }
          }
        }
      }
      else if(message_type == SERIAL_Set_DIO_Port_DefaultValue_ID)
      {
        unsigned char ShieldID,PortID,messageindex,messagecount;
        unsigned char v1,v2,v3,v4,v5,v6,v7,v8;
        int status = serialmessagehandler.decode_Set_DIO_Port_DefaultValueSerial(packet,&ShieldID,&PortID,&messageindex,&messagecount,
            &v1,&v2,&v3,&v4,&v5,&v6,&v7,&v8);
        if(status == 1)
        {
          recv_set_dio_defaultvalue_counter++;
          dioportdefault_messages_expected = messagecount;
          int pinvalues[DIOPORT_SIZE] = {v1,v2,v3,v4,v5,v6,v7,v8};
          if(dioportdefault_messages_received[messageindex-1] == false) //Dont set port default with the same value
          {
            bool found_port = false;
            for(int s = 0; s < MAXNUMBER_SHIELDS; s++)
            {
              if(shields[s].id == ShieldID)
              {
                for(int p = 0; p < shields[s].dio_portcount; p++)
                {
                  if(shields[s].dio_ports[p].id == PortID)
                  {
                    found_port = true;
                    for(int j = 0; j < DIOPORT_SIZE; j++)
                    {
                      shields[s].dio_ports[p].Pin_DefaultValue[j] = pinvalues[j];
                    }
                  }
                }
              }
            }
            if(found_port == true) { dioportdefault_messages_received[messageindex-1] = true; }
            else
            {
              diagnostic_status.Diagnostic_Type = COMMUNICATIONS;
              diagnostic_status.Level = ERROR;
              diagnostic_status.Diagnostic_Message = DROPPING_PACKETS;
            }
          }
        }
      }
      else if(message_type == SERIAL_Configure_ANA_Port_ID)
      {
        unsigned char ShieldID,PortID,messageindex,messagecount;
        unsigned char v1,v2,v3,v4;
        int status = serialmessagehandler.decode_Configure_ANA_PortSerial(packet,&ShieldID,&PortID,&messageindex,&messagecount,
            &v1,&v2,&v3,&v4);
        if(status == 1)
        {
          recv_configure_anaport_counter++;
          anaportconfig_messages_expected = messagecount;
          int pinmodes[ANAPORT_SIZE] = {v1,v2,v3,v4};
          if(anaportconfigure_messages_received[messageindex] == false) //Dont reconfigure port with the same value
          {
            bool found_port = false;
            for(int s = 0; s < MAXNUMBER_SHIELDS; s++)
            {
              if(shields[s].id == ShieldID)
              {
                for(int p = 0; p < shields[s].ana_portcount; p++)
                {
                  if(shields[s].ana_ports[p].id == PortID)
                  {
                    found_port = true;
                    for(int j = 0; j < ANAPORT_SIZE; j++)
                    {
                      shields[s].ana_ports[p].Pin_Mode[j] = pinmodes[j];
                    }
                  }
                }
              }
            }
            if(found_port == true) { anaportconfigure_messages_received[messageindex] = true; }
          }
        }
      }
    }
    else
    {
      failed_checksum_counter++;
    }
    memset(recv_buffer,0,sizeof(recv_buffer));
  }
  message_complete = false;
  
}
void run_mediumrate_code() //10 Hz
{
  #if ((SHIELD1_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD2_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_LCDSHIELD))
  {
    int key = read_lcd_buttons();
    switch (key)
    {
      case BUTTON_RIGHT:
        current_menupage = MENUPAGE_LEVELCOUNT;
        break;
      case BUTTON_LEFT:
        current_menupage = MENUPAGE_DEFAULT;
        break;
      case BUTTON_UP:
        if(current_menupage == MENUPAGE_LEVELCOUNT)
        {
          if(level_selected > DEBUG) { level_selected--; }
        }
        break;
      case BUTTON_DOWN:
        if(current_menupage == MENUPAGE_LEVELCOUNT)
        {
          if(level_selected < FATAL) { level_selected++;}
        }
        
    }
  }
  #endif
  //Serial1.println(board_mode,DEC);
  {
    char buffer[16];
    int length;
    int tx_status = serialmessagehandler.encode_ModeSerial(buffer,&length,BOARD_TYPE,BOARD_ID,board_mode);
    for(int i = 0; i < length; i++)
    {
      Serial.write((byte)buffer[i]);
    }
    send_mode_counter++;
  }
  {
    char buffer[16];
    int length;
    int tx_status = serialmessagehandler.encode_DiagnosticSerial(buffer,&length,
      diagnostic_status.System,
      diagnostic_status.SubSystem,
      diagnostic_status.Component,
      diagnostic_status.Diagnostic_Type,
      diagnostic_status.Level,
      diagnostic_status.Diagnostic_Message);
      for(int i = 0; i < length; i++)
      {
        Serial.write((byte)buffer[i]);
      }
    send_diagnostic_counter++;
  }
  {
    if(board_mode == BOARDMODE_RUNNING)
    {
      for(int s = 0; s < MAXNUMBER_SHIELDS; s++)
      {
        for(int p = 0; p < shields[s].ana_portcount; p++)
        {
          char buffer[16];
          int length;
          int computed_checksum;
          
          for(int j = 0; j < ANAPORT_SIZE; j++)
          {
            switch(shields[s].ana_ports[p].Pin_Mode[j])
            {
              case PINMODE_ANALOG_INPUT:
              {
                int tx_status = serialmessagehandler.encode_Get_ANA_PortSerial(buffer,&length,shields[s].id,shields[s].ana_ports[p].id,
                  shields[s].ana_ports[p].Pin_Value[0],
                  shields[s].ana_ports[p].Pin_Value[1],
                  shields[s].ana_ports[p].Pin_Value[2],
                  shields[s].ana_ports[p].Pin_Value[3]);
                for(int i = 0; i < length; i++)
                {
                  Serial.write((byte)buffer[i]);
                }
                send_ana_counter++;
                break;
              }
              default:
                break;
            }
          }
        }
      }
    }
  }
  if(new_pps == 1)
  {
    new_pps = 0;
    char buffer[16];
    int length;
    int computed_checksum;
    int tx_status = serialmessagehandler.encode_PPSSerial(buffer,&length,pps_received);
    for(int i = 0; i < length; i++)
    {
      Serial.write((byte)buffer[i]);
    }
    send_pps_counter++;
    }
  /*
  #if(( BOARD_TYPE == BOARDTYPE_ARDUINOMEGA) && (SHIELD1_TYPE == SHIELDTYPE_TERMINALSHIELD))
  {

    if(board_mode == BOARDMODE_RUNNING)
    {
      
    }
      
  }
  #endif
  
  {
    
  }
  //DEBUG ONLY
  /*
  delay(10);
  {
    char buffer[16];
    int length;
    int computed_checksum;
    int tx_status = serialmessagehandler.encode_UserMessageSerial(buffer,&length,failed_checksum_counter,recv_configure_shield_counter,0,0,0,0,0,0,0,0,0,0);
    for(int i = 0; i < length; i++)
    {
      Serial.write((byte)buffer[i]);
    }
  }
  */
}
void run_slowrate_code() //1 Hz
{
  #if(PRINT_DEBUG_LINES == 1)
  {
    #if(BOARD_TYPE == BOARDTYPE_ARDUINOMEGA)
    {
      Serial1.print("Board Mode: ");
      Serial1.println(map_mode_tostring(board_mode));
    }
     #elif(BOARD_TYPE == BOARDTYPE_ARDUINOUNO)
     {
      softSerial.print("Board Mode: ");
      softSerial.println(map_mode_tostring(board_mode));
     }
     #endif
  }
  #endif
  #if ((SHIELD1_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD2_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_LCDSHIELD))
  {
      update_lcd();
  }
  #endif
  if((node_mode == BOARDMODE_BOOTING) and (board_mode == BOARDMODE_BOOTING)) //Do Nothing
  {
    
  }
  else if((node_mode == BOARDMODE_INITIALIZING) and (board_mode == BOARDMODE_BOOTING)) //Do Nothing
  {
    
  }
  else if((node_mode == BOARDMODE_BOOTING) and (board_mode == BOARDMODE_INITIALIZING)) //Do Nothing
  {
    
  }
  else if((node_mode == BOARDMODE_INITIALIZING) and (board_mode == BOARDMODE_INITIALIZING)) //Do Nothing
  {
    
  }
  else if(((node_mode == BOARDMODE_CONFIGURING) and (board_mode == BOARDMODE_CONFIGURING)) or 
          ((node_mode == BOARDMODE_CONFIGURED) and (board_mode == BOARDMODE_CONFIGURING))) //Received Config Command, Waiting to Config Parameters
  {
    boolean dioports_ready = true;
    if(dioportconfig_messages_expected < 0) { dioports_ready = false; }
    else if(dioportconfig_messages_expected == 0) { dioports_ready = true; }
    else
    {
      for(int i = 0; i < dioportconfig_messages_expected; i++)
      {
        if(dioportconfigure_messages_received[i] == true)
        {
          dioports_ready = dioports_ready and true;     
        }
        else
        {
          dioports_ready = false;
        }
      }
    }

    boolean anaports_ready = true;
    if(anaportconfig_messages_expected < 0) { anaports_ready = false; }
    else if(anaportconfig_messages_expected == 0) { anaports_ready = true; }
    else
    {
      for(int i = 0; i < anaportconfig_messages_expected; i++)
      {
        if(anaportconfigure_messages_received[i] == true)
        {
          anaports_ready = anaports_ready and true;     
        }
        else
        {
          anaports_ready = false;
        }
      }
    }

    if((dioports_ready == true) and (anaports_ready == true))
    {
      board_mode = BOARDMODE_CONFIGURED;
    }
    
  }
  else if(((node_mode == BOARDMODE_CONFIGURED) and (board_mode == BOARDMODE_CONFIGURED)) or 
          ((node_mode == BOARDMODE_RUNNING) and (board_mode == BOARDMODE_CONFIGURED)))
  {
      board_mode = BOARDMODE_RUNNING;
  }
  else if((node_mode == BOARDMODE_RUNNING) and (board_mode == BOARDMODE_RUNNING))
  {
    diagnostic_status.Diagnostic_Type = SOFTWARE;
    diagnostic_status.Level = INFO;
    diagnostic_status.Diagnostic_Message = NOERROR;
  }
  else
  {
    diagnostic_status.Diagnostic_Type = SOFTWARE;
    diagnostic_status.Level = WARN;
    diagnostic_status.Diagnostic_Message = UNKNOWN_STATE;
  }
  /*
  if(board_mode == BOARDMODE_INITIALIZING) //Waiting on Configuration
  {
  }
  if(board_mode == BOARDMODE_CONFIGURING)  //Receiving Config
  {
  
    boolean dio_portready = true;
    boolean ana_portready = true;
    for(int i = 0; i < dio_portconfig_messages; i++)
    {
        if(dio_portmessagesreceived[i] == true) { dio_portready & true;}
        else { dio_portready = false;}
    }
    for(int i = 0; i < ana_portconfig_messages; i++)
    {
        if(ana_portmessagesreceived[i] == true) { ana_portready & true;}
        else { ana_portready = false;}
    }
    if((dio_portready == true) && (ana_portready == true))
    {
        board_mode = BOARDMODE_CONFIGURED;
    }
  }
  else if(board_mode == BOARDMODE_CONFIGURED)  //Config complete
  {
    armed_state = ARMEDSTATUS_DISARMED;
    board_mode = BOARDMODE_RUNNING;
  }
  if(board_mode == BOARDMODE_INITIALIZING)
  {
    board_mode = BOARDMODE_RUNNING;
    armed_state = ARMEDSTATUS_DISARMED;
  }
  if((board_mode == BOARDMODE_RUNNING) && (node_mode == BOARDMODE_INITIALIZING))
  {
    #if BOARD_TYPE == BOARDTYPE_ARDUINOMEGA
    {
      Serial1.println("Arduino Board Rebooting.");
    }
    #endif
    #if ((SHIELD1_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD2_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_LCDSHIELD))
    {
      lcd.print("Rebooting");
    }
    #endif
    
    delay(500);
    resetFunc();
  }
  if(time_since_last_rx > 200)
  {
    #if ((SHIELD1_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD2_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_LCDSHIELD))
    {
      
      if(current_menupage == MENUPAGE_DEFAULT)
      {
        lcd.clear();
        lcd.setCursor(0,0);
        if(comm_established_once == 0)
        {
          lcd.print("ERROR: No Comm");
        }
        else
        {
          lcd.print("ERROR: Comm Lost");
        }
      }
      
    }
    #endif
  }
  
  */
  digitalWrite(LED,!digitalRead(LED));
  
}
void run_veryslowrate_code() //0.1 Hz
{
  
  #if BOARD_TYPE == BOARDTYPE_ARDUINOMEGA
  {
      Serial1.print("Passed Checksum: ");
      Serial1.print(passed_checksum_counter,DEC);
      Serial1.print(" Failed Checksum: ");
      Serial1.println(failed_checksum_counter,DEC);
    #if(PRINT_DEBUG_LINES == 1)
    {
      Serial1.print("Passed Checksum: ");
      Serial1.print(passed_checksum_counter,DEC);
      Serial1.print(" Failed Checksum: ");
      Serial1.println(failed_checksum_counter,DEC);
      
      Serial1.print("Send Command (0xA02) times: ");
      Serial1.print(send_command_counter,DEC);
      Serial1.print(" at: ");
      Serial1.print(1000.0*(double)send_command_counter/(double)loop_counter);
      Serial1.print(" (Hz) Received times: ");
      Serial1.print(recv_command_counter,DEC);
      Serial1.print(" at: ");
      Serial1.print(1000.0*(double)recv_command_counter/(double)loop_counter);
      Serial1.println(" (Hz)");
      
      Serial1.print("Sent Mode (0xAB17) times: ");
      Serial1.print(send_mode_counter,DEC);
      Serial1.print(" at: ");
      Serial1.print(1000.0*(double)send_mode_counter/(double)loop_counter);
      Serial1.print(" (Hz) Received times: ");
      Serial1.print(recv_mode_counter,DEC);
      Serial1.print(" at: ");
      Serial1.print(1000.0*(double)recv_mode_counter/(double)loop_counter);
      Serial1.println(" (Hz)");
      
      Serial1.print("Send Configure DIOPort (0xAB16) times: ");
      Serial1.print(send_configure_dioport_counter,DEC);
      Serial1.print(" at: ");
      Serial1.print(1000.0*(double)send_configure_dioport_counter/(double)loop_counter);
      Serial1.print(" (Hz) Received times: ");
      Serial1.print(recv_configure_dioport_counter,DEC);
      Serial1.print(" at: ");
      Serial1.print(1000.0*(double)recv_configure_dioport_counter/(double)loop_counter);
      Serial1.println(" (Hz)");

      Serial1.print("Send Configure ANAPort (0xAB36) times: ");
      Serial1.print(send_configure_anaport_counter,DEC);
      Serial1.print(" at: ");
      Serial1.print(1000.0*(double)send_configure_anaport_counter/(double)loop_counter);
      Serial1.print(" (Hz) Received times: ");
      Serial1.print(recv_configure_anaport_counter,DEC);
      Serial1.print(" at: ");
      Serial1.print(1000.0*(double)recv_configure_anaport_counter/(double)loop_counter);
      Serial1.println(" (Hz)");
      
      Serial1.print("Send Set DIO (0xAB18) times: ");
      Serial1.print(send_set_dio_counter,DEC);
      Serial1.print(" at: ");
      Serial1.print(1000.0*(double)send_set_dio_counter/(double)loop_counter);
      Serial1.print(" (Hz) Received times: ");
      Serial1.print(recv_set_dio_counter,DEC);
      Serial1.print(" at: ");
      Serial1.print(1000.0*(double)recv_set_dio_counter/(double)loop_counter);
      Serial1.println(" (Hz)");
      
      Serial1.print("Send ANA (0xAB20) times: ");
      Serial1.print(send_ana_counter,DEC);
      Serial1.print(" at: ");
      Serial1.print(1000.0*(double)send_ana_counter/(double)loop_counter);
      Serial1.print(" (Hz) Received times: ");
      Serial1.print(recv_ana_counter,DEC);
      Serial1.print(" at: ");
      Serial1.print(1000.0*(double)recv_ana_counter/(double)loop_counter);
      Serial1.println(" (Hz)");
      
      Serial1.print("Send Set DIO DefaultValue (0xAB32) times: ");
      Serial1.print(send_set_dio_defaultvalue_counter,DEC);
      Serial1.print(" at: ");
      Serial1.print(1000.0*(double)send_set_dio_defaultvalue_counter/(double)loop_counter);
      Serial1.print(" (Hz) Received times: ");
      Serial1.print(recv_set_dio_defaultvalue_counter,DEC);
      Serial1.print(" at: ");
      Serial1.print(1000.0*(double)recv_set_dio_defaultvalue_counter/(double)loop_counter);
      Serial1.println(" (Hz)");
      
      Serial1.print("Send PPS (0xAB35) times: ");
      Serial1.print(send_pps_counter,DEC);
      Serial1.print(" at: ");
      Serial1.print(1000.0*(double)send_pps_counter/(double)loop_counter);
      Serial1.print(" (Hz) Received times: ");
      Serial1.print(recv_pps_counter,DEC);
      Serial1.print(" at: ");
      Serial1.print(1000.0*(double)recv_pps_counter/(double)loop_counter);
      Serial1.println(" (Hz)");
    }
    #endif
  }
  #elif(BOARD_TYPE == BOARDTYPE_ARDUINOUNO)
  {
    #if(PRINT_DEBUG_LINES == 1)
    {
      softSerial.print("Passed Checksum: ");
      softSerial.print(passed_checksum_counter,DEC);
      softSerial.print(" Failed Checksum: ");
      softSerial.println(failed_checksum_counter,DEC);
      
      softSerial.print("Send Command (0xA02) times: ");
      softSerial.print(send_command_counter,DEC);
      softSerial.print(" at: ");
      softSerial.print(1000.0*(double)send_command_counter/(double)loop_counter);
      softSerial.print(" (Hz) Received times: ");
      softSerial.print(recv_command_counter,DEC);
      softSerial.print(" at: ");
      softSerial.print(1000.0*(double)recv_command_counter/(double)loop_counter);
      softSerial.println(" (Hz)");
      
      softSerial.print("Sent Mode (0xAB17) times: ");
      softSerial.print(send_mode_counter,DEC);
      softSerial.print(" at: ");
      softSerial.print(1000.0*(double)send_mode_counter/(double)loop_counter);
      softSerial.print(" (Hz) Received times: ");
      softSerial.print(recv_mode_counter,DEC);
      softSerial.print(" at: ");
      softSerial.print(1000.0*(double)recv_mode_counter/(double)loop_counter);
      softSerial.println(" (Hz)");
      
      softSerial.print("Send Configure DIOPort (0xAB16) times: ");
      softSerial.print(send_configure_dioport_counter,DEC);
      softSerial.print(" at: ");
      softSerial.print(1000.0*(double)send_configure_dioport_counter/(double)loop_counter);
      softSerial.print(" (Hz) Received times: ");
      softSerial.print(recv_configure_dioport_counter,DEC);
      softSerial.print(" at: ");
      softSerial.print(1000.0*(double)recv_configure_dioport_counter/(double)loop_counter);
      softSerial.println(" (Hz)");

      softSerial.print("Send Configure ANAPort (0xAB36) times: ");
      softSerial.print(send_configure_anaport_counter,DEC);
      softSerial.print(" at: ");
      softSerial.print(1000.0*(double)send_configure_anaport_counter/(double)loop_counter);
      softSerial.print(" (Hz) Received times: ");
      softSerial.print(recv_configure_anaport_counter,DEC);
      softSerial.print(" at: ");
      softSerial.print(1000.0*(double)recv_configure_anaport_counter/(double)loop_counter);
      softSerial.println(" (Hz)");
      
      softSerial.print("Send Set DIO (0xAB18) times: ");
      softSerial.print(send_set_dio_counter,DEC);
      softSerial.print(" at: ");
      softSerial.print(1000.0*(double)send_set_dio_counter/(double)loop_counter);
      softSerial.print(" (Hz) Received times: ");
      softSerial.print(recv_set_dio_counter,DEC);
      softSerial.print(" at: ");
      softSerial.print(1000.0*(double)recv_set_dio_counter/(double)loop_counter);
      softSerial.println(" (Hz)");
      
      softSerial.print("Send ANA (0xAB20) times: ");
      softSerial.print(send_ana_counter,DEC);
      softSerial.print(" at: ");
      softSerial.print(1000.0*(double)send_ana_counter/(double)loop_counter);
      softSerial.print(" (Hz) Received times: ");
      softSerial.print(recv_ana_counter,DEC);
      softSerial.print(" at: ");
      softSerial.print(1000.0*(double)recv_ana_counter/(double)loop_counter);
      softSerial.println(" (Hz)");
      
      softSerial.print("Send Set DIO DefaultValue (0xAB32) times: ");
      softSerial.print(send_set_dio_defaultvalue_counter,DEC);
      softSerial.print(" at: ");
      softSerial.print(1000.0*(double)send_set_dio_defaultvalue_counter/(double)loop_counter);
      softSerial.print(" (Hz) Received times: ");
      softSerial.print(recv_set_dio_defaultvalue_counter,DEC);
      softSerial.print(" at: ");
      softSerial.print(1000.0*(double)recv_set_dio_defaultvalue_counter/(double)loop_counter);
      softSerial.println(" (Hz)");
      
      softSerial.print("Send PPS (0xAB35) times: ");
      softSerial.print(send_pps_counter,DEC);
      softSerial.print(" at: ");
      softSerial.print(1000.0*(double)send_pps_counter/(double)loop_counter);
      softSerial.print(" (Hz) Received times: ");
      softSerial.print(recv_pps_counter,DEC);
      softSerial.print(" at: ");
      softSerial.print(1000.0*(double)recv_pps_counter/(double)loop_counter);
      softSerial.println(" (Hz)");
    }
    #endif
  }
  #endif 
}
void serialEvent() 
{
  if(message_complete == false)
  {
    char c;
    int bytestoread = SERIAL_MESSAGE_SIZE;
    int bytecounter = 0;
    while((Serial.available()) && (bytecounter < bytestoread))
    {
        c = Serial.read();  
        delay(1);   
        recv_buffer[bytecounter++] = c;
      
    }
    /*
    if((BOARD_TYPE == BOARDTYPE_ARDUINOMEGA) && (PRINT_RECV_BUFFER == 1))
    {
      for(int i = 0; i < SERIAL_MESSAGE_SIZE; i++)
      {
        Serial1.print(" ");
        Serial1.print(recv_buffer[i],HEX);
      }
      Serial1.println("");
    }
    */
    message_complete = true;

  }
}
//#if ((SHIELD1_TYPE == SHIELDTYPE_SERVOSHIELD) || (SHIELD2_TYPE == SHIELDTYPE_SERVOSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_SERVOSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_SERVOSHIELD))
void SERVOSHIELD_setServoPulse(uint8_t n, uint16_t pulse_us,uint8_t shield)
{
  double pulse = (double)(pulse_us)/1000.0;
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVOSHIELD_UPDATE_RATE;   // 60 Hz
  pulselength /= 4096;  // 12 bits of resolution
  pulse *= 1000;
  pulse /= pulselength;
  Serial1.print("s: ");
  Serial1.print(shield);
  Serial1.print("p: ");
  Serial1.println(pulse);
  NOT sending PULSE TO PIN
  #if(SHIELD1_TYPE == SHIELDTYPE_SERVOSHIELD) 
  {
    if(shield == 0)
    {
      pwm1.setPWM(n, 0, pulse);
    }
  }
  #endif
  #if(SHIELD2_TYPE == SHIELDTYPE_SERVOSHIELD) 
  {
    if(shield == 1)
    {
      pwm2.setPWM(n, 0, pulse);
    }
  }
  #endif
  #if(SHIELD3_TYPE == SHIELDTYPE_SERVOSHIELD) 
  {
    if(shield == 2)
    {
      pwm3.setPWM(n, 0, pulse);
    }
  }
  #endif
  #if(SHIELD4_TYPE == SHIELDTYPE_SERVOSHIELD) 
  {
    if(shield == 3)
    {
      pwm4.setPWM(n, 0, pulse);
    }
  }
  #endif
}
//#endif

#if ((SHIELD1_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD2_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_LCDSHIELD))
void update_lcd()
{
  //softSerial.println(level_selected,DEC);
  if(current_menupage == MENUPAGE_LEVELCOUNT)
    {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(map_level_tostring(level_selected));
    lcd.print(": ");
    switch(level_selected)
    {
      case DEBUG:
        lcd.print(level_debug_counter);
        break;
      case INFO:
        lcd.print(level_info_counter);
        break;
      case NOTICE:
        lcd.print(level_notice_counter);
        break;
      case WARN:
        lcd.print(level_warn_counter);
        break;
      case ERROR:
        lcd.print(level_error_counter);
        break;
      case FATAL:
        lcd.print(level_fatal_counter);
        break;
      default:
        lcd.print("?");
        lcd.print(level_selected);
        break;
    }
  }
}
void print_to_lcd(unsigned char System,unsigned char Subsystem,unsigned char Component,unsigned char DiagnosticType,unsigned char Level,unsigned char Message)
{
  if(current_menupage == MENUPAGE_DEFAULT)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    switch(Level)
    {
      case NOTICE:
        lcd.print(map_level_tostring(Level));
        lcd.print(":");
        if((System == ROVER) && (Subsystem == ENTIRE_SYSTEM) && (DiagnosticType == NOERROR) && (Message == NOERROR))
        {
          lcd.setCursor(0,1);
          lcd.print("Rover Is FMC.");
        }
        else
        {
          lcd.print(map_component_tostring(Component));
          lcd.setCursor(0,1);
          lcd.print(map_diagnostictype_tostring(DiagnosticType));
          lcd.print(":");
          lcd.print(map_message_tostring(Message));
        }
        break;
      default:
        lcd.print(map_level_tostring(Level));
        lcd.print(": ");
        lcd.print(map_component_tostring(Component));
        lcd.setCursor(0,1);
        lcd.print(map_diagnostictype_tostring(DiagnosticType));
        lcd.print(":");
        lcd.print(map_message_tostring(Message));
        return;
    }
  }
}
int read_lcd_buttons()
{
  int adc_key_in = analogRead(0);      // read the value from the sensor
   // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
   // we add approx 50 to those values and check to see if we are close
   if (adc_key_in > 1000) return BUTTON_NONE; // We make this the 1st option for speed reasons since it will be the most likely result
   if (adc_key_in < 50)   return BUTTON_RIGHT; 
   if (adc_key_in < 195)  return BUTTON_UP;
   if (adc_key_in < 380)  return BUTTON_DOWN;
   if (adc_key_in < 555)  return BUTTON_LEFT;
   if (adc_key_in < 790)  return BUTTON_SELECT;  
   return BUTTON_NONE;  // when all others fail, return this...
}
#endif
String map_component_tostring(int v)
{
  switch(v)
  {
    case CONTROLLER_NODE:
      return "Control";
      break;
    case DIAGNOSTIC_NODE:
      return "Diag";
      break;
    case NAVIGATION_NODE:
      return "Nav";
      break;
    case EVOLUTION_NODE:
      return "Evol";
      break;
    case TARGETING_NODE:
      return "Target";
      break;
    case TIMING_NODE:
      return "Timing";
      break;
    case VISION_NODE:
      return "Vision";
      break;
    case COMMUNICATION_NODE:
      return "Comm";
      break;
    case DYNAMICS_NODE:
      return "Dyn";
      break;
    case POWER_NODE:
      return "Power";
      break;
    case POSE_NODE:
      return "Pose";
      break;
    default:
      return "UNK";
      break;
  }
}
String map_level_tostring(int v)
{
  switch(v)
  {
    case DEBUG:
      return "DEBUG";
      break;
    case INFO:
      return "INFO";
      break;
    case NOTICE:
      return "NOTICE";
      break;
    case WARN:
      return "WARN";
      break;
    case ERROR:
      return "ERROR";
      break;
    case FATAL:
      return "FATAL";
      break;
    default:
      return "UNKNOWN";
  }
}
String map_diagnostictype_tostring(int v)
{
  switch(v)
  {
    case NOERROR:
      return "NOERROR";
      break;
    case ELECTRICAL:
      return "ELEC";
      break;
    case SOFTWARE:
      return "SFTWR";
      break;
    case COMMUNICATIONS:
      return "COMM";
      break;
    case SENSORS:
      return "SNSRS";
      break;
    case ACTUATORS:
      return "ACTRS";
      break;
    case DATA_STORAGE:
      return "STRGE";
      break;
    case REMOTE_CONTROL:
      return "RC";
      break;
    case TARGET_ACQUISITION:
      return "TGTACQ";
      break;
    case POWER:
      return "PWR";
      break;
    case POSE:
      return "POSE";
      break;
    case GENERAL_ERROR:
      return "GEN";
      break;
    default:
      return "UNK";
      break;
  }
}
String map_mode_tostring(int v)
{
  switch(v)
  {
    case BOARDMODE_UNDEFINED:
      return "UNDEFINED";
      break;
    case BOARDMODE_BOOTING:
      return "BOOTING";
      break;
    case BOARDMODE_INITIALIZING:
      return "INITIALIZING";
      break;
    case BOARDMODE_CONFIGURING:
      return "CONFIGURING";
      break;
    case BOARDMODE_CONFIGURED:
      return "CONFIGURED";
      break;
    case BOARDMODE_RUNNING:
      return "BOARDMODE_RUNNING";
      break;
    case BOARDMODE_STOPPED:
      return "STOPPED";
      break;
    default:
      return "UNKNOWN";
      break;
  }
}
String map_message_tostring(int v)
{
  switch(v)
  {
    case NOERROR:
      return "NOERROR";
      break;
    case INITIALIZING:
      return "Init";
      break;
    case INITIALIZING_ERROR:
      return "Init Error";
      break;
    case DROPPING_PACKETS:
      return "Drop Pkt";
      break;
    case MISSING_HEARTBEATS:
      return "Msng Hrtbt";
      break;
    case DEVICE_NOT_AVAILABLE:
      return "Device N/A";
      break;
    case ROVER_ARMED:
      return "Armed";
      break;
    case ROVER_DISARMED:
      return "Disarmed";
      break;
    case TEMPERATURE_HIGH:
      return "Temp High";
      break;
    case TEMPERATURE_LOW:
      return "Temp Low";
      break;
    case DIAGNOSTIC_PASSED:
      return "Diag Ok";
      break;
    case DIAGNOSTIC_FAILED:
      return "Diag Bad";
      break;
    case RESOURCE_LEAK:
      return "Res Leak";
      break;
    case HIGH_RESOURCE_USAGE:
      return "High Res Usg";
      break;
    default:
      return "Unknown";
      break;
  }
}

