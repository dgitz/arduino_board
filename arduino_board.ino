//Configuration Defines
#define BOARD_ID 19
#define BOARD_TYPE BOARDTYPE_ARDUINOUNO
#define PRINT_DEBUG_LINES 0
#define BYPASS_SHIELDCONFIG 1
#define SHIELD1_TYPE SHIELDTYPE_LCDSHIELD
#define SHIELD2_TYPE SHIELDTYPE_NONE
#define SHIELD3_TYPE SHIELDTYPE_NONE
#define SHIELD4_TYPE SHIELDTYPE_NONE



#define FIRMWARE_MAJOR_VERSION 0
#define FIRMWARE_MINOR_VERSION 3
#define FIRMWARE_BUILD_NUMBER 1

#include "Arduino.h"
#include <SoftwareSerial.h>
#include "serialmessage.h"
#include "Definitions.h"

#include <Wire.h>
#include <avr/wdt.h>


#define MAXNUMBER_SHIELDS 4
#if BOARD_TYPE == BOARDTYPE_ARDUINOUNO
#define MAXNUMBER_PORTS_PERSHIELD 2
#elif BOARD_TYPE == BOARDTYPE_ARDUINOMEGA
#define MAXNUMBER_PORTS_PERSHIELD 2
#endif
#define PORT_SIZE        8
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

  #define SERVOSHIELD_UPDATE_RATE 50
  #define SERVOSHIELD_SERVO_MIN 224
  #define SERVOSHIELD_SERVO_MAX 480
#endif

//Debugging Defines
#define PRINT_RECV_BUFFER 1

typedef struct
{
  int id;
  int Pin_Number[PORT_SIZE];
  int Pin_Mode[PORT_SIZE];
  int Pin_Value[PORT_SIZE];
  int Pin_DefaultValue[PORT_SIZE];
} port;
typedef struct
{
  int id;
  int type;
  boolean isconfigured;
  int portcount;
  port ports[MAXNUMBER_PORTS_PERSHIELD];
} shield;

shield shields[MAXNUMBER_SHIELDS];
void run_veryfastrate_code(); //1000 Hz
void run_fastrate_code(); //100 Hz
void run_mediumrate_code(); //10 Hz
void run_slowrate_code(); //1 Hz
void run_veryslowrate_code(); //0.1 Hz


#if ((SHIELD1_TYPE == SHIELDTYPE_SERVOSHIELD) || (SHIELD2_TYPE == SHIELDTYPE_SERVOSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_SERVOSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_SERVOSHIELD))
  Adafruit_PWMServoDriver pwm;
#endif

int board_mode = BOARDMODE_BOOT;
int node_mode = BOARDMODE_UNDEFINED;
unsigned int armed_command = ROVERCOMMAND_DISARM;
unsigned int armed_state = ARMEDSTATUS_DISARMED_CANNOTARM;
unsigned char recv_buffer[32];
int available_i2c_devices[MAXNUMBER_SHIELDS*2];
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
unsigned long send_configure_shield_counter = 0;
unsigned long recv_configure_shield_counter = 0;
unsigned long send_configure_dio_counter = 0;
unsigned long recv_configure_dio_counter = 0;
unsigned long send_set_dio_counter = 0;
unsigned long recv_set_dio_counter = 0;
unsigned long send_set_dio_defaultvalue_counter = 0;
unsigned long recv_set_dio_defaultvalue_counter = 0;
unsigned long send_armcommand_counter = 0;
unsigned long recv_armcommand_counter = 0;
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

//Function Prototypes for individual shields
//SERVOSHIELD
#if ((SHIELD1_TYPE == SHIELDTYPE_SERVOSHIELD) || (SHIELD2_TYPE == SHIELDTYPE_SERVOSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_SERVOSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_SERVOSHIELD))
  void SERVOSHIELD_setServoPulse(uint8_t pin_number, uint16_t pulse_us);
#endif

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

void scan_for_shields();
void init_shields();
String map_level_tostring(int v);
String map_component_tostring(int v);
String map_diagnostictype_tostring(int v);
String map_message_tostring(int v);

void(*resetFunc)(void) = 0;
void init_shields()
{
  for(int s = 0; s < (MAXNUMBER_SHIELDS*2);s++)
  {
    available_i2c_devices[s] = -1;
  }
  for(int s = 0; s < MAXNUMBER_SHIELDS;s++)
  {
    shields[s].id = -1; //0 is a valid Shield ID (which represents the Arduino Board)
    shields[s].type = SHIELDTYPE_UNDEFINED;
    shields[s].isconfigured = false;
    shields[s].portcount = 0;
    int pinindex = 0;
    for(int p=0; p < MAXNUMBER_PORTS_PERSHIELD;p++)
    {
      shields[s].ports[p].id = -1;
      for(int i = 0; i < PORT_SIZE; i++)
      {
        shields[s].ports[p].Pin_Number[i] = pinindex++;
        shields[s].ports[p].Pin_Value[i] = 0;
        shields[s].ports[p].Pin_DefaultValue[i] = 0;
        /*
        Serial1.print("port: ");
        Serial1.print(p,DEC);
        Serial1.print(" i: ");
        Serial1.print(i,DEC);
        Serial1.print(" pin: ");
        Serial1.print(pinindex,DEC);
        Serial1.print("/");
        Serial1.println(shields[s].ports[p].Pin_Number[i]);
        */
      }
    }
  }
}
void setup() {
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
  }
  #endif
  delay(500);
  wdt_reset();
  init_shields();
  
  wdt_reset();
  scan_for_shields();
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

void scan_for_shields()
{
  Wire.begin();
  byte error;
  int found_index = 0;
  for(int i = 1; i < 127; i++)
  {
    wdt_reset();
    Wire.beginTransmission(i);
    //Wire.write(1);
    error = Wire.endTransmission();
    if(error == 0)
    {
      available_i2c_devices[found_index++] = i;
    }
    else
    {
    }
    delay(10);
  }
  #if BOARD_TYPE == BOARDTYPE_ARDUINOMEGA
  {  
    for(int i = 0; i < found_index; i++)
    {
      Serial1.print("Found I2C Device at Address: ");
      Serial1.println(available_i2c_devices[i],HEX);
    }
  }
  #elif(BOARD_TYPE == BOARDTYPE_ARDUINOUNO)
  {
    for(int i = 0; i < found_index; i++)
    {
      softSerial.print("Found I2C Device at Address: ");
      softSerial.println(available_i2c_devices[i],HEX);
    }
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
    send_configure_shield_counter = 0;
    recv_configure_shield_counter = 0;
    send_configure_dio_counter = 0;
    recv_configure_dio_counter = 0;
    send_set_dio_counter = 0;
    recv_set_dio_counter = 0;
    send_set_dio_defaultvalue_counter = 0;
    recv_set_dio_defaultvalue_counter = 0;
    send_armcommand_counter = 0;
    recv_armcommand_counter = 0;

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
  
}
void run_fastrate_code() //100 Hz
{
  time_since_last_rx++;
  
  if(board_mode == BOARDMODE_RUNNING)
  {
    for(int s = 0; s < MAXNUMBER_SHIELDS;s++)
    {
      for(int p = 0; p < MAXNUMBER_PORTS_PERSHIELD;p++)
      {
        for(int j = 0; j < PORT_SIZE;j++)
        {
          #if ((SHIELD1_TYPE == SHIELDTYPE_SERVOSHIELD) || (SHIELD2_TYPE == SHIELDTYPE_SERVOSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_SERVOSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_SERVOSHIELD))
          if(shields[s].ports[p].Pin_Mode[j] == PINMODE_PWM_OUTPUT)
          {
            if(armed_state == ARMEDSTATUS_ARMED)
            {
              if(shields[s].type == SHIELDTYPE_SERVOSHIELD)
              {
                unsigned int pulse = 1000 + 3.90625*(double)(shields[s].ports[p].Pin_Value[j]);
                if(pulse > 2000) { pulse = 2000;}
                else if(pulse < 1000) { pulse = 1000; }
                SERVOSHIELD_setServoPulse(shields[s].ports[p].Pin_Number[j], pulse);
              }
            }
            else
            {
              if(shields[s].type == SHIELDTYPE_SERVOSHIELD)
              {
                unsigned int pulse = 1000 + 3.90625*(double)(shields[s].ports[p].Pin_DefaultValue[j]);
                if(pulse > 2000) { pulse = 2000;}
                else if(pulse < 1000) { pulse = 1000; }
                SERVOSHIELD_setServoPulse(shields[s].ports[p].Pin_Number[j], pulse);
              }
              
            }
            
          }
          #endif
          if(shields[s].ports[p].Pin_Mode[j] == PINMODE_DIGITAL_OUTPUT_NON_ACTUATOR)
          {
              if(shields[s].type == SHIELDTYPE_RELAYSHIELD)
              {
                if(shields[s].ports[p].Pin_Value[j] == 1)
                {
                   digitalWrite(shields[s].ports[p].Pin_Number[j],HIGH);
                }
                else
                {
                  digitalWrite(shields[s].ports[p].Pin_Number[j],LOW);
                }
              }
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
      if(message_type == SERIAL_Configure_Shield_ID)
      {
        recv_configure_shield_counter++;
        if(board_mode == BOARDMODE_INITIALIZING)
        {
          unsigned char ShieldCount,ShieldType,ShieldID,PortCount;
          int status = serialmessagehandler.decode_Configure_ShieldSerial(packet,&ShieldCount,&ShieldType,&ShieldID,&PortCount);
          if(shield_count == -1)
          {
            shield_count = ShieldCount;
          }
          bool add_new_shield = false;
          int shield_info_received_counter = 0;
          
          int i = 0;
          for(i = 0; i < MAXNUMBER_SHIELDS; i++)
          {
            if(shields[i].id == -1)
            {
              add_new_shield = true;
            }
            else
            {
              shield_info_received_counter++;
            }
            if(add_new_shield == true)
            {
              break;
            }
          }
          if((shield_count > 0) &&
             (shield_info_received_counter == shield_count))
          {
            board_mode = BOARDMODE_SHIELDS_CONFIGURED;
            add_new_shield = false;
            bool device_available = false;
            bool matched = false;
            #if ((SHIELD1_TYPE == SHIELDTYPE_SERVOSHIELD) || (SHIELD2_TYPE == SHIELDTYPE_SERVOSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_SERVOSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_SERVOSHIELD))
            if((ShieldID > 0) &&
               (ShieldType == SHIELDTYPE_SERVOSHIELD)) //Need to see if device is I2C that is available
            {
              for(int s = 0; s < (MAXNUMBER_SHIELDS*2);s++)
              {
                if(available_i2c_devices[s] == ShieldID)
                {
                  pwm = Adafruit_PWMServoDriver(ShieldID);
                  pwm.begin();
                  pwm.setPWMFreq(SERVOSHIELD_UPDATE_RATE);
                  
                  
                }
              }
            }
            #endif
            if(matched == true)
            {
              
            }
            
          }
          if(add_new_shield == true)
          {
            shields[i].id = ShieldID;
            shields[i].type = ShieldType;
            shields[i].portcount = PortCount;
          }
          
        }
      }
      else if (message_type == SERIAL_Diagnostic_ID)
      {
        //int decode_DiagnosticSerial(unsigned char* inpacket,unsigned char* System,unsigned char* SubSystem,unsigned char* Component,unsigned char* Diagnostic_Type,unsigned char* Level,unsigned char* Diagnostic_Message);
        unsigned char System, Subsystem,Component,DiagnosticType,Level,Message;
        int status = serialmessagehandler.decode_DiagnosticSerial(packet,&System,&Subsystem,&Component,&DiagnosticType,&Level,&Message);
        if(status == 1)
        {
          #if ((SHIELD1_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD2_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_LCDSHIELD))
          {
            if(Level == DEBUG) { level_debug_counter++;  }
            else if(Level == INFO)  { level_info_counter++;  }
            else if(Level == NOTICE) { level_notice_counter++;  }
            else if(Level == WARN)  { level_warn_counter++; }
            else if(Level == ERROR) { level_error_counter++; }
            else if(Level == FATAL) { level_fatal_counter++; }
            print_to_lcd(System,Subsystem,Component,DiagnosticType,Level,Message);
          }
          #endif
        }
      }
      else if(message_type == SERIAL_Set_DIO_Port_ID)
      {
        unsigned char ShieldID,PortID;
        unsigned char v1,v2,v3,v4,v5,v6,v7,v8;
        int status = serialmessagehandler.decode_Set_DIO_PortSerial(packet,&ShieldID,&PortID,&v1,&v2,&v3,&v4,&v5,&v6,&v7,&v8);
        for(int s = 0; s < MAXNUMBER_SHIELDS;s++)
        {
          if(shields[s].id == ShieldID)
          {
            for(int p = 0; p < MAXNUMBER_PORTS_PERSHIELD;p++)
            {
              
              if(shields[s].ports[p].id == PortID)
              {
                shields[s].ports[p].Pin_Value[0] = v1;
                shields[s].ports[p].Pin_Value[1] = v2;
                shields[s].ports[p].Pin_Value[2] = v3;
                shields[s].ports[p].Pin_Value[3] = v4;
                shields[s].ports[p].Pin_Value[4] = v5;
                shields[s].ports[p].Pin_Value[5] = v6;
                shields[s].ports[p].Pin_Value[6] = v7;
                shields[s].ports[p].Pin_Value[7] = v8;
              }
            }
          }
        }
        recv_set_dio_counter++;
      }
      else if(message_type == SERIAL_Mode_ID)
      {
        unsigned char node_id,node_status,node_type;
        int status = serialmessagehandler.decode_ModeSerial(packet,&node_type,&node_id,&node_status);
        node_mode = node_status;
        recv_mode_counter++;
      }
      else if(message_type == SERIAL_Set_DIO_Port_DefaultValue_ID)
      {
        unsigned char ShieldID,PortID;
        unsigned char v1,v2,v3,v4,v5,v6,v7,v8;
        int status = serialmessagehandler.decode_Set_DIO_Port_DefaultValueSerial(packet,&ShieldID,&PortID,&v1,&v2,&v3,&v4,&v5,&v6,&v7,&v8);
        for(int s = 0; s < MAXNUMBER_SHIELDS;s++)
        {
          if(shields[s].id == ShieldID)
          {
            for(int p = 0; p < MAXNUMBER_PORTS_PERSHIELD;p++)
            {
              if(shields[s].ports[p].id == PortID)
              {
                shields[s].ports[p].Pin_DefaultValue[0] = v1;
                shields[s].ports[p].Pin_Value[0] = v1;
                shields[s].ports[p].Pin_DefaultValue[1] = v2;
                shields[s].ports[p].Pin_Value[1] = v2;
                shields[s].ports[p].Pin_DefaultValue[2] = v3;
                shields[s].ports[p].Pin_Value[2] = v3;
                shields[s].ports[p].Pin_DefaultValue[3] = v4;
                shields[s].ports[p].Pin_Value[3] = v4;
                shields[s].ports[p].Pin_DefaultValue[4] = v5;
                shields[s].ports[p].Pin_Value[4] = v5;
                shields[s].ports[p].Pin_DefaultValue[5] = v6;
                shields[s].ports[p].Pin_Value[5] = v6;
                shields[s].ports[p].Pin_DefaultValue[6] = v7;
                shields[s].ports[p].Pin_Value[6] = v7;
                shields[s].ports[p].Pin_DefaultValue[7] = v8;
                shields[s].ports[p].Pin_Value[7] = v8;
              }
            }
          }
        }
        recv_set_dio_defaultvalue_counter++;
      }
      else if(message_type == SERIAL_Arm_Command_ID)
      {
        recv_armcommand_counter++;
        unsigned char Command;
        int status = serialmessagehandler.decode_Arm_CommandSerial(packet,&Command);
        armed_command = Command;
      }
      else if(message_type == SERIAL_Configure_DIO_Port_ID)
      {
        
        recv_configure_dio_counter++;
        unsigned char ShieldID,PortID;
        unsigned char v1,v2,v3,v4,v5,v6,v7,v8;
        
        int status = serialmessagehandler.decode_Configure_DIO_PortSerial(packet,&ShieldID,&PortID,&v1,&v2,&v3,&v4,&v5,&v6,&v7,&v8);
        
        for(int s = 0; s < MAXNUMBER_SHIELDS; s++)
        {
          if(shields[s].id == ShieldID)
          {
            bool add_new_port = false;
            int p;
            for(p = 0; p < PORT_SIZE; p++)
            {
              if(shields[s].ports[p].id == -1)
              {
                add_new_port = true;
              }
              if(add_new_port == true)
              {
                break;
              }
            }
            if(add_new_port == true)
            {             
              shields[s].ports[p].id = PortID;
              shields[s].ports[p].Pin_Mode[0] = v1;
              if(v1 == PINMODE_PWM_OUTPUT) 
              { 
                shields[s].ports[p].Pin_DefaultValue[0] = 127; 
              }
              else if(shields[s].id == 0)
              {
                if(v1 == PINMODE_DIGITAL_INPUT) { pinMode(shields[s].ports[p].Pin_Number[0],INPUT); }
                else { pinMode(shields[s].ports[p].Pin_Number[0],OUTPUT); }
              }
              shields[s].ports[p].Pin_Mode[1] = v2;
              if(v2 == PINMODE_PWM_OUTPUT) 
              { 
                shields[s].ports[p].Pin_DefaultValue[1] = 127; 
              }
              else if(shields[s].id == 0)
              {
                if(v2 == PINMODE_DIGITAL_INPUT) { pinMode(shields[s].ports[p].Pin_Number[1],INPUT); }
                else { pinMode(shields[s].ports[p].Pin_Number[1],OUTPUT); }
              }
              shields[s].ports[p].Pin_Mode[2] = v3;
              if(v3 == PINMODE_PWM_OUTPUT) 
              { 
                shields[s].ports[p].Pin_DefaultValue[2] = 127; 
              }
              else if(shields[s].id == 0)
              {
                if(v3 == PINMODE_DIGITAL_INPUT) { pinMode(shields[s].ports[p].Pin_Number[2],INPUT); }
                else { pinMode(shields[s].ports[p].Pin_Number[2],OUTPUT); }
              }
              shields[s].ports[p].Pin_Mode[3] = v4;
              if(v4 == PINMODE_PWM_OUTPUT) 
              { 
                shields[s].ports[p].Pin_DefaultValue[3] = 127; 
              }
              else if(shields[s].id == 0)
              {
                if(v4 == PINMODE_DIGITAL_INPUT) { pinMode(shields[s].ports[p].Pin_Number[3],INPUT); }
                else { pinMode(shields[s].ports[p].Pin_Number[3],OUTPUT); }
              }
              shields[s].ports[p].Pin_Mode[4] = v5;
              if(v5 == PINMODE_PWM_OUTPUT) 
              {
                shields[s].ports[p].Pin_DefaultValue[4] = 127; 
              }
              else if(shields[s].id == 0)
              {
                if(v5 == PINMODE_DIGITAL_INPUT) { pinMode(shields[s].ports[p].Pin_Number[4],INPUT); }
                else { pinMode(shields[s].ports[p].Pin_Number[4],OUTPUT); }
              }
              shields[s].ports[p].Pin_Mode[5] = v6;
              if(v6 == PINMODE_PWM_OUTPUT) 
              { 
                shields[s].ports[p].Pin_DefaultValue[5] = 127; 
              }
              else if(shields[s].id == 0)
              {
                if(v6 == PINMODE_DIGITAL_INPUT) { pinMode(shields[s].ports[p].Pin_Number[5],INPUT); }
                else { pinMode(shields[s].ports[p].Pin_Number[5],OUTPUT); }
              }
              shields[s].ports[p].Pin_Mode[6] = v7;
              if(v7 == PINMODE_PWM_OUTPUT) 
              { 
                shields[s].ports[p].Pin_DefaultValue[6] = 127; 
              }
              else if(shields[s].id == 0)
              {
                if(v7 == PINMODE_DIGITAL_INPUT) { pinMode(shields[s].ports[p].Pin_Number[6],INPUT); }
                else { pinMode(shields[s].ports[p].Pin_Number[7],OUTPUT); }
              }
              shields[s].ports[p].Pin_Mode[7] = v8;
              if(v8 == PINMODE_PWM_OUTPUT) 
              { 
                shields[s].ports[p].Pin_DefaultValue[7] = 127; 
              }
              else if(shields[s].id == 0)
              {
                if(v8 == PINMODE_DIGITAL_INPUT) { pinMode(shields[s].ports[p].Pin_Number[7],INPUT); }
                else { pinMode(shields[s].ports[p].Pin_Number[7],OUTPUT); }
              }
            }
            int configured_port_counter = 0;
            for(int i = 0; i < MAXNUMBER_PORTS_PERSHIELD; i++)
            {
              if(shields[s].ports[i].id >= 0)
              {
                configured_port_counter++;
              }
            }
            if((shields[s].portcount > 0) &&
             (configured_port_counter == shields[s].portcount))
            {
              board_mode = BOARDMODE_INITIALIZED;
            }
            
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
      case BUTTON_SELECT:
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
    int computed_checksum;
    int tx_status = serialmessagehandler.encode_ModeSerial(buffer,&length,BOARD_TYPE,BOARD_ID,board_mode);
    for(int i = 0; i < length; i++)
    {
      Serial.write((byte)buffer[i]);
    }
    send_mode_counter++;
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
  #if ((SHIELD1_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD2_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_LCDSHIELD))
  {
      update_lcd();
  }
  #endif
  if(board_mode == BOARDMODE_BOOT)
  {
    board_mode = BOARDMODE_INITIALIZING;
  }
  if(board_mode == BOARDMODE_INITIALIZED)
  {
    board_mode = BOARDMODE_RUNNING;
    armed_state = ARMEDSTATUS_DISARMED;
  }
  if((BYPASS_SHIELDCONFIG == 1) && (board_mode == BOARDMODE_SHIELDS_CONFIGURED))
  {
    board_mode = BOARDMODE_RUNNING;
  }
  if((board_mode == BOARDMODE_RUNNING) && (node_mode == BOARDMODE_INITIALIZING))
  {
    #if BOARD_TYPE == BOARDTYPE_ARDUINOMEGA
    {
      Serial1.println("Arduino Board Rebooting.");
    }
    #endif
    lcd.print("Rebooting");
    
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
  
  
  digitalWrite(LED,!digitalRead(LED));
  
}
void run_veryslowrate_code() //0.1 Hz
{
  
  #if BOARD_TYPE == BOARDTYPE_ARDUINOMEGA
  {
    #if PRINT_DEBUG_LINES == 1
      Serial1.print("Passed Checksum: ");
      Serial1.print(passed_checksum_counter,DEC);
      Serial1.print(" Failed Checksum: ");
      Serial1.println(failed_checksum_counter,DEC);
      
      Serial1.print("Sent Mode (0xAB17) times: ");
      Serial1.print(send_mode_counter,DEC);
      Serial1.print(" at: ");
      Serial1.print(1000.0*(double)send_mode_counter/(double)loop_counter);
      Serial1.print(" (Hz) Received times: ");
      Serial1.print(recv_mode_counter,DEC);
      Serial1.print(" at: ");
      Serial1.print(1000.0*(double)recv_mode_counter/(double)loop_counter);
      Serial1.println(" (Hz)");
      
      Serial1.print("Sent Shield Configure (0xAB33) times: ");
      Serial1.print(send_configure_shield_counter,DEC);
      Serial1.print(" at: ");
      Serial1.print(1000.0*(double)send_configure_shield_counter/(double)loop_counter);
      Serial1.print(" (Hz) Received times: ");
      Serial1.print(recv_configure_shield_counter,DEC);
      Serial1.print(" at: ");
      Serial1.print(1000.0*(double)recv_configure_shield_counter/(double)loop_counter);
      Serial1.println(" (Hz)");
      
      Serial1.print("Send DIO Configure (0xAB16) times: ");
      Serial1.print(send_configure_dio_counter,DEC);
      Serial1.print(" at: ");
      Serial1.print(1000.0*(double)send_configure_dio_counter/(double)loop_counter);
      Serial1.print(" (Hz) Received times: ");
      Serial1.print(recv_configure_dio_counter,DEC);
      Serial1.print(" at: ");
      Serial1.print(1000.0*(double)recv_configure_dio_counter/(double)loop_counter);
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
      
      Serial1.print("Send Set DIO DefaultValue (0xAB32) times: ");
      Serial1.print(send_set_dio_defaultvalue_counter,DEC);
      Serial1.print(" at: ");
      Serial1.print(1000.0*(double)send_set_dio_defaultvalue_counter/(double)loop_counter);
      Serial1.print(" (Hz) Received times: ");
      Serial1.print(recv_set_dio_defaultvalue_counter,DEC);
      Serial1.print(" at: ");
      Serial1.print(1000.0*(double)recv_set_dio_defaultvalue_counter/(double)loop_counter);
      Serial1.println(" (Hz)");
      
      Serial1.print("Send ArmCommand (0xAB27) times: ");
      Serial1.print(send_armcommand_counter,DEC);
      Serial1.print(" at: ");
      Serial1.print(1000.0*(double)send_armcommand_counter/(double)loop_counter);
      Serial1.print(" (Hz) Received times: ");
      Serial1.print(recv_armcommand_counter,DEC);
      Serial1.print(" at: ");
      Serial1.print(1000.0*(double)recv_armcommand_counter/(double)loop_counter);
      Serial1.println(" (Hz)");
    #endif
  }
  #elif(BOARD_TYPE == BOARDTYPE_ARDUINOUNO)
  {
    #if PRINT_DEBUG_LINES == 1
      softSerial.print("Passed Checksum: ");
      softSerial.print(passed_checksum_counter,DEC);
      softSerial.print(" Failed Checksum: ");
      softSerial.println(failed_checksum_counter,DEC);
      
      softSerial.print("Sent Mode (0xAB17) times: ");
      softSerial.print(send_mode_counter,DEC);
      softSerial.print(" at: ");
      softSerial.print(1000.0*(double)send_mode_counter/(double)loop_counter);
      softSerial.print(" (Hz) Received times: ");
      softSerial.print(recv_mode_counter,DEC);
      softSerial.print(" at: ");
      softSerial.print(1000.0*(double)recv_mode_counter/(double)loop_counter);
      softSerial.println(" (Hz)");
      
      softSerial.print("Sent Shield Configure (0xAB33) times: ");
      softSerial.print(send_configure_shield_counter,DEC);
      softSerial.print(" at: ");
      softSerial.print(1000.0*(double)send_configure_shield_counter/(double)loop_counter);
      softSerial.print(" (Hz) Received times: ");
      softSerial.print(recv_configure_shield_counter,DEC);
      softSerial.print(" at: ");
      softSerial.print(1000.0*(double)recv_configure_shield_counter/(double)loop_counter);
      softSerial.println(" (Hz)");
      
      softSerial.print("Send DIO Configure (0xAB16) times: ");
      softSerial.print(send_configure_dio_counter,DEC);
      softSerial.print(" at: ");
      softSerial.print(1000.0*(double)send_configure_dio_counter/(double)loop_counter);
      softSerial.print(" (Hz) Received times: ");
      softSerial.print(recv_configure_dio_counter,DEC);
      softSerial.print(" at: ");
      softSerial.print(1000.0*(double)recv_configure_dio_counter/(double)loop_counter);
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
      
      softSerial.print("Send Set DIO DefaultValue (0xAB32) times: ");
      softSerial.print(send_set_dio_defaultvalue_counter,DEC);
      softSerial.print(" at: ");
      softSerial.print(1000.0*(double)send_set_dio_defaultvalue_counter/(double)loop_counter);
      softSerial.print(" (Hz) Received times: ");
      softSerial.print(recv_set_dio_defaultvalue_counter,DEC);
      softSerial.print(" at: ");
      softSerial.print(1000.0*(double)recv_set_dio_defaultvalue_counter/(double)loop_counter);
      softSerial.println(" (Hz)");
      
      softSerial.print("Send ArmCommand (0xAB27) times: ");
      softSerial.print(send_armcommand_counter,DEC);
      softSerial.print(" at: ");
      softSerial.print(1000.0*(double)send_armcommand_counter/(double)loop_counter);
      softSerial.print(" (Hz) Received times: ");
      softSerial.print(recv_armcommand_counter,DEC);
      softSerial.print(" at: ");
      softSerial.print(1000.0*(double)recv_armcommand_counter/(double)loop_counter);
      softSerial.println(" (Hz)");   
    #endif
  }
  #endif
  
  
  for(int s = 0; s < MAXNUMBER_SHIELDS;s++)
  {
    int pinindex = 0;
    for(int p=0; p < MAXNUMBER_PORTS_PERSHIELD;p++)
    {
      for(int i = 0; i < PORT_SIZE; i++)
      {
       /* Serial.print("Shield ID: ");
        Serial.print(shields[s].id,DEC);
        Serial.print(" port ID: ");
        Serial.print(shields[s].ports[p].id,DEC);
        Serial.print(" pin num: ");
        Serial.print(shields[s].ports[p].Pin_Number[i],DEC);
        Serial.println("");
        */
      }
    }
  }
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
#if ((SHIELD1_TYPE == SHIELDTYPE_SERVOSHIELD) || (SHIELD2_TYPE == SHIELDTYPE_SERVOSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_SERVOSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_SERVOSHIELD))
void SERVOSHIELD_setServoPulse(uint8_t n, uint16_t pulse_us)
{
  double pulse = (double)(pulse_us)/1000.0;
  double pulselength;
  
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= SERVOSHIELD_UPDATE_RATE;   // 60 Hz
  pulselength /= 4096;  // 12 bits of resolution
  pulse *= 1000;
  pulse /= pulselength;
  pwm.setPWM(n, 0, pulse);
}
#endif

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

