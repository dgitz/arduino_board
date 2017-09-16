//Configuration Defines

#define FIRMWARE_MAJOR_VERSION 1
#define FIRMWARE_MINOR_VERSION 0
#define FIRMWARE_BUILD_NUMBER 0

#include "Arduino.h"
#include <SoftwareSerial.h>
#include "spimessage.h"
#include "Definitions.h"

#include <Wire.h>
#include <avr/wdt.h>
#include "config.h"


#define MAXNUMBER_SHIELDS 4
#if BOARD_TYPE == BOARDTYPE_ARDUINOUNO
#define MAXNUMBER_PORTS_PERSHIELD 2
#elif BOARD_TYPE == BOARDTYPE_ARDUINOMEGA
#define MAXNUMBER_PORTS_PERSHIELD 2
#endif
#define DIO_PORT_SIZE    8
#define ANA_PORT_SIZE    6

//Defines for SPI Comm between Raspberry Pi and Arduino Board
//unsigned char transmitBuffer[14];
//unsigned char receiveBuffer[14];
unsigned char outputBuffer_AB14[13];
unsigned char outputBuffer_AB19[13];
unsigned char outputBuffer_AB20[13];
bool run_spi_handler = false;
byte current_command = 0;
int outputBuffer_index = 0;
int received_command = 0;
int message_ready_to_send = 0;
int receive_index = 0;
int message_index = 0;
byte marker = 0;
unsigned char dat;
int compute_checksum(unsigned char * outputbuffer);



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
  int Pin_Mode[DIO_PORT_SIZE];
  int Pin_Number[DIO_PORT_SIZE];
  unsigned int Pin_Value[DIO_PORT_SIZE];
  unsigned int Pin_DefaultValue[DIO_PORT_SIZE];
  bool Pin_Used[DIO_PORT_SIZE];
} dio_port;

typedef struct
{
  int id;
  int Pin_Number[ANA_PORT_SIZE];
  unsigned int Pin_Value[ANA_PORT_SIZE];
  unsigned int Pin_DefaultValue[ANA_PORT_SIZE];
  bool Pin_Used[ANA_PORT_SIZE];
} ana_port;

ana_port ANAPORT1;
dio_port DIOPORT1;
void run_veryfastrate_code(); //1000 Hz
void run_fastrate_code(); //100 Hz
void run_mediumrate_code(); //10 Hz
void run_slowrate_code(); //1 Hz
void run_veryslowrate_code(); //0.1 Hz


#if ((SHIELD1_TYPE == SHIELDTYPE_SERVOSHIELD) || (SHIELD2_TYPE == SHIELDTYPE_SERVOSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_SERVOSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_SERVOSHIELD))
  Adafruit_PWMServoDriver pwm;
#endif

unsigned int armed_command = ROVERCOMMAND_DISARM;
unsigned int armed_state = ARMEDSTATUS_DISARMED_CANNOTARM;
unsigned char recv_buffer[32];
int available_i2c_devices[MAXNUMBER_SHIELDS*2];
int passed_checksum_counter = 0;
int failed_checksum_counter = 0;

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
unsigned long send_pps_counter = 0;
unsigned long recv_pps_counter = 0;
unsigned long time_since_last_rx = 0;
unsigned long level_debug_counter = 0;
unsigned long level_info_counter = 0;
unsigned long level_notice_counter = 0;
unsigned long level_warn_counter = 0;
unsigned long level_error_counter = 0;
unsigned long level_fatal_counter = 0;
byte transmit_testcounter = 0;
int comm_established_once = 0;



int temp_counter = 1000;
bool reverse = false;
int shield_count = -1;
int new_pps = 0;
int pps_received = 0;

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

int process_AB14_Query();
int process_AB19_Query();
int process_AB20_Query();


void(*resetFunc)(void) = 0;

//Message processing functions.  This should be as fast as possible
int process_AB14_Query()
{
  int msg_length;
  encode_TestMessageCounterSPI(outputBuffer_AB14,&msg_length,
    transmit_testcounter++,
    transmit_testcounter++,
    transmit_testcounter++,
    transmit_testcounter++,
    transmit_testcounter++,
    transmit_testcounter++,
    transmit_testcounter++,
    transmit_testcounter++,
    transmit_testcounter++,
    transmit_testcounter--,
    transmit_testcounter--,
    transmit_testcounter--);
}
int process_AB19_Query()
{
  int msg_length;
  encode_Get_DIO_Port1SPI(outputBuffer_AB19,&msg_length,
    DIOPORT1.Pin_Value[0],
    DIOPORT1.Pin_Value[1],
    DIOPORT1.Pin_Value[2],
    DIOPORT1.Pin_Value[3],
    DIOPORT1.Pin_Value[4],
    DIOPORT1.Pin_Value[5],
    DIOPORT1.Pin_Value[6],
    DIOPORT1.Pin_Value[7]);
}
int process_AB20_Query()
{
  int msg_length;
  encode_Get_ANA_Port1SPI(outputBuffer_AB20,&msg_length,
    ANAPORT1.Pin_Value[0],
    ANAPORT1.Pin_Value[1],
    ANAPORT1.Pin_Value[2],
    ANAPORT1.Pin_Value[3],
    ANAPORT1.Pin_Value[4],
    ANAPORT1.Pin_Value[5]);
}
void init_shields()
{
 
}
void setup() {
  wdt_disable();
  wdt_enable(WDTO_1S);
  //Setup pins

  //Setup ANAPORT1
  ANAPORT1.id = 1;
  for(int i = 0; i < ANA_PORT_SIZE; i++)
  {
    if(AnalogInputPort1[i] < 0) 
    {  
      ANAPORT1.Pin_Used[i] = false;   
      ANAPORT1.Pin_Value[i] = 0;
    }
    else
    {
      ANAPORT1.Pin_Used[i] = true;
      ANAPORT1.Pin_Number[i] = AnalogInputPort1[i];
      ANAPORT1.Pin_Value[i] = 0;
      ANAPORT1.Pin_DefaultValue[i] = 0;
    }
  }

  //Setup DIOPORT1
  DIOPORT1.id = 1;
  for(int i = 0; i < DIO_PORT_SIZE; i++)
  {
    if(DigitalPort1_Pins[i] < 0)
    {
      DIOPORT1.Pin_Used[i] = false;   
      DIOPORT1.Pin_Value[i] = 0;
      DIOPORT1.Pin_Mode[i] = PINMODE_NOTAVAILABLE;
    }
    else
    {
      DIOPORT1.Pin_Used[i] = true;
      DIOPORT1.Pin_Number[i] = DigitalPort1_Pins[i];
      DIOPORT1.Pin_Value[i] = 0;
      DIOPORT1.Pin_DefaultValue[i] = DigitalPort1_DefaultValue[i];
      DIOPORT1.Pin_Mode[i] = DigitalPort1_Mode[i];
    }
  }
  pinMode(LED,OUTPUT);
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);
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

    #if ((SHIELD1_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD2_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_LCDSHIELD) || (SHIELD3_TYPE == SHIELDTYPE_LCDSHIELD))
    {
      lcd.print("Booted");
    }
    #endif
  }
  #endif
  delay(500);
  wdt_reset();
  
  wdt_reset();
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
  if((SPSR & (1 << SPIF)) != 0)
  {
    run_spi_handler = true;
    spiHandler();
  }
  if(run_spi_handler == false)
  {
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
    send_pps_counter = 0;
    recv_pps_counter = 0;

  }
}

void run_veryfastrate_code() //1000 Hz
{
  
}
void run_fastrate_code() //100 Hz
{
  //Read ANAPORT1
  for(int i = 0; i < ANA_PORT_SIZE; i++)
  {
    if(ANAPORT1.Pin_Used[i] == true)
    {
      ANAPORT1.Pin_Value[i] = analogRead(ANAPORT1.Pin_Number[i]);
    }  
  }
  //Read DIOPORT1 and Set DIOPORT1
  for(int i = 0; i < DIO_PORT_SIZE; i++)
  {
    if(DIOPORT1.Pin_Used[i] == true)
    {
      if(DIOPORT1.Pin_Mode[i] == PINMODE_DIGITAL_INPUT)
      {
        DIOPORT1.Pin_Value[i] = digitalRead(DIOPORT1.Pin_Number[i]);
      }
      else //Not currently supported
      {
        
      }
    }  
  }
  
      
}
void run_mediumrate_code() //10 Hz
{
}
void run_slowrate_code() //1 Hz
{  
  
  digitalWrite(LED,!digitalRead(LED));
 // Serial.println("Running...");
}
void run_veryslowrate_code() //0.1 Hz
{
  
  #if BOARD_TYPE == BOARDTYPE_ARDUINOMEGA
  {
    #if PRINT_DEBUG_LINES == 1
    
    #endif
  }
  #elif(BOARD_TYPE == BOARDTYPE_ARDUINOUNO)
  {
    #if PRINT_DEBUG_LINES == 1
      
    #endif
  }
  #endif
}
void spiHandler()
{
  if(marker == 0)
  {
    dat = SPDR;
    if(dat == 0xAB)
    {
      SPDR = 'a';
      marker++;
    }
  }
  else if(marker == 1)
  {
    dat = SPDR;
    current_command = dat;
    if(current_command == SPI_TestMessageCounter_ID)
    {
      process_AB14_Query();
    }
    if(current_command == SPI_Get_DIO_Port1_ID)
    {
      process_AB19_Query();
    }
    else if(current_command == SPI_Get_ANA_Port1_ID)
    {
      process_AB20_Query();
    }
    marker++;
  }
  else
  {
    if(current_command == SPI_TestMessageCounter_ID)
    {
      SPDR = outputBuffer_AB14[outputBuffer_index];
    }
    else if(current_command == SPI_Get_DIO_Port1_ID)
    {
      SPDR = outputBuffer_AB19[outputBuffer_index];
    }
    else if(current_command == SPI_Get_ANA_Port1_ID)
    {
      SPDR = outputBuffer_AB20[outputBuffer_index];
    }
    outputBuffer_index++;
    marker++;
    if(outputBuffer_index == 13)
    {
      outputBuffer_index = 0;
      marker = 0;
      run_spi_handler = false;
    }
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
int compute_checksum(unsigned char * outputbuffer)
{
  int checksum = 0;
  for(int i = 0; i < 12; i++)
  {
    checksum ^= outputbuffer[i];
  }
  return checksum;
}

