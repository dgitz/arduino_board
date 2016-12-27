#define FIRMWARE_MAJOR_VERSION 0
#define FIRMWARE_MINOR_VERSION 1
#define FIRMWARE_BUILD_NUMBER 0

#include "Arduino.h"
#include <SoftwareSerial.h>
#include "serialmessage.h"
#include "Definitions.h"
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>

//Debugging Defines
#define PRINT_RECV_BUFFER 0
#define PRINT_DEBUG_LINES 0


#define BOARD_ID 18
#define BOARD_TYPE BOARDTYPE_ARDUINOMEGA
#define MAXNUMBER_SHIELDS 8
#define MAXNUMBER_PORTS_PERSHIELD 4
#define PORT_SIZE        8
#define SERIAL_MESSAGE_SIZE 16 //Start Delimiter thru Checksum

//Defines for Individual Shields
//SERVOSHIELD
#define SERVOSHIELD_UPDATE_RATE 50
#define SERVOSHIELD_SERVO_MIN 224
#define SERVOSHIELD_SERVO_MAX 480

String InBuffer = "";
String ProcessBuffer = "";

typedef struct
{
  int id;
  int Pin_Number[PORT_SIZE];
  int Pin_Mode[PORT_SIZE];
  int Pin_Value[PORT_SIZE];
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

int led = 13;

Adafruit_PWMServoDriver pwm;

int board_mode = BOARDMODE_BOOT;
int node_mode = BOARDMODE_UNDEFINED;
//String message_buffer = ""; // a string to hold incoming data
unsigned char in_buffer[64];
unsigned char recv_buffer[64];
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

int temp_counter = 1000;
bool reverse = false;
int shield_count = -1;

//Function Prototypes for individual shields
//SERVOSHIELD
void SERVOSHIELD_setServoPulse(uint8_t pin_number, uint16_t pulse_us);


void scan_for_shields();
void init_shields();
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
  InBuffer.reserve(200);
  ProcessBuffer.reserve(200);
  memset(recv_buffer,0,sizeof(recv_buffer));
  pinMode(led,OUTPUT);
  Serial.begin(115200);
  //Serial.setTimeout(1000);
  while(Serial.read() >= 0);
  Serial.flush();
  if(BOARD_TYPE == BOARDTYPE_ARDUINOMEGA)
  {
    Serial1.begin(115200);
    Serial1.setTimeout(1000);
    while(Serial1.read() >= 0);
    Serial1.flush();
    Serial1.println("Board Booting");
  }
  
  

  init_shields();
  scan_for_shields();
  if(BOARD_TYPE == BOARDTYPE_ARDUINOMEGA)
  {
    Serial1.println("Board Executing");
  }
}

void scan_for_shields()
{
  Wire.begin();
  byte error;
  int found_index = 0;
  for(int i = 1; i < 127; i++)
  {
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
  if(BOARD_TYPE == BOARDTYPE_ARDUINOMEGA)
  {  
    for(int i = 0; i < found_index; i++)
    {
      Serial1.print("Found I2C Device at Address: ");
      Serial1.println(available_i2c_devices[i],HEX);
    }
  }
}
void loop() 
{ 

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

  }
}

void run_veryfastrate_code() //1000 Hz
{
}
void run_fastrate_code() //100 Hz
{
  if(board_mode == BOARDMODE_RUNNING)
  {
    
    for(int s = 0; s < MAXNUMBER_SHIELDS;s++)
    {
      for(int p = 0; p < MAXNUMBER_PORTS_PERSHIELD;p++)
      {
        for(int j = 0; j < PORT_SIZE;j++)
        {
          if(shields[s].ports[p].Pin_Mode[j] == PINMODE_PWM_OUTPUT)
          {
            unsigned int pulse = 1000 + 3.90625*(double)(shields[s].ports[p].Pin_Value[j]);
            if(pulse > 2000) { pulse = 2000;}
            else if(pulse < 1000) { pulse = 1000; }
            SERVOSHIELD_setServoPulse(shields[s].ports[p].Pin_Number[j], pulse);
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
          char ShieldCount,ShieldType,ShieldID,PortCount;
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
            if(matched == true)
            {
              
            }
            //Do config stuff for all shields
            
          }
          if(add_new_shield == true)
          {
            shields[i].id = ShieldID;
            shields[i].type = ShieldType;
            shields[i].portcount = PortCount;
          }
          
        }
      }
      else if(message_type == SERIAL_Set_DIO_Port_ID)
      {
        char ShieldID,PortID;
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
        char node_id,node_status,node_type;
        int status = serialmessagehandler.decode_ModeSerial(packet,&node_type,&node_id,&node_status);
        node_mode = node_status;
        recv_mode_counter++;
      }
      else if(message_type == SERIAL_Configure_DIO_Port_ID)
      {
        recv_configure_dio_counter++;
        char ShieldID,PortID;
        char v1,v2,v3,v4,v5,v6,v7,v8;
        
        int status = serialmessagehandler.decode_Configure_DIO_PortSerial(packet,&ShieldID,&PortID,&v1,&v2,&v3,&v4,&v5,&v6,&v7,&v8);
        for(int s = 0; s < MAXNUMBER_SHIELDS; s++)
        {
          if(shields[s].id == ShieldID)
          {
            bool add_new_port = false;
            int configured_port_counter = 0;
            int p;
            for(p = 0; p < PORT_SIZE; p++)
            {
              if(shields[s].ports[p].id == -1)
              {
                add_new_port = true;
              }
              else
              {
                configured_port_counter++;
              }
              if(add_new_port == true)
              {
                break;
              }
            }
            if((shields[s].portcount > 0) &&
             (configured_port_counter == shields[s].portcount))
            {
              board_mode = BOARDMODE_INITIALIZED;
              add_new_port = false;
            }
            if(add_new_port == true)
            {             
              shields[s].ports[p].id = PortID;
              shields[s].ports[p].Pin_Mode[0] = v1;
              shields[s].ports[p].Pin_Mode[1] = v2;
              shields[s].ports[p].Pin_Mode[2] = v3;
              shields[s].ports[p].Pin_Mode[3] = v4;
              shields[s].ports[p].Pin_Mode[4] = v5;
              shields[s].ports[p].Pin_Mode[5] = v6;
              shields[s].ports[p].Pin_Mode[6] = v7;
              shields[s].ports[p].Pin_Mode[7] = v8;
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
  if(board_mode == BOARDMODE_BOOT)
  {
    board_mode = BOARDMODE_INITIALIZING;
  }
  if(board_mode == BOARDMODE_INITIALIZED)
  {
    board_mode = BOARDMODE_RUNNING;
  }
  if((board_mode == BOARDMODE_RUNNING) && (node_mode == BOARDMODE_INITIALIZING))
  {
    Serial1.println("Rebooting");
      resetFunc();
  }
  
  digitalWrite(led,!digitalRead(led));
  
}
void run_veryslowrate_code() //0.1 Hz
{
  if(BOARD_TYPE == BOARDTYPE_ARDUINOMEGA)
  {
    
    
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
    
    Serial1.print("Set DIO Configure (0xAB18) times: ");
    Serial1.print(send_set_dio_counter,DEC);
    Serial1.print(" at: ");
    Serial1.print(1000.0*(double)send_set_dio_counter/(double)loop_counter);
    Serial1.print(" (Hz) Received times: ");
    Serial1.print(recv_set_dio_counter,DEC);
    Serial1.print(" at: ");
    Serial1.print(1000.0*(double)recv_set_dio_counter/(double)loop_counter);
    Serial1.println(" (Hz)");
    
    
    /*
    if((BOARD_TYPE == BOARDTYPE_ARDUINOMEGA) && (board_mode == BOARDMODE_RUNNING))
    {
      for(int s = 0; s < MAXNUMBER_SHIELDS; s++)
      {
        if(shields[s].id > -1)
        {
          for(int p = 0; p < MAXNUMBER_PORTS_PERSHIELD; p++)
          {
            if(shields[s].ports[p].id > -1)
            {
              Serial1.print("Shield Configuration: ID");
              Serial1.print(shields[s].id,DEC);
              Serial1.print(" Port ID: ");
              Serial1.println(shields[s].ports[p].id);
              for(int i = 0; i < PORT_SIZE; i++)
              {
                Serial1.print(" i: ");
                Serial1.print(shields[s].ports[p].Pin_Number[i],DEC);
                Serial1.print(" mode: ");
                Serial1.print(shields[s].ports[p].Pin_Mode[i],DEC);
              }
              Serial1.println("");
            }
          }
        }
      }
    }
    */
   
  }
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
    InBuffer = "";
    //Serial1.print("in: ");
    while((Serial.available()) && (bytecounter < bytestoread))
    {
      c = Serial.read();  
     delay(1);   
      //Serial1.print(" ");
      //Serial1.print(c,HEX);
      //Serial1.print(" count: ");
      //Serial1.print(bytecounter,DEC);
      recv_buffer[bytecounter++] = c;
      
    }
    //Serial1.println("");
    if((BOARD_TYPE == BOARDTYPE_ARDUINOMEGA) && (PRINT_RECV_BUFFER == 1))
    {
      for(int i = 0; i < SERIAL_MESSAGE_SIZE; i++)
      {
        Serial1.print(" ");
        Serial1.print(recv_buffer[i],HEX);
      }
      Serial1.println("");
    }
    message_complete = true;

  }
}
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


