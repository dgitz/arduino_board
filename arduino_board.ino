#define FIRMWARE_MAJOR_VERSION 0
#define FIRMWARE_MINOR_VERSION 0
#define FIRMWARE_BUILD_NUMBER 0

#include "Arduino.h"
#include <SoftwareSerial.h>
#include "serialmessage.h"
#include "Definitions.h"
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <avr/wdt.h>

//Debugging Defines
#define PRINT_RECV_BUFFER 1
#define PRINT_DEBUG_LINES 0


#define BOARD_ID 17
#define BOARD_TYPE BOARDTYPE_ARDUINOMEGA
#define MAXNUMBER_SHIELDS 4
#if BOARD_TYPE == BOARDTYPE_ARDUINOUNO
#define MAXNUMBER_PORTS_PERSHIELD 2
#elif BOARD_TYPE == BOARDTYPE_ARDUINOMEGA
#define MAXNUMBER_PORTS_PERSHIELD 2
#endif
#define PORT_SIZE        8
#define SERIAL_MESSAGE_SIZE 16 //Start Delimiter thru Checksum
#define TICK_PER_CDEG 0.08f

//Defines for Individual Shields
//SERVOSHIELD
#define SERVOSHIELD_UPDATE_RATE 50
#define SERVOSHIELD_SERVO_MIN 224
#define SERVOSHIELD_SERVO_MAX 480

typedef struct
{
  int id;
  int Level; //Diagnostic Level
  int State;
  int Available;
  int Joint_Type;
  int Output_Type;
  int Pin_Output;
  int LimitSwitchA_Type;
  int Pin_LimitSwitchA;
  int LimitSwitchB_Type;
  int Pin_LimitSwitchB;
  int PositionSensor_Type;
  int Pin_PositionSensorA;
  int Pin_PositionSensorB;
  double Rotation_Ratio; //The ratio of how many rotations the Angle Sensor makes and how many rotations the Joint makes
  double Current_Position_cdeg; //centidegrees, ranges from [-18000,18000] 
  double Target_Position_cdeg; //centidegrees, ranges from [-18000,18000] 
  double Angle_Traveled_cdeg; //centidegrees, ranges from -inf to inf.  This is not suitable for any type of trig functions
  double P_Gain;
  double I_Gain;
  double D_Gain;
  
} joint;
joint Joints[3]; //Can only support 3 Joints, due to Arduino Mega only supporting 6 interrupts
void run_veryfastrate_code(); //1000 Hz
void run_fastrate_code(); //100 Hz
void run_mediumrate_code(); //10 Hz
void run_slowrate_code(); //1 Hz
void run_veryslowrate_code(); //0.1 Hz
int led = 13;



Adafruit_PWMServoDriver pwm;

int board_mode = BOARDMODE_BOOT;
int node_mode = BOARDMODE_UNDEFINED;
unsigned int armed_command = ARMEDCOMMAND_DISARM;
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

#if(BOARD_TYPE == BOARDTYPE_ARDUINOUNO)
SoftwareSerial softSerial(2, 3); // RX, TX
#endif

int veryfastrate_counter = 0;
int fastrate_counter = 0;
int mediumrate_counter = 0;
int slowrate_counter = 0;
int veryslowrate_counter = 0;

unsigned long loop_counter = 0;
unsigned long send_mode_counter = 0;
unsigned long recv_mode_counter = 0;
unsigned long send_armcommand_counter = 0;
unsigned long recv_armcommand_counter = 0;
unsigned long time_since_last_rx = 0;

int temp_counter = 1000;
bool reverse = false;
int shield_count = -1;

//Function Prototypes for individual shields
//SERVOSHIELD
void SERVOSHIELD_setServoPulse(uint8_t pin_number, uint16_t pulse_us);

void Joint0_InputA();
void Joint0_InputB();
void Joint1_InputA();
void Joint1_InputB();
void Joint2_InputA();
void Joint2_InputB();


void scan_for_shields();
void init_joints();
void(*resetFunc)(void) = 0;
void setup() {
  wdt_disable();
  wdt_enable(WDTO_1S);
  memset(recv_buffer,0,sizeof(recv_buffer));
  pinMode(led,OUTPUT);
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
    softSerial.begin(115200);
    softSerial.println("ArduinoBoard Booting");
    softSerial.print("FW Major Version: ");
    softSerial.println(FIRMWARE_MAJOR_VERSION,DEC);
    softSerial.print("FW Minor Version: ");
    softSerial.println(FIRMWARE_MINOR_VERSION,DEC);
    softSerial.print("FW Build Number: ");
    softSerial.println(FIRMWARE_BUILD_NUMBER,DEC);
  }
  #endif
  delay(500);
  wdt_reset();
  
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
void init_joints()
{
  int jointindex = 0;
  Joints[jointindex].id = jointindex+1;//Shoulder
  Joints[jointindex].Level = INFO;
  Joints[jointindex].State = JOINTSTATE_INITIALIZING;
  Joints[jointindex].Available = 0;
  Joints[jointindex].Joint_Type = JOINTTYPE_REVOLUTE;
  Joints[jointindex].Pin_Output = -1;
  Joints[jointindex].LimitSwitchA_Type = PINMODE_DIGITAL_INPUT;
  Joints[jointindex].Pin_LimitSwitchA = -1;
  Joints[jointindex].LimitSwitchB_Type = PINMODE_DIGITAL_INPUT;
  Joints[jointindex].Pin_LimitSwitchB = -1;
  Joints[jointindex].PositionSensor_Type = PINMODE_QUADRATUREENCODER_INPUT;
  Joints[jointindex].Pin_PositionSensorA = -1;
  Joints[jointindex].Pin_PositionSensorB = -1;
  Joints[jointindex].Current_Position_cdeg = 0.0f;
  Joints[jointindex].Target_Position_cdeg = 0.0f;
  Joints[jointindex].Rotation_Ratio = 0.0f;
  jointindex++;

  for(int j = 0; j < 3; j++)
  {
     pinMode(Joints[j].Pin_Output, OUTPUT);  
     pinMode(Joints[j].Pin_LimitSwitchA, INPUT); 
     pinMode(Joints[j].Pin_LimitSwitchB, INPUT); 
     pinMode(Joints[j].Pin_PositionSensorA, INPUT); 
     pinMode(Joints[j].Pin_PositionSensorB, INPUT);   
  }
  attachInterrupt(digitalPinToInterrupt(Joints[0].Pin_PositionSensorA), Joint0_InputA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Joints[0].Pin_PositionSensorB), Joint0_InputB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Joints[1].Pin_PositionSensorA), Joint1_InputA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Joints[1].Pin_PositionSensorB), Joint1_InputB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Joints[2].Pin_PositionSensorA), Joint2_InputA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Joints[2].Pin_PositionSensorB), Joint2_InputB, CHANGE);
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
    if(armed_command == ARMEDCOMMAND_ARM)
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
    if(armed_command == ARMEDCOMMAND_DISARM)
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
        node_mode = node_status;
        recv_mode_counter++;
      }
      else if(message_type == SERIAL_Arm_Command_ID)
      {
        recv_armcommand_counter++;
        unsigned char Command;
        int status = serialmessagehandler.decode_Arm_CommandSerial(packet,&Command);
        armed_command = Command;
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
    armed_state = ARMEDSTATUS_DISARMED;
  }
  if((board_mode == BOARDMODE_RUNNING) && (node_mode == BOARDMODE_INITIALIZING))
  {
    #if BOARD_TYPE == BOARDTYPE_ARDUINOMEGA
    {
      Serial1.println("Arduino Board Rebooting.");
    }
    #endif
    delay(500);
    resetFunc();
  }
  
  digitalWrite(led,!digitalRead(led));
  
}
void run_veryslowrate_code() //0.1 Hz
{
  
  #if BOARD_TYPE == BOARDTYPE_ARDUINOMEGA
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
    
    
    Serial1.print("Send ArmCommand (0xAB27) times: ");
    Serial1.print(send_armcommand_counter,DEC);
    Serial1.print(" at: ");
    Serial1.print(1000.0*(double)send_armcommand_counter/(double)loop_counter);
    Serial1.print(" (Hz) Received times: ");
    Serial1.print(recv_armcommand_counter,DEC);
    Serial1.print(" at: ");
    Serial1.print(1000.0*(double)recv_armcommand_counter/(double)loop_counter);
    Serial1.println(" (Hz)");   
  }
  #elif(BOARD_TYPE == BOARDTYPE_ARDUINOUNO)
  {
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
    
    
    softSerial.print("Send ArmCommand (0xAB27) times: ");
    softSerial.print(send_armcommand_counter,DEC);
    softSerial.print(" at: ");
    softSerial.print(1000.0*(double)send_armcommand_counter/(double)loop_counter);
    softSerial.print(" (Hz) Received times: ");
    softSerial.print(recv_armcommand_counter,DEC);
    softSerial.print(" at: ");
    softSerial.print(1000.0*(double)recv_armcommand_counter/(double)loop_counter);
    softSerial.println(" (Hz)");   
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

void Joint0_InputA()
{
  bool set_current_position = false;
  if(Joints[0].State == JOINTSTATE_RUNNING) { set_current_position = true; }
  // look for a low-to-high on channel A
  if (digitalRead(Joints[0].Pin_PositionSensorA) == HIGH) 
  { 
  // check channel B to see which way encoder is turning
    if (digitalRead(Joints[0].Pin_PositionSensorB) == LOW) //CW
    {  
      if(set_current_position == true) { Joints[0].Current_Position_cdeg += TICK_PER_CDEG*Joints[0].Rotation_Ratio; }
      else { Joints[0].Angle_Traveled_cdeg += TICK_PER_CDEG*Joints[0].Rotation_Ratio; }
    } 
    else //CCW
    {
      if(set_current_position == true) { Joints[0].Current_Position_cdeg -= TICK_PER_CDEG*Joints[0].Rotation_Ratio; }
      else { Joints[0].Angle_Traveled_cdeg -= TICK_PER_CDEG*Joints[0].Rotation_Ratio; }
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(Joints[0].Pin_PositionSensorB) == HIGH) //CW
    {   
      if(set_current_position == true) { Joints[0].Current_Position_cdeg += TICK_PER_CDEG*Joints[0].Rotation_Ratio; }
      else { Joints[0].Angle_Traveled_cdeg += TICK_PER_CDEG*Joints[0].Rotation_Ratio; }
    } 
    else //CCW
    {
      if(set_current_position == true) { Joints[0].Current_Position_cdeg -= TICK_PER_CDEG*Joints[0].Rotation_Ratio; }
      else { Joints[0].Angle_Traveled_cdeg -= TICK_PER_CDEG*Joints[0].Rotation_Ratio; }
    } 
  }
}
void Joint0_InputB()
{
  bool set_current_position = false;
  if(Joints[0].State == JOINTSTATE_RUNNING) { set_current_position = true; }
  // look for a low-to-high on channel B
  if (digitalRead(Joints[0].Pin_PositionSensorB) == HIGH) 
  {   
    // check channel A to see which way encoder is turning
    if (digitalRead(Joints[0].Pin_PositionSensorA) == HIGH) //CW
    {  
      if(set_current_position == true) { Joints[0].Current_Position_cdeg += TICK_PER_CDEG*Joints[0].Rotation_Ratio; }
      else { Joints[0].Angle_Traveled_cdeg += TICK_PER_CDEG*Joints[0].Rotation_Ratio; }
    } 
    else //CCW
    {
      if(set_current_position == true) { Joints[0].Current_Position_cdeg -= TICK_PER_CDEG*Joints[0].Rotation_Ratio; }
      else { Joints[0].Angle_Traveled_cdeg -= TICK_PER_CDEG*Joints[0].Rotation_Ratio; }
    }
  }
  // Look for a high-to-low on channel B
  else 
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(Joints[0].Pin_PositionSensorA) == LOW) //CW
    {   
      if(set_current_position == true) { Joints[0].Current_Position_cdeg += TICK_PER_CDEG*Joints[0].Rotation_Ratio; }
      else { Joints[0].Angle_Traveled_cdeg += TICK_PER_CDEG*Joints[0].Rotation_Ratio; }
    } 
    else //CCW
    {
      if(set_current_position == true) { Joints[0].Current_Position_cdeg -= TICK_PER_CDEG*Joints[0].Rotation_Ratio; }
      else { Joints[0].Angle_Traveled_cdeg -= TICK_PER_CDEG*Joints[0].Rotation_Ratio; }
    }
  }
}

void Joint1_InputA()
{
  bool set_current_position = false;
  if(Joints[1].State == JOINTSTATE_RUNNING) { set_current_position = true; }
  // look for a low-to-high on channel A
  if (digitalRead(Joints[1].Pin_PositionSensorA) == HIGH) 
  { 
  // check channel B to see which way encoder is turning
    if (digitalRead(Joints[1].Pin_PositionSensorB) == LOW) //CW
    {  
      if(set_current_position == true) { Joints[1].Current_Position_cdeg += TICK_PER_CDEG*Joints[1].Rotation_Ratio; }
      else { Joints[1].Angle_Traveled_cdeg += TICK_PER_CDEG*Joints[1].Rotation_Ratio; }
    } 
    else //CCW
    {
      if(set_current_position == true) { Joints[1].Current_Position_cdeg -= TICK_PER_CDEG*Joints[1].Rotation_Ratio; }
      else { Joints[1].Angle_Traveled_cdeg -= TICK_PER_CDEG*Joints[1].Rotation_Ratio; }
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(Joints[1].Pin_PositionSensorB) == HIGH) //CW
    {   
      if(set_current_position == true) { Joints[1].Current_Position_cdeg += TICK_PER_CDEG*Joints[1].Rotation_Ratio; }
      else { Joints[1].Angle_Traveled_cdeg += TICK_PER_CDEG*Joints[1].Rotation_Ratio; }
    } 
    else //CCW
    {
      if(set_current_position == true) { Joints[1].Current_Position_cdeg -= TICK_PER_CDEG*Joints[1].Rotation_Ratio; }
      else { Joints[1].Angle_Traveled_cdeg -= TICK_PER_CDEG*Joints[1].Rotation_Ratio; }
    } 
  }
}
void Joint1_InputB()
{
  bool set_current_position = false;
  if(Joints[1].State == JOINTSTATE_RUNNING) { set_current_position = true; }
  // look for a low-to-high on channel B
  if (digitalRead(Joints[1].Pin_PositionSensorB) == HIGH) 
  {   
    // check channel A to see which way encoder is turning
    if (digitalRead(Joints[1].Pin_PositionSensorA) == HIGH) //CW
    {  
      if(set_current_position == true) { Joints[1].Current_Position_cdeg += TICK_PER_CDEG*Joints[1].Rotation_Ratio; }
      else { Joints[1].Angle_Traveled_cdeg += TICK_PER_CDEG*Joints[1].Rotation_Ratio; }
    } 
    else //CCW
    {
      if(set_current_position == true) { Joints[1].Current_Position_cdeg -= TICK_PER_CDEG*Joints[1].Rotation_Ratio; }
      else { Joints[1].Angle_Traveled_cdeg -= TICK_PER_CDEG*Joints[1].Rotation_Ratio; }
    }
  }
  // Look for a high-to-low on channel B
  else 
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(Joints[1].Pin_PositionSensorA) == LOW) //CW
    {   
      if(set_current_position == true) { Joints[1].Current_Position_cdeg += TICK_PER_CDEG*Joints[1].Rotation_Ratio; }
      else { Joints[1].Angle_Traveled_cdeg += TICK_PER_CDEG*Joints[1].Rotation_Ratio; }
    } 
    else //CCW
    {
      if(set_current_position == true) { Joints[1].Current_Position_cdeg -= TICK_PER_CDEG*Joints[1].Rotation_Ratio; }
      else { Joints[1].Angle_Traveled_cdeg -= TICK_PER_CDEG*Joints[1].Rotation_Ratio; }
    }
  }
}

void Joint2_InputA()
{
  bool set_current_position = false;
  if(Joints[2].State == JOINTSTATE_RUNNING) { set_current_position = true; }
  // look for a low-to-high on channel A
  if (digitalRead(Joints[2].Pin_PositionSensorA) == HIGH) 
  { 
  // check channel B to see which way encoder is turning
    if (digitalRead(Joints[2].Pin_PositionSensorB) == LOW) //CW
    {  
      if(set_current_position == true) { Joints[2].Current_Position_cdeg += TICK_PER_CDEG*Joints[2].Rotation_Ratio; }
      else { Joints[2].Angle_Traveled_cdeg += TICK_PER_CDEG*Joints[2].Rotation_Ratio; }
    } 
    else //CCW
    {
      if(set_current_position == true) { Joints[2].Current_Position_cdeg -= TICK_PER_CDEG*Joints[2].Rotation_Ratio; }
      else { Joints[2].Angle_Traveled_cdeg -= TICK_PER_CDEG*Joints[2].Rotation_Ratio; }
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(Joints[2].Pin_PositionSensorB) == HIGH) //CW
    {   
      if(set_current_position == true) { Joints[2].Current_Position_cdeg += TICK_PER_CDEG*Joints[2].Rotation_Ratio; }
      else { Joints[2].Angle_Traveled_cdeg += TICK_PER_CDEG*Joints[2].Rotation_Ratio; }
    } 
    else //CCW
    {
      if(set_current_position == true) { Joints[2].Current_Position_cdeg -= TICK_PER_CDEG*Joints[2].Rotation_Ratio; }
      else { Joints[2].Angle_Traveled_cdeg -= TICK_PER_CDEG*Joints[2].Rotation_Ratio; }
    } 
  }
}
void Joint2_InputB()
{
  bool set_current_position = false;
  if(Joints[2].State == JOINTSTATE_RUNNING) { set_current_position = true; }
  // look for a low-to-high on channel B
  if (digitalRead(Joints[2].Pin_PositionSensorB) == HIGH) 
  {   
    // check channel A to see which way encoder is turning
    if (digitalRead(Joints[2].Pin_PositionSensorA) == HIGH) //CW
    {  
      if(set_current_position == true) { Joints[2].Current_Position_cdeg += TICK_PER_CDEG*Joints[2].Rotation_Ratio; }
      else { Joints[2].Angle_Traveled_cdeg += TICK_PER_CDEG*Joints[2].Rotation_Ratio; }
    } 
    else //CCW
    {
      if(set_current_position == true) { Joints[2].Current_Position_cdeg -= TICK_PER_CDEG*Joints[2].Rotation_Ratio; }
      else { Joints[2].Angle_Traveled_cdeg -= TICK_PER_CDEG*Joints[2].Rotation_Ratio; }
    }
  }
  // Look for a high-to-low on channel B
  else 
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(Joints[2].Pin_PositionSensorA) == LOW) //CW
    {   
      if(set_current_position == true) { Joints[2].Current_Position_cdeg += TICK_PER_CDEG*Joints[2].Rotation_Ratio; }
      else { Joints[2].Angle_Traveled_cdeg += TICK_PER_CDEG*Joints[2].Rotation_Ratio; }
    } 
    else //CCW
    {
      if(set_current_position == true) { Joints[2].Current_Position_cdeg -= TICK_PER_CDEG*Joints[2].Rotation_Ratio; }
      else { Joints[2].Angle_Traveled_cdeg -= TICK_PER_CDEG*Joints[2].Rotation_Ratio; }
    }
  }
}

