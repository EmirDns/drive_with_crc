#include <Arduino.h>
#include "HardwareSerial.h"

#define MASTER

#define ARRAY_LEN 2
#define MSG_LEN 3*ARRAY_LEN + 2
#define MSG_LEN_WCRC MSG_LEN + 2
#define MAPPING_COEFF 255

typedef struct
{
  uint8_t start_byte = 'S';
  uint8_t stop_byte = 'F'; 
  uint8_t directions[ARRAY_LEN];
  uint8_t directions_to_sent[ARRAY_LEN];
  uint8_t magnitudes[ARRAY_LEN];
  uint8_t magnitudes_to_sent[ARRAY_LEN];
  uint16_t slave_crc;
  uint16_t master_crc;
  uint8_t input_arr[MSG_LEN];
  uint8_t output_arr[MSG_LEN];
} DriveMsg;

#ifdef SLAVE
void readMsg(DriveMsg *message, HardwareSerial& mySerial);
void assignSlaveVars(DriveMsg *message);
uint16_t getCRC(byte buf[MSG_LEN]);
#endif

#ifdef MASTER
void assignMasterVars(DriveMsg *message, float command_list[]);
void createMsg(DriveMsg *message, float command_list[]);
void printMsg(DriveMsg *message, HardwareSerial& mySerial);
uint16_t getCRC(byte buf[MSG_LEN + 2]);
#endif

DriveMsg msg;
DriveMsg *msg_ptr = &msg;

float motorValues[ARRAY_LEN]={0.56,-0.86};

void setup() {
  
}

void loop() {
  
}

#ifdef SLAVE
void readMsg(DriveMsg *message, HardwareSerial& mySerial){
  uint8_t inc_byte;
  static bool receive_flag=false;
  static uint8_t receive_counter=0; 
  while (mySerial.available()>0){
    inc_byte=mySerial.read();
    delay(1);
    if(inc_byte = message->start_byte){
      message->input_arr[receive_counter]=inc_byte;
      receive_flag=true;
      receive_counter++;
    }
    if(receive_flag && inc_byte!=message->start_byte && inc_byte!=message->stop_byte){
      message->input_arr[receive_counter]=inc_byte;
      receive_counter++;
    }
    if(inc_byte==message->stop_byte){
      message->input_arr[receive_counter]=inc_byte;
      receive_counter++;
      delay(1);
      inc_byte=mySerial.read();
      message->input_arr[receive_counter]=inc_byte;
      receive_counter++;
      delay(1);
      inc_byte=mySerial.read();
      message->input_arr[receive_counter]=inc_byte;
      receive_counter=0;
      receive_flag=false;
      uint16_t new_crc=getCRC(message->input_arr);
      message->slave_crc=(message->input_arr[MSG_LEN_WCRC - 2] << 8) | message->input_arr[MSG_LEN_WCRC - 1];
      if(new_crc==message->slave_crc){
        assignSlaveVars(message);
      }
    } 
  }
}

void assignSlaveVars(DriveMsg *message){
  String str_buffer;
  int int_buffer;
  if(message->input_arr[0]==message->start_byte && message->input_arr[MSG_LEN+1]==message->stop_byte){
    for(int i=0;i<ARRAY_LEN;i++){
      for(int j=1;j<4;j++){
        str_buffer=str_buffer+message->input_arr[(i*ARRAY_LEN)+j];
      }
      if(str_buffer.toInt()<128){
        message->directions[i]=0;
        int_buffer=(str_buffer.toInt()*MAPPING_COEFF/127)-MAPPING_COEFF;
        message->magnitudes[i]=abs(int_buffer)/MAPPING_COEFF;
        str_buffer="";
      }
      if(127<str_buffer.toInt()){
        message->directions[i]=1;
        int_buffer=(str_buffer.toInt()-128)*MAPPING_COEFF/127;
        message->magnitudes[i]=int_buffer/MAPPING_COEFF;
        str_buffer="";
      }
    }
  }
}

uint16_t getCRC(byte buf[MSG_LEN + 2])
{
    unsigned int crc = 0xFFFF;
    for (int pos = 0; pos < MSG_LEN-1; pos++)
    {
        crc ^= (unsigned int)buf[pos];
        for (int i = 8; i != 0; i--)
        {
            if ((crc & 0x0001) != 0)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}
#endif

#ifdef MASTER
void assignMasterVars(DriveMsg *message, float command_list[]){
  int mapped_magnitude;
  for(int i=0;i<ARRAY_LEN;i++){
    if(command_list[i]<0){
      message->directions_to_sent[i]=0;
    }
    if(0<command_list[i]){
      message->directions_to_sent[i]=1;
    }
  }
  for(int i=0;i<ARRAY_LEN;i++){
    mapped_magnitude=command_list[i]*MAPPING_COEFF;
    if(mapped_magnitude<0){
      message->magnitudes_to_sent[i]=(command_list[i]+MAPPING_COEFF)*127/MAPPING_COEFF;
    }
    if(0<mapped_magnitude){
      message->magnitudes_to_sent[i]=(command_list[i]*128/MAPPING_COEFF)+128;
    }
  }
}

void createMsg(DriveMsg *message, float command_list[]){
  uint8_t output_arr_buf[MSG_LEN];
  output_arr_buf[0]=message->start_byte;
  for(int i=0;i<ARRAY_LEN;i++){
    output_arr_buf[i+1]=message->magnitudes_to_sent[i];
  }
  output_arr_buf[MSG_LEN-1]=message->stop_byte;
  message->master_crc=getCRC(output_arr_buf);
  message->output_arr[0]=message->start_byte;
  for(int i=0;i<ARRAY_LEN;i++){
    message->output_arr[i+1]=message->magnitudes_to_sent[i];
  }
  message->output_arr[MSG_LEN-1]=message->stop_byte;
  message->output_arr[MSG_LEN_WCRC-2]=((message->master_crc)>>8) & 0xFF;
  message->output_arr[MSG_LEN_WCRC-1]=(message->master_crc) & 0xFF;
}

void printMsg(DriveMsg *message, HardwareSerial& mySerial){
  for(int i=0;i<MSG_LEN_WCRC;i++){
    mySerial.write(message->output_arr[i]);
    delay(1);
  }
}

uint16_t getCRC(byte buf[MSG_LEN + 2])
{
    unsigned int crc = 0xFFFF;
    for (int pos = 0; pos < MSG_LEN-1; pos++)
    {
        crc ^= (unsigned int)buf[pos];
        for (int i = 8; i != 0; i--)
        {
            if ((crc & 0x0001) != 0)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}
#endif

// S xxx yyy F crc_high crc_low
//the code generates crc according to S xxx yyy F interval
