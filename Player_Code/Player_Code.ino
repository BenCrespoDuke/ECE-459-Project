//slave
#define button 8
#define grav 9.807
#define USE_TIMER_1     true
#include "TimerInterrupt.h"
#define TIMER1_INTERVAL_MS    100
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
enum hand_state{ret,ext_a,ext_d,ext,ret_1,ret_a,ret_d};
enum hand_state right_hand_state; // assuming we start with hand retracted

Adafruit_ICM20948 body;
Adafruit_ICM20948 right_hand;
int toggle = 0;


float angX_body = 0.0;
float angY_body = 0.0;
float angZ_body = 0.0;

float velX_right_hand = 0.0;
float velY_right_hand = 0.0;
float velZ_right_hand = 0.0;

float mag_magnitude;
float mag_z_comp;
float mag_x_comp;
float mag_y_comp;

float accel_y_comp;
float accel_z_comp;
float accel_x_comp;
float accel_z_avg;

float accel_z_past_raw[4] = {0,0,0,0};
float accel_z_past_in[4] = {0,0,0,0};
float accel_z_past_out[4] = {0,0,0,0};
float punch_iir_a[5] = {1,-3.26231,4.04702664,-2.2564272,0.4762797};
float punch_iir_b[5] = {0.00028314, 0.00113258, 0.00169887, 0.00113258, 0.00028314};
char bluetooth_data[7] = {0,"hello",0};

union conv32
{
    uint32_t u32; // here_write_bits
    float    f32; // here_read_float
};

/*float dot_prod(float vec1[],float vec2[]){
  float sum = 0.0;
  int len = 10;
  //Serial.println(sizeof(vec1));
  for(int i = 0; i<len; i++){
    sum = sum+vec1[i]*vec2[i];
  }
  
  return sum;
}*/

float irr_filt(float input, float past_in[], float past_out[],float a[],float b[],int order){
  float sum = b[0]*input;
  for(int i = 0; i<order; i++){
    sum = sum+b[i+1]*past_in[i]-a[i+1]*past_out[i];
  }
  
  return (sum * (1/a[0]));
}

SoftwareSerial bluetooth(3,2);
int state = 20;
int buttonState = 0;





void imu_calcs(){
  for(int i = 0; i<10;i++){
    sensors_event_t accel;
    sensors_event_t gyro_body;
    sensors_event_t mag;
    sensors_event_t temp;
    //Serial.println("waiting for data");
    right_hand.getEvent(&accel, &gyro_body, &temp, &mag);
    //Serial.println("Recived data");
    mag_magnitude = sqrt(mag.magnetic.x*mag.magnetic.x+mag.magnetic.y*mag.magnetic.y+mag.magnetic.z*mag.magnetic.z);
    mag_z_comp = mag.magnetic.z/mag_magnitude;
    mag_x_comp = mag.magnetic.x/mag_magnitude;
    mag_y_comp = mag.magnetic.y/mag_magnitude;

    //accel_x_comp = accel.acceleration.x+mag_x_comp*(grav);
    //accel_y_comp = accel.acceleration.y-mag_y_comp*(grav);
    accel_z_avg = (accel.acceleration.z*0.2+accel_z_past_raw[0]*.2+accel_z_past_raw[1]*.2+accel_z_past_raw[2]*.2+accel_z_past_raw[3]*.2);
    accel_z_comp = accel.acceleration.z-accel_z_avg;
    accel_z_past_raw[3] = accel_z_past_raw[2];
    accel_z_past_raw[2] = accel_z_past_raw[1];
    accel_z_past_raw[1] = accel_z_past_raw[0];
    accel_z_past_raw[0] = accel.acceleration.z;
    
    
    
    buttonState = digitalRead(button);
    

    float decsion_num = irr_filt(accel_z_comp, accel_z_past_in, accel_z_past_out,punch_iir_a,punch_iir_b,4);
    accel_z_past_out[3] =  accel_z_past_out[2];
    accel_z_past_out[2] =  accel_z_past_out[1];
    accel_z_past_out[1] =  accel_z_past_out[0];
    accel_z_past_out[0] =  decsion_num;
    
    accel_z_past_in[3] =  accel_z_past_in[2];
    accel_z_past_in[2] =  accel_z_past_in[1];
    accel_z_past_in[1] =  accel_z_past_in[0];
    accel_z_past_in[0] =  accel_z_comp;
    if(buttonState  == HIGH){
      Serial.print(accel.acceleration.x);
      Serial.print(","); 
    }
    // Right Hand Decsion tree
    if(decsion_num < -5.0){
      if(right_hand_state == ret){
        right_hand_state =  ext_a;
       Serial.println(right_hand_state);
      } if(right_hand_state == ext){
        right_hand_state =  ret_1;
        Serial.println(right_hand_state);
      }
    } else if(decsion_num < -3){
      if(right_hand_state == ret_a){
        right_hand_state =  ret_d;
        Serial.println(right_hand_state);
      }
    } else if(decsion_num > 5.0){
      if(right_hand_state == ext_a){
        right_hand_state =  ext_d;
       Serial.println(right_hand_state);
      } if(right_hand_state == ret_1){
        right_hand_state =  ret_a;
        Serial.println(right_hand_state);
      } 
    
    } else if(abs(decsion_num) < 3){
        if(right_hand_state == ext_d){
          right_hand_state =  ext;
          Serial.println(right_hand_state);
        } else if(right_hand_state == ret_d){
          right_hand_state =  ret;
          Serial.println(right_hand_state);
        }
      }
    
    delayMicroseconds(888);
  }
   
  
  
}

void send_movement_data(){
  if(bluetooth.available() > 0){
    state = bluetooth.read();
    Serial.print("Reading \n");
  }
  
  bluetooth_data[0] = 0x0A;
 // bluetooth_data[1] = bluetooth_data[1]+1;
  //Serial.print(byte(bluetooth_data[1]));
  //Serial.println("");
  bluetooth_data[6] = 0xFF;
  
  String data = bluetooth_data;
  if(toggle == 1){
    bluetooth.write('A');
    toggle = 0;
  } else {
    bluetooth.write('B');
    toggle = 1;
  }
  float test = 5.58;
  union conv32 x = {.f32 = test};
  
  bluetooth.write((x.u32 >> 24 & 0xFF));
  bluetooth.write((x.u32 >> 16 & 0xFF));  
  bluetooth.write((x.u32 >> 8 & 0xFF));  
  bluetooth.write((x.u32 >> 0 & 0xFF));  
    bluetooth.write('E');
  

  
}




void setup() {
  right_hand_state = ret;
  pinMode(3,INPUT);
  pinMode(2,OUTPUT);
  pinMode(button, INPUT);
  
  bluetooth.begin(38400);
  Serial.begin(38400);

  ITimer1.init();
  ITimer1.attachInterruptInterval(TIMER1_INTERVAL_MS, send_movement_data);
  
  if (bluetooth.isListening()) { 
         Serial.println("Bluetooth is listening!");
     }

     if(!body.begin_I2C(0x69,&Wire)){
    Serial.print("Body Not Detected\n");
  } else{
    Serial.print("Body Detected and Starting\n");
  }

 if(!right_hand.begin_I2C(0x68,&Wire)){
    Serial.print("Right Hand Not Detected\n");
  } else{
    Serial.print("Right Hand Detected and Starting\n");
  }
  right_hand.setAccelRateDivisor(0);
  uint16_t accel_divisor = right_hand.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);

  Serial.print("Accelerometer data rate divisor set to: ");
  Serial.println(accel_divisor);
  Serial.print("Accelerometer data rate (Hz) is approximately: ");
  Serial.println(accel_rate);
  
}








void loop() {
  //bluetooth.print("AT\r\n");
  
  
 /* buttonState = digitalRead(button);
  if(buttonState == HIGH) {
    bluetooth.write('1');
    //Serial.write("1\n");
  }
  else {
    bluetooth.write('0');
    //Serial.write("0\n");
  }
  //char* trans_data = {angX_body,angY_body,angZ_body};
  char trans_data[12] = {'H','e','l','l','0'};
  bluetooth.write(trans_data);*/
  imu_calcs();
  //Serial.print(velX_right_hand);

}
