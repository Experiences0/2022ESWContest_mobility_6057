#include <SoftwareSerial.h>
#include "MedianFilterLib.h"

#define ANGLE_PIN 4
#define BUZZER 12

MedianFilter<float> medianFilter(5);
MedianFilter<float> medianFilter_angle(3);


SoftwareSerial UltraSonic(11,10); // RX, TX
unsigned char data[4]={};
float distance;
float Filtered_distance;
float Filtered_angle;
float angle;

uint32_t start_tick=0;
uint8_t flag=0;
uint8_t Sound_toggle=1;
uint8_t toggle_flag=0;


void setup() {
  Serial.begin(115200);
  UltraSonic.begin(9600); 
  
  Serial.println("HI");
  pinMode(ANGLE_PIN,INPUT);
  pinMode(BUZZER, OUTPUT);
}

void loop() {
  
  unsigned long value = pulseIn(ANGLE_PIN,HIGH); 
  //long angle=map_long(value,263,8255,0,360);
  angle=map_long(value,360,2586,-35,35);
 
  get_Distance();
  Median_Filter();
  Send_Data();
  
   delay(30);
  
  if(Serial.available()){
    char data = Serial.read();
    if(data == 'a'){
      flag=1;
    }
    else if(data == 'b'){
      flag=2;
    }
    else if(data == 'c'){
      flag=3;
    }
    else if(data == 'd'){
      flag=4;
    }
  }

  
  delay(25);
  
  if(flag==1){
    Sound(500, 500);
  }
  else if(flag==2){
    Sound(100, 100);
  }
  else if(flag==3){
    digitalWrite(BUZZER,HIGH);
  }
  else{
    digitalWrite(BUZZER, 0);
  }

  
  

}
void Send_Data(){
  Serial.print(Filtered_distance);
  Serial.print(',');
  Serial.println(Filtered_angle);
}
long map_long(long x, long in_min, long in_max, long out_min, long out_max) {
  long x_out=(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return x_out;
}

void get_Distance(){
  do{
     for(int i=0;i<4;i++)
     {
       data[i]=UltraSonic.read();
     }
  }while(UltraSonic.read()==0xff);

  UltraSonic.flush();

  if(data[0]==0xff)
    {
      int sum;
      sum=(data[0]+data[1]+data[2])&0x00FF;
      if(sum==data[3])
      {
        distance=(data[1]<<8)+data[2];
      }
      
     }
}


void Median_Filter(){
  Filtered_distance=medianFilter.AddValue(distance);
  Filtered_angle=medianFilter_angle.AddValue(angle);
}

void Sound(uint32_t delay_time_on, uint32_t delay_time_off){
  if(toggle_flag==0){

    if(millis()-start_tick>delay_time_on){
       start_tick=millis();
       Sound_toggle=!Sound_toggle;
       toggle_flag=!toggle_flag;
       digitalWrite(BUZZER,Sound_toggle);
    }  
    
  }
  else{

    if(millis()-start_tick>delay_time_off){
       start_tick=millis();
       Sound_toggle=!Sound_toggle;
       toggle_flag=!toggle_flag;
       digitalWrite(BUZZER,Sound_toggle);
    }
  }
  
    
}
