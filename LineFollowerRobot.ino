#include "QTRSensors.h"

QTRSensors qtr;
const int trig = 9;     // chân trig của SRF-05.
const int echo = 10; 
void dokhoangcach();

int IN1 = 4; 
int IN2 = 5;  // Right motor 

int IN3 = 6; 
int IN4 = 7;  // Left motor 

const uint8_t SensorCount = 5;
int PID_out;
int khoangcach; 
int count;
unsigned long thoigian;
uint16_t sensorValues[SensorCount];
bool manualCalibration = 1;
int  lastError;
void setup()
{
  Serial.begin(9600);
  pinMode(IN1, OUTPUT);  // sets the pin as output
  pinMode(IN2, OUTPUT);  // sets the pin as output
  pinMode(IN3, OUTPUT);  // sets the pin as output
  pinMode(IN4, OUTPUT);  // sets the pin as output
  pinMode(trig,OUTPUT);   // chân trig sẽ phát tín hiệu
  pinMode(echo,INPUT); 
  // configure the sensors
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){A1, A2, A3, A4, A5}, SensorCount);
  qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
  Serial.begin(115200);
if (manualCalibration)
{
  uint16_t minVal[5] = {444,420,416,412,352};
  uint16_t maxVal[5] = {2500,2500,2500,2500,2500};
  qtr.calibrationOn.initialized = true;
  qtr.calibrationOn.minimum = (uint16_t *)realloc(qtr.calibrationOn.minimum,sizeof(uint16_t) * 5);
  qtr.calibrationOn.maximum = (uint16_t *)realloc(qtr.calibrationOn.maximum,sizeof(uint16_t) * 5);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    qtr.calibrationOn.minimum[i] = minVal[i];
    qtr.calibrationOn.maximum[i] = maxVal[i];
  }
}
else
{
  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
}
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  
  // print the calibration minimum values measured when emitters were on

  Serial.print("Minimum: ");
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print('\t');
  }
  Serial.println();
  //print the calibration maximum values measured when emitters were on
  Serial.print("Maximun: ");
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print('\t');
  }

  Serial.println();

  delay(1000);
}
 
void loop()
{
  // if((digitalRead(A1) == 0) && (digitalRead(A2) == 0) && (digitalRead(A3) == 0) && (digitalRead(A4) == 0)&& (digitalRead(A5) == 0)){
   
  //    Go_ahead(200);
  //   Go_ahead(200);
  //   Go_ahead(200);
    
  // }
    

  
 khoangcach=0;
 count = 1;
    dokhoangcach();
    Serial.print("Khoang Cach: ");
    Serial.println(khoangcach);
    if(khoangcach > 13) 
    {
      int rate=0;
      uint16_t position = qtr.readLineWhite(sensorValues);
      int error = position - 2000;
  
      Serial.print(error);
 
      PID_out = 0.10005 * error + 0.0022 *( (error - lastError)/0.01);//0.06 0.0022

      lastError = error;
  
      int Speed0 = 165;
      int Left_Out = max(min(Speed0 + PID_out*(1+rate),255),-255);
      int Right_Out = max(min(Speed0 - PID_out*(1-rate),255),-255);
    if (Left_Out >= 0)  {
      Motor_Left(Left_Out,1);
    }
    else  {
      Motor_Left(abs(Left_Out),0);
    }
    if (Right_Out >=0)  {
      Motor_Right(Right_Out,1);
    }
    else  {
      Motor_Right(abs(Right_Out),0);
    }
  
    delay(10);
    }
    else {
      // if (count <2){  
      Motor_Right(200, 0);
      Motor_Left(0, 0);
      delay(200);
     Motor_Right(100, 1);
     Motor_Left(100,1);
     delay(450);
     Motor_Right(200, 1);
     Motor_Left(0,1);
     delay(600);
     Motor_Right(100, 1);
     Motor_Left(100,1);
     delay(200);
     Motor_Left(100,1);
     delay(200);
     digitalWrite(IN1,LOW);
     digitalWrite(IN2,LOW);
     digitalWrite(IN3,LOW);
     digitalWrite(IN4,LOW);
     delay(5000);
     // count++
      // }
    //  digitalWrite(IN1,LOW);
    // digitalWrite(IN2,LOW);
    // digitalWrite(IN3,LOW);
    // digitalWrite(IN4,LOW);
    // else{
    //  digitalWrite(IN1,LOW);
    //  digitalWrite(IN2,LOW);
    //  digitalWrite(IN3,LOW);
    //  digitalWrite(IN4,LOW);
    // }
    }
  
  }
  // Serial.print("SENSOR VALUE = ");
  // Serial.print(position);
  // Serial.print(" PID OUT = ");
  // Serial.println(PID_out);


void Motor_Right(int turnspeed, bool direction) // 1 forward, 0 reverse
{
  if (direction)
  {
    analogWrite(IN1 , 0);
    analogWrite(IN2 , turnspeed);
  }
  else
  {
    analogWrite(IN1 , turnspeed);
    analogWrite(IN2 , 0);
  }
}

void Motor_Left(int speed, bool direction) // 1 forward, 0 reverse
{
  if (direction)
  {
    analogWrite(IN3 , speed);
    analogWrite(IN4 , 0);
  }
  else
  {
    analogWrite(IN3 , 0);
    analogWrite(IN4 , speed);
  }
}

void Turn_Right(int speed)
{
  Motor_Right(speed,1);
  Motor_Left(speed,0);
}

void Turn_Left(int speed)
{
  Motor_Right(speed,0);
  Motor_Left(speed,1);
}
void Go_ahead(int speed)
{
  Motor_Right(speed,1);
  Motor_Left(speed,1);
}
void dokhoangcach()
{
/* Phát xung từ chân trig */
    digitalWrite(trig,0);   // tắt chân trig
    delayMicroseconds(2);
    digitalWrite(trig,1);   // phát xung từ chân trig
    delayMicroseconds(10);  // xung có độ dài 5 microSeconds
    digitalWrite(trig,0);   // tắt chân trig
    
    /* Tính toán thời gian */
    // Đo độ rộng xung HIGH ở chân echo. 
    thoigian = pulseIn(echo,HIGH);  
    // Tính khoảng cách đến vật.
    khoangcach = int(thoigian/2/29.412);
    

}
//tổng: 8V+, dong quâ LN: 7.6-7.5Vg



