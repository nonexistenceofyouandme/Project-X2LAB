#define NUM_SENSORS 8
#define IN1_L 8   // Left motor forward pin
#define IN2_L 7   // Left motor backward pin
#define PWM_L 6   // Left motor speed control (PWM)
#include<Servo.h>
// Motor R pins
#define FAN 3
#define IN1_R 11  // Right motor forward pin
#define IN2_R 10   // Right motor backward pin
#define PWM_R 9   // Right motor speed control (PWM)
Servo ESC;
int LastError;


uint16_t MinValue[NUM_SENSORS] = {70, 70, 70, 70, 70, 70, 115, 120}; 
uint16_t MaxValue[NUM_SENSORS] = {640, 640, 640, 640, 640, 640, 640, 640};

uint16_t F[NUM_SENSORS];
int LTurnSpdL, LTurnSpdR, TurnDelayL;
int RTurnSpdL, RTurnSpdR, TurnDelayR;
int LineColor = 0;
int Toturndelay;



void setup() {
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(PWM_L, OUTPUT);

  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);
  pinMode(PWM_R, OUTPUT);

  pinMode(2, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);

  // Code Below

  Serial.begin(9600);
   ESC.attach(FAN);
   ESC.writeMicroseconds(1000);
  WaitSW1();
  ESC.writeMicroseconds(1400);
  delay(1000);
  Motor(40,40);
  delay(200);
  TrackCrossC(50,0.015,0.3,'p');

   TrackTime(80,0.03,0.7, 1000);
  TrackTime(70,0.03,0.5,5500);
  TrackCrossC(40,0.015,0.3,'s');
  TurnLeft();
  TrackCrossC(100,0.032,0.7,'p');
  TrackCrossC(100,0.032,0.7,'p');
  TrackCrossC(100,0.032,0.7,'p');
  TrackCrossC(100,0.032,0.7,'p');
  TrackCrossC(100,0.032,0.7,'p');
  TrackCrossC(100,0.032,0.7,'p');
   TrackCrossC(100,0.032,0.7,'p');
   TrackCrossC(40,0.015,0.3,'s');

 
  TurnRight();
  delay(100);
  TurnLeft();
  TrackCrossC(100,0.032,0.7,'p');
  TrackCrossC(100,0.032,0.7,'p');
  TrackCrossC(100,0.032,0.7,'p');
  TrackCrossC(100,0.032,0.7,'p');
  TrackCrossC(100,0.032,0.7,'p');
  TrackCrossC(100,0.032,0.7,'p');
   TrackCrossC(100,0.032,0.7,'p');
   TrackCrossC(40,0.015,0.3,'s');
  
  TurnRight();
  delay(100);
  TurnLeft();
   TrackCrossC(100,0.032,0.7,'p');
  TrackCrossC(100,0.032,0.7,'p');
  TrackCrossC(100,0.032,0.7,'p');
  TrackCrossC(100,0.032,0.7,'p');
  TrackCrossC(100,0.032,0.7,'p');
  TrackCrossC(100,0.032,0.7,'p');
   TrackCrossC(100,0.032,0.7,'p');
   TrackCrossC(40,0.015,0.,'s');
    TurnRight();
    delay(100);
  TurnLeft();
  TrackCrossC(100,0.032,0.7,'p');
  TrackCrossC(100,0.032,0.7,'p');
  TrackCrossC(100,0.032,0.7,'p');
  TrackCrossC(100,0.032,0.7,'p');
  TrackCrossC(100,0.032,0.7,'p');
  TrackCrossC(100,0.032,0.7,'p');
   TrackCrossC(100,0.032,0.7,'p');
   
   TrackCrossC(40,0.015,0.3,'s');
  TurnRight();
  Motor(40,40);
  delay(100);
  TurnLeft();
     TrackCrossL(50,0.02,0.3,'p');
     TurnRight();
    delay(100);
    TurnLeft();
    TrackTime(50,0.01,0.3,'p');
    Motor(50,50);
    delay(100);

     TrackTime(0,0.02,0.3,200);
      TrackCrossC(100,0.032,0.7,'p');
     TrackCrossC(100,0.032,0.7,'p');
    TrackCrossC(100,0.032,0.7,'p');
    
      TrackTime(100,0.03,0.8,500);
     TrackCrossC(50,0.015,0.5,'s');
      TurnLeft();
//     // TurnLeft();
 TrackTime(100,0.03,0.8,500);
 TrackCrossC(50,0.015,0.5,'s');
  ESC.writeMicroseconds(1000);
  // TrackTime(0,0.02,0.3,200);
  Stop();

}

void loop() {
  // ReadLight();
}

// ||-----------------Functions Lib-----------------||

void WaitSW1() {
  while (true) {
    if (digitalRead(2) == LOW) {  
      delay(50);                  
      if (digitalRead(2) == LOW) { 
        break;                   
      }
    }
    delay(10); 
  }
}
void WaitSW2() {

  while (true) {
    if (digitalRead(4) == LOW) {  
      delay(50);                  
      if (digitalRead(4) == LOW) { 
        break;                   
      }
    }
    delay(10); 
  }
}

void ReadSensor() {
 for (uint8_t i = 0; i < NUM_SENSORS; i++) {
  F[i] = analogRead(A7 - i);  // Reversed: A7, A6, ..., A0
}

}

void ReadLight(){
 for (uint8_t i = 0; i < NUM_SENSORS; i++) {
  Serial.print(analogRead(A7 - i));
  Serial.print("\t");
}
Serial.print("\n");
}

void ReadCalibrate() {
  ReadSensor();
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    uint16_t calMin = MinValue[i];
    uint16_t calMax = MaxValue[i];
    int x = map(F[i], calMin, calMax, 0, 1000);
    x = constrain(x, 0, 1000);
    if (LineColor == 0) {
      F[i] = x;
    } else {
      F[i] = 1000 - x;
    }
  }
}

void Motor(int LeftSpeed, int RightSpeed) {
  MotorL(LeftSpeed);
  MotorR(RightSpeed);
}
 
void MotorL(int Speed) {
  int pwm = abs(Speed) * 255 / 100;  // Scale 0~100 to 0~255
  pwm = constrain(pwm, 0, 255);      // Safety
  
  if (Speed > 0) {
    digitalWrite(IN1_L, HIGH);
    digitalWrite(IN2_L, LOW);
    analogWrite(PWM_L, pwm);
  } else if (Speed < 0) {
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, HIGH);
    analogWrite(PWM_L, pwm);
  } else { // Speed == 0
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, LOW);
    analogWrite(PWM_L, 0);
  }
}

void MotorR(int Speed) {
  int pwm = abs(Speed) * 255 / 100;  // Scale 0~100 to 0~255
  pwm = constrain(pwm, 0, 255);      // Safety
  
  if (Speed > 0) {
    digitalWrite(IN1_R, HIGH);
    digitalWrite(IN2_R, LOW);
    analogWrite(PWM_R, pwm);
  } else if (Speed < 0) {
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, HIGH);
    analogWrite(PWM_R, pwm);
  } else { // Speed == 0
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, LOW);
    analogWrite(PWM_R, 0);
  }
}



void Stop() {
  Motor(0, 0);
  delay(10);
}


void TrackLineColor(int Col) {
  LineColor = Col;
}

void ToTurnDelay(int deleaytoturn) {
  Toturndelay = deleaytoturn;
}

void TurnSpeedLeft(int l, int r, int de) {
  LTurnSpdL = l;
  LTurnSpdR = r;
  TurnDelayL = de;
}

void TurnSpeedRight(int l, int r, int de) {
  RTurnSpdL = l;
  RTurnSpdR = r;
  TurnDelayR = de;
}


void TurnLeft() {
  Motor(-50, 65);
  delay(100);
  while (1) {
    Motor(-50, 65);
    ReadCalibrate();
    if (F[2] >= 500) {
      Motor(-LTurnSpdL, LTurnSpdR);
      delay(5);
      Stop();
      break;
    }
  }
}

void TurnRight() {
  Motor(65, -50);
  delay(100);
  while (1) {
    Motor(65, -50);
    ReadCalibrate();
    if (F[6] >= 500) {
      Motor(RTurnSpdL, -RTurnSpdR);
      delay(5);
      Stop();
      break;
    }
  }
}

int readPosition(int Track, int noise) {
  unsigned long avg = 0;
  unsigned int sum = 0;
  static int last_value = 0;
  bool online = false;

  ReadCalibrate();

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (F[i] > Track) online = true;
    if (F[i] > noise) {
      avg += (unsigned long)F[i] * ((NUM_SENSORS - 1 - i) * 1000);
      sum += F[i];
    }
  }

  if (!online) {
    return (last_value < (NUM_SENSORS - 1) * 1000 / 2) ? 0 : (NUM_SENSORS - 1) * 1000;
  }
  last_value = avg / sum;
  return last_value;
}


void PID(int Speed, float Kp, float Kd) {
  int Pos = readPosition(250, 50);
  int Error = 3500 - Pos;
  int PID_Value = (Kp * Error) + (Kd * (Error - LastError));
  LastError = Error;
  int LeftPower = Speed + PID_Value;
  int RightPower = Speed - PID_Value;
  if (LeftPower > 100) LeftPower = 100;
  if (LeftPower < 0) LeftPower = -30;
  if (RightPower > 100) RightPower = 100;
  if (RightPower < 0) RightPower = -30;
  Motor(LeftPower, RightPower);
}
void TR(){
  
}
void TrackSelect(int spd, char x) {
  if (x == 's') {
    Stop();
    delay(20);
  } else if (x == 'p') {
    Motor(spd, spd);
    delay(30);
    ReadCalibrate();
    while (1) {
      Motor(spd, spd);
      ReadCalibrate();
      if (F[0] < 500 && F[7] < 500) {
        break;
      }
    }
  } else if (x == 'l') {
    Motor(spd, spd);
    delay(Toturndelay);
    TurnLeft();
  } else if (x == 'r') {
    Motor(spd, spd);
    delay(Toturndelay);
    TurnRight();
  }
}


void TrackTime(int Speed, float Kp, float Kd, int TotalTime) {
  unsigned long StartTime = millis();
  unsigned long EndTime = StartTime + TotalTime;
  while (millis() <= EndTime) {
    PID(Speed, Kp, Kd);
  }
}


void TrackCross(int Speed, float Kp, float Kd, char select) {
  while (1) {
    PID(Speed, Kp, Kd);
    ReadCalibrate();
    if ((F[2] > 550 && F[5] > 550) || (F[0] > 550 && F[2] > 550) || (F[5] > 550 && F[7] > 550)) {
      break;
    }
  }
  TrackSelect(Speed, select);
}

void TrackCrossC(int Speed, float Kp, float Kd, char select) {
  while (1) {
    PID(Speed, Kp, Kd);
    ReadCalibrate();
    if ((F[2] > 500 && F[5] > 500)) {
      break;
    }
  }
  TrackSelect(Speed, select);
}

void TrackCrossR(int Speed, float Kp, float Kd, char select) {
  while (1) {
    PID(Speed, Kp, Kd);
    ReadCalibrate();
    if ((F[5] > 500 && F[7] > 500)) {
      break;
    }
  }
  TrackSelect(Speed, select);
}

void TrackCrossL(int Speed, float Kp, float Kd, char select) {
  while (1) {
    PID(Speed, Kp, Kd);
    ReadCalibrate();
    if ((F[7] > 500 && F[6] > 500)) {
      break;
    }
  }
  TrackSelect(Speed, select);
}
