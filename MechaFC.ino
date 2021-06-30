/* Mechatronics Floor Cleaner
By Eddy Martínez, 1053420.
Instituto Tecnógico de Santo Domingo */

//#include <Wire.h>
#include <Event.h>
#include <Timer.h>

// Button ports
#define Start 40
#define Stop 42
#define Bluetooth 44
#define MetalDetector 46

//LEDs ports
#define LStart 51
#define LBluetooth 50
#define LMetalDetector 49

//Bluetooth ports
#define BtState 14
#define BtEnable 13

//Sensor ports
#define Trigger 36
#define EchoF1 34
#define EchoF2 32
#define EchoL 38
#define EchoR 30
#define EchoT 28
#define Metal 48
#define EncoderLA 18
#define EncoderLB 19
#define EncoderRA 20
#define EncoderRB 21

//Motor ports
#define MotorL1 3
#define MotorL2 4
#define MotorLen 2
#define MotorR1 5
#define MotorR2 6
#define MotorRen 7
#define MotorB 9
#define MotorBen 8
#define Valve 11
#define MotorV 10
#define MotorS 12
#define MotorSen 13

//Constant times in milliseconds
#define button 500
#define BombOn 2000
#define BombPause 60000
#define ahead 500

//Ultrasonic
#define Distance 600
#define USTime 60
#define F1 0
#define F2 1
#define L 2
#define R 3
#define T 4

//Constant PWMs
#define BombPWM 255
#define VPWM 255
#define SPWM 180
#define drag 25

//Pulses per PTime seconds at 255 PWM
#define LPulses 1854
#define RPulses 3675
#define PTime 1000

//MPU6050
#define MPU 0x68 //MPU6050 I2C address
#define accelAdd 0x3B //Acceleration address
#define gyroAdd 0x43 //Angular velocity address
#define CountError 200
#define MPUtime ((MPUact-MPUprev)/1000)
#define x 0
#define y 1
#define z 2
#define yaw 10
#define forward (abs(angle[z]-0)<yaw)
#define left (abs(angle[z]-(-90))<yaw)
#define right (abs(angle[z]-90)<yaw)

bool Over=0, FC_ON=1, AutoManual=1, Sponge=0, Vacuum=0, Level=0, rside=0, lside=0;
bool Forward=0, Back=0, Left=0, Right=0, TManual=0, TBomb=0, TMotion=0, US=0;
unsigned char  LPWM=0, RPWM=0, BMD=0, Timep=0, Time=0, Speed=0, UScount=0, motion=0;
const unsigned char Echo[]={EchoF1, EchoF2, EchoL, EchoR, EchoT};
short distance[]={0,0,0,0,0}, pulseL, pulseR, pulsesL, pulsesR;
short MPUact, MPUprev, TManual0, TManual1, TBomb0, TBomb1, TMotion0, TMotion1, TUS0, TUS1;
float A[]={0,0,0}, G[]={0,0,0}, AError[]={0,0}, GError[]={0,0,0};
float angle[]={0,0,0}, angleA[]={0,0}, angleG[]={0,0,0};
int TimeEvent, MetalEvent, LevelEvent, OverEvent, EncoderEvent;
Timer t;

void TimeToStart(){
  Time--;

  if(Time==0){
    AutoManual = 1;
    FC_ON = 1;
    t.stop(TimeEvent);
  }
}

void BluetoothRX() {
  int data=0;
  
  if(Serial.available()){
    data = Serial.read();

    switch(data){
      case '0':
       break;

      case '1':
       AutoManual = !AutoManual;
       motion = 0;
       break;

      case '2':
       FC_ON = 1;
       break;

      case '3':
       FC_ON = 0;
       break;

      case '4':
       break;

      case '5':
       break;

      case '6':
       Sponge = !Sponge;
       break;

      case '7':
       Vacuum = !Vacuum;
       break;

      case '8':
       Forward = !Forward;
       Back = 0;
       Left = 0;
       Right = 0;
       break;

      case '9':
       Forward = 0;
       Back = !Back;
       Left = 0;
       Right = 0;
       break;

      case 'a':
       Forward = 0;
       Back = 0;
       Left = !Left;
       Right = 0;
       break;

      case 'b':
       Forward = 0;
       Back = 0;
       Left = 0;
       Right = !Right;
       break;

      case 'c':
       if(Time>0) t.stop(TimeEvent);
       delay(100);
       Time = Serial.read();
       TimeEvent = t.every(1000, TimeToStart);
       break;

      case 'd':
       delay(100);
       Speed = Serial.read();
       Serial.write('d');
       Serial.write(Speed);
       break;
    }
  }
}

void StateInd() {
  if(!Over){
    t.stop(OverEvent);
    digitalWrite(LStart, FC_ON);
  }
  else OverEvent = t.oscillate(LStart, 500, HIGH);
  
  if(Level){
    t.stop(LevelEvent);
    digitalWrite(LBluetooth, (!digitalRead(Metal)));
  } else LevelEvent = t.oscillate(LBluetooth, 500, HIGH);
  
  if(Timep!=Time){
    Timep = Time;
    Serial.write('c');
    Serial.write(Time);
  }

  if(distance[T] > 90){
    Level = 0;
  } else {
    Level = 1;
  }
}

void Ultrasonic() {
  unsigned short sonic;
  
  if(!US){
    US = 1;
    TUS0 = millis();
  } else {
    TUS1 = millis();
  
    if(TUS1-TUS0 >= USTime){
      US = 0;
      if(UScount > 4) UScount = 0;
      else{
        digitalWrite(Trigger, HIGH);
        delayMicroseconds(10);
        digitalWrite(Trigger, LOW);
        sonic = pulseIn(Echo[UScount], HIGH);
        sonic /= 5.9;
        if(sonic > 50) distance[UScount] = sonic;
        /*Serial.print("\n");
        Serial.print(UScount);
        Serial.print("\t");
        Serial.print(distance[UScount]);
        delay(1000);*/
        //Serial.print("   \t Metal: "); Serial.print(digitalRead(Metal));
        UScount++;
      }  
    }
  }
}

void MPUbegin(){
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
}

void MPUrequest(int address){
  Wire.beginTransmission(MPU);
  Wire.write(address);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
}

void MPUaccel(){
  MPUrequest(accelAdd);

  for(char axis=x; axis<=z; axis++)
    A[axis] = (Wire.read() << 8 | Wire.read());

    angleA[x] = (atan((A[y])/sqrt(pow(A[x],2) + pow(A[z],2))))*180/PI;
    angleA[y] = (atan((-A[x])/sqrt(pow(A[y],2) + pow(A[z], 2))))*180/PI;
}

void MPUgyro(){
  MPUrequest(gyroAdd);

  for(char axis=x; axis<=z; axis++)
    G[axis] = (Wire.read() << 8 | Wire.read())/131;
}

void MPUerror(){
  for(short count=0; count<CountError; count++){
    MPUaccel();
    MPUgyro();
    
    AError[x] += angleA[x];
    AError[y] += angleA[y];
    GError[x] += G[x];
    GError[y] += G[y];
    GError[z] += G[z];
  }

  AError[x] /= CountError;
  AError[y] /= CountError;
  GError[x] /= CountError;
  GError[y] /= CountError;
  GError[z] /= CountError;
}

void MPUangle(){
  int factor;
  MPUaccel();
  MPUgyro();
  MPUprev = MPUact;
  MPUact = millis();
  
  G[x] -= GError[x];
  G[y] -= GError[y];
  G[z] -= GError[z];
  angleA[x] -= AError[x];
  angleA[y] -= AError[y];
  angleG[x] += G[x]*MPUtime;
  angleG[y] += G[y]*MPUtime;
  angleG[z] += G[z]*MPUtime;

  angle[x] = 0.96*angleG[x] + 0.04*angleA[x];
  angle[y] = 0.96*angleG[y] + 0.04*angleA[y];
  angle[z] = angleG[z];

  for(char axis=x; axis<=z; axis++){
    factor = angle[axis]/360;
    angle[axis] -= factor*360;
  }
}

void EncoderL() { 
  if ( digitalRead(EncoderLB) == 0 ) {
    if ( digitalRead(EncoderLA) == 0 ) {
      // A fell, B is low
      pulseL--; // Moving forward
    } else {
      // A rose, B is high
      pulseL++; // Moving reverse
    }
  } else {
    if ( digitalRead(EncoderLA) == 0 ) {
      pulseL++; // Moving reverse
    } else {
      // A rose, B is low
      pulseL--; // Moving forward
    }
  }
}

void EncoderR() { 
  if ( digitalRead(EncoderRB) == 0 ) {
    if ( digitalRead(EncoderRA) == 0 ) {
      // A fell, B is low
      pulseR--; // Moving forward
    } else {
      // A rose, B is high
      pulseR++; // Moving reverse
    }
  } else {
    if ( digitalRead(EncoderRA) == 0 ) {
      pulseR++; // Moving reverse
    } else {
      // A rose, B is low
      pulseR--; // Moving forward
    }
  }
}*/

void Encoder() {
  Serial.print("\nLeft pulses: "); Serial.print(abs(pulseL));
  Serial.print("   \tRight pulses: "); Serial.print(abs(pulseR));
  pulsesL = abs(pulseL);
  pulsesR = abs(pulseR);
  pulseL = 0;
  pulseR = 0;
}

void Motor (bool L1, bool L2, bool R1, bool R2, int Lpwm, int Rpwm) {
  if(((L1!=0)||(L2!=0))||((R1!=0)||(R2!=0))) TManual = 0;
  
  digitalWrite(MotorL1, L1);
  digitalWrite(MotorL2, L2);
  analogWrite(MotorLen, Lpwm);
  
  digitalWrite(MotorR1, R1);
  digitalWrite(MotorR2, R2);
  analogWrite(MotorRen, Rpwm);
}

void MotorStop () {Motor(0, 0, 0, 0, 0, 0);}
void MoveForward (int Lpwm, int Rpwm) {Motor(1, 0, 1, 0, Lpwm, Rpwm);}
void MoveBack (int Lpwm, int Rpwm) {Motor(0, 1, 0, 1, Lpwm, Rpwm);}
void TurnLeft (int Lpwm, int Rpwm) {Motor(0, 1, 1, 0, Lpwm, Rpwm);}
void TurnRight (int Lpwm, int Rpwm) {Motor(1, 0, 0, 1, Lpwm, Rpwm);}

void Cleaner(){
  digitalWrite(MotorV, Vacuum);
  digitalWrite(MotorS, Sponge);
  analogWrite(MotorSen, SPWM);
  
  if(Sponge){
    if(!TBomb){
      TBomb = 1;
      TBomb0 = millis();
    } else {
      TBomb1 = millis();

      if(TBomb1-TBomb0 > BombPause){
        analogWrite(MotorBen, BombPWM);
        t.pulse(Valve, BombOn, HIGH);
        t.pulse(MotorB, BombOn, HIGH);
        TBomb = 0;
      }
    }
  } else {
    TBomb = 0;
    analogWrite(MotorBen, 0);
    digitalWrite(Valve, LOW);
    digitalWrite(MotorB, LOW);
  }
}

void Robot(){
  switch(motion){
    case 0:
      if(!forward) TurnLeft(Speed, Speed);
      else {
        MotorStop();
        motion = 1;
      }

      break;
      
    case 1:
      if(!forward) motion = 0;
      else if(distance[L] > Distance) motion = 2;
      else if(distance[R] > Distance) motion = 8;
      else motion = 7;
      break;
      
    case 2:
      if(!left) TurnLeft(Speed, Speed);
      else {
        MotorStop();
        motion = 3;
      }

      break;
      
    case 3:
      if(!left) motion = 2;
      else motion = 4;
      break;
      
    case 4:
      if((distance[F1] > Distance) && (distance[F2] > Distance)) MoveForward(Speed, Speed-drag);
      else {
        MotorStop();

        if((lside&&(!rside)) && (distance[R] <= Distance)) Over = 1;
        else motion = 5;
      }

      break;
      
    case 5:
      if(!forward) TurnRight(Speed, Speed);
      else {
        MotorStop();
        motion = 6;
      }

      break;
      
    case 6:
      if(!forward) motion = 5;
      else motion = 7;
      break;
      
    case 7:
      MoveForward(Speed,Speed-drag);
      
      if(!TMotion){
        TMotion = 1;
        TMotion0 = millis();
      } else {
        TMotion1 = millis();

        if(TMotion1-TMotion0 >= ahead){
          MotorStop();
          TMotion = 0;
          motion = 1;
        }
      }

      if((distance[F1] <= Distance) || (distance[F2] <= Distance)){
        MotorStop();
        TMotion = 0;
        motion = 1;

        if(distance[L] > Distance) lside = 1;
        if(distance[R] > Distance) rside = 1;
      }

      break;
      
    case 8:
      if(!right) TurnRight(Speed, Speed);
      else {
        MotorStop();
        motion = 9;
      }

      break;
      
    case 9:
      if(!right) motion = 8;
      else motion = 10;
      break;
      
    case 10:
      if((distance[F1] > Distance) && (distance[F2] > Distance)) MoveForward(Speed, Speed-drag);
      else {
        MotorStop();

        if(rside && (distance[L] <= Distance)) Over = 1;
        else motion = 11;
      }

      break;
      
    case 11:
      if(!forward) TurnLeft(Speed, Speed);
      else {
        MotorStop();
        motion = 12;
      }

      break;
      
    case 12:
      if(!forward) motion = 11;
      else motion = 7;
      break;
  }
}

void setup() {
// put your setup code here, to run once:
  Serial.begin(9600);
  MPUbegin();
  MPUerror();
  
// Button ports
  pinMode(Start, INPUT);
  pinMode(Stop, INPUT);
  pinMode(Bluetooth, INPUT);
  pinMode(MetalDetector, INPUT);

//LEDs ports
  pinMode(LStart, OUTPUT);
  pinMode(LBluetooth, OUTPUT);
  pinMode(LMetalDetector, OUTPUT);

//Bluetooth ports
  pinMode(BtState, INPUT);
  pinMode(BtEnable, OUTPUT);

//Sensor ports
  pinMode(Trigger, OUTPUT);
  pinMode(EchoF1, INPUT);
  pinMode(EchoF2, INPUT);
  pinMode(EchoL, INPUT);
  pinMode(EchoR, INPUT);
  pinMode(EchoT, INPUT);
  pinMode(Metal, INPUT);
  pinMode(EncoderLA, INPUT);
  pinMode(EncoderLB, INPUT);
  pinMode(EncoderRA, INPUT);
  pinMode(EncoderRB, INPUT);

//Motor ports
  pinMode(MotorL1, OUTPUT);
  pinMode(MotorL2, OUTPUT);
  pinMode(MotorLen, OUTPUT);
  pinMode(MotorR1, OUTPUT);
  pinMode(MotorR2, OUTPUT);
  pinMode(MotorRen, OUTPUT);
  pinMode(MotorB, OUTPUT);
  pinMode(MotorBen, OUTPUT);
  pinMode(MotorV, OUTPUT);
  pinMode(MotorS, OUTPUT);
  pinMode(MotorSen, OUTPUT);

//Interrupts
  attachInterrupt(digitalPinToInterrupt(EncoderLA), EncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncoderRA), EncoderR, CHANGE);

//Timer functions
  EncoderEvent = t.every(PTime*3, Encoder);
  MPUact = millis();
}

void loop() {
// put your main code here, to run repeatedly:
  MPUangle();
  BluetoothRX();
  Ultrasonic();
  StateInd();
  Cleaner();
  
  if(FC_ON&&AutoManual){
    Sponge = 1;
    Vacuum = 1;
    Speed = 255;
    
    if(!TMotion){
      TMotion = 1;
      TMotion0 = millis();
    } else {
      TMotion1 = millis();

      if(TMotion1-TMotion0 >= 180000){
        FC_ON = 0;
        TMotion = 0;
      }
    }

    if(distance[F1]>Distance&&distance[F2]>Distance){
      MoveForward(Speed,Speed-drag);
    } else if(distance[F1]>Distance){
      TurnLeft(Speed,Speed);
    } else if(distance[F2]>Distance){
      TurnRight(Speed,Speed);
    } else if(distance[L]>Distance){
      TurnLeft(Speed,Speed);
    } else if(distance[R]>Distance){
      TurnRight(Speed,Speed);
    } else {
      MoveBack(Speed,Speed-drag);
    }
  } else if(FC_ON&&Forward) MoveForward(Speed,Speed-drag);
    else if(FC_ON&&Back) MoveBack(Speed,Speed-drag);
    else if(FC_ON&&Left) TurnLeft(Speed,Speed);
    else if(FC_ON&&Right) TurnRight(Speed,Speed);
  else {
    if(!FC_ON) {
      Sponge = 0;
      Vacuum = 0;
    }

    if(!TManual){
      MotorStop();
      TManual = 1;
      TManual0 = millis();
    } else {
      TManual1 = millis();

      if((TManual1-TManual0)>60000){
        AutoManual = 1;
        FC_ON = 0;
        TManual = 0;
      }
    }
  }

  t.update();
}