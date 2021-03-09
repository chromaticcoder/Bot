  #include <NewPing.h>

/********************************************************
   PID Basic Example
   Reading analog input 0 to control analog PWM output 3
 ********************************************************/
NewPing sonar(4, 5, 400);
int ir_f,dist;

#include <PID_v1.h>


//Define Variables we'll be connecting to
double Setpoint, Input, Output;
int flag=1;
//Specify the links and initial tuning parameters
double Kp = 2.97, Ki = 0.5, Kd = 0.33;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int current,prev,var=0;

//MOTOR_CONTROL


class motor {
    const int Motor1Pin1 = 8;
    const int Motor1Pin2 = 9;
    const int Motor2Pin1 = 10;
    const int Motor2Pin2 = 11;

    //Backward
  public: void moveBackward(int vel) {
      digitalWrite(Motor1Pin1, vel);
      digitalWrite(Motor1Pin2, LOW);
      digitalWrite(Motor2Pin1, vel);
      digitalWrite(Motor2Pin2, LOW);
    }

    //Forward
  public: void moveForward(int vel) {
      digitalWrite(Motor1Pin1, LOW);
      digitalWrite(Motor1Pin2, vel);
      digitalWrite(Motor2Pin1, LOW);
      digitalWrite(Motor2Pin2, vel);
    }

    //Rotating the bot
  public: void rotate(int angle) {
      if(angle<0){
      digitalWrite(Motor1Pin1, LOW);
      digitalWrite(Motor1Pin2, HIGH);
      digitalWrite(Motor2Pin1, HIGH);
      digitalWrite(Motor2Pin2, LOW);
    }
    else{
      digitalWrite(Motor1Pin1, HIGH);
      digitalWrite(Motor1Pin2, LOW);
      digitalWrite(Motor2Pin1, LOW);
      digitalWrite(Motor2Pin2, HIGH);
    }
    delay(abs(angle)*10);
  }

    //Stop
  public: void moveStop() {
      digitalWrite(Motor1Pin1, LOW);
      digitalWrite(Motor1Pin2, LOW);
      digitalWrite(Motor2Pin1, LOW);
      digitalWrite(Motor2Pin2, LOW);
    }

  public: void justMove(int vel, strength) {
      digitalWrite(Motor1Pin1, LOW);
      analogWrite(Motor1Pin2, vel);
      digitalWrite(Motor2Pin1, LOW);
      analogWrite(Motor2Pin2, vel-strength);

  public: void Right(int vel, strength) {
      digitalWrite(Motor1Pin1, LOW);
      analogWrite(Motor1Pin2, vel);
      digitalWrite(Motor2Pin1, LOW);
      analogWrite(Motor2Pin2, vel-strength);
    }
  public: void Left(int vel, strength) {
      digitalWrite(Motor1Pin1, LOW);
      analogWrite(Motor1Pin2, vel-strength);
      digitalWrite(Motor2Pin1, LOW);
      analogWrite(Motor2Pin2, vel);
    }
  public: void pidLeft() {
      analogWrite(8, 100-Output);
      analogWrite(10, 100);
      digitalWrite(9, LOW);
      digitalWrite(11, LOW);
     
    }
  public: void pidRight() {
      digitalWrite(8, 100);
      analogWrite(11, 100);
      digitalWrite(9, LOW);
      analogWrite(10, 100-Output);

    }
  public: void pidRight() {
      digitalWrite(8, 100);
      analogWrite(11, 100);
      digitalWrite(9, LOW); 
      analogWrite(10, 100-Output);
    } 

};

motor m;

void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);//initialize the variables we're linked to
  Input = sonar.ping_cm();
  Setpoint = 9;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  pinMode(2, INPUT);
  delay(40);
  current = sonar.ping_cm();
  prev = current;
  
}

void loop()
{
  Input = sonar.ping_cm();
  delay(100);
  current = Input;
  var = abs(current-prev);//to check difference between current and previous for detecting a right turn
  ir_f = digitalRead(2);
  if(ir_f ==1&&var<7)
  {
    m.moveStop();
    delay(100);
    m.rotate(-90);
  }

 
  myPID.Compute();
  m.justMove(Output);
  Serial.print(ir_f);
  Serial.print("ultra");
  Serial.println(Input);
  Serial.print("pid value:");
  Serial.println(Output);
  

  prev = current;
  delay(40);
}
