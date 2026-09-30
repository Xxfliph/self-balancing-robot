#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

/*
General idea: intilaize the acceleratmoer, range, and filterbandwith and set their range which dpends on what ur using the mpu 6050 for.
THen create variables for each measuermnt "event" (a, g, temp) and then access the data stored in them using "." and then print it out
*/

Adafruit_MPU6050 mpu;


unsigned long newtime = 0;
unsigned long oldtime = 0;
double oldangle = 0;
double newangle = 0;
const unsigned long loop_period = 10000;
unsigned long last_loop_time = 0;
unsigned long current_time;
double olderror = 0;

int AIN1 = 25;
int AIN2 = 26;
int PWMA = 18;

int BIN1 = 32;
int BIN2 = 33;
int PWMB = 19;


void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("Adafruit MPU6050 test");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found");

/*
This block of code sets the mpu acceloromter range, which the user sets to +-8G. 
Instead of harcoding the user inputted range, the code actually checks what hte range 
is by using the switch, case, break chain. The value to be tested is mpu.getaccelerometer range. 
If its in the rnage 2_G (the first case tests for) then print 2_G, break out of the logic 
chain then go to next block of code. same idea for 4, 8 and 16. Same idea for setgyroscope and filter bandwith
*/

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

/*
Filter bandwith is used to smooth out noise (fast small fluctuations  in raw sensor readings, vibrations, electrical interference)
lower Hz = more smoothing but slower to respond to changes. Higher Hz = less smoothing (noisier) but more responsive to changes
*/
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // or 44_HZ

  
  
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  oldtime = micros();   
}

double integral_sum = 0;

void loop() {
  /* Get new sensor events with the readings 
    a, g and temp are variables of type "sensors_event_t". That condensed line is the same as this:
    sensors_event_t a;
    sensors_event_t g;
    sensors_event_t temp;

    a, g and temp are sort of like python objects, where you use "." to access attributes. They are actually "structs"
    in the c++ language which is like python object but only with attributes and no functions.

    Then use "." to access the data stored in a, g and temp. So a.acceleration.x gets the x acceleration
  */
  
  double angle_accel;
  double angle_gyro;
  double deltatime;
  double setpoint = 0.0;
  double error;
  double P = 0.0;
  double K_p = 10; //will need to adjust these based on behaviour
  double D = 0.0;
  double K_d = 0.7; //will need to adjust these based on behaviour
  double I = 0.0;
  double K_i = 0.0;//will need to adjust these based on behaviour

  current_time = micros();
  if (current_time - last_loop_time >= loop_period) {
    last_loop_time = current_time;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    newtime = current_time;
    deltatime = (double)(newtime - oldtime)/1000000.0;
    if (deltatime <= 0) {
      deltatime = 0.01;
    }
    oldtime = newtime;

    double gyro_reading = g.gyro.y*(180.0/M_PI);

    angle_accel = atan2(a.acceleration.x, -a.acceleration.z)*(180.0/M_PI); //put negative sign before z acceleration to invert it becasue mpu is upside down in the robot. Not tested yet, so may need to fix other parts if this doenst work.
    newangle = (0.98*(oldangle+gyro_reading*deltatime) + 0.02*angle_accel);
    oldangle = newangle;
  

    error = setpoint - newangle;
    P = K_p*error;

    D = K_d*((error - olderror)/deltatime);
    olderror = error;

    integral_sum += error*deltatime;
    integral_sum = constrain(integral_sum, -100.0, 100.0); // can change the -100 100 clamping values according to behaviour. range = pwm/K_i (255/K-i)
    I = K_i*integral_sum;

    double pid_output = P + I + D;

    pid_output = constrain(pid_output, -255.0, 255.0);

    if (abs(error) < 1.0) {
      integral_sum = 0; // Clear stored memory when near setpoint
    }

    int motor_speed = (int)abs(pid_output);
    if (motor_speed > 0) {
      motor_speed = map(motor_speed, 1,255, 50, 255);
    }

    if (abs(newangle) > 45.0) {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW);
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, LOW);
      analogWrite(PWMA, 0);
      analogWrite(PWMB, 0);

      integral_sum = 0;
    }
    else {
      if (pid_output < 0) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        digitalWrite (BIN1, HIGH);
        digitalWrite(BIN2, LOW);

        analogWrite(PWMA, motor_speed);
        analogWrite(PWMB, motor_speed);
      }
      else if (pid_output > 0) {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);

        analogWrite(PWMA, motor_speed);
        analogWrite(PWMB, motor_speed);
      }
      else {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);

        analogWrite(PWMA, 0);
        analogWrite(PWMB, 0);

      }
    }



    

    /*
    int AIN1 = 25;
    int AIN2 = 26;
    int PWMA = 18;

    int BIN1 = 32;
    int BIN2 = 33;
    int PWMB = 19;

    */

    Serial.print("Angle: ");
    Serial.print(newangle);
    Serial.print(" | PID: ");
    Serial.print(pid_output);
    Serial.print(" | PWM: ");
    Serial.println(motor_speed);
  }



  

  
}

