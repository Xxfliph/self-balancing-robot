#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>


Adafruit_MPU6050 mpu;


unsigned long newtime = 0;
unsigned long oldtime = 0;
double oldangle = 0;
double newangle = 0;
const unsigned long loop_period = 10000;
unsigned long last_loop_time = 0;
unsigned long current_time;


unsigned long speed_last_time = 0;
long lastCountA = 0;
long lastCountB = 0;
double wheel_speed = 0;
double speed_lean = 0;
const double K_SPEED = 0.06;
double speed_integral = 0;
double pos_hold = 0;
const double K_POS = 0.01;
const double KI_SPEED = 0.002;

double olderror = 0;
double gyro_bias = 0;


const double LEAN_ANGLE = 2.0;
double lean_target = 0;
double lean_now = 0;
double speed_target = 0;
const double TARGET_SPEED = 30;
double speed_cmd = 0;
const double SPEED_RAMP = 2.0;
const double LEAN_I_MAX = 2.5;
const int MIN_PWM = 25;
const double DEAD_BAND = 13;

class CmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) {
    auto v = c->getValue();
    for (size_t i = 0; i < v.length(); i++) {
      char cmd = v[i];
      if (cmd == 'F' || cmd == 'f') {
        speed_target = -TARGET_SPEED;
      }
      else if (cmd == 'B' || cmd == 'b') {
        speed_target = TARGET_SPEED;
      }
      else if (cmd == 'S' || cmd == 's') {
        speed_target = 0;
      }
    }
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onDisconnect(BLEServer *s) {
    speed_target = 0;
    BLEDevice::startAdvertising();
  }
};
//SDA goes to GPIO 21 and SCL goes to GPIO 22 by default

int AIN1 = 25;
int AIN2 = 26;
int PWMA = 18;

int BIN1 = 32;
int BIN2 = 33;
int PWMB = 19;

int encoderAA = 13;
int encoderAB = 14;
int encoderBA = 27;
int encoderBB = 16;
volatile int pulseCountA = 0;
volatile int pulseCountB = 0;

void IRAM_ATTR onEncoderPulseA() {
  if (digitalRead(encoderAB) == LOW) {
    pulseCountA++;
  }
  else {
    pulseCountA--;
  }
}

void IRAM_ATTR onEncoderPulseB() {
  if (digitalRead(encoderBB) == LOW) {
    pulseCountB++;
  }
  else {
    pulseCountB--;
  }
}



void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("Adafruit MPU6050 test");


  if (!mpu.begin(0x69)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found");
  Wire.setClock(400000);


  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

/*
Filter bandwith is used to smooth out noise (fast small fluctuations  in raw sensor readings, vibrations, electrical interference)
lower Hz = more smoothing but slower to respond to changes. Higher Hz = less smoothing (noisier) but more responsive to changes
*/
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ); // or 44_HZ

  
  
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  analogWriteFrequency(PWMA, 20000);
  analogWriteFrequency(PWMB, 20000);

  pinMode(encoderAA, INPUT_PULLUP);
  pinMode(encoderAB, INPUT_PULLUP);
  pinMode(encoderBA, INPUT_PULLUP);
  pinMode(encoderBB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderAA), onEncoderPulseA, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderBA), onEncoderPulseB, FALLING);  

  BLEDevice::init("balancebot");
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  BLEService *service = server->createService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
  BLECharacteristic *rx = service->createCharacteristic(
    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E",
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new CmdCallbacks());
  service->start();
  BLEDevice::getAdvertising()->addServiceUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
  BLEDevice::getAdvertising()->start();

  double gyro_sum = 0;
  for (int i = 0; i < 500; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyro_sum += g.gyro.y;
    delay(3);
  }
  gyro_bias = gyro_sum / 500.0;

  oldtime = micros();   
}

double integral_sum = 0;

void loop() {

  double angle_accel;
  double angle_gyro;
  double deltatime;
  double setpoint = 2.65; //if you want it to lean negative, decrease. if you want it to lean positive, increase
  double error;
  double P = 0.0;
  double K_p = 58; //will need to adjust these based on behaviour
  double D = 0.0;
  double K_d = 2.2; //will need to adjust these based on behaviour
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

    double gyro_reading = (g.gyro.y - gyro_bias)*(180.0/M_PI);

    angle_accel = atan2(a.acceleration.x, -a.acceleration.z)*(180.0/M_PI); //put negative sign before z acceleration to invert it becasue mpu is upside down.
    newangle = (0.98*(oldangle+gyro_reading*deltatime) + 0.02*angle_accel);
    oldangle = newangle;
  

    lean_now += constrain(lean_target - lean_now, -0.05, 0.05);
    error = (setpoint + lean_now + speed_lean) - newangle;

    P = K_p*error;

    static double gyro_filtered = 0;
    gyro_filtered = 0.5*gyro_filtered + 0.5*gyro_reading;
    D = -K_d*gyro_filtered;
    olderror = error;

    integral_sum += error*deltatime;
    integral_sum = constrain(integral_sum, -5.0, 5.0); // can change the -100 100 clamping values according to behaviour. range = pwm/K_i (255/K-i)
    I = K_i*integral_sum;

    double pid_output = P + I + D;

    pid_output = constrain(pid_output, -255.0, 255.0);



    double mag = fabs(pid_output);
    int motor_speed;
    if (mag < DEAD_BAND) {
      motor_speed = (int)(mag * MIN_PWM / DEAD_BAND);
    } else {
      motor_speed = MIN_PWM + (int)((mag - DEAD_BAND) * (255 - MIN_PWM) / (255.0 - DEAD_BAND));
    }

    if (abs(newangle) > 45.0) {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, LOW);
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, LOW);
      analogWrite(PWMA, 0);
      analogWrite(PWMB, 0);

      integral_sum = 0;
      speed_integral = 0;
    }
    else {
      if (pid_output < 0) {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);

        analogWrite(PWMA, motor_speed);
        analogWrite(PWMB, motor_speed);
      }
      else if (pid_output > 0) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);

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
    if (current_time - speed_last_time >= 50000) {
      long countA = pulseCountA;
      long countB = pulseCountB;
      double raw_speed = ((countA - lastCountA) + (countB - lastCountB)) / 2.0;
      wheel_speed = 0.7 * wheel_speed + 0.3 * raw_speed;
      lastCountA = countA;
      lastCountB = countB;
      speed_last_time = current_time;

      speed_cmd += constrain(speed_target - speed_cmd, -SPEED_RAMP, SPEED_RAMP);

      double speed_error = wheel_speed - speed_cmd;
      double i_lim = LEAN_I_MAX / KI_SPEED;
      speed_integral = constrain(speed_integral + speed_error, -i_lim, i_lim);
      double pos = (countA + countB) / 2.0;
      if (speed_target != 0 || abs(wheel_speed) > 2) {
        pos_hold = pos;
      }
      speed_lean = constrain(K_SPEED * speed_error + KI_SPEED * speed_integral + K_POS * (pos - pos_hold), -3.0, 3.0);
    }

    
  static unsigned long last_print = 0;
  if (millis() - last_print >= 20) {
    last_print = millis();
    Serial.print("P:");     Serial.print(P);
    Serial.print(",D:");    Serial.print(D);
    Serial.print(",angle:"); Serial.print(newangle * 10);
    Serial.print("speedlean:"); Serial.println(speed_lean * 10);
  }

    /*
    int AIN1 = 25;
    int AIN2 = 26;
    int PWMA = 18;

    int BIN1 = 32;
    int BIN2 = 33;
    int PWMB = 19;

    */
    /*
    Serial.println("Angle: ");
    
    Serial.println(newangle);
    
    Serial.print(" | PID: ");
    Serial.print(pid_output);
    Serial.print(" | PWM: ");
    Serial.println(motor_speed);
    */
    
  }



  

  
}

