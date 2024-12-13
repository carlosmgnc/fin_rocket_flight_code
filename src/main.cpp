#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>
#include <Servo.h>
#include <cmath>

#define BNO08X_CS 10
#define BNO08X_INT 9

#define BNO08X_RESET -1
#define RAD_TO_DEG 180/PI

#define PYRO_PIN 21

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

struct quat_t{
  double b0;
  double b1;
  double b2;
  double b3;
};

struct euler_t{
  double roll;
  double pitch;
  double yaw;
};

quat_t quat = {1.0, 0.0, 0.0, 0.0};
euler_t ypr = {0.0, 0.0, 0.0};

unsigned long last_millis = 0;

void integrate_quat(sh2_Gyroscope_t* gyro, quat_t* q){

  //store current quaternion into array for indexing
  double quat_array[4] = {q->b0, q->b1, q->b2, q->b3};
  
  //get measured gyro measurements
  double w1 = gyro->x;
  double w2 = gyro->y;
  double w3 = gyro->z;

  //measure sample time
  unsigned long current_millis = millis();
  double dt = (double)(current_millis - last_millis)/1000.0;
  last_millis = current_millis;

  //quat kinematic A matrix (q_dot = A*q)
  double A[4][4] = {
    {0, -w1/2, -w2/2, -w3/2},
    {w1/2, 0, w3/2, -w2/2},
    {w2/2, -w3/2, 0, w1/2},
    {w3/2, w2/2, -w1/2, 0}
  };

  //define intermediate vectors and matrices used for taylor expansion
  double Adtq_plus_q[4] = {0, 0, 0, 0};
  double A2dt2[4][4] = {
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0}
  };
  double taylor_expansion[4] = {0, 0, 0, 0};

  //get (A*dt*q) + q
  for (int i = 0; i < 4; i++){
    for (int j = 0; j < 4; j++){
      Adtq_plus_q[i] += A[i][j] * dt * quat_array[j];
    }
    Adtq_plus_q[i] += quat_array[i];
  }

  //get A^2 * (dt^2/2)
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4; j++){
      for(int k = 0; k < 4; k++){
        A2dt2[i][j] += A[i][k] * A[k][j] * (dt*dt/2);
      }
    }
  }

  //get ((A^2 * (dt^2/2)) * q) + (A*dt*q) + q 
  //(which is the last step to get the taylor expansion)
  for (int i = 0; i < 4; i++){
    for (int j = 0; j < 4; j++){
      taylor_expansion[i] += A2dt2[i][j] * quat_array[j];
    }
    taylor_expansion[i] += Adtq_plus_q[i];
  }
  
  //normalize quat
  double temp = 0;
  for (int i = 0; i < 4; i++){
    temp += taylor_expansion[i] * taylor_expansion[i];
  }
  temp = sqrt(temp);
  for (int i = 0; i < 4; i++){
    taylor_expansion[i] = taylor_expansion[i] / temp;
  }

  q->b0 = taylor_expansion[0];
  q->b1 = taylor_expansion[1];
  q->b2 = taylor_expansion[2];
  q->b3 = taylor_expansion[3];
}

void integrate_quat_o1(sh2_Gyroscope_t* gyro, quat_t* q){

  //store current quaternion into array for indexing
  double quat_array[4] = {q->b0, q->b1, q->b2, q->b3};
  
  //get measured gyro measurements
  double w1 = gyro->x;
  double w2 = gyro->y;
  double w3 = gyro->z;

  //measure sample time
  unsigned long current_millis = millis();
  double dt = (double)(current_millis - last_millis)/1000.0;
  last_millis = current_millis;

  //quat kinematic A matrix (q_dot = A*q)
  double A[4][4] = {
    {0, -w1/2, -w2/2, -w3/2},
    {w1/2, 0, w3/2, -w2/2},
    {w2/2, -w3/2, 0, w1/2},
    {w3/2, w2/2, -w1/2, 0}
  };

  //define intermediate vectors and matrices used for taylor expansion
  double Adtq_plus_q[4] = {0, 0, 0, 0};

  //get (A*dt*q) + q
  for (int i = 0; i < 4; i++){
    for (int j = 0; j < 4; j++){
      Adtq_plus_q[i] += A[i][j] * dt * quat_array[j];
    }
    Adtq_plus_q[i] += quat_array[i];
  }
  
  //normalize quat
  double temp = 0;
  for (int i = 0; i < 4; i++){
    temp += Adtq_plus_q[i] * Adtq_plus_q[i];
  }
  temp = sqrt(temp);
  for (int i = 0; i < 4; i++){
    Adtq_plus_q[i] = Adtq_plus_q[i] / temp;
  }

  q->b0 = Adtq_plus_q[0];
  q->b1 = Adtq_plus_q[1];
  q->b2 = Adtq_plus_q[2];
  q->b3 = Adtq_plus_q[3];
}

unsigned long take_off_millis = 0;
void detect_takeoff(sh2_Accelerometer_t* accel, int* state, unsigned long* take_off_millis){
  if(accel->x > 20){
    *take_off_millis = millis();
    *state = 1;
  }
}

unsigned long delay_time = 5600;

void ejection_timer(int* state){
  if (millis() > take_off_millis + delay_time){
    *state = 2;
  }
}

void eject_parachute(int* state){
  digitalWrite(PYRO_PIN, HIGH);
  delay(2000);
  digitalWrite(PYRO_PIN, LOW);
  *state = 3;
}

void quat_to_euler(quat_t* quat, euler_t* ypr){
  double b0 = quat->b0;
  double b1 = quat->b1;
  double b2 = quat->b2;
  double b3 = quat->b3;

  ypr->yaw = atan2( 2.0*(b1*b2+b0*b3), sq(b0) + sq(b1) - sq(b2) - sq(b3) ) * RAD_TO_DEG;
  ypr->pitch = asin( -2.0*(b1*b3-b0*b2) ) * RAD_TO_DEG;
  ypr->roll = atan2( 2.0*(b2*b3+b0*b1), sq(b0) - sq(b1) - sq(b2) + sq(b3) ) * RAD_TO_DEG;
}

void quaternionToEuler(quat_t* quat, euler_t* ypr) {
    float qr = quat->b0;
    float qi = quat->b1;
    float qj = quat->b2;
    float qk = quat->b3;

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
}


class pid {
  private:
    double kp, ki, kd;
    double integral, integral_proposed, last_error, max_integral;
    unsigned long last_millis;

  public: 
    pid (double kp, double ki, double kd) {
      this->kp = kp;
      this->ki = ki;
      this->kd = kd;
      integral = 0.0;
      integral_proposed = 0.0;
      max_integral = (5 / ki);
      last_error = 0.0;
      last_millis = 0.0;
    }

    double calculate(double error) {
      unsigned long current_millis = millis();
      double dt = (double)(current_millis - last_millis)/1000.0;
      last_millis = current_millis;
  
      double derivative = (error - last_error) / dt;
      integral += error * dt;

      //hard limit on integral value
      if (integral > max_integral){
        integral = max_integral;
      }
      if (integral < -max_integral){
        integral = -max_integral;
      }

      last_error = error;

      //proposed output before checking for integral windup
      double output = error*kp + integral*ki + derivative*kd;

      //anti-windup
      if((abs(output) > 9) && ((output<0) == (error<0))){
        integral -= error*dt;
        output = error*kp + integral*ki + derivative*kd;
      }
      return output;
    }
};

pid roll_pid(0.5, 0, 0.02);
pid pitch_pid(2, 0, 0.2);
pid yaw_pid(2, 0, 0.2);

double clamp_value(double value, double min, double max){
  if(value < min){
    value = min;
  }
  else if (value > max){
    value = max;
  }
  return value;
}

double fin_to_servo(double theta){
  return (0.00001159*pow(theta,4) + 0.00051471*pow(theta, 3) - 0.00607428*pow(theta,2) + 1.90145221*pow(theta, 1) + 0.05012628);
}

double max_amp = 14;
double delta1_0 = 85;
double delta2_0 = 85;
double delta3_0 = 85;
double delta4_0 = 84;

void vert_attitude_control(euler_t* ypr){

  //calculate euler angle errors
  double error_roll = 0 - ypr->roll;
  double error_pitch = 0 - ypr->pitch;
  double error_yaw = 0 - ypr->yaw;

  //calulate pid outputs
  double u_roll = roll_pid.calculate(error_roll);
  double u_pitch = pitch_pid.calculate(error_pitch);
  double u_yaw = yaw_pid.calculate(error_yaw);

  //control mixing
  double delta1 = u_yaw - u_roll;
  double delta2 = - u_pitch - u_roll;
  double delta3 = - u_yaw - u_roll;
  double delta4 = u_pitch - u_roll;

  //limit fin angle outputs
  delta1 = clamp_value(delta1, -max_amp, max_amp);
  delta2 = clamp_value(delta2, -max_amp, max_amp);
  delta3 = clamp_value(delta3, -max_amp, max_amp);
  delta4 = clamp_value(delta4, -max_amp, max_amp);

  //convert fin angle request to servo angle request
  delta1 = fin_to_servo(delta1);
  delta2 = fin_to_servo(delta2);
  delta3 = fin_to_servo(delta3);
  delta4 = fin_to_servo(delta4);

  Serial.print(delta1);
  Serial.println();

  delta1 += delta1_0;
  delta2 += delta2_0;
  delta3 += delta3_0;
  delta4 += delta4_0;

  //write to the servos
  servo1.write(delta1);
  servo2.write(delta2);
  servo3.write(delta3);
  servo4.write(delta4);
}

void setReports(void) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_ACCELEROMETER, 1000000/50)) {
    Serial.println("Could not enable accelerometer");
  }
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 1000000/400)) {
    Serial.println("Could not enable gyroscope");
  }
  
}

void setup(){
  Serial.begin(9600);

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) {delay(10);}
  }
  Serial.println("BNO08x Found!");
  setReports();

  servo1.attach(1, 900, 2100);
  servo2.attach(2, 900, 2100);
  servo3.attach(3, 900, 2100);
  servo4.attach(4, 900, 2100);

  pinMode(PYRO_PIN, OUTPUT);
  delay(100);
}

int angle = 82;
bool direction = 0;
int state = 0;
double accel = 0;

bool gyro_ready = false;

void loop() {
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }
  if (!bno08x.getSensorEvent(&sensorValue)) {return;}

  //wait 5 seconds to enable gyro integration (to let gyro calibrate)
  if (!gyro_ready){
    if(millis() > 5000){
      gyro_ready = true;
    }
  }

  switch (state){
    //armed
    case 0:
      if(true){
        switch (sensorValue.sensorId) {
          case SH2_ACCELEROMETER:
            if(gyro_ready){
              detect_takeoff(&sensorValue.un.accelerometer, &state, &take_off_millis);
            }
            break;
          case SH2_GYROSCOPE_CALIBRATED:
            integrate_quat_o1(&sensorValue.un.gyroscope, &quat);
            quaternionToEuler(&quat, &ypr);
            break;
        }
      }
      vert_attitude_control(&ypr);
      break;

    //flight
    case 1:
      Serial.println("flight");

      if(true){
        switch (sensorValue.sensorId) {
          case SH2_ACCELEROMETER:
            break;
          case SH2_GYROSCOPE_CALIBRATED:
            integrate_quat_o1(&sensorValue.un.gyroscope, &quat);
            quaternionToEuler(&quat, &ypr);
            break;
        }
      }
      vert_attitude_control(&ypr);
      ejection_timer(&state);
      break;
    //eject parachute
    case 2:
      Serial.println("ejecting parachute!");
      eject_parachute(&state);
      break;
    //idle
    case 3:
      Serial.println("idle");
      break;
  }
  // // //print for the 3d model web viewer
  // Serial.print(F("Quaternion: "));
  // Serial.print((float)quat.b0, 4);
  // Serial.print(F(", "));
  // Serial.print((float)quat.b1, 4);
  // Serial.print(F(", "));
  // Serial.print((float)quat.b2, 4);
  // Serial.print(F(", "));
  // Serial.print((float)quat.b3, 4);
  // Serial.println(F(""));

  // Serial.print(F("Orientation: "));
  // Serial.print((float)ypr.roll);
  // Serial.print(F(", "));
  // Serial.print((float)ypr.pitch);
  // Serial.print(F(", "));
  // Serial.print((float)ypr.yaw);
  // Serial.println(F(""));

  //print euler angles
  // Serial.print("yaw: ");
  // Serial.print((float)ypr.yaw, 4);
  // Serial.print(", pitch: ");
  // Serial.print((float)ypr.pitch, 4);
  // Serial.print(", roll: ");
  // Serial.print((float)ypr.roll, 4);
  // Serial.println();
  
  // Serial.print("gyro:");
  // Serial.print(sensorValue.un.gyroscope.x);
  // Serial.print(", ");
  // Serial.print(sensorValue.un.gyroscope.y);
  // Serial.print(", ");
  // Serial.print(sensorValue.un.gyroscope.z);
  // Serial.println("");
}