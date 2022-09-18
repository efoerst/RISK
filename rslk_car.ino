#include <ECE3.h>
#include "excel.h"

float findPID(float error);

uint16_t sensorValues[8];
float sensorIntVals[8];

float elapsedTime = 0.006;
float error = 0;
float speed_change = 0;

// default speeds
int default_speed = 60;
int max_speed = 175;
int min_speed = 20;
int left_speed = default_speed;
int right_speed = default_speed;

long cur_time;
long donut_time;
long start_time;
int turning = 0;
int boost = 0;
int boost_delay = 3300;
int boost_end = 5000;
int count = 0;
int boost_after = 1700;
int boost_before = 1500;
int boost2 = 1;

float fuse = 0, prevFuse = 0;
float Kd_boost = 0.003;


void setup() {
  ECE3_Init();
  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);
  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(left_nslp_pin, HIGH);
  digitalWrite(right_dir_pin, LOW);
  digitalWrite(right_nslp_pin, HIGH);

  delay(1000);

  analogWrite(left_pwm_pin, left_speed);
  analogWrite(right_pwm_pin, right_speed);
  
  Serial.begin(9600);

  //=== PID K Coeffs ===
  Kp = 0.050;
  Ki = 0;
  Kd = 0.005;
  // ===================
  
  
}

void loop() {
  fuse = 0;
  // Read in the sensor values
  ECE3_read_IR(sensorValues);
  
  error = 0;
  // Get the value -> apply normalization principle -> apply weighting scheme
  for (int i = 0; i < 8; i++){
    sensorIntVals[i] = (float)sensorValues[i];
    fuse += sensorIntVals[i];
    error += (((sensorIntVals[i] - minimum[i])/maximum[i]) * 1000) * weight[i];
  }
  //error = error/4;
  error = error/8;
  speed_change = findPID(error);
  //Serial.println(speed_change);
  //Serial.println(error);
  // ===============================================================

  // keeping track of current time
  cur_time = millis();
  if (boost == 0)
  {
    start_time = millis();
    boost = 1;
  }

  // check if all sensors are at maximum (black bar signaling donut)
  if (fuse >= fuseMax && prevFuse >= fuseMax)
  {
    // might need COUNT variable if car stops at a random spot on the track
    if (turning == 0)
    { // reached end of track (do donut)
      // stop wheels
      analogWrite(left_pwm_pin, 0);
      analogWrite(right_pwm_pin, 0);
      digitalWrite(left_dir_pin, HIGH);   // change direction of left wheel
      delay(50);
      turning = 1;
      donut_time = millis();
    }
    if (turning == 2)       // reached end of track (stop)
      turning = 3;
  }

  // stop doing donut after 1 second
  if (turning == 1 && cur_time - donut_time > 350) 
  {
    digitalWrite(left_dir_pin, LOW);
    turning = 2;
    boost = 0; // reset boost
    boost2 = 2;
    boost_delay += 100;
    boost_end += 100;
  }
  

  // if doing donut, do not change speed
  // Turning left
  if (error > 0 && (turning != 1)){
    // Slow the left motor and increase the speed of the right motor
    left_speed -= speed_change;
    right_speed += speed_change;
    //Serial.println("turning left!");
    
  }
  //Turning right
  else if (error < 0 && turning != 1) {
    // Slow the right motor and increase the speed of the left motor
    left_speed += speed_change;
    right_speed -= speed_change; 
    //Serial.println("turning right!");
  }
  
  // place limits on speed
  if (left_speed > max_speed) left_speed = max_speed;
  if (right_speed > max_speed) right_speed = max_speed;
  if (left_speed < min_speed) left_speed = min_speed;
  if (right_speed < min_speed) right_speed = min_speed;

  // reached end of track, make speeds 0
  if (turning == 3)
  {
    left_speed = 0;
    right_speed = 0;
  }

  // increase speed when on the straight
  if (((cur_time - start_time) > boost_delay) && ((cur_time - start_time) < boost_end))
  {
    left_speed += 145;
    right_speed += 145;
  }

  if (boost2 == 1 && ((cur_time - start_time) > boost_end) && ((cur_time - start_time) < boost_end + boost_after))
  {
    left_speed += 80;
    right_speed += 80;
  }

  if (boost2 == 2 && ((cur_time - start_time) > boost_before) && ((cur_time - start_time) < boost_delay - 100))
  {
    left_speed += 80;
    right_speed += 80;
  }


  //fast donut spin
  if (turning == 1)
  {
    left_speed = max_speed;
    right_speed = max_speed;
  }

  // apply speed change to wheels
  analogWrite(left_pwm_pin, left_speed);
  analogWrite(right_pwm_pin, right_speed);

  // reset speed to default
  left_speed = default_speed;
  right_speed = default_speed;
  prevFuse = fuse;

  //Serial.print("Left speed ");
  //Serial.println(left_speed);
  //Serial.print("Right speed ");
  //Serial.println(right_speed);
  //Serial.print("speed change ");
  //Serial.println(speed_change);

}


// Define findPID
float findPID(float error){
  error = abs(error);
  //Serial.println(error);
  // Start timer
  currTime = millis();
  cumError += error * elapsedTime;
  rateError = (error - prevError)/elapsedTime;

  speed_change = Kp * error + Ki * cumError + Kd * rateError;
  //Serial.println(speed_change);

  prevError = error;
  prevTime = currTime;

  return speed_change;
}
