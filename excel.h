#ifndef excel.h
#define excel.h

// Define left motor pin numbers
const int left_nslp_pin = 31;
const int left_dir_pin = 29;
const int left_pwm_pin = 40;

// Define right motor pin numbers
const int right_nslp_pin = 11;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

// Declare min/max arrays found in Excel Doc
const float minimum[] = {785, 718, 625, 560, 574, 546, 634, 639};
const float maximum[] = {1381, 1487, 962, 896, 898, 921, 1171, 1608};
const float fuseMax = 17000;

// Apply weighting scheme
//const float weight[] = {-8, -4, -2, -1, 1, 2, 4, 8};
const float weight[] = {-15, -14, -12, -8, 8, 12, 14, 15};

// Constants
float Kp = 0, Kd = 0, Ki = 0;
unsigned long currTime = 0, prevTime = 0; 
float prevError = 0, cumError = 0, rateError = 0;

#endif
