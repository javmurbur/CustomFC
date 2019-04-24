#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_ADC.h>
#include <StorageManager.h>
#include <AP_InertialSensor.h>
#include <AP_Notify.h>
#include <AP_GPS.h>
#include <AP_Baro.h>
#include <Filter.h>
#include <GCS_MAVLink.h>
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_Compass.h>
#include <AP_NavEKF.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

#include <PID.h>

// Motor numbers definitions
#define MOTOR_FL   2    // Front left    
#define MOTOR_FR   0    // Front right
#define MOTOR_BL   1    // back left
#define MOTOR_BR   3    // back right

// PID array (6 pids, two for each axis)
PID pids[6];
#define PID_PITCH_RATE 0
#define PID_ROLL_RATE 1
#define PID_PITCH_STAB 2
#define PID_ROLL_STAB 3
#define PID_YAW_RATE 4
#define PID_YAW_STAB 5

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))

// ArduPilot Hardware Abstraction Layer
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;

// MPU6050 accel/gyro chip
AP_InertialSensor ins;
AP_InertialSensor_MPU6000 imu(ins);
AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi);
AP_GPS gps;
AP_Compass_HMC5843 compass;
AP_AHRS_DCM ahrs(ins,baro,gps);

// Radio min/max values for each stick for my radio (worked out at beginning of article)
#define RC_THR_MIN   1000
#define RC_YAW_MIN   1000
#define RC_YAW_MAX   2000
#define RC_PIT_MIN   1000
#define RC_PIT_MAX   2000
#define RC_ROL_MIN   1000
#define RC_ROL_MAX   2000



// Arduino map function
float map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void setup() 
{
  // Enable the motors and set at 490Hz update
  hal.rcout->set_freq(0xF, 490);
  //hal.rcout->enable_mask(0xFF);

   for (uint8_t i=0; i<8; i++) {
    hal.rcout->enable_ch(i);
  }

  // Turn off Barometer to avoid bus collisions
  hal.gpio->pinMode(40, HAL_GPIO_OUTPUT);
  hal.gpio->write(40, 1);

  // Turn on MPU6050 - quad must be kept still as gyros will calibrate
  ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_100HZ);

  ins.init_accel();

  ahrs.init();

  if( compass.init() ) {
    hal.console->printf("Enabling compass\n");
    ahrs.set_compass(&compass);
  }
  else {
    hal.console->printf("No compass detected\n");
  }

  gps.init(NULL);

  
  // PID Configuration
  pids[PID_PITCH_STAB].kP(110.714);
  pids[PID_PITCH_STAB].kI(553.57);
  pids[PID_PITCH_STAB].kD(5.536);
  pids[PID_PITCH_STAB].imax(8);

  pids[PID_ROLL_STAB].kP(69.354);
  pids[PID_ROLL_STAB].kI(346.77);
  pids[PID_ROLL_STAB].kD(3.468);
  pids[PID_ROLL_STAB].imax(8);

  pids[PID_YAW_STAB].kP(146);
  pids[PID_YAW_STAB].kI(561.54);
  pids[PID_YAW_STAB].kD(9.49);
  pids[PID_YAW_STAB].imax(8);

  pids[PID_PITCH_RATE].kP(3.67);
  pids[PID_PITCH_RATE].kI(73.429);
  pids[PID_PITCH_RATE].kD(0);
  pids[PID_PITCH_RATE].imax(50);

  pids[PID_ROLL_RATE].kP(2.78);
  pids[PID_ROLL_RATE].kI(55.61);
  pids[PID_ROLL_RATE].kD(0);
  pids[PID_ROLL_RATE].imax(50);
  
  pids[PID_YAW_RATE].kP(3.908);
  pids[PID_YAW_RATE].kI(39.08);
  pids[PID_YAW_RATE].kD(0);
  pids[PID_YAW_RATE].imax(50);
  


  // initialise sensor fusion on MPU6050 chip (aka DigitalMotionProcessing/DMP)
  //hal.scheduler->suspend_timer_procs();  // stop bus collisions
  //ins.dmp_init();
  //hal.scheduler->resume_timer_procs();
  
  // We're ready to go! Now over to loop()
}

void loop() 
{
  uint16_t channels[8];
  float rcthr, rcyaw, rcpit, rcroll;  // Variables to store radio in
  Vector3f gyro;
  float gyroroll,gyropitch,gyroyaw;
  float roll,pitch,yaw;
  static uint16_t counter;
  static uint32_t last_print,last_compass;
  static float yaw_target = 0;
  float pitch_stab_output;
  float roll_stab_output;
  float yaw_stab_output;
  float pitch_output;
  float roll_output;
  float yaw_output; 

  uint32_t now = hal.scheduler->micros();
  if (last_t == 0) {
    last_t = now;
    return;
  }

  last_t = now;

  // Read RC transmitter and map to sensible values  
  hal.rcin->read(channels, 8);

  rcthr = channels[2];
  rcyaw = map(channels[3], RC_YAW_MIN, RC_YAW_MAX, -180, 180);
  rcpit = map(channels[1], RC_PIT_MIN, RC_PIT_MAX, -45, 45);
  rcroll = map(channels[0], RC_ROL_MIN, RC_ROL_MAX, -45, 45);
  
  ins.wait_for_sample();

  // Ask MPU6050 for orientation
  ins.update();
  gyro = ins.get_gyro();
  gyropitch = ToDeg(gyro.y);
  gyroroll = ToDeg(gyro.x);
  gyroyaw = ToDeg(gyro.z);

  ahrs.update();
  //ins.quaternion.to_euler(&roll, &pitch, &yaw);
  roll = ToDeg(ahrs.roll) ;
  pitch = ToDeg(ahrs.pitch) ;
  yaw = ToDeg(ahrs.yaw) ;
  counter++;
  
  // Do the magic
  if(rcthr > RC_THR_MIN + 100) {  // Throttle raised, turn on stablisation.
    // Stablise PIDS
    pitch_stab_output = constrain_float(pids[PID_PITCH_STAB].get_pid((float)rcpit - pitch, 1), -250, 250); 
    roll_stab_output = constrain_float(pids[PID_ROLL_STAB].get_pid((float)rcroll - roll, 1), -250, 250);
    yaw_stab_output = constrain_float(pids[PID_YAW_STAB].get_pid(wrap_180(yaw_target - yaw), 1), -360, 360);
  
    // is pilot asking for yaw change - if so feed directly to rate pid (overwriting yaw stab output)
    if(abs(rcyaw ) > 5) {
      yaw_stab_output = rcyaw;
      yaw_target = yaw;   // remember this yaw for when pilot stops
    }
    
    // rate PIDS
    pitch_output =  constrain_float(pids[PID_PITCH_RATE].get_pid(pitch_stab_output - gyroPitch, 1), - 500, 500);  
    roll_output =  constrain_float(pids[PID_ROLL_RATE].get_pid(roll_stab_output - gyroRoll, 1), -500, 500);  
    yaw_output =  -constrain_float(pids[PID_YAW_RATE].get_pid(yaw_stab_output - gyroYaw, 1), -500, 500);  

    // mix pid outputs and send to the motors.
    hal.rcout->write(MOTOR_FL, rcthr + roll_output + pitch_output - yaw_output);
    hal.rcout->write(MOTOR_BL, rcthr + roll_output - pitch_output + yaw_output);
    hal.rcout->write(MOTOR_FR, rcthr - roll_output + pitch_output + yaw_output);
    hal.rcout->write(MOTOR_BR, rcthr - roll_output - pitch_output - yaw_output);
  } else {
    // motors off
    hal.rcout->write(MOTOR_FL, 1000);
    hal.rcout->write(MOTOR_BL, 1000);
    hal.rcout->write(MOTOR_FR, 1000);
    hal.rcout->write(MOTOR_BR, 1000);
       
    // reset yaw target so we maintain this on takeoff
    yaw_target = yaw;
    
    // reset PID integrals whilst on the ground
    for(int i=0; i<6; i++)
      pids[i].reset_I();

  }
}

AP_HAL_MAIN();





