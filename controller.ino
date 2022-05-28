#include <Adafruit_BME280.h>

#include <Adafruit_MPU6050.h>

#include<Servo.h>

Adafruit_MPU6050 mpu;
Adafruit_BME280 bme;

Servo lock_servo;

float prev_height;
float height; 
float init_height; 

int counter;

enum State
  {
    IDLE = 0, 
    READY = 1,  
    POWERED_ASCENT = 2,
    MECO = 3, 
    APOGEE = 4,
    PARACHUTE_DESCENT = 5,
    LANDED = 6,
  };

State state;

#define SEALEVELPRESSURE_HPA 1013.25  //sea level pressure
#define LIFTOFF_THRESHOLD 100 //if accleration is higher, rocket is lifted 
#define POWER_THRESHOLD 50 //if acceleration is lower, engine burned out
#define LOWERING_THRESHOLD 10 //amount of consecutive height decreases to trigger parachute
#define APOGEE_PARACHUTE_DELAY 1000 //number of miliseconds to release parachute after apogee
#define LANDING_HEIGHT 60 //if the rocket is lower that thin height from the starting location, it landed



#define SRVO_PIN 9
#define SRVO_OPEN 255
#define SRVO_LOCKED 0
#define BEEPER_PIN 7

void setup() {
  pinMode(SRVO_PIN, OUTPUT);
  pinMode(BEEPER_PIN, OUTPUT);
  
  // put your setup code here, to run once:
  state=IDLE;

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  
  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
   if (!bme.begin()) {  
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  init_height = bme.readAltitude(SEALEVELPRESSURE_HPA);
  height=init_height;

  lock_servo.attach(SRVO_PIN);
  lock_servo.write(SRVO_LOCKED);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  switch(state){
    case IDLE:
      //unlock manually?
      state=READY;
    break;
    case READY:
       if((a.acceleration.z*a.acceleration.z)>LIFTOFF_THRESHOLD){
          state=POWERED_ASCENT;
        }
    break;
    case POWERED_ASCENT:
      if((a.acceleration.x*a.acceleration.x) + 
       (a.acceleration.y*a.acceleration.y) + 
       (a.acceleration.z*a.acceleration.z)<POWER_THRESHOLD){
          state=POWERED_ASCENT;
          delay(3000); //safety delay
        }  
    break;
    case MECO:
        height = bme.readAltitude(SEALEVELPRESSURE_HPA);
        if(height<prev_height){
          if(counter>LOWERING_THRESHOLD){
            state=APOGEE;
            delay(APOGEE_PARACHUTE_DELAY);
          }
        }
        else{
          counter=0;
        }
    break;
    case APOGEE:
      lock_servo.write(SRVO_OPEN);
      state=PARACHUTE_DESCENT;
    break;
    case PARACHUTE_DESCENT:
      if (bme.readAltitude(SEALEVELPRESSURE_HPA) <= init_height+LANDING_HEIGHT)
        state=LANDED;
        delay(5000);
    break;
    case LANDED:
      
    break;
  }
  beep(state);
}

State prev_state;
int beep_timer=0;
bool beeping;

int beep_codes[] = {
  200,5000, //IDLE
  200,2000, //READY
  1000,0, //POWERED_ASCENT
  200,1000, //MECO
  200,200, //APOGEE
  300,500, //PARACHUTE_DESCEND
  1000,500 //LANDED
  };


void beep(State curr_state){
    if(prev_state!=curr_state){
      beep_timer=0; 
      beeping=true; 
    }
    if(beep_timer>beep_codes[(2*int(curr_state))+beeping?1:0])
    {
      beep_timer=0;
      beeping=!beeping;
    }
    if(beep_timer=0){
       digitalWrite(BEEPER_PIN, beeping);
    }
}
