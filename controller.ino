#include <Adafruit_BMP280.h>

#include <Adafruit_MPU6050.h>

#include<Servo.h>
#define IS_DEBUG true

Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp;


Servo lock_servo;

float prev_height;
float height; 
float init_height; 

int counter;
long waitTime;

enum State
  {
    IDLE = 0, 
    READY = 1,  
    POWERED_ASCENT = 2,
    MECO = 3, 
    APOGEE = 4,
    PARACHUTE_DESCENT = 5,
    LANDED = 6,
    ERROR=7,
  };

State state;

#define SEALEVELPRESSURE_HPA 1013.25  //sea level pressure
#define LIFTOFF_THRESHOLD 300 //if accleration is higher, rocket is lifted 
#define POWER_THRESHOLD 200 //if acceleration is lower, engine burned out

#define APOGEE_PARACHUTE_DELAY 1000 //number of miliseconds to release parachute after apogee
#define LANDING_HEIGHT 60 //if the rocket is lower that thin height from the starting location, it landed

#define CONSECUTIVE_COUNTER 1 //amount of consecutive height decreases to trigger parachute

#define SRVO_PIN 9
#define SRVO_OPEN 255
#define SRVO_LOCKED 0
#define BEEPER_PIN 7
#define READY_PIN 7

void setup() {
  pinMode(SRVO_PIN, OUTPUT);
  pinMode(BEEPER_PIN, OUTPUT);
  pinMode(BEEPER_PIN, OUTPUT);

  Serial.begin(9600);
  
  // put your setup code here, to run once:
  state=IDLE;

  if (!bmp.begin()) {  
    Serial.println("Could not find a valid bmp280 sensor, check wiring!");
    state=ERROR;
  }

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    state=ERROR;
  }
  
  // set accelerometer range to +-16G
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  
  init_height = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  height=init_height;

  lock_servo.attach(SRVO_PIN);
  lock_servo.write(SRVO_LOCKED);
  waitTime=0;
  if(state==IDLE){
    Serial.println("Initialised without problems");
  }
  Serial.println("x y z sqrMagnitude STATE");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  #if IS_DEBUG
    Serial.print(a.acceleration.x);
    Serial.print(" "); // a space ' ' or  tab '\t' character is printed between the two values.
    Serial.print(a.acceleration.y);
    Serial.print(" "); // a space ' ' or  tab '\t' character is printed between the two values.
    Serial.print(a.acceleration.z);
    Serial.print(" "); // a space ' ' or  tab '\t' character is printed between the two values.
    Serial.print(sqrt((a.acceleration.x*a.acceleration.x) + 
       (a.acceleration.y*a.acceleration.y) + 
       (a.acceleration.z*a.acceleration.z)));
    Serial.print(" "); // a space ' ' or  tab '\t' character is printed between the two values.
    Serial.println(state);
  #endif
  if(millis() > waitTime ){
  switch(state){
    case IDLE:
      //unlock manually?
      state=READY;
    break;
    case READY:
       if((a.acceleration.z*a.acceleration.z)>LIFTOFF_THRESHOLD){
          counter++;
          if(counter>CONSECUTIVE_COUNTER){
            state=POWERED_ASCENT;
            waitTime=millis()+3000; //safety delay
            counter=0;
          }
        }
        else{
          counter=0;
        }  
    break;
    case POWERED_ASCENT:
      if((a.acceleration.x*a.acceleration.x) + 
       (a.acceleration.y*a.acceleration.y) + 
       (a.acceleration.z*a.acceleration.z)<POWER_THRESHOLD){
          counter++;
          if(counter>CONSECUTIVE_COUNTER){
            state=MECO;
            waitTime=millis()+3000; //safety delay
            counter=0;
          }
        }
        else{
          counter=0;
        }  
    break;
    case MECO:
        height = bmp.readAltitude(SEALEVELPRESSURE_HPA);
        if(height<prev_height){
          counter++;
          if(counter>CONSECUTIVE_COUNTER){
            state=APOGEE;
            waitTime=millis()+APOGEE_PARACHUTE_DELAY;
            counter=0;
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
      if (bmp.readAltitude(SEALEVELPRESSURE_HPA) <= init_height+LANDING_HEIGHT)
        state=LANDED;
        waitTime=millis()+5000;
    break;
    case LANDED:
      
    break;
    case ERROR:
    break;
  }
  }
  beep(state);
}

State prev_state;
long beep_timer=0;
bool beeping;

int beep_codes[] = {
  200,5000, //IDLE
  200,2000, //READY
  1000,0, //POWERED_ASCENT
  500,1000, //MECO
  500,200, //APOGEE
  1000,2000, //PARACHUTE_DESCEND
  1000,500, //LANDED
  200,200 //ERROR
  };


void beep(State curr_state){
    if(prev_state!=curr_state){
      beep_timer=0;
      beeping=true; 
      prev_state=curr_state;
    }
    if(millis()>beep_timer)
    {
      beep_timer=millis() + beep_codes[(2*curr_state)+(beeping?1:0)];
      beeping=!beeping;
      if(beeping){
        tone(BEEPER_PIN, 500);
      }
      else{
        noTone(BEEPER_PIN);
      }
    }
}
