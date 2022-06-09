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

// PARAMETERS
#define SEALEVELPRESSURE_HPA 1013.25  //sea level pressure
#define LIFTOFF_THRESHOLD 1000 //if accleration is higher, rocket is lifted 
#define POWER_THRESHOLD 500 //if acceleration is lower, engine burned out

#define APOGEE_PARACHUTE_DELAY 500 //number of miliseconds to release parachute after apogee
#define LANDING_HEIGHT 60 //if the rocket is lower that thin height from the starting location, it landed

#define IS_DEBUG true // if true, serial debgging is on
#define CONSECUTIVE_COUNTER 10 //amount of consecutive height decreases to trigger parachute
                              //SHOULD BE MORE THAN 1 IF NOT TESTING
#define SRVO_PIN 5
#define SRVO_OPEN 255
#define SRVO_LOCKED 0
#define BEEPER_PIN 6
#define READY_PIN 17 //analog
#define OVERRIDE_PIN 16 //analog


State prev_state;
long beep_timer=0;
bool beeping;

// sound signal codes
int beep_codes[] = {
  100,5000, //IDLE
  200,2000, //READY
  1000,0, //POWERED_ASCENT
  500,1000, //MECO
  500,200, //APOGEE
  1000,2000, //PARACHUTE_DESCEND
  1000,500, //LANDED
  100,100 //ERROR
 };



void setup() {
  // initialise pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SRVO_PIN, OUTPUT);
  pinMode(BEEPER_PIN, OUTPUT);
  pinMode(READY_PIN, INPUT);
  pinMode(OVERRIDE_PIN, INPUT);


#if IS_DEBUG
  Serial.begin(9600);
#endif

  state=IDLE;

  // connect sensors
  if (!bme.begin(0x76)) {  
#if IS_DEBUG
    Serial.println("Could not find a valid bme280 sensor, check wiring!");
#endif
    state=ERROR;
  }

  if (!mpu.begin()) {
#if IS_DEBUG
    Serial.println("Failed to find MPU6050 chip");
#endif
    state=ERROR;
  }  
  // set accelerometer range to +-16G
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // set initial height
  init_height = bme.readAltitude(SEALEVELPRESSURE_HPA);
  height=init_height;

  // initialize and loch parachute servo
  lock_servo.attach(SRVO_PIN);
  lock_servo.write(SRVO_LOCKED);
  waitTime=0;
#if IS_DEBUG
  if(state==IDLE){
    Serial.println("Initialised without problems");
  }
  Serial.println("x y z sqrMagnitude STATE");
#endif
  if( pulseIn(OVERRIDE_PIN,HIGH)<1200 && pulseIn(OVERRIDE_PIN,HIGH)>1000 ){
    state = ERROR;
  }
  delay(2000);
}

sensors_event_t a, g, temp;
 
void loop() {
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
    Serial.print(state);
#endif

  // manual parachute override
  Serial.print(" "); // a space ' ' or  tab '\t' character is printed between the two values.
  Serial.print(pulseIn(OVERRIDE_PIN,HIGH)); 
  Serial.print(" "); // a space ' ' or  tab '\t' character is printed between the two values.
  Serial.print(pulseIn(READY_PIN,HIGH)); 
  if(state<4 && state>0 && pulseIn(OVERRIDE_PIN,HIGH)<1200 && pulseIn(OVERRIDE_PIN,HIGH)>1000 ){
    state = APOGEE;
    waitTime=0;
  }

  // perform state check
  if(millis() > waitTime ){
  switch(state){
    case IDLE:  // rocket waits for signal
      if(pulseIn(READY_PIN,HIGH)>1600){
        state=READY;
      }
    break;
    case READY: // rocket is ready for launch and waits for acceleration
       // read sensors
       mpu.getEvent(&a, &g, &temp);

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
    case POWERED_ASCENT: // engine is on, waiting for accelearation to decrease
      // read sensors
      mpu.getEvent(&a, &g, &temp);

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
    case MECO: // engine is off, altitude increases, waiting for altitude to start decreasing
        height = bme.readAltitude(SEALEVELPRESSURE_HPA);
        if(height<=prev_height){
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
        prev_height=height;
    break;
    case APOGEE: // opening parachute
      lock_servo.write(SRVO_OPEN);
      state=PARACHUTE_DESCENT;
    break;
    case PARACHUTE_DESCENT: // parachute is out, waiting to reach the ground
      if (bme.readAltitude(SEALEVELPRESSURE_HPA) <= init_height+LANDING_HEIGHT)
        state=LANDED;
        waitTime=millis()+5000;
    break;
    case LANDED:
      
    break;
    case ERROR:
    break;
  }
  }

  // sound signal
  beep(state);
#if IS_DEBUG
  Serial.println();
#endif
}

// sound signal for every stage of flight
void beep(State curr_state){
    if(prev_state!=curr_state){
      beep_timer=0;
      beeping=true; 
      prev_state=curr_state;
    }
    if(millis()>beep_timer)
    {
      beep_timer=millis() + beep_codes[(2*curr_state)+(beeping?1:0)];
      beeping=beeping?false:true;
      if(beeping){
        tone(BEEPER_PIN, 500);
        digitalWrite(LED_BUILTIN, HIGH);
      }
      else{
        noTone(BEEPER_PIN);
        digitalWrite(LED_BUILTIN, LOW);
      }
    }
}
