#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <arduino-timer.h>

auto timer = timer_create_default();

Adafruit_MPU6050 mpu;

#define WINDOW_SIZE 65
#define VIBRATIONS_THRESHOLD 0.0096
#define AMPLITUDE_THRESHOLD 12.45
#define VARIANCE_THRESHOLD 0.329
#define ZCR_TOLERANCE 10.75
#define ZCR_WINDOW_SIZE 8
#define ZCR_THRESHOLD 1


int get_zcr(double a[], int n) 
{
  byte zcr=0;
  byte index=0;
  
  while(index < n){
    double sub[ZCR_WINDOW_SIZE];
    double window_mean = 0;

    slice_zcr(a, sub, index);
    
    window_mean = get_mean(sub, ZCR_WINDOW_SIZE );
    
    if(window_mean < ZCR_TOLERANCE){
      zcr++;
    }
    index = index+ZCR_WINDOW_SIZE;
  }
  
  return zcr;
}

void slice_zcr(double a[], double(&subarr)[ZCR_WINDOW_SIZE], int first){

  for (int i=0; i < ZCR_WINDOW_SIZE; i++){
    subarr[i] = a[i+first];
  }

}

float get_mean(double a[], int n){
       
    double sum = 0; 
    for (int i = 0; i < n; i++) {sum += a[i];}

    return (double)sum /(double)n;
}

float get_variance(double a[], int n) 
{
    double mean = get_mean(a, n); 
    
    double square_diff = 0; 
    for (int i = 0; i < n; i++) 
        square_diff += (a[i] - mean) * (a[i] - mean); 
    return (float)square_diff / n; 
}

float get_max(double a[], int n){
       
    double m = a[0]; 
    for (int i = 1; i < n; i++) {
      if(a[i] > m){
        m = a[i];
      }
    }

    return m;
}

void setup() {
  Serial.begin(115200);
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  //setup motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(0.001);
  mpu.setMotionDetectionDuration(2);
  mpu.setInterruptPinLatch(true);  // Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  delay(100);
}

void loop() {
  timer.tick();
  sensors_event_t a, g, temp;

  double x,y,z;
  double vibrations [WINDOW_SIZE];

  for(int i=0; i < WINDOW_SIZE; i++){
    mpu.getEvent(&a, &g, &temp);
    x = a.acceleration.x;
    y = a.acceleration.y;
    z = a.acceleration.z;

    vibrations[i] = sqrt(x*x + y*y + z*z);
    delay(10);
  }

  double vibrations_var;
  vibrations_var = get_variance(vibrations, WINDOW_SIZE);
  Serial.printf("Vibrations variance: %f\n", vibrations_var);


  if(vibrations_var > VIBRATIONS_THRESHOLD){

    Serial.println("Some activity detected!");
    Serial.printf("Vibrations variance: %f\n", vibrations_var);

    double vibrations_zcr;
    vibrations_zcr = get_zcr(vibrations, WINDOW_SIZE);
    Serial.printf("Vibrations zero-crossing rate: %f\n", vibrations_zcr);

    double vibrations_amplitude;
    vibrations_amplitude = get_max(vibrations, WINDOW_SIZE);
    Serial.printf("Vibrations amplitude: %f\n", vibrations_amplitude);

    if(vibrations_var < VARIANCE_THRESHOLD && 
    (vibrations_zcr > ZCR_THRESHOLD || 
    vibrations_amplitude > AMPLITUDE_THRESHOLD)){
        Serial.println("Train detected! Going to sleep...");
        delay(500);
      }else{
        Serial.println("Not the train.");
        Serial.printf("Response time: %f\n", timer.tick());
    }
  }
}
