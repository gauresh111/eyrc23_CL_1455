/*
 * Created by ArduinoGetStarted, https://arduinogetstarted.com
 *
 * Arduino - Ultrasonic Sensor HC-SR04
 *
 * Wiring: Ultrasonic Sensor -> Arduino:
 * - VCC  -> 5VDC
 * - TRIG -> Pin 9
 * - ECHO -> Pin 8
 * - GND  -> GND
 *
 * Tutorial is available here: https://arduinogetstarted.com/tutorials/arduino-ultrasonic-sensor
 */
#define WINDOW_SIZE 5
int LefttrigPin = 14;    // TRIG pin
int leftechoPin = 27;    // ECHO pin
int righttrigPin = 4;
int rightechoPin=15;
float duration_usleft, distance_cmleft;
float duration_usright, distance_cmright;
float LeftDistance,RightDistance;

const int numReadings  = 30;
int readings [numReadings];
int readIndex  = 0;
long total  = 0;

void leftUltraSonic(void* pvParameters) {

  Serial.print("leftUltraSonic ");

  Serial.println(xPortGetCoreID());

  for (;;) {
digitalWrite(LefttrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(LefttrigPin, LOW);

  // measure duration of pulse from ECHO pin
  duration_usleft = pulseIn(leftechoPin, HIGH);
  // duration_us2 = pulseIn(rightechoPin, HIGH);
  
  // // calculate the distance
  distance_cmleft = 0.017 * duration_usleft;
  LeftDistance = distance_cmleft;
  // print the value to Serial Monitor
 
 
  delay(50);
    
  }
}

void RightUltraSonic(void* pvParameters) {

  Serial.print(" RightUltraSonic ");

  Serial.println(xPortGetCoreID());

  for (;;) {

  long average;
  // subtract the last reading:
  total = total - readings[readIndex];
  // read the sensor:
  if (millis()%random(1,10) == 0){
    LeftDistance += random(0,30);
  }
  readings[readIndex] = LeftDistance;
  // add value to total:
  total = total + readings[readIndex];
  // handle index
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) {
    readIndex = 0;
  }
  // calculate the average:
  average = total / numReadings;

   Serial.print("LeftDistance:");
  Serial.println(LeftDistance);
  // Serial.print(" cm       ");
  Serial.print("average:");
  Serial.println(average);
  float error = abs(LeftDistance-average);
  // Serial.print(" cm   ");
  Serial.print("error:");
  Serial.println(error);

  }
}




void setup() {
  // begin serial port
  Serial.begin (9600);
   xTaskCreatePinnedToCore(leftUltraSonic, "Task1", 10000, NULL, 1, NULL,  1); 

  delay(500); 

  xTaskCreatePinnedToCore(RightUltraSonic, "Task2", 10000, NULL, 0, NULL,  0); 
  // configure the trigger pin to output mode
  pinMode(LefttrigPin, OUTPUT);
  // configure the echo pin to input mode
  pinMode(leftechoPin, INPUT);
  pinMode(righttrigPin,OUTPUT);
  pinMode(rightechoPin,INPUT);
}

void loop() {
  // generate 10-microsecond pulse to TRIG pin
  
 
}
