/*
 * CanSat AlphaSpot - Main Flight Computer
 * 
 * High School Project: Complete satellite that fits in a soda can
 * 
 * This program runs on Arduino Feather M0 Express and:
 * - Collects atmospheric data (pressure, temperature, altitude)
 * - Measures acceleration (X, Y, Z axes)
 * - Retrieves GPS coordinates
 * - Stores all data on SD card
 * - Transmits GPS coordinates via RFM69 radio module
 * - Controls stepper motor for parachute deployment
 * 
 * Hardware:
 * - BMP280 (Pressure/Temperature sensor)
 * - MPU6050 (Accelerometer)
 * - GPS Breakout V3
 * - RFM69 Radio Module (transmitter)
 * - SD Card Logger
 * - Stepper motor (parachute deployment mechanism)
 * 
 * Author: Jean Knecht (High School Project)
 * Year: [Your graduation year]
 */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SD.h>
#include <RH_RF69.h>
#include <Adafruit_GPS.h>

#define GPSSerial Serial1

// GPS Coordinate Conversion Constant
// GPS returns coordinates in fixed-point format (degrees * 10000000)
#define GPS_COORD_DIVISOR 10000000.0

// Stepper motor pins (for parachute deployment mechanism)
#define STEPPER_PIN_1 10
#define STEPPER_PIN_2 6
#define STEPPER_PIN_3 A4
#define STEPPER_PIN_4 A2

// Sensor Update Intervals
#define GPS_UPDATE_INTERVAL 200    // GPS read interval (ms)
#define STEPPER_DELAY_INTERVAL 4000 // Stepper motor delay interval (ms)

// BMP280 Pressure Sensor
Adafruit_BMP280 bmp;
float pressure;
float pressure_calibration; // Calibration value at launch height
float temperature;
float altitude1;

// MPU6050 Accelerometer
Adafruit_MPU6050 mpu;
float x_acceleration;
float y_acceleration;
float z_acceleration;

// SD Card
File data;
File gps;
const int pin_cs = 5;

// GPS Module
Adafruit_GPS GPS(&GPSSerial);
String NMEA1;
String NMEA2;
char c;
float gps_longitude;
float gps_latitude;
float gps_altitude;
float gps_speed;

// Timing variables
unsigned long Time;
unsigned long prevTime;
unsigned long startT;
unsigned long prevTime2;
unsigned long prevTime3;

// Radio Configuration (RFM69)
const float RF69_FREQ = 434.50; // MHz
const int RFM69_CS = 11;
const int RFM69_INT = 9;
const int RFM69_RST = 12;
RH_RF69 rf69(RFM69_CS, RFM69_INT);
char packet[50];
static const int sendlength = 50;

bool error_flag = false;

// Buzzer
const int buzzerPin = A1;
const int powerPin = A5;

// Stepper motor state
int step_number = 0;
boolean figureCompleted = false;

void setup() {
  Serial.begin(115200);
  pinMode(buzzerPin, OUTPUT);

  // GPS Setup
  GPS.begin(9600);
  GPS.sendCommand("$PGCMD,33,0*6D"); // Turn Off GPS Antenna Update
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // RMC and GGA NMEA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);   // 5 Hz update rate
  delay(1000);

  // Stepper motor setup
  pinMode(STEPPER_PIN_1, OUTPUT);
  pinMode(STEPPER_PIN_2, OUTPUT);
  pinMode(STEPPER_PIN_3, OUTPUT);
  pinMode(STEPPER_PIN_4, OUTPUT);
  pinMode(powerPin, OUTPUT);
  
  digitalWrite(STEPPER_PIN_1, LOW);
  digitalWrite(STEPPER_PIN_2, LOW);
  digitalWrite(STEPPER_PIN_3, LOW);
  digitalWrite(STEPPER_PIN_4, LOW);

  // Wait for GPS signal
  Serial.println("Waiting for GPS signal...");
  while(!GPS.fix){
    readGPS();
    delay(1000);
    if(GPS.fix){
      Serial.println("GPS location fixed");
      buzzer();
      break;
    } else {
      Serial.println("Connecting to GPS satellites...");
    }
  }

  // Initialize sensors
  if (!bmp.begin()) {
    Serial.println("ERROR: BMP280 sensor not found!");
    error_loop();
  }
  
  if (!mpu.begin()) {
    Serial.println("ERROR: MPU6050 sensor not found!");
    error_loop();
  }

  // Configure accelerometer range
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);

  // SD Card Setup
  pinMode(pin_cs, OUTPUT);
  Serial.println("Initializing SD card...");
  while(!SD.begin()){
    Serial.println("Waiting for SD card...");
    delay(1000);
  }
  
  // Create data.csv file with header
  data = SD.open("data.csv", FILE_WRITE);
  if(data){
    Serial.println("Creating data.csv file...");
    data.print("time");
    data.print(",");
    data.print("pressure");
    data.print(",");
    data.print("temperature");
    data.print(",");
    data.print("altitude(MPU)");
    data.print(",");
    data.print("x_acceleration");
    data.print(",");
    data.print("y_acceleration");
    data.print(",");
    data.println("z_acceleration");
    data.close();
    Serial.println("data.csv created successfully");
    Serial.println("Data collection started");
    startT = millis();
    buzzer();
  } else {
    Serial.println("ERROR: Failed to create data.csv");
    error_loop();
  }

  // Create GPS CSV file with header
  gps = SD.open("gps.csv", FILE_WRITE);
  if(gps){
    gps.print("altitude");
    gps.print(",");
    gps.print("latitude");
    gps.print(",");
    gps.println("longitude");
    gps.close();
    Serial.println("gps.csv created successfully");
  } else {
    Serial.println("ERROR: Failed to create gps.csv");
    error_loop();
  }

  // Radio Setup
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  
  if (!rf69.init()) {
    Serial.println("ERROR: RFM69 radio init failed");
    error_loop();
  }
  
  rf69.setFrequency(RF69_FREQ);
  rf69.setTxPower(20, true); // Max power (20 dBm)
  
  // Encryption key (must match receiver)
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  Serial.print("RFM69 radio initialized @ ");
  Serial.print(RF69_FREQ);
  Serial.println(" MHz");

  // Calibrate pressure sensor (set current pressure to height = 0)
  pressure_calibration = BMP280_Pressure();
  Serial.print("Pressure calibration successful. Pressure at height 0: ");
  Serial.println(pressure_calibration);

  // Wait until altitude trigger (0+ meters) is reached
  Serial.println("Waiting for altitude trigger...");
  while(true){
    float altTrigger = BMP280_Altitude();
    if(altTrigger >= 0){
      buzzer();
      Serial.println("Altitude trigger reached - starting data collection");
      break;
    }
    delay(100);
  }

  prevTime = millis();
  Serial.println("=== CanSat AlphaSpot Ready ===");
  Serial.println("Data collection active");
}

void loop() {
  unsigned long currentTime = millis();

  // Check for errors
  if(error_flag){
    error_loop();
  }

  // Read sensor data
  pressure = BMP280_Pressure();
  temperature = BMP280_Temperature();
  altitude1 = BMP280_Altitude();
  x_acceleration = MPU_acceleration_x();
  y_acceleration = MPU_acceleration_y();
  z_acceleration = MPU_acceleration_z();
  Time = (millis()) / 1000; // Convert to seconds

  // Write sensor data to SD card
  write_to_SD(Time, pressure, temperature, altitude1, 
              x_acceleration, y_acceleration, z_acceleration);
  
  // Read GPS data at specified interval
  if((currentTime - prevTime > GPS_UPDATE_INTERVAL)){
    readGPS();
    if(GPS.fix){
      write_gps_to_SD(gps_latitude, gps_longitude, gps_altitude);
    }
    prevTime = currentTime;
  }
}

// Sensor reading functions
float BMP280_Pressure(){
  return bmp.readPressure();
}

float BMP280_Temperature(){
  return bmp.readTemperature();
}

float BMP280_Altitude(){
  return bmp.readAltitude(pressure_calibration / 100); // Convert to hPa
}

float MPU_acceleration_x(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  return a.acceleration.x;
}

float MPU_acceleration_y(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  return a.acceleration.y;
}

float MPU_acceleration_z(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  return a.acceleration.z;
}

// SD Card writing functions
void write_to_SD(float time, float pressure, float temperature, float altitude, 
                 float x_acc, float y_acc, float z_acc){
  data = SD.open("data.csv", FILE_WRITE);
  if(data){
    data.print(time);
    data.print(",");
    data.print(pressure);
    data.print(",");
    data.print(temperature);
    data.print(",");
    data.print(altitude);
    data.print(",");
    data.print(x_acc);
    data.print(",");
    data.print(y_acc);
    data.print(",");
    data.println(z_acc);
    data.close();
  } else {
    Serial.println("ERROR: Failed to open data.csv");
    error_flag = true;
  }
}

void write_gps_to_SD(float latitude, float longitude, float altitude){
  gps = SD.open("gps.csv", FILE_WRITE);
  
  if(gps){
    // Convert GPS coordinates from fixed-point to decimal degrees
    float lat_decimal = latitude / GPS_COORD_DIVISOR;
    float lon_decimal = longitude / GPS_COORD_DIVISOR;
    
    // Convert altitude to cm for transmission (multiply by 100)
    long altitude_cm = (long)(altitude * 100);
    
    // Transmit GPS data via radio
    senddata((long)latitude, (long)longitude, altitude_cm);
    
    // Write to SD card
    gps.print(altitude);
    gps.print(",");
    gps.print(lat_decimal, 7);  // 7 decimal places for accuracy
    gps.print(",");
    gps.println(lon_decimal, 7);
    gps.close();
  } else {
    Serial.println("ERROR: Failed to open gps.csv");
    error_flag = true;
  }
}

// GPS reading function
void readGPS(){
  // Read first NMEA sentence
  while(!GPS.newNMEAreceived()) {
    c = GPS.read();
  }
  GPS.parse(GPS.lastNMEA());
  NMEA1 = GPS.lastNMEA();
  
  // Read second NMEA sentence (for complete data)
  while(!GPS.newNMEAreceived()) {
    c = GPS.read();
  }
  GPS.parse(GPS.lastNMEA());
  NMEA2 = GPS.lastNMEA();
  
  // Extract GPS data
  gps_latitude = GPS.latitude_fixed;
  gps_longitude = GPS.longitude_fixed;
  gps_altitude = GPS.altitude;
  gps_speed = (GPS.speed) * 0.514444444; // Convert from knots to m/s
}

// Radio transmission function
void senddata(long lat, long lon, long alt){
  sprintf(packet, "%ld,%ld,%ld", lat, lon, alt);
  rf69.send((uint8_t *)packet, sendlength);
  rf69.waitPacketSent();
  Serial.println("GPS packet transmitted");
}

// Buzzer function (audio feedback)
void buzzer() {
  tone(buzzerPin, 1000);
  delay(500);
  noTone(buzzerPin);
  delay(500);
  tone(buzzerPin, 1000);
  delay(500);
  noTone(buzzerPin);
  delay(500);
}

// Stepper motor control functions (for parachute deployment mechanism)

void turnRight() {
  // Makes string shorter in one direction then longer back
  revolution(true);
  revolution(false);
}

void turnLeft() {
  // Makes string shorter in opposite direction
  revolution(false);
  revolution(true);
}

void corkscrew() {
  // Continuous rotation in one direction
  revolution(true);
}

void revolution(bool dir) {
  // One full revolution (2048 steps for stepper motor)
  for(int i = 0; i < 2048; i++) {
    OneStep(dir); 
    delay(2);
  }
}

void OneStep(bool dir){
  // Single step in specified direction
  if(dir){
    // Clockwise rotation
    switch(step_number){
      case 0:
        digitalWrite(STEPPER_PIN_1, HIGH);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 1:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, HIGH);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 2:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, HIGH);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 3:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, HIGH);
        break;
    }
  } else {
    // Counter-clockwise rotation
    switch(step_number){
      case 0:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, HIGH);
        break;
      case 1:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, HIGH);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 2:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, HIGH);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 3:
        digitalWrite(STEPPER_PIN_1, HIGH);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
    } 
  }
  
  // Move to next step
  step_number++;
  if(step_number > 3){
    step_number = 0;
  }
}

void stepperControl(){        
  // Control stepper motor sequence for parachute deployment
  digitalWrite(powerPin, HIGH);
  Serial.println("Starting stepper motor control sequence");
  
  if (!figureCompleted) {
    // Execute deployment sequence: right, left, right, left, corkscrew
    turnRight();
    prevTime2 = millis();
    prevTime3 = millis();
    measurementDelay();
    
    turnLeft();
    prevTime2 = millis();
    prevTime3 = millis();
    measurementDelay();
    
    turnRight();
    prevTime2 = millis();
    prevTime3 = millis();
    measurementDelay();
    
    turnLeft();
    prevTime2 = millis();
    prevTime3 = millis();
    measurementDelay();
    
    corkscrew();
    digitalWrite(powerPin, LOW);
    figureCompleted = true;
    Serial.println("Stepper motor sequence completed");
  }
}

void measurementDelay(){
  // Continue data collection during stepper motor delays
  bool measuring = true;
  
  while(measuring){
    unsigned long currentTime2 = millis();
    unsigned long currentTime3 = millis();
  
    // Read all sensors
    pressure = BMP280_Pressure();
    temperature = BMP280_Temperature();
    altitude1 = BMP280_Altitude();
    x_acceleration = MPU_acceleration_x();
    y_acceleration = MPU_acceleration_y();
    z_acceleration = MPU_acceleration_z();
    Time = (millis()) / 1000;

    // Write sensor data to SD card
    write_to_SD(Time, pressure, temperature, altitude1, 
                x_acceleration, y_acceleration, z_acceleration);
  
    // Read GPS data
    if((currentTime3 - prevTime3 > GPS_UPDATE_INTERVAL)){
      readGPS();
      if(GPS.fix){
        write_gps_to_SD(gps_latitude, gps_longitude, gps_altitude);
      }
      prevTime3 = currentTime3;
    }

    // Exit delay after specified interval
    if((currentTime2 - prevTime2 > STEPPER_DELAY_INTERVAL)){
      measuring = false;
    }
  }
}

// Error handling function
void error_loop() {
  // Enter error state - flash buzzer continuously
  Serial.println("=== ERROR STATE ===");
  Serial.println("System halted due to critical error");
  
  while(true) {
    buzzer();
    delay(2000);
  }
}
