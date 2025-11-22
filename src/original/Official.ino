#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SD.h>
#include <RH_RF69.h>
#include <Adafruit_GPS.h> //Load the GPS Library. Make sure you have installed the library form the adafruit site above
#define GPSSerial Serial1

#define STEPPER_PIN_1 10
#define STEPPER_PIN_2 6
#define STEPPER_PIN_3 A4
#define STEPPER_PIN_4 A2

int step_number = 0;
boolean figureCompleted = false;
//BMP280
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp; // I2C

float pressure;
float pressure_calibration; //for calibration of altimeter, this calibration happens when sensor is turned on
float prepressure = 0; //for calculating when the cansat is going down in altitude
bool runned = true;
//bool calibration = true; //turns false after first run. First run is assumed to be at heigth 0
float temperature;
float altitude1;

//MPU6050
Adafruit_MPU6050 mpu;

float x_acceleration; //graph with 3 lines
float y_acceleration;
float z_acceleration;

int dalen = 0;
bool descent = false;

//SD card
File data;
File gps;
int pin_cs = 5;

//gps
Adafruit_GPS GPS(&GPSSerial); //Create GPS object
String NMEA1;  //We will use this variable to hold our first NMEA sentence
String NMEA2;  //We will use this variable to hold our second NMEA sentence
char c;       //Used to read the characters spewing from the GPS module
float gps_longitude;
float gps_latitude;
float gps_altitude;
float gps_speed;
int y = 10000000.0000000000;

//time
unsigned long Time;
unsigned long prevTime;
unsigned long startT;
unsigned long stopT;
unsigned long prevTime2;
unsigned long prevTime3;

//for testing 5000 times
int x = 0;

//radio (only for gps coordinates)
int RF69_FREQ = 434.50;
int RFM69_CS = 11;
int RFM69_INT = 9;
int RFM69_RST = 12;
RH_RF69 rf69(RFM69_CS, RFM69_INT);
char packet[50];
static int sendlength = 50;

bool ex = false;

//buzzer

const int buzzerPin = A1;

int powerPin = A5;

void setup() {
  Serial.begin(115200);
  pinMode(buzzerPin, OUTPUT);

  //setup gps
  GPS.begin(9600);       //Turn GPS on at baud rate of 9600
  GPS.sendCommand("$PGCMD,33,0*6D"); // Turn Off GPS Antenna Update
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Tell GPS we want only $GPRMC and $GPGGA NMEA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);   // 1 Hz update rate - kan 1HZ, 5HZ of 10HZ zijn
  delay(1000);  //Pause

  pinMode(STEPPER_PIN_1, OUTPUT);
  pinMode(STEPPER_PIN_2, OUTPUT);
  pinMode(STEPPER_PIN_3, OUTPUT);
  pinMode(STEPPER_PIN_4, OUTPUT);

  pinMode(powerPin, OUTPUT);
  
  digitalWrite(STEPPER_PIN_1, LOW);
  digitalWrite(STEPPER_PIN_2, LOW);
  digitalWrite(STEPPER_PIN_3, LOW);
  digitalWrite(STEPPER_PIN_4, LOW);

  //wachten tot gps een signaal heeft -> wanneer latitude != 0
  bool waiting = true;
  while(waiting){
    readGPS();
    delay(1000);
    if(GPS.fix){
      Serial.println("location is fixed");
      buzzer();
      waiting = false;
      }else{
      
    Serial.println("connecting.....");
    }
  }
  //Start sensors
  bmp.begin();
  mpu.begin();

  //Setup accelerometer
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  //mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);


  //SD card
  pinMode(pin_cs, OUTPUT);
  //wachten tot sd een signaal heeft ->
  waiting = true;
  while(waiting){
    SD.begin();
    data = SD.open("data.csv", FILE_WRITE); //csv bestand maken
    if(data){
      Serial.println("\n csv bestand aanmaken");
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
      Serial.println("csv_bestand aangemaakt");
      
      Serial.println("metingen zijn gestart");
      startT = millis();
      buzzer();
      waiting = false;
    
      }
      else {
      Serial.println("waiting for SD card");
    }
 }

  gps = SD.open("gps.csv", FILE_WRITE);
  
  if(gps){
    //Serial.println("gps_bestand aangemaakt");
    gps.print("altitude");
    gps.print(",");
    gps.print("latitude");
    gps.print(",");
    gps.println("longitude");
    gps.close();
    }
    else {
    Serial.println("error opening gps.csv");

    }

  //Setup radio
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);
  rf69.init();
  rf69.setFrequency(434.50);
  rf69.setTxPower(20, true); //max power
  uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
                    0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
  rf69.setEncryptionKey(key);
  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz"); //in code -> verstuurt via freq

  //calibrating pressure sensor (setting current pressure to height = 0)
  pressure_calibration = BMP280_Pressure();
  Serial.print("calibration is succesfull. Pressure at height 0 is: ");
  Serial.println(pressure_calibration);

  //waiting till a heigth of 3 meters has been reached
  waiting = false;
  while(waiting){
    float altTrigger = BMP280_Altitude();
    if(altTrigger >= 0){
      buzzer();
      waiting = false;
      }
    }

  //threading (getting gps data takes 2 seconds and a lot of data from the other sensors is lost in that time period. With threading we can run different tasks at the same time.
  prevTime = millis();
  Serial.println("start");
}

void loop() {
  unsigned long currentTime = millis(); //for threading

  //retrieving sensor data
  
  pressure = BMP280_Pressure();
  temperature = BMP280_Temperature();
  altitude1 = BMP280_Altitude();
  x_acceleration = (MPU_acceleration_x()); //for absolute values use -> abs()
  y_acceleration = (MPU_acceleration_y());
  z_acceleration = (MPU_acceleration_z());
  Time = (millis())/1000; //convert to seconds

  /*if (pressure > prepressure){
    dalen++;
    }else{
      dalen = 0;
      }
      
  if(dalen > -1){
    descent = true;
    Serial.println("dalen");
    }    

  if(descent) {
    stepperControl();
    }
    
  prepressure = pressure;*/
  
  /*Serial.print(Time);
  Serial.print('\t');
  Serial.print(pressure);
  Serial.print('\t');
  Serial.print(temperature);
  Serial.print('\t');
  Serial.print(altitude1);
  Serial.print('\t');
  Serial.print(x_acceleration);
  Serial.print('\t');
  Serial.print(y_acceleration);
  Serial.print('\t');
  Serial.print(z_acceleration);*/

  write_to_SD(Time,pressure,temperature,altitude1,x_acceleration,y_acceleration,z_acceleration);
  
  //retrieving GPS data
  if((currentTime - prevTime > 200)){ //this is the most optimal waiting time for having enough data but also coorinates.
    readGPS();
    if(GPS.fix){
      /*Serial.print(gps_latitude);
      Serial.print('\t');
      Serial.print(gps_longitude);
      Serial.print('\t');
      Serial.print(gps_altitude);
      //Serial.print('\t');
      //Serial.print(gps_speed);*/
      write_gps_to_SD(gps_latitude,gps_longitude,gps_altitude);
    }
      prevTime = currentTime;
    }

   if(ex){
    stopT = millis();

    Serial.println(stopT - startT);
    exit(0);
    }
  x++;
  //delay(100);

}

float BMP280_Pressure(){
  pressure = bmp.readPressure();

  return pressure;
  
  }

float BMP280_Temperature(){
  temperature = bmp.readTemperature();

  return temperature;
}

float BMP280_Altitude(){
  altitude1 = bmp.readAltitude(pressure_calibration/100); //converting to hPa

  return altitude1;
  
  }
float MPU_acceleration_x(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  x_acceleration = a.acceleration.x;

  return x_acceleration;
  
}

float MPU_acceleration_y(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  y_acceleration = a.acceleration.y;

  return y_acceleration;
  
}

float MPU_acceleration_z(){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  z_acceleration = a.acceleration.z;

  return z_acceleration;
  
}

void write_to_SD(float a,float b,float c,float d,float e, float f, float g){
  data = SD.open("data.csv", FILE_WRITE);
  if(data){
    //Serial.println("\n data opslaan naar kaart....");
    data.print(a);
    data.print(",");
    data.print(b);
    data.print(",");
    data.print(c);
    data.print(",");
    data.print(d);
    data.print(",");
    data.print(e);
    data.print(",");
    data.print(f);
    data.print(",");
    data.println(g);
    data.close();
    //Serial.println("Done");
    
    }
    else {
    Serial.println("error opening data.csv");
    ex = true;
  }
}

void write_gps_to_SD(float a, float b, float c){
  gps = SD.open("gps.csv", FILE_WRITE);
  gps_altitude = int(gps_altitude * 100);
  senddata(gps_latitude, gps_longitude, gps_altitude);
  if(gps){
    //Serial.println("gps data opslaan naar kaart");
    a = a/y;
    gps.print(c);
    gps.print(",");
    b = b/y;
    gps.print(a,7);
    gps.print(",");
    gps.println(b,7);
    gps.close();
    //Serial.println("Done");
    }
    else{
      Serial.println("error opening gps.txt");
      }
  
  }
  
void readGPS(){  //This function will read and remember two NMEA sentences from GPS
  //clearGPS();    //Serial port probably has old or corrupt data, so begin by clearing it all out

  //Serial.println("checking");
  while(!GPS.newNMEAreceived()) { //Keep reading characters in this loop until a good NMEA sentence is received    
     c=GPS.read(); //read a character from the GPS
    }
    
  GPS.parse(GPS.lastNMEA());  //Once you get a good NMEA, parse it
  NMEA1=GPS.lastNMEA();      //Once parsed, save NMEA sentence into NMEA1
  
  while(!GPS.newNMEAreceived()) {  //Go out and get the second NMEA sentence, should be different type than the first one read above.
      c=GPS.read();
    }
    
  GPS.parse(GPS.lastNMEA());
  NMEA2=GPS.lastNMEA();
  
  gps_latitude = GPS.latitude_fixed;
  gps_longitude = GPS.longitude_fixed;
  gps_altitude = GPS.altitude;
  gps_speed = (GPS.speed)*0.514444444;//convert from knots to m/s
}

void clearGPS() {  //Since between GPS reads, we still have data streaming in, we need to clear the old data by reading a few sentences, and discarding these
  
  while(!GPS.newNMEAreceived()) {
      c=GPS.read();
    }
    
  GPS.parse(GPS.lastNMEA());
  
  while(!GPS.newNMEAreceived()) {
      c=GPS.read();
    }
    
  GPS.parse(GPS.lastNMEA());
 
}

void senddata(int x, int y, int z){
  //Serial.println("making packet....");
  sprintf(packet, "%d,%d,%d", x,y,z);
  //Serial.println(packet);
  rf69.send((uint8_t *)packet, sendlength); //sent encoded packet
  rf69.waitPacketSent();
  Serial.println("packet is sent");
  
  //delay(1000);

}

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

void turnRight() {                        //maakt effe touwtje korter in de ene richting en dan terug langer
  revolution(true);
  revolution(false);
}

void turnLeft() {
  revolution(false);
  revolution(true);
}

void corkscrew() {
  revolution(true);
}

void revolution(bool dir) {
  for(int i = 0; i < 2048; i++) {
    OneStep(dir); 
    delay(2);
    }
}

void OneStep(bool dir){
    if(dir){
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
  }else{
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
     
      
    } 
  }
  step_number++;
    if(step_number > 3){
      step_number = 0;
  }
}

void stepperControl(){        
    digitalWrite(powerPin, HIGH);
    Serial.println("stepper1");
    if (figureCompleted == false) {
    turnRight();
    prevTime2 = millis();
    prevTime3 = millis();
    metingenDelay_4();
    turnLeft();
    Serial.println("stepper2");
    prevTime2 = millis();
    prevTime3 = millis();
    metingenDelay_4();
    turnRight();
    Serial.println("stepper3");
    prevTime2 = millis();
    prevTime3 = millis();
    metingenDelay_4();
    turnLeft();
    prevTime2 = millis();
    prevTime3 = millis();
    metingenDelay_4();
    corkscrew();
    digitalWrite(powerPin, LOW);
    figureCompleted = true;
    Serial.println("verder meten");
  }
}

void metingenDelay_4(){
  bool meten = true;
  while(meten){
    unsigned long currentTime2 = millis(); //for threading
    unsigned long currentTime3 = millis(); //for threading
  
    pressure = BMP280_Pressure();
    temperature = BMP280_Temperature();
    altitude1 = BMP280_Altitude();
    x_acceleration = (MPU_acceleration_x()); //for absolute values use -> abs()
    y_acceleration = (MPU_acceleration_y());
    z_acceleration = (MPU_acceleration_z());
    Time = (millis())/1000; //convert to seconds

    write_to_SD(Time,pressure,temperature,altitude1,x_acceleration,y_acceleration,z_acceleration);
  
  //retrieving GPS data
    if((currentTime3 - prevTime3 > 200)){ //this is the most optimal waiting time for having enough data but also coorinates.
      readGPS();
      if(GPS.fix){
      /*Serial.print(gps_latitude);
      Serial.print('\t');
      Serial.print(gps_longitude);
      Serial.print('\t');
      Serial.print(gps_altitude);
      //Serial.print('\t');
      //Serial.print(gps_speed);*/
        write_gps_to_SD(gps_latitude,gps_longitude,gps_altitude);
      }
        prevTime3 = currentTime3;
      }

   if((currentTime2 - prevTime2 > 4000)){

    meten = false;}
  
}
  }
