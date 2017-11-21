/***************************************************************************
  This is a library for the BMP280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BMEP280 Breakout 
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required 
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define VCC 7
#define GND 6
#define BMP_SCK 5 //scl
#define BMP_MISO 2 //sdo
#define BMP_MOSI 4 //sda
#define BMP_CS 3//csb

int initWait =10;
int count = 0;
//Adafruit_BMP280 bme; // I2C
//Adafruit_BMP280 bme(BMP_CS); // hardware SPI
Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
double pres=0.0;
double alt=0.0;
double sealevel = 1013.0;
double alt0=0.0;
double pres0=0.0;
double height0=0.0;
double relpres=0.0;
double relalt=0.0;
double height = 0.0;
double relHeight = 0.0;


int Vin = A4;
int XPIN = A3;
int YPIN = A2;
int ZPIN = A1;
int Gnd = A0;
double z0 = 0.0;
double zValue = 0.0;
double yValue = 0.0;
double xValue = 0.0;

double calculateHeight(double p) {
  //Serial.println((p-initPressure)/initPressure);
  //Serial.println(pow((p-initPressure)/initPressure,0.19));

  return 44330*(1-pow(pres/pres0,0.19));
}

void setup() {
  pinMode(VCC,OUTPUT);
  pinMode(GND,OUTPUT);
  pinMode(A0,OUTPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  pinMode(A4,OUTPUT);
  
  digitalWrite(VCC,HIGH);
  digitalWrite(GND,LOW);
    
  Serial.begin(9600);
  
  Serial.println(F("BMP280 test"));
  
  if (!bmp.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  Serial.print("Acceleration, ");
  Serial.print("Temperature, ");
  Serial.print("Pressure,  ");
  Serial.print("Relative Pressure, ");
  Serial.print("Approx altitude, ");
  Serial.print("Relative altitude, ");
  Serial.print("Calculated Height, ");
  Serial.println("Relative Calculated Height, ");
  
}
  
void loop() {
  
    digitalWrite(VCC,HIGH);
    digitalWrite(GND,LOW);
    analogWrite(Gnd,0);
    analogWrite(Vin,(3.3/5)*255);
//getting initial reading
    for (count=initWait;count>0;count--){
      //xValue = analogRead(XPIN);
      //yValue = analogRead(YPIN);
      zValue += analogRead(ZPIN);
      pres += bmp.readPressure();  
      alt += bmp.readAltitude(sealevel);
    }
    z0 = zValue/initWait;
    pres0 = pres / initWait;
    alt0 = alt /initWait;
    
    for (count=initWait;count>0;count--){
          height += calculateHeight(pres);
    }
    height0 = height/initWait;
    
//acceleration
    //xValue = analogRead(XPIN);
    //yValue = analogRead(YPIN);
    zValue = analogRead(ZPIN);
    Serial.print(zValue);
    //Serial.print(" ");
    //Serial.print(yValue);
    //Serial.print(" ");
    //Serial.print(xValue);
    Serial.print(",         ");
//temperature
    //Serial.print("Temperature = ");
    Serial.print(bmp.readTemperature());
    Serial.print(",       ");
    //Serial.println(" *C");

//pressure
    // Serial.print("Pressure = ");
    pres=bmp.readPressure();
    Serial.print(pres);
    Serial.print(", ");
    
//relative pressure
    // Serial.print("Relative Pressure = ");
    relpres = pres-pres0;
    Serial.print(relpres);
    Serial.print(",         ");

//Altitude
    //Serial.print("Approx altitude = ");
    alt = bmp.readAltitude(sealevel);
    Serial.print(alt); // this should be adjusted to your local forcase
    Serial.print(",           ");
    //Serial.println(" m");

//relative altitude
    //Serial.print("Approx altitude = ");
    relalt = alt-alt0;
    Serial.print(relalt); // this should be adjusted to your local forcase
    //Serial.println(" m");
    Serial.print(",             ");

//Calculated height
    height = calculateHeight(pres);
    Serial.print(height);
    //Serial.println(" Pa");
    Serial.print(",            ");

//Relative Calculated height
    relHeight = height - height0;
    Serial.print(relHeight);
    //Serial.println(" Pa");
    Serial.print(",    ");
    
    Serial.println();
    delay(1);
    
}
