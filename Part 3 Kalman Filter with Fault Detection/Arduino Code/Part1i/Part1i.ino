
#include <MatrixMath.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

//ACCELEROMETER PINS
int Vin = A4;
int XPIN = A3;
int YPIN = A2;
int ZPIN = A1;
int CHECKPIN = A7;
int Gnd = A0;

int Vin2 = A12;
int XPIN2 = A11;
int YPIN2 = A10;
int ZPIN2 = A9;
int CHECKPIN2 = A15;
int Gnd2 = A8;

//ACCELEROMETER VARIABLES
double zValue1 = 0.0;
double yValue1 = 0.0;
double xValue1 = 0.0;

double zValue2 = 0.0;
double yValue2 = 0.0;
double xValue2 = 0.0;


void setup() {

  pinMode(A0,OUTPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  pinMode(A4,OUTPUT);
  
  pinMode(A8,OUTPUT);
  pinMode(A9,INPUT);
  pinMode(A10,INPUT);
  pinMode(A11,INPUT);
  pinMode(A12,OUTPUT);
    
  Serial.begin(9600);
  

}

void loop() {
  
  analogWrite(Gnd,0);
  analogWrite(Vin,(3.3/5)*255);
  analogWrite(Gnd2,0);
  analogWrite(Vin2,(3.3/5)*255);
  
 

    
    zValue1 = analogRead(ZPIN);
    zValue2 = analogRead(ZPIN2);
    int check1 = analogRead(CHECKPIN);
    int check2 = analogRead(CHECKPIN2);
    
    double zValue = zValue1;
    //double zValue = faultdetect(zValue1,zValue2, zValue);
    
    double U= (zValue-420)/70; //calibration
 
    Serial.print("SENSOR READINGS: "); 
    Serial.print(U); 
    Serial.print(", ");
    Serial.print((zValue2-420)/70);
    Serial.print(", ");
    Serial.print(check1);
    Serial.print(", ");
    Serial.print(check2);
    Serial.println();
  delay(1);
}



