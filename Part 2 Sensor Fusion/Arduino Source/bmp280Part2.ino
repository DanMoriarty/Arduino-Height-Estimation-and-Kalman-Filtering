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
#include <MatrixMath.h>
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

int initWait =50;
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

int c = 5000;
int Vin = A4;
int XPIN = A3;
int YPIN = A2;
int ZPIN = A1;
int Gnd = A0;
double z0 = 0.0;
double zValue = 0.0;
double yValue = 0.0;
double xValue = 0.0;


//State Space Matrices
#define N 3
#define M 2
double X[N][1];
double Xdot[1][N];
double A[N][N];
double B[N][1];
double U=0;
double C1[1][N];
double C3[1][N];
double C[2][N];
double x1=0.0;
double x2=0.0;
double x3=0.0;

//Kalman Filter Matrices
double Q[N][N];
double R[N][N];
double P[N][N];
double K[N][2];
double L1[N] = {0.0018,0.0000,0.0020};
double L3[N] = {0.0235,0.0000,0.0255};

//Covariances
double wcovar[N] = {0.6088, 0.001, 9.1561};
double W[1][N];
double vcovar[M] = {1, 0.5962};//0.1367
double V[1][M];

double m = 37 + 1.3 + 1.27;

void printDouble( double val, unsigned int precision){
// prints val with number of decimal places determine by precision
// NOTE: precision is 1 followed by the number of zeros for the desired number of decimial places
// example: printDouble( 3.1415, 100); // prints 3.14 (two decimal places)

   Serial.print (int(val));  //prints the int part
   Serial.print("."); // print the decimal point
   unsigned int frac;
   if(val >= 0)
     frac = (val - int(val)) * precision;
   else
      frac = (int(val)- val ) * precision;
   int frac1 = frac;
   while( frac1 /= 10 )
       precision /= 10;
   precision /= 10;
   while(  precision /= 10)
       Serial.print("0");

   Serial.print(frac,DEC) ;
}

void printDoubleln( double val, unsigned int precision){
  printDouble(val, precision);
  Serial.println();
}

void MatrixPrint(double* A, int m, int n, String label)
{
  // A = input matrix (m x n)
  int i, j;
  Serial.println();
  Serial.println(label);
  for (i = 0; i < m; i++)
  {
    for (j = 0; j < n; j++)
    {
      printDouble(A[n * i + j],1000);
      Serial.print("\t");
    }
    Serial.println();
  }
}

void MatrixTranspose(double* A, int m, int n, double* C)
{
  // A = input matrix (m x n)
  // m = number of rows in A
  // n = number of columns in A
  // C = output matrix = the transpose of A (n x m)
  int i, j;
  for (i = 0; i < m; i++)
    for(j = 0; j < n; j++)
      C[m * j + i] = A[n * i + j];
}
void MatrixAdd(double* A, double* B, int m, int n, double* C)
{
  // A = input matrix (m x n)
  // B = input matrix (m x n)
  // m = number of rows in A = number of rows in B
  // n = number of columns in A = number of columns in B
  // C = output matrix = A+B (m x n)
  int i, j;
  for (i = 0; i < m; i++)
    for(j = 0; j < n; j++)
      C[n * i + j] = A[n * i + j] + B[n * i + j];
}

void MatrixMultiply(double* A, double* B, int m, int p, int n, double* C)
{
  // A = input matrix (m x p)
  // B = input matrix (p x n)
  // m = number of rows in A
  // p = number of columns in A = number of rows in B
  // n = number of columns in B
  // C = output matrix = A*B (m x n)
  int i, j, k;
  for (i = 0; i < m; i++)
    for(j = 0; j < n; j++)
    {
      C[n * i + j] = 0;
      for (k = 0; k < p; k++)
        C[n * i + j] = C[n * i + j] + A[p * i + k] * B[n * k + j];
    }
}
int MatrixInvert(double* A, int n)
{
  // A = input matrix AND result matrix
  // n = number of rows = number of columns in A (n x n)
  int pivrow;   // keeps track of current pivot row
  int k, i, j;    // k: overall index along diagonal; i: row index; j: col index
  int pivrows[n]; // keeps track of rows swaps to undo at end
  float tmp;    // used for finding max value and making column swaps

  for (k = 0; k < n; k++)
  {
    // find pivot row, the row with biggest entry in current column
    tmp = 0;
    for (i = k; i < n; i++)
    {
      if (abs(A[i * n + k]) >= tmp) // 'Avoid using other functions inside abs()?'
      {
        tmp = abs(A[i * n + k]);
        pivrow = i;
      }
    }

    // check for singular matrix
    if (A[pivrow * n + k] == 0.0f)
    {
      Serial.println("Inversion failed due to singular matrix");
      return 0;
    }

    // Execute pivot (row swap) if needed
    if (pivrow != k)
    {
      // swap row k with pivrow
      for (j = 0; j < n; j++)
      {
        tmp = A[k * n + j];
        A[k * n + j] = A[pivrow * n + j];
        A[pivrow * n + j] = tmp;
      }
    }
    pivrows[k] = pivrow;  // record row swap (even if no swap happened)

    tmp = 1.0f / A[k * n + k];  // invert pivot element
    A[k * n + k] = 1.0f;    // This element of input matrix becomes result matrix

    // Perform row reduction (divide every element by pivot)
    for (j = 0; j < n; j++)
    {
      A[k * n + j] = A[k * n + j] * tmp;
    }

    // Now eliminate all other entries in this column
    for (i = 0; i < n; i++)
    {
      if (i != k)
      {
        tmp = A[i * n + k];
        A[i * n + k] = 0.0f; // The other place where in matrix becomes result mat
        for (j = 0; j < n; j++)
        {
          A[i * n + j] = A[i * n + j] - A[k * n + j] * tmp;
        }
      }
    }
  }

  // Done, now need to undo pivot row swaps by doing column swaps in reverse order
  for (k = n - 1; k >= 0; k--)
  {
    if (pivrows[k] != k)
    {
      for (i = 0; i < n; i++)
      {
        tmp = A[i * n + k];
        A[i * n + k] = A[i * n + pivrows[k]];
        A[i * n + pivrows[k]] = tmp;
      }
    }
  }
  return 1;
}
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
  //Initial Conditions
  Serial.println("\nINITIAL CONDITIONS:");
  double p[N]={0.0, 0.0, 0.0};
  MatrixTranspose((double*)p, 1, N, (double*) X);
  MatrixTranspose((double*)p, 1, N, (double*) Xdot);
  MatrixPrint((double*)X, N, 1, "x0");
  MatrixPrint((double*)Xdot, N, 1, "xdot0");
  Serial.println("\n u0:");
  Serial.println(U);


  //Covariances
  Serial.println("\nCOVARIANCES");
    //Q
  MatrixTranspose((double*) wcovar, 1, N, (double*) W);
  MatrixMultiply((double*)W, (double*)wcovar, N, 1, N, (double*)Q);  
  MatrixPrint((double*)Q, N, N, "Q");
  
    //R
  MatrixTranspose((double*) vcovar, 1, M, (double*) V);
  MatrixMultiply((double*)V, (double*)vcovar, M, 1, M, (double*)R);  
  MatrixPrint((double*)R, M, M, "R");
  
   //P  
  MatrixPrint((double*)P, N, N, "P0");
  
  
  //State Space
  Serial.println("\nSTATE SPACE");
    //A
  A[0][1] = 1;
  A[1][2] = 1;
  MatrixPrint((double*)A, N, N, "A");
   //B
  B[0][0]=0.0;
  B[1][0]=0.0;
  B[2][0]=1/m;
  MatrixPrint((double*)B, N, 1, "B");
    //C1
  for (int i = 0;i<N;i++) {
    C1[0][i] = 0;
    C[0][i] = 0;
    C[1][i] = 0;
  }
  C1[0][0]=1;
  C[0][0]=1;  
  MatrixPrint((double*)C1, 1, N, "C1");  
    //C3
  for (int i = 0;i<N;i++) {
    C3[0][i] = 0.0;
  }
  C3[0][2]=1.0;
  C[1][2]=1.0;  
  MatrixPrint((double*)C3, 1, N, "C3");

  MatrixPrint((double*)C, 2, N, "C");
  
  Serial.println("\nSAMPLE TIME");
  Serial.print(1);
  Serial.println(" ms");

  x1 = X[0][0];
  x2 = X[0][1];
  x3 = X[0][2];
  
  
  
  Serial.println("........................");
  Serial.println("\nSTART");
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
    double t1 = micros();  
    digitalWrite(VCC,HIGH);
    digitalWrite(GND,LOW);
    analogWrite(Gnd,0);
    analogWrite(Vin,(3.3/5)*255);
    //Serial.print("c: ");
    //Serial.println(c);
    if(c==0)
      alt0=alt;
      //alt0 = bmp.readAltitude(sealevel);
    if (c>0){
//getting initial reading
    /*
    for (count=initWait;count>0;count--){
      //xValue = analogRead(XPIN);
      //yValue = analogRead(YPIN);
      zValue += analogRead(ZPIN);

      alt += bmp.readAltitude(sealevel);
      
    }

    alt0 = alt /initWait;
    */
    alt = (alt +bmp.readAltitude(sealevel))/2;
    }
    if (c<0) {
//acceleration
    //xValue = analogRead(XPIN);
    //yValue = analogRead(YPIN);
    zValue = analogRead(ZPIN);

    alt = bmp.readAltitude(sealevel);

    relalt = alt-alt0;


  
  
  //HEIGHT FILTERING
    //Predicted X
  double x1pred = A[0][0]*x1 + A[0][1]*x2 + A[0][2]*x3 + B[0][0]*U;
  double x2pred = A[1][0]*x1 + A[1][1]*x2 + A[1][2]*x3 + B[1][0]*U;
  double x3pred = A[2][0]*x1 + A[2][1]*x2 + A[2][2]*x3 + B[2][0]*U;
   
    //Predicted P
  double AT[N][N];
  MatrixTranspose((double*) A, N, N, (double*) AT);
  double a[N][N];
  double b[N][N];
  MatrixMultiply((double*)AT, (double*)P, N, N, N, (double*)a);
  MatrixMultiply((double*)a, (double*)A, N, N, N, (double*)b);
  MatrixAdd((double*) A, (double*) Q, N, N, (double*) P); 
  

    //Finding K
  double PCT[N][2];  
  double CT[N][2];
  double d[2][2];
  double k[2][2];
  MatrixTranspose((double*) C, 2, N, (double*) CT);
  MatrixMultiply((double*)P, (double*)CT, N, N, 2, (double*)PCT);
  MatrixMultiply((double*)C, (double*)PCT, 2, N, 2, (double*)d);
  MatrixAdd((double*) d, (double*) R, 2, 2, (double*) k);
  if(MatrixInvert((double*) k, 2)) {
    MatrixMultiply((double*)PCT, (double*)k, N, 2, 2, (double*)K);

    double L = K[0][0]; 
    double CCC = C[0][0]; 

    double predX;
    if ((c>-1000))
      predX = 0;
    if ((c<=-1000)&&(c>-2000))
      predX = 1;
    if ((c<=-2000)&&(c>-3000))
      predX = 2;
    if (c<=-3000)
      predX = 0;
    
    x1 =predX-L*(CCC*x1-relalt);
    x1 = CCC*x1;    
    
    Serial.print(L);
    Serial.print(", ");
    Serial.print(alt0);
    Serial.print(", ");
    Serial.print(relalt);
    Serial.print(", ");
    //if (L<0)
      //printDouble(-1*x1,10000);
    //else
      printDouble(x1,10000);
    Serial.print(", ");
    Serial.print(predX);
    
    }else {
      Serial.println("inversion error");
    } 
    if ((c>-1000))
      Serial.println();
    if ((c<=-1000)&&(c>-2000))
    Serial.println(",      1m................ ");
    if ((c<=-2000)&&(c>-3000))
    Serial.println(",      2m++++++++++++++++++++ ");
    if (c<=-3000)
    Serial.println(",      0m/////////////////////// ");
  }
  
  c--;
    double t2 = micros();
    delay(1-0.001*(t2-t1));
    
}
