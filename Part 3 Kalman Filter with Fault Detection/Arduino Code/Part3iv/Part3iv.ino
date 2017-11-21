
#include <MatrixMath.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

//BMP
#define VCC 7
#define GND 6
#define BMP_SCK 5 //scl
#define BMP_MISO 2 //sdo
#define BMP_MOSI 4 //sda
#define BMP_CS 3//csb

//Adafruit_BMP280 bme; // I2C
//Adafruit_BMP280 bme(BMP_CS); // hardware SPI
Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

boolean faultyalt=false;
double faultysensor=0;
boolean check1out=false;
boolean check2out=false;
boolean faultyaccels=false;
//DATA SAMPLING
int initWait =50;
int count = 0;
int initflush=0;
double dt = 0.1;
double m = 37 + 1.3 + 1.27;

//BMP VARIABLES
double pres=0.0;
double alt=0.0;
double lastalt=0.0;
double alt0=999;
double sealevel = 1008.66;//1013.0;
double pres0=0.0;

double base=419;
boolean restdetected=true;
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

double zValue=0;
double z1init=999;
double z2init=999;
double initbiasdif=999;

//STATE SPACE MATRICES
#define N 2
#define M 1
double X[N][1];
double Xdot[1][N];
double A[N][N];
double B[N][1];
double U=0;
double C1[1][N];
double C3[1][N];
//double C[2][N];
double x1=0.0;
double x2=0.0;
double x3=0.0;
double x1pred;
double x2pred;
double x1model;
double x2model;

//Kalman Filter Matrices
double Q[N][N];
double R[N][N];
double P[N][N];
double K[N][2];
//double L1[N] = {0.0018,0.0000,0.0020};
//double L3[N] = {0.0235,0.0000,0.0255};

//Covariances
double wcovar[N] = {0.1, 0.1};//, 9.1561};
double W[1][N];
double vcovar[M] = {0.1};//, 0.5962};//0.1367
double V[1][M];

//FAULT DETECTION
#define WINDOW 10
double accels[WINDOW];
double accels1[WINDOW];
double accels2[WINDOW];
double honestaccels[WINDOW];
boolean faultdetected = false;
int faultcode=0;
boolean initdivergence=false;
boolean corrected=false;

void(* resetFunc) (void) = 0; //declare reset function @ address 0

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
  pinMode(CHECKPIN,INPUT);
  
  pinMode(A8,OUTPUT);
  pinMode(A9,INPUT);
  pinMode(A10,INPUT);
  pinMode(A11,INPUT);
  pinMode(A12,OUTPUT);
  pinMode(CHECKPIN2,INPUT);
  
  digitalWrite(VCC,HIGH);
  digitalWrite(GND,LOW);
    
  Serial.begin(115200);
  
  //Serial.println(F("BMP280 test"));
  
  if (!bmp.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check that it has been installed");
    delay(1000);
    resetFunc();
    while (1);
  }
  //Initial Conditions
  //Serial.println("\nINITIAL CONDITIONS:");
  double p[N]={0.0, 0.0};//, 0.0};
  MatrixTranspose((double*)p, 1, N, (double*) X);
  MatrixTranspose((double*)p, 1, N, (double*) Xdot);
  //MatrixPrint((double*)X, N, 1, "x0");
  //MatrixPrint((double*)Xdot, N, 1, "xdot0");
  //Serial.println("\n u0:");
  //Serial.println(U);


  //Covariances
  //Serial.println("\nCOVARIANCES");
    //Q
  MatrixTranspose((double*) wcovar, 1, N, (double*) W);
  MatrixMultiply((double*)W, (double*)wcovar, N, 1, N, (double*)Q);  
  //MatrixPrint((double*)Q, N, N, "Q");
  
    //R
  MatrixTranspose((double*) vcovar, 1, M, (double*) V);
  MatrixMultiply((double*)V, (double*)vcovar, M, 1, M, (double*)R);  
  //MatrixPrint((double*)R, M, M, "R");
  
   //P  
  //MatrixPrint((double*)P, N, N, "P0");
  
  
  //State Space
  //Serial.println("\nSTATE SPACE");
    //A
    /////CONTINUOUS
    //A[0][1]=1;  
    /////DISCRETE
  A[0][0]=1;  
  A[0][1]=dt;  
  A[1][1]=1;
  

  //MatrixPrint((double*)A, N, N, "A");
   //B
/////CONTINUOUS
   //B[1][0]=1;
/////DISCRETE
  B[0][0]=0.5*dt*dt;
  B[1][0]=dt;

  //MatrixPrint((double*)B, N, 1, "B");
    //C1

  C1[0][0]=1;

  //MatrixPrint((double*)C1, 2, N, "C");

  x1pred = X[0][0];
  x2pred = X[1][0];
  x1model = X[0][0];
  x2model = X[1][0];
}
double readsensor(double ref){
  bmp.readTemperature();
  
  return bmp.readAltitude(ref);
}
void loop() {
  count++;
  double t1 = micros();  
  
  digitalWrite(VCC,HIGH);
  digitalWrite(GND,LOW);
  analogWrite(Gnd,0);
  analogWrite(Vin,(3.3/5)*255);

    analogWrite(Vin2,(3.3/5)*255);
  analogWrite(Gnd2,0);
  
  ///INITIAL FLUSH
  if (initflush<7) {
    bmp.readPressure();
    //sealevel = 0.01*bmp.readPressure();
    lastalt=alt;
    alt = readsensor(sealevel);

    initflush++;
  } else {

    ///SENSOR READ
    //acceleration
    //xValue = analogRead(XPIN);
    //yValue = analogRead(YPIN);

    zValue1 = analogRead(ZPIN);
    zValue2 = analogRead(ZPIN2);

    
    //calibration
    zValue1 = (zValue1-base)/70;
    zValue2 = (zValue2-base)/70;
    
    //bias profiling
    if (initbiasdif==999) {
//      Serial.println("INITBIASDIFCALCULATION: "); 
//      Serial.println(zValue1); 
//      Serial.println(zValue2); 
      initbiasdif=zValue1-zValue2;
//      Serial.print("INITBIASDIF: "); 
//      Serial.println(initbiasdif);
    }
    if (z1init==999) {
        z1init=zValue1;
        z2init=zValue2;
//        Serial.print("ZINITS: "); 
//        Serial.println(z1init); 
//        Serial.println(z2init); 
    }
    lastalt=alt;
    alt=readsensor(sealevel); // this should be adjusted to your local forcast
    
    if(faultyalt) {
      if (readsensor(sealevel)>2000) {
        Serial.println("Altimeter replaced, restarting...");
        delay(1000);
        lastalt=alt;
        resetFunc();
     }
    } else {
        if (alt<-100) {
          Serial.println("altimeter pulled out");

          faultyalt=true;
        }
    }

        
    if (alt0 ==999)
      alt0 = alt;
    alt-=alt0;

    
      
    zValue=faultdetect(zValue1,zValue2, zValue,alt,lastalt);
    
    U= zValue; //calibration


    Serial.print("ACCEL READINGS (1/2/merged): "); 

    Serial.print(zValue1); 
    Serial.print(", ");
    Serial.print(zValue2);
    Serial.print(", ");
    Serial.print(zValue);
    Serial.print(", ");

    if (faultyalt)
      alt=0;
    Serial.print("ALT: ");
    Serial.print(alt);

    if (initflush==7) {
      Serial.print(".;");
      initflush++;
    }
    Serial.print(", ");

    Serial.print("ESTIMATES: "); 
    
    //HEIGHT FILTERING
    double CCC = C1[0][0]; 
    //Model X
    double x1modelprev = x1model;
    double x2modelprev = x2model;
    if (restdetected)
      x2modelprev=0;
    x1model = A[0][0]*x1modelprev + A[0][1]*x2modelprev + B[0][0]*U;//A[0][2]*x3 + B[0][0]*U;
    x2model = A[1][0]*x1modelprev + A[1][1]*x2modelprev - B[1][0]*U;//A[1][2]*x3 + B[1][0]*U;
 

    double ymodel = CCC*x1model;   

     if(!faultysensor){
    Serial.print(ymodel);
    Serial.print(", ");
     }
    //Predicted X
    
    double x1predprev = x1pred;
    double x2predprev = x2pred;
    if (restdetected)
      x2predprev=0;
    x1pred = A[0][0]*x1predprev + A[0][1]*x2predprev + B[0][0]*U;//A[0][2]*x3 + B[0][0]*U;
    x2pred = A[1][0]*x1predprev + A[1][1]*x2predprev - B[1][0]*U;//A[1][2]*x3 + B[1][0]*U;
    
    //without update:
//    if (alt<0)
//      Serial.print("-");
//    printDouble(x1pred,10000);
//    Serial.print(", ");
    
      //Predicted P
    double AT[N][N];
    MatrixTranspose((double*) A, N, N, (double*) AT);
    double a[N][N];
    double b[N][N];
    MatrixMultiply((double*)AT, (double*)P, N, N, N, (double*)a);
    MatrixMultiply((double*)a, (double*)A, N, N, N, (double*)b);
    MatrixAdd((double*) A, (double*) Q, N, N, (double*) P); 
    
      //Finding K
    double PCT[N][1];  
    double CT[N][1];
    double d[1][1];
    double k[1][1];
    MatrixTranspose((double*) C1, 1, N, (double*) CT);
    MatrixMultiply((double*)P, (double*)CT, N, N, 1, (double*)PCT);
    MatrixMultiply((double*)C1, (double*)PCT, 1, N, 1, (double*)d);
    MatrixAdd((double*) d, (double*) R, 1, 1, (double*) k);
  
    if(MatrixInvert((double*) k, 1)) {
      MatrixMultiply((double*)PCT, (double*)k, N, 1, 1, (double*)K);
  
    double L = K[0][0]; 
    //L=0.1;
    L=0.1;
    if (!faultyalt) 
      x1pred =x1pred-L*(CCC*x1pred-alt);
    else
      x1pred=x1model;

    if (!faultyaccels) 
      x1pred =x1pred-L*(CCC*x1pred-alt);
    else
      x1pred =alt;
      
    double ypred = CCC*x1pred;    
    

    //THIS MUST BE TUNED!!!!
    //if ((alt<-0.2)&&(alt>-1))
     // Serial.print("-");
     
    if (abs(x1-x1predprev)>x1predprev){
      //if (ypred<0) {
        //Serial.println("ENTERED");
        ypred*=-1;
        Serial.print("-");
        printDouble(ypred,10000); 
        ypred*=-1;
      //}else{
      //Serial.print("-");
      //printDouble(ypred,10000);
      //}
    } else {
    if (ypred<0){
      ypred*=-1;
      Serial.print("-");
        printDouble(ypred,10000); 
        ypred*=-1;
    } else
    printDouble(ypred,10000);
    }
    
    //printDouble(ypred,10000);
    
    }else
      Serial.println("inversion error");
    Serial.print(", ");
    Serial.print("FAULTCODE: ");    
    Serial.print(faultcode);
    
  }

  if (restdetected)
    Serial.print(" REST DETECTED");

  Serial.println();
  double t2 = micros();
  //int ttt = (int)(1000-0.001*(t2-t1));
  //Serial.println(ttt);
  //delay(ttt);
  //typically takes 60us to process

  delay(1);
}



double faultdetect(double accel1, double accel2, double lastzvalue, double lastalt, double alt) {

  //sensor 1 typical covars = 16 17
  
  int samples=10;
  
  double dif = (accel1-accel2+initbiasdif);
  
  ///THESE MUST BE TUNED
  double meanthresh=0.22;//0.1;////0.22;
  double convergencethresh=0.02;
  double divergencethresh=1;
  double covarthresh = 0.6;//0.4;//0.6;
  
  
  /*take the rolling average*/
  int i;
  for (i=0; i<WINDOW-1;i++) {
   accels[i]=accels[i+1];
   accels1[i]=accels1[i+1];
   accels2[i]=accels2[i+1];
   honestaccels[i]=honestaccels[i+1];
  }

  accels[i]=lastzvalue;
  accels1[i]=accel1;
  accels2[i]=accel2;
  honestaccels[i]=(accels1[i+1]-accels2[i+1])/2;

  for (i=1;i<2;i++) {
    
  
    /*check the checkpins*/
      int check1 = analogRead(CHECKPIN);
      int check2 = analogRead(CHECKPIN2);
      
      Serial.print(" ::: ");
      //Serial.print(check1);
      Serial.print(" ");
      //Serial.print(check2);
      Serial.print(" ::: ");
      
      if(check1<1022) {
        if (!check1out)
          Serial.println("\n::Accelerometer (1) removed::");
        faultysensor=1;
        faultcode=1;
        check1out=true;
        
      } else {
        if (check1out)
          Serial.println("\n::Accelerometer (1) replaced::");
        check1out=false;
      }
      
      if(check2<1022) {
        if (!check2out)
          Serial.println("\n::Accelerometer (2) removed::");
        faultysensor=2;
        faultcode=1;
        check2out=true;
        
      } else {
        if (check2out)
            Serial.println("\n::Accelerometer (2) replaced::");
        check2out=false;
      }

      if (check1out||check2out)
       break;
      
    if (!faultcode) {
      
      
      /*check for initial divergence*/
      if((abs(initbiasdif)>divergencethresh)&&(!initdivergence)&&(!faultcode)) {
        faultcode = 3;
        initdivergence = true;
        Serial.println(" ::FAULT DETECTED. Code (2) - Immediate Divergence upon Starting::");
        for (int i=1;i<samples;i++) {
          //take a number of so that we are later analysing behaviour well into divergence
          analogRead(ZPIN);
          analogRead(ZPIN2);
        }
        break;
      }
  //        Serial.println("values::::");
  //    Serial.println(abs(accel1-(accel2+initbiasdif)));
  //    Serial.println(divergencethresh);
      /*Check for divergence*/
      if ((abs(accel1-(accel2+initbiasdif))>divergencethresh)&&(!faultcode)) {
        Serial.println(" ::FAULT DETECTED. Code (3) - Divergence::");
        faultcode = 3; 
        //take a number of samples so that we are later analysing behaviour well into divergence
         for (int i=1;i<samples;i++) {
            //take a number of so that we are later analysing behaviour well into divergence
            analogRead(ZPIN);
            analogRead(ZPIN2);
          }
      }
  
    } 
  }

  /*calculate means and covars*/
  double mean=0;
  double mean1=0;
  double mean2=0;
  double zmean=0;
  for (i=0; i<WINDOW;i++) {
    mean+=honestaccels[i];
    mean1+=accels1[i];
    mean2+=accels2[i];
    zmean+=accels[i];
  }
  mean/=WINDOW;
  mean1/=WINDOW;
  mean2/=WINDOW;
  zmean/=WINDOW;

   
  /*rest detection*/
  double covar=0;
  double covar1=0;
  double covar2=0;
  double zcovar=0;

  for (i=0; i<WINDOW;i++) {
    covar+=abs(honestaccels[i]-mean);
    covar1+=abs(accels1[i]-mean1);
    covar2+=abs(accels2[i]-mean2);
  }

  if (!faultcode) {
//          Serial.print(" MEAN is: ");
//          Serial.print(mean);
//          Serial.print(" COVAR is: ");
//          Serial.print(covar);
    /*rest detection*/      
    if ((abs(mean)<(meanthresh))&&(covar<covarthresh)) {
      restdetected=true;
        //Serial.println("MEAN : ");
        double m=(mean1+mean2-initbiasdif)/2;
        //Serial.println(m);
        base+=round(m*10);
        return 0;
    } else  restdetected=false;
    
  } else { //faultcode
    
    if (faultysensor==1) {
//      Serial.print(" MEAN2 is: ");
//      Serial.print(mean2);
//      Serial.print(" COVAR is: ");
//      Serial.print(covar2);
      /*rest detection*/
      if ((abs(mean2)-initbiasdif<(meanthresh))&&(covar2<covarthresh)) {
        restdetected=true;
        //Serial.println("MEAN: ");
        base+=round((mean2-initbiasdif)*10);
      } else  restdetected=false;
      
    } 
    if (faultysensor==2) {
//      Serial.print(" MEAN1 is: ");
//      Serial.print(mean1);
//      Serial.print(" COVAR is: ");
//      Serial.print(covar1);
      /*rest detection*/
      if ((abs(mean1)<(meanthresh))&&(covar1<covarthresh)) {
        restdetected=true;
        //Serial.println("MEAN: ");
        base+=round(mean1*10);
      } restdetected=false;
    }     
//            return 0;
  }
      

  /*FAULT CORRECTION*/
  switch (faultcode) {

    /*removed checkpins*/
    case 1: {
      if (check1out||check2out) {
        if (check1out&&check2out&&!faultyaccels) {
          faultyaccels=true;
          return 0;
        } else faultyaccels=false;
         
        if (check1out)
          return accel2-initbiasdif;
        if (check2out)
          return accel1;
        //return (accel1+(accel2-initbiasdif))/2;
        } else
          faultcode=0;
    }
    break;
    
    /*Divergence*/
    case 3: {
//      Serial.println("DIVERGENCE");
      //Divergence detected - which sensor is diverging
      double sum1=0;
      double sum2=0;
      for (i=0;i<5;i++) {
        double a1 = analogRead(ZPIN);
        double a2 = analogRead(ZPIN2);
        sum1+= (a1-420)/70;
        sum2+= (a2-420)/70;
      }
      if (!faultysensor) {
        if (abs(sum1)>abs(sum2)) {
          faultysensor=1;
        }else{
          faultysensor=2;
        }
      }
      if (!corrected) {
      Serial.print("\n Faulty sensor detected as: Sensor (");  
      Serial.println(faultysensor);
      Serial.print(")\n If flatlining, check that VCC is supplying power");  
      Serial.println("\n If resulting in a very piecewise curve, check that the sensor is oriented the correct way");  
      }
      faultcode = 4;

      if (restdetected)
        return 0;
      if (faultysensor==1)
        return accel2-initbiasdif;
      else
        return accel1;
    }
    break;

    /*Divergence correction*/
    case 4: {
//      Serial.println("DIVERGENCE CORRECTION");
      //Previous divergence, check if no longer divergent
      if ((abs(accel1-accel2+initbiasdif)<divergencethresh)&&(!initdivergence)) {
        Serial.println("Faulty sensor no longer divergent");
        if(corrected)
          corrected=false;
        initbiasdif = accel1-accel2;
        faultysensor=0;
        faultcode=0;
        
      } else {
        
//        if (initdivergence) {
//          initbiasdif = accel1-accel2;
//        }
        //if (initdivergence) {
          if (faultysensor==1)
          initbiasdif=mean2;

          if (faultysensor==2)
          initbiasdif=mean1;
        //}
        if((abs(accel1-accel2)<divergencethresh)&&(initdivergence)) {
          //initial divergence resolved
          Serial.println("Initial Faulty sensor no longer divergent");
          corrected=true;
          initdivergence=false;
          analogRead(ZPIN);
          analogRead(ZPIN2);
          faultcode = 0;
          faultysensor=0;
        }
          if (restdetected)
             return 0;
          if (faultysensor==1) 
          
            return accel2-initbiasdif;
          if (faultysensor==2)
            return accel1;
      }
    }
    break;
    
    case 0: {
      return (accel1+(accel2-initbiasdif))/2;
    }
}

  return (accel1+accel2-initbiasdif)/2;  
}

