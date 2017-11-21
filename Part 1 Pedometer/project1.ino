//Make sure to use analogue pins

//IO
int Print = 0;
boolean START = false;
boolean STOP = false;


//Data Sampling
int readIndex = 0;
int DELAY = 200;//50
int initialWait = 15000;
int NUMREADINGS = 1000+initialWait/DELAY;

          
//Pin setup
int Vin = A4;
int XPIN = A3;
int YPIN = A2;
int ZPIN = A1;
int Gnd = A0;

//Accelerometer Reading
int xValue=0;
int yValue=0;
int zValue=0;

//Data Processing;
double xs = 0;
double ys = 0;
double zs = 0;
double Xsum = 0;
double Ysum = 0;
double Zsum = 0;
double xDev = 0;
double yDev = 0;
double zDev = 0;
double dev = 0;
double xDevSum = 0;
double yDevSum = 0;
double zDevSum = 0;
double avgDevSum = 0;
double X = 0;
double Y = 0;
double Z = 0;

double posbound = 500.0;
double negbound = 0.0;

double xScale =1;
double yScale =1;
double zScale =1;
double avgsum=0;
double avg = 0;
double avgDev = 0;
double readValue = 0;

double xScaleSum = 0;
double yScaleSum = 0;
double zScaleSum = 0;
double r;

//Step Recording
int count =0;
int stepnumber = 0;
boolean isstep = false;
double sum = 0;
boolean recordstep = false;


void setup() {
  // put your setup code here, to run once:
  //Begin Serial communication
  Serial.begin(9600);
  START = true;
  Serial.println("START");
  Serial.println("Calibrating....please go for a walk for 10 sec");
  pinMode(A0,OUTPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  pinMode(A4,OUTPUT);  
}

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void loop() {
  analogWrite(Gnd,0);
  analogWrite(Vin,(3.3/5)*255);
  
  if (Serial.read())
    STOP = true;
  if (START) {
      xValue = analogRead(XPIN);
      yValue = analogRead(YPIN);
      zValue = analogRead(ZPIN);
      /*Serial.print("raw: ");
      Serial.print(xValue);
      Serial.print(" ");
      Serial.print(yValue);
      Serial.print(" ");
      Serial.println(zValue);
      Serial.print("calibrated: ");
      Serial.print((xValue-340)/70.0);
      Serial.print(" ");
      Serial.print((yValue-340)/70.0);
      Serial.print(" ");
      Serial.println((zValue-340)/70.0);
      */
      
    if (readIndex==0) {
    //first reading is usually off, take it and ignore it
      readIndex++;
    } else {
       
        if (readIndex < NUMREADINGS) {
          ////READING
          xs +=xValue;
          ys +=yValue;
          zs +=zValue;
          
          /// ZERO MEAN
          xValue-=(xs/readIndex);
          yValue-=(ys/readIndex);
          zValue-=(zs/readIndex);
          
          //// CALIBRATION    
          if (readIndex>(initialWait/DELAY)) {
            //enough samples taken in order to have a good average estimate for data processing
            
            if (!Print) {
              Print = true;
              Serial.println("initial wait over");
            }
            
            //use this or just keep board in favour of 1 axis and 
            //read that axis' value
            //->scale the dimensions by their respective deviation weights to get the final filtered readings
            X = pow(xValue,xScaleSum);
            Y = pow(yValue,yScaleSum);
            Z = pow(zValue,zScaleSum);

            //number corrections
            if (isnan(X))
              X=0.0;
            if (isnan(Y))
              Y=0.0;
            if (isnan(Z))
              Z=0.0;
            if (isinf(X))
              X=1024;
            if (isinf(Y))
              Y=1024;
            if (isinf(Z))
              Z=1024;

            // STEP CHECKING
            r = readValue;            
            readValue = (X+Y+Z)/3.0; 
            
            if (readValue<posbound) {
              readValue=0;
              count = 0;
              isstep = 0;
            } else {
              //check if within a new time window
              if (count>1000) {
                isstep = 1;
                count = 0;
              } else isstep = 0;
              
              if (r==0) {
              isstep = 1;
              }
              count++;
            }
            
            // STEP RECORDING
            if (isstep) {
                //Serial.println("Step");  
                recordstep = true;
            } else recordstep = false;
          
          }
          else{

            //integrating
            Xsum+=xValue;
            Ysum+=yValue;
            Zsum+=zValue;
            
            //deviation
            xDev = xValue-Xsum/readIndex;
            yDev = yValue-Ysum/readIndex;
            zDev = zValue-Zsum/readIndex;
  
            //Deviation sum
            xDevSum += xDev;
            yDevSum += yDev;
            zDevSum += zDev;
            
            avgDevSum = (xDevSum+yDevSum+zDevSum)/3.0;
      
            //SCALING
            //get integral of each axis' deviations from overall avg axis deviation
            //eg if z deviates more than the average of x,y,z, it's readings will be amplified
            xScale = xDev/avgDevSum;
            yScale = yDev/avgDevSum;
            zScale = zDev/avgDevSum;
  
            //square it to get stronger deviation weights          
            xScale *=xScale;
            yScale *=yScale;
            zScale *=zScale;
  
            //To make scales stronger, use their sum; but only after disregarding first few readings as they usually equate to nan
            if (readIndex>50) {
            xScaleSum+=xScale;
            yScaleSum+=yScale;
            zScaleSum+=zScale;
            }
            
            //scale the dimensions by their respective deviation weights to get the final filtered readings
            X = pow(xValue,xScale);
            Y = pow(yValue,yScale);
            Z = pow(zValue,zScale);
  
            //number correction
            if (isnan(X))
              X=0.0;
            if (isnan(Y))
              Y=0.0;
            if (isnan(Z))
              Z=0.0;
            if (isinf(X))
              X=40;
            if (isinf(Y))
              Y=40;
            if (isinf(Z))
              Z=40;
            
              
            //calculate rolling averages and deviations
            if (!isnan(X+Y+Z)) {
            avgsum+=(X+Y+Z)/3.0;
            avg = avgsum/readIndex;
            avgDev = (X+Y+Z)/3.0-avg;
            }

          }
          readIndex++;
        }
        else {
          //Enough readings gathered
          if (Print) {
            Serial.println("STOP");
            Print = 0;
            STOP = true; //comment this line out if you want continuous back-to-back operation
          }
        }
    }
  }
    if (Print) {
      if (recordstep) {
        stepnumber++;
        Serial.print("Step ");
        Serial.println(stepnumber);
      }
    } else {
      if (!STOP)
      resetFunc();
    }
  delay(DELAY); // wait, ie enforcing the sample period
}
