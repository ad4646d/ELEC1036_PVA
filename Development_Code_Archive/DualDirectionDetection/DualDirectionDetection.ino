#include <VL53L1X.h> //Library from: https://github.com/pololu/vl53l1x-arduino
#include <Wire.h>   //Standard arduino library

VL53L1X SEN_LEFT; //Create an object for left sensor 
VL53L1X SEN_RGHT; //Create an object for right sensor

int readFreq = 50; // Used to determine how frequently readings are taken from the sensors in ms, cannot be less than the timing budget
int startRead = millis(); //Used for timing loop

long loopStart = 0;
long newLeftTime = 0;
long prevLeftTime = 0;
long deltLeftTime = 0;
long newRghtTime = 0;
long prevRghtTime = 0;
long deltRghtTime = 0;

long rghtStart = 0;
long rghtRead = 0;
long loopEnd = 0;

float newLeftDist = 0;
float prevLeftDist = 0;
float deltLeftDist = 0;
float leftVelEst = 0;

float newRghtDist = 0;
float prevRghtDist = 0;
float deltRghtDist = 0;
float rghtVelEst = 0;

#define XSHUT_LEFT 2
#define XSHUT_RGHT 4

int objDirectionLeft = 0;
int objDirectionRght = 0;

//~~Variables for left sensor averaging~~//
//~~Velocity Averaging~~/
#define leftVelVals 3
float leftVelArray[leftVelVals];
float leftVelAvg = 0;
byte leftVelLoc;

//~~Variables for right sensor averaging~~//
//~~Velocity Averaging~~/
#define rghtVelVals 3
float rghtVelArray[rghtVelVals];
float rghtVelAvg = 0;
byte rghtVelLoc;




void setup() {

    Serial.begin(115200);
    
    Wire.begin(); //initialise I2C
    Wire.setClock(400000); // Set I2C to 400kHz

    //Set the pin mode to output
    pinMode(XSHUT_LEFT, OUTPUT);
    pinMode(XSHUT_RGHT, OUTPUT);
    
    //Disable all sensors
    digitalWrite(XSHUT_LEFT, LOW);
    digitalWrite(XSHUT_RGHT, LOW);
        
      //------------------------------------------------//
     //        Configuration for left sensor           //
    //------------------------------------------------//
    
    delay(50);
    digitalWrite(XSHUT_LEFT, HIGH); //Enable left sensor
    delay(50);
    
    SEN_LEFT.setTimeout(500); //Time in ms that the sensor will abort the read process if it does not return a result, 0 disables the timeout
    
    if (!SEN_LEFT.init()) 
    {
        Serial.println("Left sensor hasn't initialised, is it connected?"); 
        //If sensor hasn't initialised, print warning message
        //In future this could provoke a fault code via haptics
    }
    else
    {
        SEN_LEFT.setAddress(50); //Set the I2C address for left sensor
        SEN_LEFT.setDistanceMode(VL53L1X::Long); //Configures the maximum range, arguments are: short, medium, long
        SEN_LEFT.setMeasurementTimingBudget(50000); //Configures the maximum allowed time (uS) to make a single distance measurement, longer times = higher accuracy
        SEN_LEFT.startContinuous(50); //Sets the sensor to read continuously 
    }   

      //-------------------------------------------------//
     //        Configuration for right sensor           //
    //-------------------------------------------------//
    
    delay(50);
    digitalWrite(XSHUT_RGHT, HIGH); 
    delay(50);
    
    SEN_RGHT.setTimeout(500); 
    
    if (!SEN_RGHT.init())
    {
        Serial.println("Right sensor hasn't initialised, is it connected?");
   }
    else
    {
        SEN_RGHT.setAddress(45); 
        SEN_RGHT.setDistanceMode(VL53L1X::Long); 
        SEN_RGHT.setMeasurementTimingBudget(50000); 
        SEN_RGHT.startContinuous(50); 
    } 

    for (int x = 0; x < leftVelVals; x++) //empty left vel est array
    {
        leftVelArray[x] = 0;
    } 

    for (int x = 0; x < rghtVelVals; x++) //empty right vel est array
    {
        rghtVelArray[x] = 0;
    }
}

void loop() {
    if((millis()- startRead) > readFreq)
    {
        //~~left sensor
        leftVelEst_func();
        leftVelAvg_func();
        objLeftDirectionClassification_func();
        objDirectionLeftPrint_func();
        //~~right sensor
        rghtVelEst_func();
        rghtVelAvg_func();
        objRghtDirectionClassification_func();
        objDirectionRghtPrint_func();
               
        
        startRead = millis();
    }
} //End of "loop"

//~~~~~ Left Sensor Functions ~~~~~//

void leftVelEst_func() //Estimating velocity of objects approaching left sensor
{
    //Left Sensor Read
    newLeftDist = SEN_LEFT.read();
    
    newLeftTime = millis();
    deltLeftTime = (newLeftTime-prevLeftTime);
    
    deltLeftDist = (prevLeftDist - newLeftDist);
    
    leftVelEst=(deltLeftDist/deltLeftTime);
       
    prevLeftDist = newLeftDist;
    prevLeftTime = newLeftTime;
    
}

void leftVelAvg_func() //Calculates rolling average for left velocity
{
    leftVelArray[leftVelLoc] = leftVelEst;
    if (++leftVelLoc == leftVelVals)
    {
        leftVelLoc = 0;
    }
        
    for (int x = 0; x < leftVelVals; x++)
    {
        leftVelAvg += leftVelArray[x];
    }

    leftVelAvg /= leftVelVals;
}

void objLeftDirectionClassification_func()
{
  //if (leftVelEst < leftVelAvg && leftVelEst >0.1) // Object is approaching
  if (leftVelEst < leftVelAvg)
  {
    objDirectionLeft = 1;
  }

  //if (leftVelEst > leftVelAvg && leftVelEst < -0.1) // Object is departing
  if (leftVelEst > leftVelAvg)
  {
    objDirectionLeft = 2;
  }

  if (leftVelEst <0.1 && leftVelEst > -0.1) // Object is stationary -- Change to use "avgVelEst" at a later date
  {
    objDirectionLeft = 3;
  }
}

void objDirectionLeftPrint_func()
{
  switch(objDirectionLeft)
  {
    case 1:
      Serial.print("!!Warning!! Object is approaching left at: ");
      Serial.print(leftVelEst);
      Serial.print("m/s.");
      break;
    case 2:
      Serial.print("Object is departing left at: ");
      Serial.print(leftVelEst);
      Serial.print("m/s.");
      break;
    case 3:
      Serial.print("Object left is presumed to be stationary.");
      
      break;
    default:
      break;
  }
}

//~~~~~ Right Sensor Functions ~~~~~//

void rghtVelEst_func() //Estimating velocity of objects approaching right sensor
{
    
    //Right Sensor Read
    newRghtDist = SEN_RGHT.read();
    
    newRghtTime = millis();
    deltRghtTime = (newRghtTime-prevRghtTime);
    
    deltRghtDist = (prevRghtDist - newRghtDist);
    
    rghtVelEst=(deltRghtDist/deltRghtTime);
    
    prevRghtDist = newRghtDist;
    prevRghtTime = newRghtTime;
}

void rghtVelAvg_func() //Calculates rolling average for right velocity
{
    rghtVelArray[rghtVelLoc] = rghtVelEst;
    if (++rghtVelLoc == rghtVelVals)
    {
        rghtVelLoc = 0;
    }
        
    for (int x = 0; x < rghtVelVals; x++)
    {
        rghtVelAvg += rghtVelArray[x];
    }

    rghtVelAvg /= rghtVelVals;
}

void objRghtDirectionClassification_func()
{
  //if (rghtVelEst < rghtVelAvg && rghtVelEst >0.1) // Object is approaching
  if (rghtVelEst < rghtVelAvg)
  {
    objDirectionRght = 1;
  }

  //if (rghtVelEst > rghtVelAvg && rghtVelEst < -0.1) // Object is departing
  if (rghtVelEst > rghtVelAvg)
  {
    objDirectionRght = 2;
  }

  if (rghtVelEst <0.1 && rghtVelEst > -0.1) // Object is stationary -- Change to use "avgVelEst" at a later date
  {
    objDirectionRght = 3;
  }
}

void objDirectionRghtPrint_func()
{
  switch(objDirectionRght)
  {
    case 1:
      Serial.print("\t\t!!Warning!! Object is approaching right at: ");
      Serial.print(rghtVelEst);
      Serial.print("m/s.");
      Serial.println("");
      break;
    case 2:
      Serial.print("\t\tObject is departing right at: ");
      Serial.print(rghtVelEst);
      Serial.print("m/s.");
      Serial.println("");
      break;
    case 3:
      Serial.print("\t\tObject right is presumed to be stationary.");
      Serial.println("");
      break;
    default:
      break;
  }
}
