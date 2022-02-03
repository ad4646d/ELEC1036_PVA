#include <VL53L1X.h> //Library from: https://github.com/pololu/vl53l1x-arduino
#include <Wire.h>   //Standard arduino library

VL53L1X ToF; //Create ToF as an object

//~~Velocity Estimate Variables~~//
float newDist = 0;
float prevDist = 0;
float deltDist = 0;
long startTime = 0;
long deltTime = 0;
float velEst = 0;

//~~Velocity Averaging Variables~~//
#define HISTORY_SIZE 3 // Number of velocity values that are stored for averaging
float history[HISTORY_SIZE]; // Array to hold velocity estimates
byte historySpot; // Used in for-loop for moving through the array
float avgVelEst = 0;

//~~Object direction and impact ETA variables~~//
float ETA = 0;
int objDirection = 0;
int impactWarn = 0;

void setup()
{
  Serial.begin(115200); // High serial baud rate
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C
  ToF.setTimeout(500);

  if (!ToF.init())
  {
    Serial.println("Failed to detect ToF! Is it connected?");
    while (1);
  }

  ToF.setDistanceMode(VL53L1X::Long); // Long = ToF set to range at max capable distance
  ToF.setMeasurementTimingBudget(50000); // Time in microseconds
  ToF.startContinuous(50); // Time in milliseconds
  
  for (int x = 0; x < HISTORY_SIZE; x++) 
  {
    history[x] = 0;
  }
}

void loop()
{
  velEst_func();
  avgVelEst_func();
  objDirectionClassification_func();
  impactETA_func();
  hazardClassification_func();

  if (ToF.timeoutOccurred()) 
  { 
      Serial.print(" Sensor timed out."); 
  }
} //End of 'loop'

void velEst_func ()
{
  startTime = millis();
      
  newDist = ToF.read();
  
  deltTime = (millis()-startTime);
          
  deltDist = (prevDist - newDist);
  
  velEst = (deltDist / deltTime);
  
  prevDist = newDist; 
}

void avgVelEst_func()
{
  //Averaging algorithm based on SparkFun VL53L1X example: https://bit.ly/3qt83Yh
  //SparkFun's VL53L1X library wasn't used as it was not as well documented as Pololu's but it does feature some useful examples.
  history[historySpot] = velEst;
  if (++historySpot == HISTORY_SIZE)
  {
    historySpot = 0;
  }
    
  for (int x = 0; x < HISTORY_SIZE; x++)
  {
    avgVelEst += history[x];
  }

  avgVelEst /= HISTORY_SIZE;
}

void objDirectionClassification_func()
{
  if (velEst < avgVelEst && velEst >0.1) // Object is approaching
  {
    objDirection = 1;
  }

  if (velEst > avgVelEst && velEst < -0.1) // Object is departing
  {
    objDirection = 2;
  }

  if (velEst <0.1 && velEst > -0.1) // Object is stationary -- Change to use "avgVelEst" at a later date
  {
    objDirection = 3;
  }
}

void objDirectionPrint_func()
{
  switch(objDirection)
  {
    case 1:
      Serial.println("!!Warning!! Object is approaching at: ");
      Serial.print(velEst);
      Serial.print("m/s.");
      Serial.println("");
      break;
    case 2:
      Serial.println("Object is departing at: ");
      Serial.print(velEst);
      Serial.print("m/s.");
      Serial.println("");
      break;
    case 3:
      Serial.println("Object is presumed to be stationary.");
      break;
    default:
      break;
  }
}

void hazardClassification_func()
{
  switch(impactWarn)
  {
    case 1:
      Serial.println("!!WARNING!! -- !!IMPACT PROBABLE!!");
      break;
    case 2:
      switch(objDirection)
      {
        case 1:
          Serial.println("!!Warning!! Object is approaching at: ");
          Serial.print(velEst);
          Serial.print("m/s.");
          Serial.println("");
          break;
        case 2:
          Serial.println("Object is departing at: ");
          Serial.print(velEst);
          Serial.print("m/s.");
          Serial.println("");
          break;
        case 3:
          Serial.println("Object is presumed to be stationary.");
          break;
        default:
          break;
      }
      break;
    default:
      break;      
  }
}

void impactETA_func()
{
  ETA = newDist / velEst;
   
  if (ETA < 500.00 && ETA > 0.00)
  {
    impactWarn = 1;
  }

  else
  {
    impactWarn = 2;
  }
}