#include <Wire.h>
#include <VL53L1X.h>

VL53L1X ToF; //Create ToF as an object
long startTime = 0;
long deltTime = 0;

float newDist = 0;
float prevDist = 0;
float deltDist = 0;
float velEst = 0;
float ETA = 0;
float avgETA = 0;

int objDirection;

#define HISTORY_SIZE 3 // Number of velocity values that are stored for averaging
float history[HISTORY_SIZE]; // Array to hold velocity estimates
byte historySpot; // Used in for-loop for moving through the array
float avgVelEst = 0;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  ToF.setTimeout(500);
  if (!ToF.init())
  {
    Serial.println("Failed to detect and initialize ToF!");
    while (1);
    
  }

  ToF.setDistanceMode(VL53L1X::Long);
  ToF.setMeasurementTimingBudget(50000);
  ToF.startContinuous(50);
  
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
  //objDirectionPrint_func();
  
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

void impactETA_func()
{
  ETA = newDist / velEst;
   
  if (ETA < 1000.00 && ETA > 0.00)
  {
    Serial.print(ETA);
    Serial.print(",");
    Serial.println("");
  }

  else
  {
    Serial.println("Object is likely stationary or moving away.");
  }
}