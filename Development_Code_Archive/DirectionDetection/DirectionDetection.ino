#include <Wire.h>
#include <VL53L1X.h>

VL53L1X ToF; //Create ToF as an object
long startTime = 0;
long endTime = 0;

int readFreq = 20; // Used to determine how frequently readings are taken from the ToFs in ms, cannot be less than the timing budget
int startRead = millis();

float newDist = 0;
float prevDist = 0;
float deltDist = 0;
float velEst = 0;

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
  Serial.print(velEst);//Real-time distance
  Serial.print(",");
  avgVelEst_func();
  objDirectionClassification_func();
  //objDirectionPrint_func();
  Serial.print(avgVelEst);//Average distance 
  Serial.print(",");
  Serial.println("");
  if (ToF.timeoutOccurred()) 
  { 
    Serial.print(" Sensor timed out."); 
  }
} //End of 'loop'

void velEst_func ()
{
    startTime = millis();
        
    newDist = ToF.read();
    /*Serial.print("New Distance: ");
    Serial.print(newDist);
    Serial.print("mm - ");
    
    Serial.print("Previous Distance: ");
    Serial.print(prevDist);
    Serial.print("mm - ");*/
    
    endTime = (millis()-startTime);
            
    deltDist = (prevDist - newDist);
    /*Serial.print("Distance delta: ");
    Serial.print(deltDist);
    Serial.print("mm - ");

    Serial.print("Read time: ");
    Serial.print(endTime);
    Serial.print("ms - ");*/
    
    velEst = (deltDist / endTime);
    /*Serial.print("Velocity Estimate: ");
    Serial.print(velEst);
    Serial.print("mm/ms");*/
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