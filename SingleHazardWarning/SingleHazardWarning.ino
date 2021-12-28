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

//~~DistanceThresholds
int twoStridesAway = 500;
int oneStrideAway = 250;

//~~Haptic feedback variables
int hapticScore = 0;
const int leftHaptPin = 3;
int leftHaptState = 0; //Will either be 0 or a val between 0-255
int leftHaptSet = 0; //Sets the analogOut value
int leftMaxHapt = 0;
int leftHaptCount = 0;
unsigned long prevLeftHaptMillis = 0;
long leftInteval = 0;
long leftDwell = 0;
long leftHaptStart = 0;


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
  hapticFeedbac_func();

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
      hapticScore = 4;
      break;
    case 2:
      switch(objDirection)
      {
        case 1: // Object moving towards sensor
            // Distance threshold detection
            if(newDist > twoStridesAway)
            {
                hapticScore = 0;
                Serial.println("Object is more than two strides away.");
            }

            if((newDist < twoStridesAway) && (newDist > oneStrideAway))
            {
                hapticScore = 2;
                Serial.println("Object is within two strides.");
            }

            if(newDist < oneStrideAway)
            {
                hapticScore = 3;
                Serial.println("Object is within one stride.");
            }

            /*Serial.println("!!Warning!! Object is approaching at: ");
            Serial.print(velEst);
            Serial.print("m/s.");
            Serial.println("");*/
            break;
        case 2: // Object moving away from sensor
            // Distance threshold detection
            hapticScore = 0;
            
            Serial.println("Object is departing.");
            /*Serial.print(velEst);
            Serial.print("m/s.");
            Serial.println("");*/
            break;
        case 3: // Object probably stationary
            // Distance threshold detection?
            if(newDist > twoStridesAway)
            {
                hapticScore = 0;
                Serial.println("Object is more than two strides away and is stationary.");
            }

            if((newDist < twoStridesAway) && (newDist > oneStrideAway))
            {
                hapticScore = 6;
                Serial.println("Object is within two strides and is stationary.");
            }

            if(newDist < oneStrideAway)
            {
                hapticScore = 7;
                Serial.println("Object is within one stride and is stationary.");
            }

            //Serial.println("Object is presumed to be stationary.");
            
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

void hapticFeedbac_func ()
{
    leftHaptStart = millis();
    leftHaptSet = map(newDist, 4000, 0, 130, 255);
    switch(hapticScore)
    {
        case 0:
            //leftHaptSet= 0;
            leftMaxHapt = 0;
            leftInteval = 0;
            leftDwell = 0;
            break;
        case 1:
            //leftHaptSet= 175;
            leftMaxHapt = 1;
            leftInteval = 75;
            leftDwell = 1500;
            break;
        case 2:
            //leftHaptSet= 175;
            leftMaxHapt = 2;
            leftInteval = 75;
            leftDwell = 1000;
            break;
        case 3:
            //leftHaptSet= 175;
            leftMaxHapt = 3;
            leftInteval = 75;
            leftDwell = 750;
            break;
        case 4:
            //leftHaptSet= 200;
            leftMaxHapt = 5;
            leftInteval = 20;
            leftDwell = 500;
            break;
        case 5:
            //leftHaptSet= 175;
            leftMaxHapt = 1;
            leftInteval = 75;
            leftDwell = 1000;
            break;
        case 6:
            leftHaptSet= 120;
            leftMaxHapt = 2;
            leftInteval = 50;
            leftDwell = 1000;
            break;
        case 7:
            leftHaptSet= 100;
            leftMaxHapt = 1;
            leftInteval = 75;
            leftDwell = 1000;
            break;
        default:
            break;
    }
     
    
    if (leftHaptStart - prevLeftHaptMillis >= leftInteval && leftHaptCount < leftMaxHapt) // The 'latching on' vibrate bug might lurk here
    {
        prevLeftHaptMillis = leftHaptStart;
        if (leftHaptState == 0)
        {
            leftHaptState = leftHaptSet;
            //leftHaptState = HIGH;
        }
        else //Possibly where pin is not set to low on changing hapticScore case?
        {
            leftHaptState = 0;
            //leftHaptState = LOW;
            leftHaptCount++;
        }
    analogWrite(leftHaptPin, leftHaptState); 
    }

    if(leftHaptStart - prevLeftHaptMillis >= leftDwell)
    {
        leftHaptCount = 0;
    }
}