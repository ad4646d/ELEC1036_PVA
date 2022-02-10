/*
  --------------------------------------------------------------
  Project Title: Peripheral Vision Aid for the Partially Sighted
  Name: Andrew Dean
  Student ID: 000962066
  Email: ad4646d@gre.ac.uk
  --------------------------------------------------------------
  **************************************************************
  --------------------------------------------------------------
  Revision: V1.3
  --------------------------------------------------------------
  Revision History:
  V1 - Initial release
  
  V1.1 - Housekeeping ~~ Fixing indents
  
  V1.2 - Housekeeping ~~ Creating common variables for distance
                         thresholds
 
  V1.3 - Enhancement  ~~ Changed haptics function to drive PWM 
                         proportionally to sensed distance, 
                         added a duty cycle for 1 step and 
                         2 step distance thresholds

  --------------------------------------------------------------
*/

#include <VL53L1X.h> //Library from: https://github.com/pololu/vl53l1x-arduino
#include <Wire.h>   //Standard arduino library

VL53L1X SEN_LEFT; //Create an object for left sensor 
VL53L1X SEN_RGHT; //Create an object for right sensor

bool serialDebugPrint = false;

//~~Pin Declarations~~//
#define XSHUT_LEFT 2 //Enable pin for left sensor
#define XSHUT_RGHT 4 //Enable pin for right sensor

const int leftHaptPin = 3; //PWM pin controlling left vibrate motor
const int rghtHaptPin = 5; //PWM pin controlling right vibrate motor

//~~~~~~~MIGHT BE REMOVED~~~~~~~~~//
int readFreq = 50; // Used to determine how frequently readings are taken from the sensors in ms, cannot be less than the timing budget
int startRead = millis(); //Used for timing loop
////~~~~~~~MIGHT BE REMOVED~~~~~~~~~//

//~~Velocity Estimation Variables~~//
long newLeftTime = 0;
long prevLeftTime = 0;
long deltLeftTime = 0;
float newLeftDist = 0;
float prevLeftDist = 0;
float deltLeftDist = 0;
float leftVelEst = 0;

long newRghtTime = 0;
long prevRghtTime = 0;
long deltRghtTime = 0;
float newRghtDist = 0;
float prevRghtDist = 0;
float deltRghtDist = 0;
float rghtVelEst = 0;

//~~Distance Thresholds~~//
int twoStridesAway = 2100;
int oneStrideAway = 1060;

//~~Object Direction and Impact Warning Variables~~//
int objDirectionLeft = 0;
int objDirectionRght = 0;
float leftImpactETA = 0.0;
float rghtImpactETA = 0.0;
int leftImpendingImpact = 0; //previously impactWarn
int rghtImpendingImpact = 0;

//~~Hazard Classification Variables and Haptic Feedback~~//
int leftHazardScore = 0; //previously hapticScore
int rghtHazardScore = 0;

const int leftHapticPin = 3; //previously vibePin
const int rghtHapticPin = 5;

int leftHapticState = 0; //previously vibeState
int rghtHapticState = 0;

int leftHapticSet = 0; //previously vibeSet
int rghtHapticSet = 0;

unsigned long leftHapticTimerStart = 0; //previously currentMillis
unsigned long rghtHapticTimerStart = 0;

unsigned long leftHapticTimerEnd = 0; //previously previousMillis
unsigned long rghtHapticTimerEnd = 0;

long leftHapticInterval = 0; //previously interval
long rghtHapticInterval = 0;

//~~Velocity Averaging~~/
#define leftVelVals 3
float leftVelArray[leftVelVals];
float leftVelAvg = 0;
byte leftVelLoc;

#define rghtVelVals 3
float rghtVelArray[rghtVelVals];
float rghtVelAvg = 0;
byte rghtVelLoc;

void setup() 
{
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

void loop() 
{
  if((millis()- startRead) > readFreq)
  {
    //~~left sensor
    leftVelEst_func();
    leftVelAvg_func();
    objLeftDirectionClassification_func();
    leftImpactETA_func();
    leftHazardClassification_func();
    leftHapticFeedback_func();
    
    //~~right sensor
    rghtVelEst_func();
    rghtVelAvg_func();
    objRghtDirectionClassification_func();
    rghtImpactETA_func();
    rghtHazardClassification_func();
    rghtHapticFeedback_func();
    

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
  if (leftVelEst < leftVelAvg && leftVelEst >0.1) // Object is approaching
  {
    objDirectionLeft = 1;
    if(serialDebugPrint == true)
    {
      Serial.print("!!Warning!! Object is approaching left at: ");
      Serial.print(leftVelEst);
      Serial.print("m/s.");
    }
  }

  if (leftVelEst > leftVelAvg && leftVelEst < -0.1) // Object is departing
  {
    objDirectionLeft = 2;
    if(serialDebugPrint == true)
    {
      Serial.print("Object is departing left at: ");
      Serial.print(leftVelEst);
      Serial.print("m/s.");
    }
  }

  if (leftVelEst < 0.1 && leftVelEst > -0.1) // Object is stationary -- Change to use "avgVelEst" at a later date
  {
    objDirectionLeft = 3;
    if(serialDebugPrint == true)
    {
      Serial.print("Object left is presumed to be stationary.");
    }
  }
}

void leftHazardClassification_func()
{
  switch(leftImpendingImpact)
  {
    case 1:
      if(serialDebugPrint == true)
      {
        Serial.println("!!WARNING!! -- !!IMPACT PROBABLE!!");
      }
      
      leftHazardScore = 5;
      break;
    case 2:
      switch(objDirectionLeft)
      {
        case 1: // Object moving towards sensor
          // Distance threshold detection
          if(newLeftDist > twoStridesAway)
          {
            leftHazardScore = 0;
            if(serialDebugPrint == true)
            {
              Serial.print("Object is more than two strides away, leftHazardScore = ");
              Serial.print(leftHazardScore);
              Serial.print(", leftHapticState = ");
              Serial.println(leftHapticState);
            }
          }

          if((newLeftDist < twoStridesAway) && (newLeftDist > oneStrideAway))
          {
            leftHazardScore = 2;
            if(serialDebugPrint == true)
            {
              Serial.print("Object is within two strides, leftHazardScore = ");
              Serial.print(leftHazardScore);
              Serial.print(", leftHapticState = ");
              Serial.println(leftHapticState);
            }
          }

          if(newLeftDist < oneStrideAway)
          {
            leftHazardScore = 1;
            if(serialDebugPrint == true)
            {
              Serial.print("Object is within one stride, leftHazardScore = ");
              Serial.print(leftHazardScore);
              Serial.print(", leftHapticState = ");
              Serial.println(leftHapticState);
            }
          }
          break;
        case 2: // Object moving away from sensor
          leftHazardScore = 0;
          if(serialDebugPrint == true)
          {
            Serial.print("Object is departing, leftHazardScore = ");
            Serial.print(leftHazardScore);
            Serial.print(", leftHapticState = ");
            Serial.println(leftHapticState);
            /*Serial.print(leftVelEst);
            Serial.print("m/s.");
            Serial.println("");*/
          }
          break;
        case 3: // Object probably stationary
          // Distance threshold detection?
          if(newLeftDist > twoStridesAway)
          {
            leftHazardScore = 0;
            if(serialDebugPrint == true)
            {
              Serial.print("Object is more than two strides away and is stationary, leftHazardScore = ");
              Serial.print(leftHazardScore);
              Serial.print(", leftHapticState = ");
              Serial.print(leftHapticState);
              Serial.print(", velocity = ");
              Serial.print(leftVelEst);
            }
          }

          if((newLeftDist < twoStridesAway) && (newLeftDist > oneStrideAway))
          {
            leftHazardScore = 4;
            if(serialDebugPrint == true)
            {
              Serial.print("Object is within two strides and is stationary, leftHazardScore = ");
              Serial.print(leftHazardScore);
              Serial.print(", leftHapticState = ");
              Serial.print(leftHapticState);
              Serial.print(", velocity = ");
              Serial.print(leftVelEst);
            }
          }

          if(newLeftDist < oneStrideAway)
          {
            leftHazardScore = 3;
            if(serialDebugPrint == true)
            {
              Serial.print("Object is within one stride and is stationary, leftHazardScore = ");
              Serial.print(leftHazardScore);
              Serial.print(", leftHapticState = ");
              Serial.print(leftHapticState);
              Serial.print(", velocity = ");
              Serial.print(leftVelEst);
            }
          }
          if(serialDebugPrint == true)
          {
            Serial.println("Is out of range and is presumed stationary.");
          }
          break;
        default:
          break;
      }
      break;
    default:
      break;      
  }
}

void leftImpactETA_func()
{
  leftImpactETA = newLeftDist / leftVelEst;
  if (leftImpactETA < 500.00 && leftImpactETA > 0.00)
  {
    leftImpendingImpact = 1;
  }
  else
  {
    leftImpendingImpact = 2;
  }
}

void leftHapticFeedback_func ()
{
  leftHapticTimerStart = millis();
  
  switch(leftHazardScore)
  {
    case 0:
      leftHapticSet = 0;
      leftHapticInterval = 0;
      break;
    case 1: // Moving towards within 1 stride -- medium priority
      leftHapticSet = map(newLeftDist, twoStridesAway, 10, 55, 255);
      leftHapticInterval = 25;
      break;
    case 2: // Moving towards within 2 strides -- medium priority
      leftHapticSet = map(newLeftDist, twoStridesAway, 10, 55, 255);
      leftHapticInterval = 50;
      break;
    case 3: // Stationary within one stride -- low priority
      leftHapticSet = 0;
      leftHapticInterval = 75;
      break;
    case 4: // Stationary within two strides -- low priority
      leftHapticSet = 0;
      leftHapticInterval = 60;
      break;
    case 5: // Impact possible -- high priority
      leftHapticSet = 255;
      leftHapticInterval = 10;
      break;
    default:
      break;
  }
    
  if (leftHapticTimerStart - leftHapticTimerEnd >= leftHapticInterval)
  {
    leftHapticTimerEnd = leftHapticTimerStart;
    if (leftHapticState == 0)
    {
      leftHapticState = leftHapticSet;
      if(serialDebugPrint == true)
      {
        Serial.print("\t\tVibrate ON, leftHapticState = ");
        Serial.print(leftHapticState);
        Serial.println("");
      }
    }
    else
    {
      leftHapticState = 0;
      if(serialDebugPrint == true)
      {
        Serial.print("\t\tVibrate OFF, leftHapticState = ");
        Serial.print(leftHapticState);
        Serial.println("");
      }
    }
    analogWrite(leftHapticPin, leftHapticState); 
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
  if (rghtVelEst < rghtVelAvg && rghtVelEst >0.1) // Object is approaching
  {
    objDirectionRght = 1;
    if(serialDebugPrint == true)
    {
      Serial.print("\t\t!!Warning!! Object is approaching right at: ");
      Serial.print(rghtVelEst);
      Serial.print("m/s.");
      Serial.println("");
    }
  }

  if (rghtVelEst > rghtVelAvg && rghtVelEst < -0.1) // Object is departing
  {
    objDirectionRght = 2;
    if(serialDebugPrint == true)
    {
      Serial.print("\t\tObject is departing right at: ");
      Serial.print(rghtVelEst);
      Serial.print("m/s.");
      Serial.println("");
    }
  }

  if (rghtVelEst <0.1 && rghtVelEst > -0.1) // Object is stationary -- Change to use "avgVelEst" at a later date
  {
    objDirectionRght = 3;
    if(serialDebugPrint == true)
    {
      Serial.print("\t\tObject right is presumed to be stationary.");
      Serial.println("");
    }
  }
}

void rghtHazardClassification_func()
{
  switch(rghtImpendingImpact)
  {
    case 1:
      if(serialDebugPrint == true)
      {
        Serial.println("!!WARNING!! -- !!IMPACT PROBABLE!!");
      }
      rghtHazardScore = 5;
      break;
    case 2:
      switch(objDirectionRght)
      {
        case 1: // Object moving towards sensor
          // Distance threshold detection
          if(newRghtDist > twoStridesAway)
          {
            rghtHazardScore = 0;
            if(serialDebugPrint == true)
            {
              Serial.print("Object is more than two strides away, rghtHazardScore = ");
              Serial.print(rghtHazardScore);
              Serial.print(", rghtHapticState = ");
              Serial.println(rghtHapticState);
            }
          }

          if((newRghtDist < twoStridesAway) && (newRghtDist > oneStrideAway))
          {
            rghtHazardScore = 2;
            if(serialDebugPrint == true)
            {
              Serial.print("Object is within two strides, rghtHazardScore = ");
              Serial.print(rghtHazardScore);
              Serial.print(", rghtHapticState = ");
              Serial.println(rghtHapticState);
            }
          }

          if(newRghtDist < oneStrideAway)
          {
            rghtHazardScore = 1;
            if(serialDebugPrint == true)
            {
              Serial.print("Object is within one stride, rghtHazardScore = ");
              Serial.print(rghtHazardScore);
              Serial.print(", rghtHapticState = ");
              Serial.println(rghtHapticState);
            }
          }
          break;
        case 2: // Object moving away from sensor
          rghtHazardScore = 0;
          if(serialDebugPrint == true)
          {
            Serial.print("Object is departing, rghtHazardScore = ");
            Serial.print(rghtHazardScore);
            Serial.print(", rghtHapticState = ");
            Serial.println(rghtHapticState);
            /*Serial.print(rghtVelEst);
            Serial.print("m/s.");
            Serial.println("");*/
          }
          break;
        case 3: // Object probably stationary
          // Distance threshold detection?
          if(newRghtDist > twoStridesAway)
          {
            rghtHazardScore = 0;
            if(serialDebugPrint == true)
            {
              Serial.print("Object is more than two strides away and is stationary, rghtHazardScore = ");
              Serial.print(rghtHazardScore);
              Serial.print(", rghtHapticState = ");
              Serial.print(rghtHapticState);
              Serial.print(", velocity = ");
              Serial.print(rghtVelEst);
            }
          }

          if((newRghtDist < twoStridesAway) && (newRghtDist > oneStrideAway))
          {
            rghtHazardScore = 4;
            if(serialDebugPrint == true)
            {
              Serial.print("Object is within two strides and is stationary, rghtHazardScore = ");
              Serial.print(rghtHazardScore);
              Serial.print(", rghtHapticState = ");
              Serial.print(rghtHapticState);
              Serial.print(", velocity = ");
              Serial.print(rghtVelEst);
            }
          }

          if(newRghtDist < oneStrideAway)
          {
            rghtHazardScore = 3;
            if(serialDebugPrint == true)
            {
              Serial.print("Object is within one stride and is stationary, rghtHazardScore = ");
              Serial.print(rghtHazardScore);
              Serial.print(", rghtHapticState = ");
              Serial.print(rghtHapticState);
              Serial.print(", velocity = ");
              Serial.print(rghtVelEst);
            }
          }
          if(serialDebugPrint == true)
          {
            Serial.println("Is out of range and is presumed stationary.");
          }
          break;
        default:
          break;
      }
      break;
    default:
      break;      
  }
}

void rghtImpactETA_func()
{
  rghtImpactETA = newLeftDist / rghtVelEst;
  if (rghtImpactETA < 500.00 && rghtImpactETA > 0.00)
  {
    rghtImpendingImpact = 1;
  }
  else
  {
    rghtImpendingImpact = 2;
  }
}

void rghtHapticFeedback_func ()
{
  rghtHapticTimerStart = millis();
  
  switch(rghtHazardScore)
  {
    case 0:
      rghtHapticSet = 0;
      rghtHapticInterval = 0;
      break;
    case 1: // Moving towards within 1 stride -- medium priority
      rghtHapticInterval = 25; //Duty cycle
      rghtHapticSet = map(newRghtDist, twoStridesAway, 10, 55, 255);
      break;
    case 2: // Moving towards within 2 strides -- medium priority
      rghtHapticInterval = 50;
      rghtHapticSet = map(newRghtDist, twoStridesAway, 10, 55, 255);
      break;
    case 3: // Stationary within one stride -- low priority
      rghtHapticSet = 0;
      rghtHapticInterval = 75;
      break;
    case 4: // Stationary within two strides -- low priority
      rghtHapticSet = 0;
      rghtHapticInterval = 60;
      break;
    case 5: // Impact possible -- high priority
      rghtHapticSet = 255;
      rghtHapticInterval = 10;
      break;
    default:
      break;
  }
    
  if (rghtHapticTimerStart - rghtHapticTimerEnd >= rghtHapticInterval)
  {
    rghtHapticTimerEnd = rghtHapticTimerStart;
    if (rghtHapticState == 0)
    {
      rghtHapticState = rghtHapticSet;
      if(serialDebugPrint == true)
      {
        Serial.print("\t\tVibrate ON, rghtHapticState = ");
        Serial.print(rghtHapticState);
        Serial.println("");
      }
    }
    else
    {
      rghtHapticState = 0;
      if(serialDebugPrint == true)
      {
        Serial.print("\t\tVibrate OFF, rghtHapticState = ");
        Serial.print(rghtHapticState);
        Serial.println("");
      }
    }
  analogWrite(rghtHapticPin, rghtHapticState); 
  }
}