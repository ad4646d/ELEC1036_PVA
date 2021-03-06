/*
  --------------------------------------------------------------
  Project Title: Peripheral Vision Aid for the Partially Sighted
  Name: Andrew Dean
  Student ID: 000962066
  Email: ad4646d@gre.ac.uk
  --------------------------------------------------------------
  **************************************************************
  --------------------------------------------------------------
  Revision: V1.7
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
  
  V1.4 - Modification ~~ Commented out 'ImpactETA' func as 
                         it is no longer required
  
  V1.5 - Modification ~~ Added serial debugging commands to
                         check that I2C addresses have been set
  
  V1.6 - Modification ~~ Added '<sensor>.dataReady()' library
                         reference to velocity estimate funcs
                         to ensure latest distance is requested
                         from sensor when it is ready
  
  V1.7 - Modification ~~ Improved serial debug printout for 
                         testing purposes and general tidying of
                         code  
  --------------------------------------------------------------
*/

#include <VL53L1X.h> //Library from: https://github.com/pololu/vl53l1x-arduino
#include <Wire.h>   //Standard arduino library

VL53L1X SEN_LEFT; //Create an object for left sensor 
VL53L1X SEN_RGHT; //Create an object for right sensor

bool serialDebugPrint = true;

//~~Pin Declarations~~//
#define XSHUT_LEFT 2 //Enable pin for left sensor
#define XSHUT_RGHT 4 //Enable pin for right sensor

const int leftHaptPin = 3; //PWM pin controlling left vibrate motor
const int rghtHaptPin = 5; //PWM pin controlling right vibrate motor

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
int twoStridesAway = 2007;
int oneStrideAway = 947;

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

long leftHapticDutyCyc = 0; //previously interval
long rghtHapticDutyCyc = 0;

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
    SEN_LEFT.setDistanceMode(VL53L1X::Medium); //Configures the maximum range, arguments are: short, medium, long
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
    SEN_RGHT.setDistanceMode(VL53L1X::Medium); 
    SEN_RGHT.setMeasurementTimingBudget(50000); 
    SEN_RGHT.startContinuous(50); 
  }

  if(serialDebugPrint == true) 
  {
    Serial.print("I2C address for left sensor is: ");
    Serial.print(SEN_LEFT.getAddress());
    Serial.println("I2C address for right sensor is: ");
    Serial.print(SEN_RGHT.getAddress());
    Serial.println("_________________---____________");
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
  //~~left sensor
  leftVelEst_func();
  leftVelAvg_func();
  objLeftDirectionClassification_func();
  leftHazardClassification_func();
  leftHapticFeedback_func();
  
  //~~right sensor
  rghtVelEst_func();
  rghtVelAvg_func();
  objRghtDirectionClassification_func();
  rghtHazardClassification_func();
  rghtHapticFeedback_func();
} //End of "loop"

//~~~~~ Left Sensor Functions ~~~~~//

void leftVelEst_func() //Estimating velocity of objects approaching left sensor
{
  //Left Sensor Read
  if(SEN_LEFT.dataReady() == true) //Will only run if left sensor has data ready
  {
    newLeftDist = SEN_LEFT.read();
  
    newLeftTime = millis();
    deltLeftTime = (newLeftTime-prevLeftTime);
    
    deltLeftDist = (prevLeftDist - newLeftDist);
    
    leftVelEst=(deltLeftDist/deltLeftTime);
        
    prevLeftDist = newLeftDist;
    prevLeftTime = newLeftTime; 
  }
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

void objLeftDirectionClassification_func() //Deduces the direction of motion of an object
{
  if (leftVelEst < leftVelAvg && leftVelEst >0.1) // Object is approaching
  {
    objDirectionLeft = 1;
    /*if(serialDebugPrint == true)
    {
      Serial.print("!!Warning!! Object is approaching left at: ");
      Serial.print(leftVelEst);
      Serial.print("m/s.");
    }*/
  }

  if (leftVelEst > leftVelAvg && leftVelEst < -0.1) // Object is departing
  {
    objDirectionLeft = 2;
    /*if(serialDebugPrint == true)
    {
      Serial.print("Object is departing left at: ");
      Serial.print(leftVelEst);
      Serial.print("m/s.");
    }*/
  }

  if (leftVelEst < 0.1 && leftVelEst > -0.1) // Object is stationary -- Change to use "avgVelEst" at a later date
  {
    objDirectionLeft = 3;
    /*if(serialDebugPrint == true)
    {
      Serial.print("Object left is presumed to be stationary.");
    }*/
  }
}

void leftHazardClassification_func() //Main hazard classification function
{
  switch(objDirectionLeft)
  {
    case 1: // Object moving towards sensor
      // Distance threshold detection
      if(newLeftDist > twoStridesAway)
      {
        leftHazardScore = 0;
        if(serialDebugPrint == true)
        {
          Serial.print("| Left Sensor | ");
          Serial.print("Direction: Approaching - Distance: ");
          Serial.print(newLeftDist);
          Serial.print("mm - Object ignored.");
        }
      }
      if((newLeftDist < twoStridesAway) && (newLeftDist > oneStrideAway))
      {
        leftHazardScore = 1;
        if(serialDebugPrint == true)
        {
          Serial.print("| Left Sensor | ");
          Serial.print("Direction: Approaching ");
          Serial.print("Distance: ");
          Serial.print(newLeftDist);
          Serial.print("mm - Within two strides - Velocity: ");
          Serial.print(leftVelEst);
          Serial.print("m/s - Avg Velocity: ");
          Serial.print(leftVelAvg);
          Serial.print("m/s");
        }
      }

      if(newLeftDist < oneStrideAway)
      {
        leftHazardScore = 2;
        if(serialDebugPrint == true)
        {
          Serial.print("| Left Sensor | ");
          Serial.print("Direction: Approaching ");
          Serial.print("Distance: ");
          Serial.print(newLeftDist);
          Serial.print("mm - Within one strides - Velocity: ");
          Serial.print(leftVelEst);
          Serial.print("m/s - Avg Velocity: ");
          Serial.print(leftVelAvg);
          Serial.print("m/s");
        }
      }
      break;
    case 2: // Object moving away from sensor
      leftHazardScore = 0;
      if(serialDebugPrint == true)
      {
        Serial.print("| Left Sensor | ");
        Serial.print("Direction: Departing - Distance: ");
        Serial.print(newLeftDist);
        Serial.print("mm - Object ignored.");
      }
      break;
    case 3: // Object probably stationary
      leftHazardScore = 0;
      if(serialDebugPrint == true)
      {
        Serial.print("| Left Sensor | ");
        Serial.print("Direction: Stationary - Distance: ");
        Serial.print(newLeftDist);
        Serial.print("mm - Object ignored.");
      }
      break;
    default:
      break;
  }
}

void leftHapticFeedback_func () //Responsible for generating haptic feedback 
{
  leftHapticTimerStart = millis();
  
  switch(leftHazardScore)
  {
    case 0:
      leftHapticSet = 0;
      leftHapticDutyCyc = 0;
      break;
    case 1: // Moving towards within 2 strides -- low priority
      leftHapticSet = map(newLeftDist, twoStridesAway, 10, 55, 255);
      leftHapticDutyCyc = 50;
      break;
    case 2: // Moving towards within 1 stride -- medium priority
      leftHapticSet = map(newLeftDist, twoStridesAway, 10, 55, 255);
      leftHapticDutyCyc = 25;
      break;
    default:
      break;
  }
    
  if (leftHapticTimerStart - leftHapticTimerEnd >= leftHapticDutyCyc)
  {
    leftHapticTimerEnd = leftHapticTimerStart;
    if (leftHapticState == 0)
    {
      leftHapticState = leftHapticSet;
      /*if(serialDebugPrint == true)
      {
        Serial.print("\t\tVibrate ON, leftHapticState = ");
        Serial.print(leftHapticState);
        Serial.println("");
      }*/
    }
    else
    {
      leftHapticState = 0;
      /*if(serialDebugPrint == true)
      {
        Serial.print("\t\tVibrate OFF, leftHapticState = ");
        Serial.print(leftHapticState);
        Serial.println("");
      }*/
    }
    analogWrite(leftHapticPin, leftHapticState); 
  }
}

//~~~~~ Right Sensor Functions ~~~~~//

void rghtVelEst_func() //Estimating velocity of objects approaching right sensor
{
  //Right Sensor Read
  if(SEN_RGHT.dataReady() == true) //Will only run if right sensor has data ready
  {
    newRghtDist = SEN_RGHT.read();
    
    newRghtTime = millis();
    deltRghtTime = (newRghtTime-prevRghtTime);
    
    deltRghtDist = (prevRghtDist - newRghtDist);
    
    rghtVelEst=(deltRghtDist/deltRghtTime);
    
    prevRghtDist = newRghtDist;
    prevRghtTime = newRghtTime;
  }
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

void objRghtDirectionClassification_func() //Deduces the direction of motion of an object
{
  if (rghtVelEst < rghtVelAvg && rghtVelEst >0.1) // Object is approaching
  {
    objDirectionRght = 1;
    /*if(serialDebugPrint == true)
    {
      Serial.print("\t\t!!Warning!! Object is approaching right at: ");
      Serial.print(rghtVelEst);
      Serial.print("m/s.");
      Serial.println("");
    }*/
  }

  if (rghtVelEst > rghtVelAvg && rghtVelEst < -0.1) // Object is departing
  {
    objDirectionRght = 2;
    /*if(serialDebugPrint == true)
    {
      Serial.print("\t\tObject is departing right at: ");
      Serial.print(rghtVelEst);
      Serial.print("m/s.");
      Serial.println("");
    }*/
  }

  if (rghtVelEst <0.1 && rghtVelEst > -0.1) // Object is stationary -- Change to use "avgVelEst" at a later date
  {
    objDirectionRght = 3;
    /*if(serialDebugPrint == true)
    {
      Serial.print("\t\tObject right is presumed to be stationary.");
      Serial.println("");
    }*/
  }
}

void rghtHazardClassification_func() //Main hazard classification function
{
  switch(objDirectionRght)
  {
    case 1: // Object moving towards sensor
      // Distance threshold detection
      if(newRghtDist > twoStridesAway)
      {
        rghtHazardScore = 0;
        if(serialDebugPrint == true)
        {
          Serial.print("\t\t| Right Sensor | ");
          Serial.print("Direction: Approaching - Distance: ");
          Serial.print(newRghtDist);
          Serial.print("mm - Object ignored.");
          Serial.println("");
        }
      }

      if((newRghtDist < twoStridesAway) && (newRghtDist > oneStrideAway))
      {
        rghtHazardScore = 1;
        if(serialDebugPrint == true)
        {
          Serial.print("\t\t| Right Sensor | ");
          Serial.print("Direction: Approaching ");
          Serial.print("Distance: ");
          Serial.print(newRghtDist);
          Serial.print("mm - Within two strides - Velocity: ");
          Serial.print(rghtVelEst);
          Serial.print("m/s - Avg Velocity: ");
          Serial.print(rghtVelAvg);
          Serial.print("m/s");
          Serial.println("");
        }
      }

      if(newRghtDist < oneStrideAway)
      {
        rghtHazardScore = 2;
        if(serialDebugPrint == true)
        {
          Serial.print("\t\t| Right Sensor | ");
          Serial.print("Direction: Approaching ");
          Serial.print("Distance: ");
          Serial.print(newRghtDist);
          Serial.print("mm - Within one strides - Velocity: ");
          Serial.print(rghtVelEst);
          Serial.print("m/s - Avg Velocity: ");
          Serial.print(rghtVelAvg);
          Serial.print("m/s");
          Serial.println("");
        }
      }
      break;
    case 2: // Object moving away from sensor
      rghtHazardScore = 0;
      if(serialDebugPrint == true)
      {
        Serial.print("\t\t| Right Sensor | ");
        Serial.print("Direction: Departing - Distance: ");
        Serial.print(newRghtDist);
        Serial.print("mm - Object ignored.");
        Serial.println("");
      }
      break;
    case 3: // Object probably stationary
      rghtHazardScore = 0;
      if(serialDebugPrint == true)
      {
        Serial.print("\t\t| Right Sensor | ");
        Serial.print("Direction: Stationary - Distance: ");
        Serial.print(newRghtDist);
        Serial.print("mm - Object ignored.");
        Serial.println("");
      }
      break;
    default:
      break;
  }
}

void rghtHapticFeedback_func () //Responsible for generating haptic feedback 
{
  rghtHapticTimerStart = millis();
  
  switch(rghtHazardScore)
  {
    case 0:
      rghtHapticSet = 0;
      rghtHapticDutyCyc = 0;
      break;
    case 1: // Moving towards within 2 stride -- low priority
      rghtHapticDutyCyc = 50; //Duty cycle
      rghtHapticSet = map(newRghtDist, twoStridesAway, 10, 55, 255);
      break;
    case 2: // Moving towards within 1 stride -- medium priority
      rghtHapticDutyCyc = 25;
      rghtHapticSet = map(newRghtDist, twoStridesAway, 10, 55, 255);
      break;
    default:
      break;
  }
    
  if (rghtHapticTimerStart - rghtHapticTimerEnd >= rghtHapticDutyCyc)
  {
    rghtHapticTimerEnd = rghtHapticTimerStart;
    if (rghtHapticState == 0)
    {
      rghtHapticState = rghtHapticSet;
      /*if(serialDebugPrint == true)
      {
        Serial.print("\t\tVibrate ON, rghtHapticState = ");
        Serial.print(rghtHapticState);
        Serial.println("");
      }*/
    }
    else
    {
      rghtHapticState = 0;
      /*if(serialDebugPrint == true)
      {
        Serial.print("\t\tVibrate OFF, rghtHapticState = ");
        Serial.print(rghtHapticState);
        Serial.println("");
      }*/
    }
    analogWrite(rghtHapticPin, rghtHapticState); 
  }
}