#include <VL53L1X.h> //Library from: https://github.com/pololu/vl53l1x-arduino
#include <Wire.h>   //Standard arduino library

VL53L1X SEN_LEFT; //Create an object for left sensor 
VL53L1X SEN_RGHT; //Create an object for right sensor

int readFreq = 50; // Used to determine how frequently readings are taken from the sensors in ms, cannot be less than the timing budget
int startRead = millis(); //Used for timing loop

//Velocity estimates
long newLeftTime = 0;
long prevLeftTime = 0;
long deltLeftTime = 0;
long newRghtTime = 0;
long prevRghtTime = 0;
long deltRghtTime = 0;

float newLeftDist = 0;
float prevLeftDist = 0;
float deltLeftDist = 0;
float leftVelEst = 0;

float newRghtDist = 0;
float prevRghtDist = 0;
float deltRghtDist = 0;
float rghtVelEst = 0;
//----------------------//

//Variables for left hazard haptics
int leftHazardSeverity = 0;
const int leftHaptPin = 3;
int leftHaptState = 0; //Will either be 0 or a val between 0-255
int leftHaptSet = 0; //Sets the analogOut value
int leftMaxHapt = 0;
int leftHaptCount = 0;
unsigned long prevLeftHaptMillis = 0;
long leftInteval = 0;
long leftDwell = 0;
long leftHaptStart = 0;

//Variables for right hazard haptics
int rghtHazardSeverity = 0;
const int rghtHaptPin = 5;
int rghtHaptState = 0; //Will either be 0 or a val between 0-255
int rghtHaptSet = 0; //Sets the analogOut value
int rghtMaxHapt = 0;
int rghtHaptCount = 0;
unsigned long prevRghtHaptMillis = 0;
long rghtInteval = 0;
long rghtDwell = 0;
long rghtHaptStart = 0;

#define XSHUT_LEFT 10
#define XSHUT_RGHT 9

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
}

void loop() 
{
    leftHaptics();
    rghtHaptics();
    if((millis()- startRead) > readFreq)
    {
      VelEstFunc();  
      startRead = millis();
    }
    leftHazardClassification();
    rghtHazardClassification();
}//End of 'loop'

void VelEstFunc ()
{
    //Left Sensor Read
    
    newLeftDist = SEN_LEFT.read();
    
    newLeftTime = millis();
    deltLeftTime = (newLeftTime-prevLeftTime);
    
    
    deltLeftDist = (prevLeftDist - newLeftDist);
    
    leftVelEst=(deltLeftDist/deltLeftTime);
    
    //Serial.print(leftVelEst);
    prevLeftDist = newLeftDist;
    prevLeftTime = newLeftTime;
    //Serial.print(",");
    //Right Sensor Read
    
    newRghtDist = SEN_RGHT.read();
    
    newRghtTime = millis();
    deltRghtTime = (newRghtTime-prevRghtTime);
    
    deltRghtDist = (prevRghtDist - newRghtDist);
    
    rghtVelEst=(deltRghtDist/deltRghtTime);
    
    //Serial.print(rghtVelEst);
    prevRghtDist = newRghtDist;
    prevRghtTime = newRghtTime;
    //Serial.println("");

    startRead = millis();
}

void leftHaptics ()
{
    leftHaptStart = millis();
    leftHaptSet = map(newLeftDist, 4000, 0, 130, 255);
    Serial.print(newLeftDist);
    Serial.print(" - ");
    Serial.print(leftHaptSet);
    Serial.print(" -- ");
    switch(leftHazardSeverity)
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
            leftInteval = 5;
            leftDwell = 500;
            break;
        default:
            break;
    }
     
    
    if (leftHaptStart - prevLeftHaptMillis >= leftInteval && leftHaptCount < leftMaxHapt)
    {
        prevLeftHaptMillis = leftHaptStart;
        if (leftHaptState == 0)
        {
            leftHaptState = leftHaptSet;
            //leftHaptState = HIGH;
        }
        else
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

void rghtHaptics ()
{
    rghtHaptStart = millis();
    rghtHaptSet = map(newRghtDist, 4000, 0, 130, 255);
    Serial.print(newRghtDist);
    Serial.print(" - ");
    Serial.print(rghtHaptSet);
    Serial.println("");
    switch(rghtHazardSeverity)
    {
        case 0:
            //rghtHaptSet= 0;
            rghtMaxHapt = 0;
            rghtInteval = 0;
            leftDwell = 0;
            break;
        case 1:
            //rghtHaptSet= 175;
            rghtMaxHapt = 1;
            rghtInteval = 75;
            rghtDwell = 1500;
            break;
        case 2:
           // rghtHaptSet= 175;
            rghtMaxHapt = 2;
            rghtInteval = 75;
            rghtDwell = 1000;
            break;
        case 3:
           // rghtHaptSet= 175;
            rghtMaxHapt = 3;
            rghtInteval = 75;
            rghtDwell = 750;
            break;
        case 4:
           // rghtHaptSet= 200;
            rghtMaxHapt = 5;
            rghtInteval = 5;
            rghtDwell = 500;
            break;
        default:
            break;
    }
     
    
    if (rghtHaptStart - prevRghtHaptMillis >= rghtInteval && rghtHaptCount < rghtMaxHapt)
    {
        prevRghtHaptMillis = rghtHaptStart;
        if (rghtHaptState == 0)
        {
            rghtHaptState = rghtHaptSet;
            //leftHaptState = HIGH;
        }
        else
        {
            rghtHaptState = 0;
            //rghtHaptState = LOW;
            rghtHaptCount++;
        }
    analogWrite(rghtHaptPin, rghtHaptState); 
    }

    if(rghtHaptStart - prevRghtHaptMillis >= rghtDwell)
    {
        rghtHaptCount = 0;
    }
}

void leftHazardClassification ()
{
    if (leftVelEst > 0.7)
    {
        //Serial.print(" - WARNING! Collision possible, move!");
        leftHazardSeverity = 4;
    }
    else
    {
        if (newLeftDist > 600)
        {
            leftHazardSeverity = 1;
        }        
        if (newLeftDist < 600)
        {
            leftHazardSeverity = 2;
        }
        if (newLeftDist <  300)
        {
            leftHazardSeverity = 3;
        }
    }
}

void rghtHazardClassification ()
{
    if (rghtVelEst > 0.7)
    {
        //Serial.print(" - WARNING! Collision possible, move!");
        rghtHazardSeverity = 4;
    }
    else
    {
        if (newRghtDist > 600)
        {
            rghtHazardSeverity = 1;
        }        
        if (newRghtDist < 600)
        {
            rghtHazardSeverity = 2;
        }
        if (newRghtDist < 300)
        {
            rghtHazardSeverity = 3;
        }
    }
}