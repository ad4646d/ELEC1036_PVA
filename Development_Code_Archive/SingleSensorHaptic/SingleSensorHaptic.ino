#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor; //Create sensor as an object
long startTime = 0;
long endTime = 0;

int readFreq = 50; // Used to determine how frequently readings are taken from the sensors in ms, cannot be less than the timing budget
int startRead = millis();

// Velocity Estimate  
float newDist = 0;    
float prevDist = 0;  
float deltDist = 0; 
float velEst = 0;  
//------------------//

int hazardSeverity = 0;
const int vibePin = 3;
int vibeState = 0;
int vibeSet = 0;
int maxVibe = 0;
int vibeCount = 0;
unsigned long previousMillis = 0;
long interval = 0;
long dwell = 0;


void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);

  sensor.startContinuous(50);

  pinMode(vibePin, OUTPUT);
}

void loop()
{
    hapticFeedback();
    if((millis()- startRead) > readFreq)
    {      
        VelEstFunc();
        
        if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
        Serial.println();
        
        startRead = millis();
    }
    
    if (velEst > 0.7)
    {
        Serial.print(" - WARNING! Collision possible, move!");
        hazardSeverity = 4;
    }
    else
    {
        if (newDist > 600)
        {
            hazardSeverity = 1;
        }        
        if (newDist < 600)
        {
            hazardSeverity = 2;
        }
        if (newDist <  300)
        {
            hazardSeverity = 3;
        }
    }
    
    
    Serial.print(" -- HazardSeverity: ");
    Serial.print(hazardSeverity);
    
    Serial.println("");
 
}// End of 'loop'

void VelEstFunc ()
{
    startTime = millis();
        
    newDist = sensor.read();
    Serial.print("New Distance: ");
    Serial.print(newDist);
    Serial.print("mm - ");
    
    Serial.print("Previous Distance: ");
    Serial.print(prevDist);
    Serial.print("mm - ");
    
    endTime = (millis()-startTime);
            
    deltDist = (prevDist - newDist);
    Serial.print("Distance delta: ");
    Serial.print(deltDist);
    Serial.print("mm - ");

    Serial.print("Read time: ");
    Serial.print(endTime);
    Serial.print("ms - ");
    
    velEst = (deltDist / endTime);
    Serial.print("Velocity Estimate: ");
    Serial.print(velEst);
    Serial.print("mm/ms");
    prevDist = newDist; 
}

void hapticFeedback ()
{
    //startTime = millis();
    switch(hazardSeverity)
    {
        case 0:
            vibeSet = 0;
            maxVibe = 0;
            interval = 0;
            dwell = 0;
            break;
        case 1:
            vibeSet = 175;
            maxVibe = 1;
            interval = 75;
            dwell = 1500;
            break;
        case 2:
            vibeSet = 175;
            maxVibe = 2;
            interval = 75;
            dwell = 1000;
            break;
        case 3:
            vibeSet = 175;
            maxVibe = 3;
            interval = 75;
            dwell = 750;
            break;
        case 4:
            vibeSet = 200;
            maxVibe = 5;
            interval = 5;
            dwell = 500;
            break;
        default:
            break;
    }
     
    
    if (startTime - previousMillis >= interval && vibeCount < maxVibe)
    {
        previousMillis = startTime;
        if (vibeState == 0)
        {
            vibeState = vibeSet;
            //vibeState = HIGH;
        }
        else
        {
            vibeState = 0;
            //vibeState = LOW;
            vibeCount++;
        }
    analogWrite(vibePin, vibeState); 
    }

    if(startTime - previousMillis >= dwell)
    {
        vibeCount = 0;
    }
}