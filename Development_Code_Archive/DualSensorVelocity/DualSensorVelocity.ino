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

void loop() {
    if((millis()- startRead) > readFreq)
    {
        //Left Sensor Read
        
        newLeftDist = SEN_LEFT.read();
        
        newLeftTime = millis();
        deltLeftTime = (newLeftTime-prevLeftTime);
        
        
        deltLeftDist = (prevLeftDist - newLeftDist);
        
        leftVelEst=(deltLeftDist/deltLeftTime);
        
        Serial.print(leftVelEst);
        prevLeftDist = newLeftDist;
        prevLeftTime = newLeftTime;
        Serial.print(",");
        //Right Sensor Read
        
        newRghtDist = SEN_RGHT.read();
        
        newRghtTime = millis();
        deltRghtTime = (newRghtTime-prevRghtTime);
        
        deltRghtDist = (prevRghtDist - newRghtDist);
        
        rghtVelEst=(deltRghtDist/deltRghtTime);
        
        Serial.print(rghtVelEst);
        prevRghtDist = newRghtDist;
        prevRghtTime = newRghtTime;
        Serial.println("");

       
         
        
        startRead = millis();        

    }
}