#include <VL53L1X.h> //Library from: https://github.com/pololu/vl53l1x-arduino
#include <Wire.h>   //Standard arduino library

VL53L1X SEN_LEFT; //Create an object for left sensor 
VL53L1X SEN_RGHT; //Create an object for right sensor
VL53L1X SEN_FRNT; //Create an object for front sensor

int readFreq = 50; // Used to determine how frequently readings are taken from the sensors in ms, cannot be less than the timing budget
int startRead = millis(); //Used for timing loop

int distLeft = 0;
int distFront = 0;
int distRight = 0;

//Detection thesholds 
int lftNrMin = 300;
int lftNrMax =  600;

int fntNrMin = 300;
int fntNrMax = 600;

int rgtNrMin = 300;
int rgtNrMax = 600;

#define XSHUT_LEFT 10
#define XSHUT_RGHT 9
#define XSHUT_FRNT 8

void setup() {

    Serial.begin(115200);
    
    Wire.begin(); //initialise I2C
    Wire.setClock(400000); // Set I2C to 400kHz

    //Set the pin mode to output
    pinMode(XSHUT_LEFT, OUTPUT);
    pinMode(XSHUT_RGHT, OUTPUT);
    pinMode(XSHUT_FRNT, OUTPUT);

    //Disable all sensors
    digitalWrite(XSHUT_LEFT, LOW);
    digitalWrite(XSHUT_RGHT, LOW);
    digitalWrite(XSHUT_FRNT, LOW);
    
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

      //-------------------------------------------------//
     //        Configuration for front sensor           //
    //-------------------------------------------------//

    delay(50);
    digitalWrite(XSHUT_FRNT, HIGH); 
    delay(50);
    
    SEN_FRNT.setTimeout(500);
    
    if (!SEN_FRNT.init())
    {
        Serial.println("Right sensor hasn't initialised, is it connected?");
    }
    else
    {
        SEN_FRNT.setAddress(40); 
        SEN_FRNT.setDistanceMode(VL53L1X::Long); 
        SEN_FRNT.setMeasurementTimingBudget(50000); 
        SEN_FRNT.startContinuous(50); 
    }
}

void loop() {
    if((millis()- startRead) > readFreq)
    {
        distLeft = SEN_LEFT.read();
        distFront = SEN_FRNT.read();
        distRight = SEN_RGHT.read();

          //-------------------------------------------------//
         //            Left sensor distance zones           //
        //-------------------------------------------------// 
       
        if ((distLeft < lftNrMax) && (distLeft > lftNrMin))
        {
            Serial.print("WARNING - Object is between 300mm and 600mm to the left! --- ");
        }
        if (distLeft < lftNrMin)
        {
            Serial.print("WARNING - Object is less than 300mm away on the left! --- ");
        }
        if (distLeft > lftNrMax)
        {
            Serial.print("Object is not close to left sensor! --- ");
        }

          //-------------------------------------------------//
         //           Front sensor distance zones           //
        //-------------------------------------------------//

        if ((distFront < fntNrMax) && (distFront > fntNrMin))
        {
            Serial.print("WARNING - Object is between 300mm and 600mm in front! --- ");
        }
        if (distFront < fntNrMin)
        {
            Serial.print("WARNING - Object is less than 300mm in front! --- ");
        }
        if (distFront > fntNrMax)
        {
            Serial.print("Object is not close to front sensor! --- ");
        }

          //-------------------------------------------------//
         //           Right sensor distance zones           //
        //-------------------------------------------------//

        if ((distRight < rgtNrMax) && (distRight > rgtNrMin))
        {
            Serial.print("WARNING - Object is between 300mm and 600mm to the right! ");
        }
        if (distRight < rgtNrMin)
        {
            Serial.print("WARNING - Object is less than 300mm to the right!");
        }
        if (distRight > rgtNrMax)
        {
            Serial.print("Object is not close to right sensor! ");
        }
      
        Serial.println("");

        startRead = millis();
    }
}