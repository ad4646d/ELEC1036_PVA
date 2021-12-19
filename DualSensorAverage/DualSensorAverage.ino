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

#define XSHUT_LEFT 10
#define XSHUT_RGHT 9

//~~Variables for left sensor averaging~~//
//~~Velocity Averaging~~/
#define leftVelVals 20
float leftVelArray[leftVelVals];
float leftVelAvg = 0;
byte leftVelLoc;

//~~Variables for right sensor averaging~~//
//~~Velocity Averaging~~/
#define rghtVelVals 20
float rghtVelArray[rghtVelVals];
float rghtVelAvg = 0;
byte rghtVelLoc;

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

    for (int x = 0; x < leftVelVals; x++) //empty left vel est array
    {
        leftVelArray[x] = 0;
    } 

    for (int x = 0; x < rghtVelVals; x++) //empty right vel est array
    {
        rghtVelArray[x] = 0;
    }       
}

void loop() {
    if((millis()- startRead) > readFreq)
    {
        leftVelEst_func();
        //leftVelEst_comma_print();
        leftVelEst_verbose_print();
        leftVelAvg_func();
        //leftVelAvg_comma_print();
        leftVelAvg_verbose_print();
        
        rghtVelEst_func();
        //rghtVelEst_comma_print();
        rghtVelEst_verbose_print();
        rghtVelAvg_func();
        //rghtVelAvg_comma_print();
        rghtVelAvg_verbose_print();
        Serial.println("");        
        
        startRead = millis();        

    }
}

//~~~~~ Left Sensor Functions ~~~~~//

void leftVelEst_func()
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

void leftVelEst_comma_print()//Prints the latest left velocity value with commas
{
    Serial.print(leftVelEst);
    Serial.print(",");
}

void leftVelEst_verbose_print()//Prints the latest left velocity value with commas
{
    Serial.print("Real-time left velocity est: ");
    Serial.print(leftVelEst);
    Serial.print("m/s -- ");
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

void leftVelAvg_comma_print() //Prints the latest left average velocity value with commas
{
    Serial.print(",");
    Serial.print(leftVelAvg);
    Serial.print(",");
}

void leftVelAvg_verbose_print() //Prints the latest left average velocity value with human readable comments
{
    Serial.print("Rolling Left Vel Avg: ");
    Serial.print(leftVelAvg);
    Serial.print("m/s -- ");
}

//~~~~~ Right Sensor Functions ~~~~~//

void rghtVelEst_func()
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

void rghtVelEst_comma_print()//Prints the latest right velocity value with commas
{
    Serial.print(",");
    Serial.print(rghtVelEst);
    Serial.print(",");
}

void rghtVelEst_verbose_print()//Prints the latest right velocity value with commas
{
    Serial.print("Real-time right velocity est: ");
    Serial.print(rghtVelEst);
    Serial.print("m/s -- ");
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

void rghtVelAvg_comma_print() //Prints the latest right average velocity value with commas
{
    Serial.print(",");
    Serial.print(rghtVelAvg);
    Serial.print(",");
}

void rghtVelAvg_verbose_print() //Prints the latest right average velocity value with human readable comments
{
    Serial.print("Rolling Right Vel Avg: ");
    Serial.print(rghtVelAvg);
    Serial.print("m/s");
}