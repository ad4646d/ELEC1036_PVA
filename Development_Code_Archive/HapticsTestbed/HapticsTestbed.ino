int hapticScore = 1;
const int vibePin = 3;
int vibeState = 0;
int vibeSet = 0;
int maxVibe = 0;
int vibeCount = 0;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
long interval = 0;
long dwell = 0;

void setup()
{
    Serial.begin(115200); // High serial baud rate
    pinMode(vibePin, OUTPUT);
}

void loop ()
{
    currentMillis = millis();
    switch(hapticScore)
    {
        case 0:
            vibeSet = 0;
            maxVibe = 0;
            interval = 0;
            dwell = 0;
            break;
        case 1: // Moving towards within 1 stride -- medium priority
            vibeSet = 160;
            maxVibe = 3;
            interval = 30;
            dwell = 500;
            break;
        case 2: // Moving towards within 2 strides -- medium priority
            vibeSet = 125;
            maxVibe = 3;
            interval = 50;
            dwell = 500;
            break;
        case 3: // Stationary within one stride -- low priority
            vibeSet = 160;
            maxVibe = 1;
            interval = 75;
            dwell = 2000;
            break;
        case 4: // Stationary within two strides -- low priority
            vibeSet = 125;
            maxVibe = 1;
            interval = 60;
            dwell = 3000;
            break;
        case 5: // Impact possible -- high priority
            vibeSet = 255;
            maxVibe = 1;
            interval = 30;
            dwell = 60;
            break;
        default:
            break;
    }
     
    if (currentMillis - previousMillis >= interval && vibeCount < maxVibe)
    {
        previousMillis = currentMillis;
        if (vibeState == 0)
        {
            vibeState = vibeSet;
            Serial.print("Vibrate ON, haptState val = ");
            Serial.print(vibeState);
            Serial.println("");
            //vibeState = HIGH;
            
        }
        else
        {
            vibeState = 0;
            Serial.print("Vibrate OFF, vibeState val = ");
            Serial.print(vibeState);
            Serial.println("");
            //vibeState = LOW;
            vibeCount++;
        }
    analogWrite(vibePin, vibeState); 
    }
    
    /*if((currentMillis - previousMillis < dwell) && (vibeState != 0))
    {
        vibeCount = 0;
        vibeState = 0;
        analogWrite(vibePin, vibeState); 
        Serial.print("Likely haven't completed vibrate cycle and moved state, hapState val = ");
        Serial.print(vibeState);
        Serial.println("~~RESET VIBECOUNT~~"); 
    }*/

    /*if((vibeCount == maxVibe || vibeCount > maxVibe ) && (vibeState > 0))
    {
        
        vibeState =0;
        Serial.println("Haptics accidentally on, haptics turned off until next cycle.");
    }*/

    if(currentMillis - previousMillis >= interval && vibeState > 0)
    {
        vibeCount = 0;
        vibeState = 0;
        Serial.println("Haptics accidentally on, haptics turned off until next cycle.");
    }
    
    if(currentMillis - previousMillis >= dwell)
    {
        vibeCount = 0;
        vibeState = 0;
        analogWrite(vibePin, vibeState); 
        Serial.print("Completed vibration cycle, vibeState val = ");
        Serial.print(vibeState);
        Serial.println("~~RESET VIBECOUNT~~");
    }
   
}
//end
