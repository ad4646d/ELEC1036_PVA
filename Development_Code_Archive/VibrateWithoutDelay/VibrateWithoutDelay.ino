//Heavily inspired by the infamous 'BlinkWithoutDelay' example sketch
const int vibePin = 3;
int vibeState = 0;
int maxVibe = 3;
int vibeCount = 0;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
const long interval = 70;
const long dwell = 500;

void setup()
{
    pinMode(vibePin, OUTPUT);
}

void loop ()
{
    currentMillis = millis();
    if (currentMillis - previousMillis >= interval && vibeCount < maxVibe)
    {
        previousMillis = currentMillis;
        if (vibeState == 0)
        {
            vibeState = 255;
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

    if(currentMillis - previousMillis >= dwell)
    {
        vibeCount = 0;
    }

}