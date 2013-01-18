#include <PinChangeInt.h>
#include <EEPROM.h>

// Betriebssystem für "Denise" (tauchende Untertasse von J.-Y- Cousteau)
// 
// Autor: Kai Laborenz (kai07@laborenz.de), Duane Banks
//
// basiert zu großen Teile auf:
// rcarduino.blogspot.com
//
// Reading an RC Receiver - What does this signal look like and how do we read it - 
// http://rcarduino.blogspot.co.uk/2012/01/how-to-read-rc-receiver-with.html
//
// The Arduino library only supports two interrupts, the Arduino pinChangeInt Library supports more than 20 - 
// http://rcarduino.blogspot.co.uk/2012/03/need-more-interrupts-to-read-more.html
//
// Using pinChangeInt library and Servo library to read three RC Channels and drive 3 RC outputs (mix of Servos and ESCs)
// http://rcarduino.blogspot.com/2012/04/how-to-read-multiple-rc-channels-draft.html
//
// RC Channels, L293D Motor Driver - Part 2 Calibration And Code
// http://rcarduino.blogspot.de/2012/05/interfacing-rc-channels-to-l293d-motor.html
// 
// rcarduino.blogspot.com
//
//
// Funktionen:
//
// Steuerung von drei Motoren via L293 Doppel-H-Brücke
// Zwei Motoren (für die Antriebspumpen) werden stufenlos in eine Richtung geregelt und zusätzlich zur Steuerung genutzt
// Der dritte Motor steuert den Tauchtank über eine Spindel (vorwärts/ rückwärts)
//
// ToDos:
// - Pumpmotoren via PWM an den Logikanälen steuern (realisierbar?)
// - Tauchtankmotor nicht Stufenlos, sondern nur ein/aus
// - Zusatzfunktionen
//
// Zusatzfunktionen: 
// Spannungsüberwachung, Test auf Wasser innen, Test RC-Signalqualität -> Notaufstieg
// Beleuchtung an über Feuchtigkeit außen
// Scheinwerfer an über Kanal 4
//


//  Basiswerte für RC-Signale, falls noch keine ermittelt wurden
#define RC_NEUTRAL 1500
#define RC_MAX 2000
#define RC_MIN 1000
#define RC_DEADBAND 40

// Steuerung
uint16_t unSteeringMin = RC_MIN;
uint16_t unSteeringMax = RC_MAX;
uint16_t unSteeringCenter = RC_NEUTRAL;
// Geschwindigkeit
uint16_t unThrottleMin = RC_MIN;
uint16_t unThrottleMax = RC_MAX;
uint16_t unThrottleCenter = RC_NEUTRAL;
// Pumpe
uint16_t unPumpMin = RC_MIN;
uint16_t unPumpMax = RC_MAX;
uint16_t unPumpCenter = RC_NEUTRAL;

#define PWM_MIN 0
#define PWM_MAX 255


#define PWM_SPEED_PUMP 10   // Geschwindigkeit Tauchpumpe
#define PWM_SPEED_MOVE 11   // Geschwindigkeit Motoren
#define PUMP1 5  // Logik Tauchpumpe
#define PUMP2 6
#define MOVE1 7  // Logik Motoren
#define MOVE2 8

#define PROGRAM_PIN A5 // Schalter, um den Programmiermodus zu starten

#define ENDSWITCH 9 // Endlagenschalter für die Position des Tauchtanks  

// Eingänge für die RC-Kanäle
#define THROTTLE_IN_PIN 2   // Geschwindigkeit
#define STEERING_IN_PIN 3   // Steuerung
#define PUMP_IN_PIN 4       // Tauchpumpe

// bit flags zur Anzeige, welche Kanäle neue Signale haben (gesetzt in bUpdateFlagsShared)
#define THROTTLE_FLAG 1
#define STEERING_FLAG 2
#define PUMP_FLAG 3

// holds the update flags defined above
volatile uint8_t bUpdateFlagsShared;  

// Änderung beim Endschalter 
volatile boolean newEndShared;        

// shared variables are updated by the ISR (interrupt service routine) and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unThrottleInShared;
volatile uint16_t unSteeringInShared;
volatile uint16_t unPumpInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulThrottleStart;
uint32_t ulSteeringStart;
uint32_t ulPumpStart;

// erst einmal alles auf "Stop" setzen
uint8_t gThrottle = 0;
uint8_t gPump = 0;

// Fälle für die Steuerung in der Loop
#define DIRECTION_STOP 0           // Stop
#define DIRECTION_FORWARD 1        // Bewegung vorwärts
#define DIRECTION_REVERSE 2        // wird verm. nicht gebraucht
#define DIRECTION_ROTATE_RIGHT 3   // Drehung nach rechts
#define DIRECTION_ROTATE_LEFT 4    // Drehung nach links
#define FILL_PUMP 5                // Tauchtank fluten
#define EMPTY_PUMP 6               // Tauchtank lenzen

uint8_t gThrottleDirection = DIRECTION_STOP;
uint8_t gDirection = DIRECTION_STOP;
uint8_t gOldDirection = DIRECTION_STOP;
uint8_t gPumpDirection = DIRECTION_STOP;
uint8_t gOldPumpDirection = DIRECTION_STOP;

// Maximalwert 
#define IDLE_MAX 80

// Programmodi
#define MODE_RUN 0      // Normaler Programmablauf, Betriebszustand
#define MODE_PROGRAM 1  // Programmiermodus, in dem die RC-Kanäle gelesen und die individuellen Maximalwerte bestimmt werden

uint8_t gMode = MODE_RUN; // Zum Start erst einmal in den RUN-Modus
uint32_t ulProgramModeExitTime = 0; 

// Im Eeprom werden die Werte für die RC-Signale gespeichert
// Index into the EEPROM Storage assuming a 0 based array of uint16_t
// Data to be stored low byte, high byte
#define EEPROM_INDEX_STEERING_MIN 0
#define EEPROM_INDEX_STEERING_MAX 1
#define EEPROM_INDEX_STEERING_CENTER 2
#define EEPROM_INDEX_THROTTLE_MIN 3
#define EEPROM_INDEX_THROTTLE_MAX 4
#define EEPROM_INDEX_THROTTLE_CENTER 5
#define EEPROM_INDEX_PUMP_MIN 6
#define EEPROM_INDEX_PUMP_MAX 7
#define EEPROM_INDEX_PUMP_CENTER 8

// Endabschaltung mit Wiederanlauf für Tauchpumpe
boolean endposReached;

int pumpPosition;    // Position der Tauchpumpe siehe nächste Werte

#define PUMP_UPPER 1 // Pumpe hat obersten Werte erreicht -> Tauchtank ist voll
#define PUMP_LOWER 2 // Pumpe hat untersten Punkt erreicht -> Tauchtank ist leer

// Letzte Richtung der Tauchpumpe für Endschalter-Umkehr
uint8_t lastPumpDirection;

// Errorstatus
int error;

// Blinkschaltung
const int signalLED =  12;        // the number of the LED pin
int ledState = LOW;            // ledState used to set the LED
long previousMillis = 0;       // will store last time LED was updated
long onMillis = 0;             // wann wurde LED eingeschaltet
long ledIinterval = 1500;      // interval at which to blink (milliseconds)
long ledDuration = 100;        // Brenndauer, Dauer eines Blitzers (milliseconds)

// Spannungsmessung
int sensorPin = A0;      // analoger Eingang
int vdiv = 1754;         // Wert des Spannnungsteilers (durch 10000 teilen)
int vmin = 3700;         // Minimalwert für Batteriespannung in Millivolt

void setup()
{
  Serial.begin(9600);

  // Mehr als 2 Interrupts gehen nur mit der PinChange-Library
  PCintPort::attachInterrupt(THROTTLE_IN_PIN ,calcThrottle,CHANGE);
  PCintPort::attachInterrupt(STEERING_IN_PIN,calcSteering,CHANGE);
  PCintPort::attachInterrupt(PUMP_IN_PIN,calcPump,CHANGE);
  
  PCintPort::attachInterrupt(ENDSWITCH,checkEnd,CHANGE);

  pinMode(PWM_SPEED_PUMP,OUTPUT);
  pinMode(PWM_SPEED_MOVE,OUTPUT);
  
  pinMode(PUMP1,OUTPUT);
  pinMode(PUMP2,OUTPUT);
  
  pinMode(MOVE1,OUTPUT);
  pinMode(MOVE2,OUTPUT);
  
  pinMode(PROGRAM_PIN,INPUT_PULLUP);
  pinMode(ENDSWITCH,INPUT_PULLUP);
  
  pinMode(signalLED, OUTPUT);
  
  analogReference(INTERNAL);
  
  readSettingsFromEEPROM();

}

void loop()
{ 
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained
  // between calls to loop.
  static uint16_t unThrottleIn;
  static uint16_t unSteeringIn;
  static uint16_t unPumpIn;
  // local copy of update flags
  static uint8_t bUpdateFlags;
  
  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;

    // in the current code, the shared values are always populated
    // so we could copy them without testing the flags
    // however in the future this could change, so lets
    // only copy when the flags tell us we can.

    if(bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;
    }
    if(bUpdateFlags & STEERING_FLAG)
    {
      unSteeringIn = unSteeringInShared;
    }
    if(bUpdateFlags & PUMP_FLAG)
    {
      unPumpIn = unPumpInShared;
    }

    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;

    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
    // service routines own these and could update them at any time. During the update, the
    // shared copies may contain junk. Luckily we have our local copies to work with :-)
  }
  
  if(!digitalRead(PROGRAM_PIN))
  {
    // give 10 seconds to program
    ulProgramModeExitTime = millis() + 10000;
    gMode = MODE_PROGRAM;
    
    unThrottleMin = RC_NEUTRAL;
    unThrottleMax = RC_NEUTRAL;
    unSteeringMin = RC_NEUTRAL;
    unSteeringMax = RC_NEUTRAL;
    unPumpMin = RC_NEUTRAL;
    unPumpMax = RC_NEUTRAL;
    
    unThrottleCenter = unThrottleIn;
    unSteeringCenter = unSteeringIn;
    unPumpCenter = unPumpIn;
    
    gDirection = DIRECTION_STOP;
    gPumpDirection = DIRECTION_STOP;
    
    delay(20);
  }
  
  if(gMode == MODE_PROGRAM)
  {
   if(ulProgramModeExitTime < millis())
   {
     // set to 0 to exit program mode
     ulProgramModeExitTime = 0;
     gMode = MODE_RUN;
     
     writeSettingsToEEPROM();
     
     digitalWrite(signalLED, HIGH);
     delay(2000); 
     digitalWrite(signalLED, LOW);
     
   }
   else
   {
     // Maximalwerte für alle Kanäle ermitteln 
     if(unThrottleIn > unThrottleMax && unThrottleIn <= RC_MAX)
     {
       unThrottleMax = unThrottleIn;
     }
     else if(unThrottleIn < unThrottleMin && unThrottleIn >= RC_MIN)
     {
       unThrottleMin = unThrottleIn;
     }
     
     if(unSteeringIn > unSteeringMax && unSteeringIn <= RC_MAX)
     {
       unSteeringMax = unSteeringIn;
     }
     else if(unSteeringIn < unSteeringMin && unSteeringIn >= RC_MIN)
     {
       unSteeringMin = unSteeringIn;
     }
     
     if(unPumpIn > unPumpMax && unPumpIn <= RC_MAX)
     {
       unPumpMax = unPumpIn;
     }
     else if(unPumpIn < unPumpMin && unPumpIn >= RC_MIN)
     {
       unPumpMin = unPumpIn;
     }     
     
   }
  }

  // do any processing from here onwards
  
  if (error) {
    ledIinterval = 200;
  }
  else {
    ledIinterval = 1500;
  }
  
  // LED-Blitzer
  
  unsigned long currentMillis = millis();
  
  if(currentMillis - previousMillis > ledIinterval) {
    // Zeitpunkt des letzten Blitzes neu setzen
    previousMillis = currentMillis;   

    // wenn LED aus, anschalten und Zeit merken
    if (ledState == LOW)
      ledState = HIGH;
      onMillis = currentMillis;     
  }
    // wenn LED aus und Brenndauer abgelaufen, wieder ausschalten
    else {
      if (currentMillis - onMillis > ledDuration){
      ledState = LOW;
      }
      
    digitalWrite(signalLED, ledState);
  }  
  
  // Spannungsüberwachung
  // 
  
  // Sensorwert lesen
  // wird später durch direkte Messung von Vcc ersetzt
  int sensorValue = analogRead(sensorPin);    
  float value = sensorValue * 1.1 / vdiv * 10000 / 1023.0 * 1000;
 
  if (value > vmin) {
    error = false;
  }
  else {
    error = true;
  }
  
  // only use the local values unAuxIn, unThrottleIn and unSteeringIn, the shared
  // variables unAuxInShared, unThrottleInShared, unSteeringInShared are always owned by
  // the interrupt routines and should not be used in loop
  
  if(gMode == MODE_RUN)
  {
    
    // Neue Signale vom Empfänger werden durch die 
    // Update-Flags angezeigt
    
    // Änderung der Geschwindigkeit
    
    if(bUpdateFlags & THROTTLE_FLAG)
    {
      // A good idea would be to check the before and after value, 
      // if they are not equal we are receiving out of range signals
      // this could be an error, interference or a transmitter setting change
      // in any case its a good idea to at least flag it to the user somehow
      unThrottleIn = constrain(unThrottleIn,unThrottleMin,unThrottleMax);
      
      // vorwärts
      if(unThrottleIn > unThrottleCenter)
      {
        gThrottle = map(unThrottleIn,unThrottleCenter,unThrottleMax,PWM_MIN,PWM_MAX);
        gThrottleDirection = DIRECTION_FORWARD;
      }
      // statt rückwärts -> stop
      else
      {
        gThrottleDirection = DIRECTION_STOP;
      }
    }
  
    // Änderung der Richtung

    // Im Moment wird einfach der Kurveninnere Motor ausgeschaltet
    // evtl. lässt sich durch Pulsen der Logikeingänge auch eine stufenlose Regelung realisieren
    
    if(bUpdateFlags & STEERING_FLAG)
    {
      uint8_t throttleLeft = gThrottle;
      uint8_t throttleRight = gThrottle;
  
      gDirection = gThrottleDirection;
      
      // see previous comments regarding trapping out of range errors
      // this is left for the user to decide how to handle and flag
      unSteeringIn = constrain(unSteeringIn,unSteeringMin,unSteeringMax);
  
        if(unSteeringIn > (unSteeringCenter + RC_DEADBAND))
        {
          gDirection = DIRECTION_ROTATE_RIGHT;
          // use steering to set throttle
          // throttleRight = throttleLeft = map(unSteeringIn,unSteeringCenter,unSteeringMax,PWM_MIN,PWM_MAX);
        }
        else if(unSteeringIn < (unSteeringCenter - RC_DEADBAND))
        {
          gDirection = DIRECTION_ROTATE_LEFT;
          // use steering to set throttle
          // throttleRight = throttleLeft = map(unSteeringIn,unSteeringMin,unSteeringCenter,PWM_MAX,PWM_MIN);
        }
      
      analogWrite(PWM_SPEED_MOVE,gThrottle);
    }
    
  // Änderung bei der Tauchpumpe

  if((bUpdateFlags & PUMP_FLAG) | newEndShared)
    {
      // A good idea would be to check the before and after value, 
      // if they are not equal we are receiving out of range signals
      // this could be an error, interference or a transmitter setting change
      // in any case its a good idea to at least flag it to the user somehow
      unPumpIn = constrain(unPumpIn,unPumpMin,unPumpMax);
            
      if(unPumpIn > (unPumpCenter + RC_DEADBAND))
      {
        gPump = map(unPumpIn,unPumpCenter,unPumpMax,PWM_MIN,PWM_MAX);
        gPumpDirection = FILL_PUMP;
        
        if (endposReached)
        {
          pumpPosition = PUMP_UPPER;
        }
        else {
          pumpPosition = 0;
        } 
      }
      else if (unPumpIn < (unPumpCenter - RC_DEADBAND))
      {
        gPump = map(unPumpIn,unPumpMin,unPumpCenter,PWM_MAX,PWM_MIN);
        gPumpDirection = EMPTY_PUMP;
        
        if (endposReached)
        {
          pumpPosition = PUMP_LOWER;
        }
        else {
          pumpPosition = 0;
        }
      }
      else {
        pumpPosition = 0;
        gPump = 0;
      }  
           
      analogWrite(PWM_SPEED_PUMP,gPump);
      Serial.println(gPump);
     
    }
    /* funktioniert noch nicht 
    if (error) {
      gPumpDirection = EMPTY_PUMP;
      analogWrite(PWM_SPEED_PUMP,gPump);
    }
    */
     
  // Steuerung 
  
  steerPump(); 
  
  moveSteer();    
    
  } // end MODE==RUN

  bUpdateFlags = 0;
  newEndShared = 0;
  
} // end loop


//
//  Subroutinen
//

void moveSteer ()

  // Steuerung von Geschwindigkeit und Richtung von Denise über Kanal 2 des L293 (Pin 9-16)

{
  if(gDirection != gOldDirection)
  {
    gOldDirection = gDirection;

    digitalWrite(MOVE1,HIGH);
    digitalWrite(MOVE2,LOW);
    analogWrite(PWM_SPEED_MOVE,LOW);

    switch(gDirection)
    {
    case DIRECTION_FORWARD:
      digitalWrite(MOVE1,LOW);
      digitalWrite(MOVE2,HIGH);
      break;    
    case DIRECTION_ROTATE_LEFT:
      digitalWrite(MOVE1,HIGH); 
      digitalWrite(MOVE2,HIGH);
      break;
    case DIRECTION_ROTATE_RIGHT:
      digitalWrite(MOVE1,LOW);
      digitalWrite(MOVE2,LOW);
      break;
    case DIRECTION_STOP:
      analogWrite(PWM_SPEED_MOVE,LOW);
      break;
    }
  }
}


void steerPump ()

  // Steuerung der Tauchpumpe über Kanal 1 des L293 (Pins 1-8)

{
    if((gPumpDirection != gOldPumpDirection) | (newEndShared) | (error))
  {
    gOldPumpDirection = gPumpDirection;
       
    digitalWrite(PUMP1,LOW);
    digitalWrite(PUMP2,LOW);
    analogWrite(PWM_SPEED_PUMP,LOW);  
   
    switch(gPumpDirection)
    {
    case FILL_PUMP:   
    if ((pumpPosition != PUMP_UPPER) | (gPumpDirection != lastPumpDirection))
    {
      digitalWrite(PUMP1,LOW);
      digitalWrite(PUMP2,HIGH);
    }
    break;
    
    case EMPTY_PUMP:    
    if ((pumpPosition != PUMP_LOWER) | (gPumpDirection != lastPumpDirection))
    {
      digitalWrite(PUMP1,HIGH);
      digitalWrite(PUMP2,LOW);
    }
    break;
    
    case DIRECTION_STOP:
      digitalWrite(PUMP1,LOW);
      digitalWrite(PUMP2,LOW);
      analogWrite(PWM_SPEED_PUMP,LOW);
      break;
    }
    
  }
  
}

//
// Interrupt-Routinen (ISR)
//

void calcThrottle()
{
  // if the pin is high, its a rising edge of the signal pulse, so lets record its value
  if(digitalRead(THROTTLE_IN_PIN) == HIGH)
  {
    ulThrottleStart = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time of the rising edge
    // this gives use the time between the rising and falling edges i.e. the pulse duration.
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
    // use set the throttle flag to indicate that a new throttle signal has been received
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

void calcSteering()
{
  if(digitalRead(STEERING_IN_PIN) == HIGH)
  {
    ulSteeringStart = micros();
  }
  else
  {
    unSteeringInShared = (uint16_t)(micros() - ulSteeringStart);
    bUpdateFlagsShared |= STEERING_FLAG;
  }
}

void calcPump()
{
  if(digitalRead(PUMP_IN_PIN) == HIGH)
  {
    ulPumpStart = micros();
  }
  else
  {
    unPumpInShared = (uint16_t)(micros() - ulPumpStart);
    bUpdateFlagsShared |= PUMP_FLAG;
  }
}

void checkEnd() 
{
  if (false == digitalRead(ENDSWITCH))
    {
      endposReached = 1;
      lastPumpDirection = gPumpDirection;
    }
    else 
    {
      endposReached = 0;
     }
    newEndShared = 1;
    
}


// Speicherung der ausgelesenen Maximal- und Minimalwerte für die RC-Kanäle im Eeprom

void readSettingsFromEEPROM()
{
  unSteeringMin = readChannelSetting(EEPROM_INDEX_STEERING_MIN);
  if(unSteeringMin < RC_MIN || unSteeringMin > RC_NEUTRAL)
  {
    unSteeringMin = RC_MIN;
  }
  //Serial.println(unSteeringMin);

  unSteeringMax = readChannelSetting(EEPROM_INDEX_STEERING_MAX);
  if(unSteeringMax > RC_MAX || unSteeringMax < RC_NEUTRAL)
  {
    unSteeringMax = RC_MAX;
  }
  //Serial.println(unSteeringMax);
  
  unSteeringCenter = readChannelSetting(EEPROM_INDEX_STEERING_CENTER);
  if(unSteeringCenter < unSteeringMin || unSteeringCenter > unSteeringMax)
  {
    unSteeringCenter = RC_NEUTRAL;
  }
  //Serial.println(unSteeringCenter);

  unThrottleMin = readChannelSetting(EEPROM_INDEX_THROTTLE_MIN);
  if(unThrottleMin < RC_MIN || unThrottleMin > RC_NEUTRAL)
  {
    unThrottleMin = RC_MIN;
  }
  //Serial.println(unThrottleMin);

  unThrottleMax = readChannelSetting(EEPROM_INDEX_THROTTLE_MAX);
  if(unThrottleMax > RC_MAX || unThrottleMax < RC_NEUTRAL)
  {
    unThrottleMax = RC_MAX;
  }
  //Serial.println(unThrottleMax);
  
  unThrottleCenter = readChannelSetting(EEPROM_INDEX_THROTTLE_CENTER);
  if(unThrottleCenter < unThrottleMin || unThrottleCenter > unThrottleMax)
  {
    unThrottleCenter = RC_NEUTRAL;
  }
  //Serial.println(unThrottleCenter);
  
  unPumpMin = readChannelSetting(EEPROM_INDEX_PUMP_MIN);
  if(unPumpMin < RC_MIN || unPumpMin > RC_NEUTRAL)
  {
    unPumpMin = RC_MIN;
  }
  //Serial.println(unPumpMin);

  unPumpMax = readChannelSetting(EEPROM_INDEX_PUMP_MAX);
  if(unPumpMax > RC_MAX || unPumpMax < RC_NEUTRAL)
  {
    unPumpMax = RC_MAX;
  }
  //Serial.println(unPumpMax);
  
  unPumpCenter = readChannelSetting(EEPROM_INDEX_PUMP_CENTER);
  if(unPumpCenter < unPumpMin || unPumpCenter > unPumpMax)
  {
    unPumpCenter = RC_NEUTRAL;
  }
  //Serial.println(unPumpCenter);
}

void writeSettingsToEEPROM()
{
  writeChannelSetting(EEPROM_INDEX_STEERING_MIN,unSteeringMin);
  writeChannelSetting(EEPROM_INDEX_STEERING_MAX,unSteeringMax);
  writeChannelSetting(EEPROM_INDEX_STEERING_CENTER,unSteeringCenter);
  writeChannelSetting(EEPROM_INDEX_THROTTLE_MIN,unThrottleMin);
  writeChannelSetting(EEPROM_INDEX_THROTTLE_MAX,unThrottleMax);
  writeChannelSetting(EEPROM_INDEX_THROTTLE_CENTER,unThrottleCenter);
  writeChannelSetting(EEPROM_INDEX_PUMP_MIN,unPumpMin);
  writeChannelSetting(EEPROM_INDEX_PUMP_MAX,unPumpMax);
  writeChannelSetting(EEPROM_INDEX_PUMP_CENTER,unPumpCenter);
            
  /* Serial.println(unSteeringMin);
  Serial.println(unSteeringMax);
  Serial.println(unSteeringCenter);
  Serial.println(unThrottleMin);
  Serial.println(unThrottleMax);
  Serial.println(unThrottleCenter);
  Serial.println(unPumpMin);
  Serial.println(unPumpMax);
  Serial.println(unPumpCenter);*/
}


uint16_t readChannelSetting(uint8_t nStart)
{
  uint16_t unSetting = (EEPROM.read((nStart*sizeof(uint16_t))+1)<<8);
  unSetting += EEPROM.read(nStart*sizeof(uint16_t));

  return unSetting;
}

void writeChannelSetting(uint8_t nIndex,uint16_t unSetting)
{
  EEPROM.write(nIndex*sizeof(uint16_t),lowByte(unSetting));
  EEPROM.write((nIndex*sizeof(uint16_t))+1,highByte(unSetting));
}
/*
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  
 
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
*/
