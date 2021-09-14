// HEADER FILES
#include <Arduino.h>
#include "tramControl.h"
#include "src/modules/servoSweep.h"
#include "src/basics/macros.h"
#include "src/basics/io.h"
#include "src/basics/stateMachineClass.h"
#include "src/modules/weistra.h"
#include "src/modules/debounceClass.h"
#include "src/basics/macros.h"

enum tramControlStates_
{
    tramControlIDLE,
    readButtons,
    accelerateTrain,
    waitDeparture,
    waitArrival,
    slowDownTrain
} ;


Debounce frontBtn( auto_manualPin ) ;
Debounce rearBtn( frontSwPin ) ; 
Debounce autoManualBtn( rearSwPin ) ;
Debounce detector( detectorPin ) ;

Weistra throttle( throttlePin ) ;
                                //   min  max spd  turn off
ServoSweep  leftPoint(  leftServoPin, 75, 105, 10, 1 ) ;
ServoSweep rightPoint( rightServoPin, 75, 105, 10, 1 ) ;

StateMachine sm ;

#define beginState readButtons
#ifndef beginState
#error beginState not yet defined
#endif

 /* RELAY PIN = HIGH -> FRONT TRACK ENABLED ) */
#define FRONT_SIDE HIGH
#define  REAR_SIDE LOW

// VARIABLES
uint32_t speedInterval ;
uint8_t frontBtnState, rearBtnState, autoManualState, detectorState ;
uint8_t setPoint, speed ;


// FUNCTIONS
extern void tramControlInit(void)
{
    throttle.begin() ;
    leftPoint.begin() ;
    rightPoint.begin() ;
    leftPoint.setState( 0 ) ;
    rightPoint.setState( 0 ) ;
    sm.nextState( beginState, 0 ) ;
}

static void debounceInput()
{
    if( digitalRead( throttlePin ) ) // we do not debounce the current sensing when the throttle pin is in the LOW side in a dutycycle
    {
        REPEAT_MS( 50 ) ;
        detector.debounceInputs() ;
        END_REPEAT
    }
        
    REPEAT_MS( 20 ) ;
    frontBtn.debounceInputs() ;
    rearBtn.debounceInputs() ;
    autoManualBtn.debounceInputs() ;
    END_REPEAT
    
    frontBtnState = frontBtn.readInput() ;
    rearBtnState = rearBtn.readInput() ;
    autoManualState = autoManualBtn.readInput() ;
    detectorState = detector.readInput() ;
}

void updateSpeed()
{
    REPEAT_MS( speedInterval ) ;
    if( speed < setPoint ) speed ++ ;
    if( speed > setPoint ) speed -- ;
    
    throttle.setSpeed( speed ) ;
    
    Serial.print(F("speed: ")) ;
    Serial.println( speed ) ;
    END_REPEAT
    
    throttle.update() ;
} 

// STATE FUNCTIONS
/* 
Read all 3 buttons and determen which train should drive.
also sets LEDs, relay and points
*/
StateFunction( readButtons )
{
    if( sm.entryState() )
    {
        Serial.println(F("train is ready to depart!")) ;
        if( autoManualState  == HIGH )              // in manual mode, turn on both LEDs
        {
            digitalWrite( statusLedFront, HIGH ) ;
            digitalWrite(  statusLedRear, HIGH ) ;
        }
    }
    
    if( sm.onState() )
    {
        if( frontBtnState == RISING             // if a button is pressed and released or mode is automatic -> exit
        ||   rearBtnState == RISING 
        ||   autoManualState == LOW ) 
        {
            sm.exit() ;
        }
    }
    
    if( sm.exitState() )
    {
        if( frontBtnState == RISING )
        {
            Serial.println(F("front track is go")) ;
            digitalWrite( relayPin, FRONT_SIDE ) ;  
            digitalWrite(  statusLedRear, LOW ) ;  // set relay and turn other led OFF
        }
        if(  rearBtnState == RISING )
        {
            Serial.println(F("rear track is go")) ;
            digitalWrite( relayPin,  REAR_SIDE ) ;
            digitalWrite( statusLedFront, LOW ) ;
        }
        if( autoManualState  == LOW ) 
        {
            Serial.println(F("automatic mode enabled")) ;
            digitalWrite( relayPin, !digitalRead( relayPin ) ) ; // if driving in automatic mode, just toggle the relay and turn both LEDs OFF
        }
        
        Serial.println(F(" setting points ")) ;
        if( digitalRead( relayPin ) == FRONT_SIDE )
        {
            leftPoint.setState( 1 ) ;
            leftPoint.setState( 1 ) ;
        }
        else
        {
            leftPoint.setState( 0 ) ;
            leftPoint.setState( 0 ) ;
        }
    }
    return sm.endState() ;
}

/*
Determens maximum speed, and sets the speed setpoint accordingly.
Than wait until there is no more current sensed
*/
StateFunction( accelerateTrain )
{
    if( sm.entryState() )
    {
        int sample = analogRead( speedPin ) ;
        setPoint = map( sample, 0 ,1023, 20, 100 ) ; // speed is set between 20 - 100 %
        speedInterval = 50 ;                         //  50ms between speed increments -> 20 updates per second, max 5 second acceleration at top speed.
        Serial.println(F("points are set\r\ntrain departing")) ;
    }
    if( sm.onState() )
    {
        if( detectorState == RISING ) sm.exit() ;     // if no current is sensed, the train is on the main track
    }
    if( sm.exitState() )
    {
        Serial.println(F("train has left the station")) ;
    }
    return sm.endState() ;
}

/*
Just wait until the train has reached the front or rear track again
*/
StateFunction( waitArrival )
{
    if( sm.entryState() )
    {
        
    }
    if( sm.onState() )
    {
        if( detectorState == FALLING ) sm.exit() ; // just wait on current sense detector...
    }
    if( sm.exitState() )
    {
        Serial.println(F("train has reached the station")) ;
    }
    return sm.endState() ;
}

/*
Slows downt the train to a full stop. Deceleration factor depends of which train is driving
and how it's matching potentiometer is set. Finished when train has stopped completely

Turns off both LEDs to indicate there is a delay
*/
StateFunction( slowDownTrain )
{
    if( sm.entryState() )
    {
        int sample ;
        if( digitalRead( relayPin ) == FRONT_SIDE ) sample = analogRead( frontBrakeSpeed ) ; 
        else                                        sample = analogRead(  rearBrakeSpeed ) ;
        speedInterval = map( sample, 0 ,1023, 1, 50 ) ;                         // determen brakespeed, depended of relay state
        
        setPoint = 0 ;                                                          // stop train
        
        Serial.println(F("slowing down train")) ;
    }
    if( sm.onState() )
    {
        if( speed == 0 ) sm.exit() ;                                            // if train has stopped -> exit
    }
    if( sm.exitState() )
    {
        digitalWrite( statusLedFront, LOW ) ;                                   // turn of leds, to indicate that there is a delay
        digitalWrite(  statusLedRear, LOW ) ;
        
        Serial.println(F("train has stopped, restarting cycle")) ;
    }
    return sm.endState() ;
}


// STATE MACHINE
extern uint8_t tramControl()
{
    debounceInput() ;
    updateSpeed() ;
    
    leftPoint.sweep() ;
    rightPoint.sweep() ;
        
    
    STATE_MACHINE_BEGIN

    State(readButtons) {
        sm.nextState( accelerateTrain, 300 ) ; } // short delay to set the points

    State(accelerateTrain) {
        sm.nextState( waitArrival, 0 ) ; }

    State(waitArrival) {
        sm.nextState( slowDownTrain, 0 ) ; }

    State(slowDownTrain) {
        sm.nextState( readButtons, 5000 ) ; } // wait 5 seconds before another train is allowed to depart

    STATE_MACHINE_END
}
