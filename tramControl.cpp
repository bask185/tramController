// HEADER FILES
#include <Arduino.h>
#include "tramControl.h"
#include "src/servoSweep.h"
#include "src/macros.h"
#include "src/io.h"
#include "src/stateMachineClass.h"
#include "src/weistra.h"
#include "src/debounceClass.h"
#include "src/macros.h"


enum tramControlStates_
{
    tramControlIDLE,
    readButtons,
    accelerateTrain,
    waitDeparture,
    waitArrival,
    slowDownTrain
} ;

uint32_t timeStamp ;
enum errors
{
    tramDepartureToLate ,
    tramArrivalToLate,
    shortCircuit,
} ;


Debounce frontBtn( frontSwPin ) ;
Debounce rearBtn( rearSwPin ) ; 
Debounce autoManualBtn( auto_manualPin) ;

Weistra throttle( throttlePin, 50, 100 ) ;
bool power ;
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
uint32_t speedInterval = 100 ;
uint32_t shorCircuitInterval = 5 ;
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

    frontBtn.debounceInputs() ;
    rearBtn.debounceInputs() ;
    autoManualBtn.debounceInputs() ;
}

static void debounceInput()
{
    static uint8_t counter ;
    REPEAT_MS( 20 )
    {
        if( digitalRead( detectorPin ) == LOW )                               // if the detector only even smells a train, the detector state is considered true
        {
            detectorState = true  ;
            counter = 100 ;                                                      // a timeout will be set at 10 x 20 = 200ms
        }
        if( counter ) counter -- ;                                              // decrement this counter every 20ms
        if( counter == 0 ) detectorState = false ;                              // if the detector has not sensed a train for atleast 200ms, the detectorstate will be false

        frontBtn.debounceInputs() ;
        rearBtn.debounceInputs() ;
        autoManualBtn.debounceInputs() ;

    } END_REPEAT
    
    frontBtnState   = frontBtn.readInput() ;
    rearBtnState    = rearBtn.readInput() ;
    autoManualState = autoManualBtn.readInput() ;
}

void updateSpeed()
{
    REPEAT_MS( speedInterval )
    {
        if( speed < setPoint ) speed ++ ;
        if( speed > setPoint ) speed -- ;
        
        static uint8_t speedPrev ;

        if( speed != speedPrev)
        {   speedPrev = speed ;

            throttle.setSpeed( speed ) ;
            //Serial.println(speed);
            
        }
    } END_REPEAT
    
    throttle.update() ;
}

#define MAX_CURRENT 100                                                         // (1A, 0.5R shunt resistors gives 0.5V 0.5/5V -->  1023 / 10 = 120 ADC sample)
void shorCircuit()
{
    static uint8_t counter = 10 ;
    if( digitalRead( throttlePin ) == false ) return ;                          // if power is enabled and the throttle pin is in the OFF cycle , return 
                                                                                // we can only measure current if the throttle pin in in ON cycle

    REPEAT_MS( shorCircuitInterval ) ;                                          // take ADC sample every 5ms, 10x overcurrent -> short circuit

    int sample = analogRead( shortCircuitPin ) ;

    if( power )
    {
        shorCircuitInterval = 5 ;                                               // keep setting interval at 5ms

        if( sample >= MAX_CURRENT ) 
        {
            if( counter ) counter -- ;                                          // if overcurrent, keep decrementing counter
        }
        else
        {
            counter = 10 ;                                                      // no more overcurrent, keep setting counter at 10
        }
        if( counter == 0 )
        {
            shorCircuitInterval = 5000 ;                                        // cut off power for atleast 5 seconds
            power = false ;
            throttle.setState( 0 ) ;
        }
    }
    else
    {
        shorCircuitInterval = 5 ;                                               // enable power again, and set interval at 5ms again
        power = true ;                                                
        throttle.setState( 1 ) ;
    }

    END_REPEAT
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
            Serial.println(F("FRONT TRACK IS GO!!")) ;
            digitalWrite( relayPin, FRONT_SIDE ) ;  
            digitalWrite(  statusLedRear, LOW ) ;  // set relay and turn other led OFF
        }
        if( rearBtnState == RISING )
        {
            Serial.println(F("REAR TRACK IS GO!!")) ;
            digitalWrite( relayPin,  REAR_SIDE ) ;
            digitalWrite( statusLedFront, LOW ) ;
        }
        if( autoManualState  == LOW ) 
        {
            Serial.println(F("automatic mode enabled")) ;
            digitalWrite( relayPin, !digitalRead( relayPin ) ) ; // if driving in automatic mode, just toggle the relay and turn both LEDs OFF
            digitalWrite( statusLedFront, LOW ) ;
            digitalWrite( statusLedRear,  LOW ) ;
        }
        
        Serial.println(F(" setting points ")) ;
        if( digitalRead( relayPin ) == FRONT_SIDE )
        {
            leftPoint.setState( 1 ) ;
            leftPoint.setState( 0 ) ;
            digitalWrite( IN1, HIGH ) ;
            digitalWrite( IN2,  LOW ) ;
        }
        else
        {
            leftPoint.setState( 0 ) ;
            leftPoint.setState( 1 ) ;
            digitalWrite( IN1,  LOW ) ;
            digitalWrite( IN2, HIGH ) ;
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
        sample = map( sample, 0 ,1023, 5, 150 ) ; 
        speedInterval = sample ; 


        setPoint = 100  ;// map( sample, 0 ,1023, 20, 100 ) ;                            // speed is set between 20 - 100 %
                                                           // 50ms between speed increments -> 20 updates per second, max 5 second acceleration at top speed.
        sm.setTimeout( 1000 ) ;                                                  // wait 1s before monitoring detector
        Serial.println(F("points are set\r\ntrain departing")) ;
    }
    if( sm.onState() )
    {
        if( detectorState == false && sm.timeout() ) sm.exit() ;                // if no current is sensed AND 1s has passed the train is on the main track
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
        Serial.println(F("waiting for train to reach the station again")) ;
    }
    if( sm.onState() )
    {
        if( detectorState == true ) sm.exit() ; // just wait on current sense detector...
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
        int sample = analogRead( speedPin );
        // if( digitalRead( relayPin ) == FRONT_SIDE ) sample = analogRead( frontBrakeSpeed ) ; 
        // else                                        sample = analogRead(  rearBrakeSpeed ) ;

        //speedInterval = 50;//map( sample, 0 ,1023, 1, 50 ) ;                         // determen brakespeed, depended of relay state
        sample = map( sample, 0 ,1023, 20, 200 ) ; 
        speedInterval = sample ; 
        
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
    // shorCircuit() ;
    updateSpeed() ;
    // digitalWrite( throttlePin, HIGH ) ;
    
    leftPoint.sweep() ;                                                         // sweep the servo motors, if set
    rightPoint.sweep() ;
        
    //if( detectorState == false ) digitalWrite( statusLedRear, LOW ) ;          // delete me, for visual feedback of the detector state.
    //if( detectorState ==  true ) digitalWrite( statusLedRear, HIGH ) ;

    if( frontBtnState == LOW && rearBtnState == LOW )                           // if both these buttons are pressed, reset the state machine
    {
        sm.nextState( readButtons, 5000 ) ;
    }

    STATE_MACHINE_BEGIN

    State(readButtons) {
        sm.nextState( accelerateTrain, 300 ) ; }                                // short delay to set the points

    State(accelerateTrain) {
        sm.nextState( waitArrival, 1000 ) ; }                                   // 1 second delay to ensure the sensor won't bounce or something

    State(waitArrival) {
        sm.nextState( slowDownTrain, 0 ) ; }

    State(slowDownTrain) {
        sm.nextState( readButtons, 5000 ) ; }                                   // wait 5 seconds before another train is allowed to depart

    STATE_MACHINE_END
}
