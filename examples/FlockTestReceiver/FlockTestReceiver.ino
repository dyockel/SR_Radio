/*

  Flock message demo -- test RECEIVER.

  this receives Flock messages from the test SENDER 
  and decomposes them. the resulting values are printed
  in the serial monitor.

  demonstrates the dispatcher() model.
    
  tom jennings

  08 aug 2015  conforms to API
  03 jul 2015  simplified
  25 jun 2015  new

*/

#include <SPI.h>
#include <SRResources.h>
#include <SRRadio.h>
#include <SRLoopTally.h>

const int LEDpin = 2;                           // Flock blinks this, if defined
SRFlock Flock;

// optional loop-tally function.
// SRLoopTally SRL;

void setup() {

  Serial.begin (9600);
  Serial.println ("#Flock test receiver");

  Flock.begin ();
  Flock.setIdentity ('@');                      // all your base, etc
  Flock.LED (LEDpin);                           // assign an LED to the radio
  Flock.addDispatcher (&Dispatcher);            // decomposed messages go here (below)
// SRL.begin (10);                                // report loop stats every 10 sec
}

void loop () {
int n;

  Flock.message();                              // Flock protocol state machine
  switch (Flock.status()) {                     // optional status info
    case 1: Serial.print (F("connected channel ")); Serial.println (Flock.getChannel()); break;
    case 2: Serial.println (F("disconnected")); break;
  }
//  SRL.tally();                                // print out loop stats
}

boolean Dispatcher (int c, unsigned nnn) {

  switch (c) {
    case 'V': Serial.print ("remote sensor = "); Serial.println (nnn); break;
    case 'G': Serial.print ("remote seconds = "); Serial.println (nnn); break;
    default: Serial.print ("unknown: "); Serial.print (nnn); 
             Serial.write (c); Serial.println (""); break;
  }
  return true;                                  // see API for details
}

