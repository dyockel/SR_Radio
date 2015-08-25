/*

  Flock message demo -- test SENDER.

  a simple test kludge, this reads a sensor (possibly) attached
  to Analog Input A0 and once a second transmits that to the
  Flock message test RECEIVER.
  
  tom jennings

  03 jul 2015  simplified
  25 jun 2015  new

*/

#include <SPI.h>
#include <SRResources.h>
#include <SRRadio.h>

SRFlock Flock;
const int LEDpin = 2;                               // optional radio-activity LED 
const char TXID = 'S';                              // our device ID
unsigned long T;                                    // ersatz timer

void setup() {

  Serial.begin(9600);
  Serial.println ("Flock demo test SENDER");

  randomSeed (analogRead (A0) ^ micros());          // Flock uses random()
  Flock.begin ();                                   // instantiate, default settings
  Flock.setIdentity (TXID);                         // our radio ID
  Flock.LED (LEDpin);                               // use this LED pin
}

void loop() {

  Flock.message();                                  // run the state machine
  switch (Flock.status()) {                         // reports protocol status (optional)
    case 1: Serial.print (F("connected channel ")); Serial.println (Flock.getChannel()); break;
    case 2: Serial.println (F("disconnected")); break;
  }

  if (millis() > T) {                               // when it's time
    T= millis() + 1000;                             // next event in 1000 mS
    Flock.messageBegin();                           // start a message
    Flock.messageTo ('@', true);                    // to Base
    Flock.messageAdd ('V', analogRead (A0));        // add sensor, command V
    Flock.messageAdd ('G', millis() / 1000);        // approx uptime, seconds; command G
    Flock.write();                                  // send the message
//    Serial.print ("sent: "); Serial.println (Flock.getTxBuff());
  }
}


