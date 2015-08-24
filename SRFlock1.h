/*

 flock protocol, part 1
 
 v5 added Peep, and added transmit queue to write(). 
    
 v4 emphasis on messaging. added packet() and message()
    methods, API changes.

 v4 unified machine (base vs. bird), switched on
    identity. the machine is now called packet(),
    to accomodate the message() layer.

 v3 new packet model; bad-channel stats and
    dynamic channel mapping.

 v2 uses separate rx, tx buffers, declared in userland.

 v1 discovers fixed base channel via
    SCISSORS protocol subset.

  tom jennings
  
 TODO:

tx queue behavior is wrong. currently writes (drains) all queued
payloads when any bird is heard from; message for X is drained
if we hear from bird Y.

does this work:

queue no longer a queue, just a list. messages for the same
bird may not deliver in order.

need a per-bird ACK list that tells how long since we've heard from bird.
ackBird()/nakBird() may not work as-is.

at write:

  if heard from bird recently, RF delivery.
  if not heard from bird, queue the message.

at dequeue:
  if heard from bird recently, RF delivery from queue, delete.

dequeueing is then simply clearing [0] eg. empty slot.

supports per-bird queue check.



 have Peep try (N times?) the last-used channel before hunting.


  24 aug 2015	added get*() for peep and ack stuff.
  11 aug 2015	stored a 0 after the last byte in received packet
  		for base (state 4) -- usage has been steadily towards
		string handling, and how thi sworked this long without
		it i will never know. likely callers did it. harmless
		for binary packets as the null is at length+1
		and rxBuff is not PACKETSIZE + 1.
  08 aug 2015	Peep working; moved transmit queue to write()
  		completing missing functions. PROBLEM: the sequence
		number is appended in write which is fine for Peep
		and Flock but bad for packet(). added setBaseID().
  07 aug 2015  	beginning Peep dev. added powerUp() method
  		that powers the radio up and restarts
		the Flock state machine.
  07 aug 2015  message() returns sender ID (char) not packet length.
  04 aug 2015  shortened LED on times.
  25 jul 2015  dispatchers are now passed (int, unsigned) to 
               accommodate checksum errors (checksums not used
	       in radio.)
  16 jul 2015  now using SRPRNG exclusively.
  13 jul 2015  added getIdentity()
  04 jul 2015  added passthrough for powerDown()
  03 jul 2015  c/Parser/Dispatcher/ to match change in SRMessage.
  29 jun 2015  Flock read and write now assumes null-terminated strings.
  25 jun 2015  unified the two models (base, bird) into one (machine).
               now contains packet() and message() methods. many
	       changes to API.
               commented out stray debug print in bad channel.
               valid packet receipt addressed to us acked, even
	       if not from base (previously, only from base).
  06 jun 2015  const int def's generate compiler complain?
               changed to #defines
  17 may 2015  badChan[] table too small! moved xor() stuff
               into SRResources.
  10 may 2015  added setPromiscuous(). nasty bug -- ackPacket()
               writes in pxBuff, called before copyN.
  05 may 2015  implemented CRC-base duplicate detection, and 
               simple double-write in write() to increase delivery
	       probability. the latter needs to be true retry for
	       writing packets requesting ack but the dup detection
	       should be run for a while first.
	       added minChannel, maxChannel settable limits; the
               SMA connectored radios don't seem to do low
	       channels well. increased LED on times.
  03 may 2015  updates for changed LED api. split RX timeout
               methods in two: RxAR (ack request) and RxAT (ack
	       timeout) to be able to do multiple requests
	       within timeout.
  26 apr 2015  rxpower works now. improved rx timeout
               methods. added connect() method and logic.
	       CRC dup idea is fail.
  25 apr 2015  eep! radio.setChannel() wasn't called at
               channel changes! stuck on 80! shortened
	       inter-channel seek delay. changed flockToBaseState()
	       to return channel number. channel 126
	       does not work! lower MAX
  23 apr 2015  added setChannel() and setRxTO(). added
               channel quality and new channel selection
	       logic for BaseToFLock().
  21 apr 2015  resets RXTIMER, RXTOTIMER if we see any
               packet from @ in getPacket(), as a way
	       to avoid requesting keepalive.
  18 apr 2015  v3 protocol seems to be working, is a hell
               of a lot simpler and smaller. added radio 
	       pass-through methods.
  17 apr 2015  v3; vastly simplified protocol.
  15 apr 2015  v2; separate rx, tx buffers, in userland,
               passed in begin(); separate, internal
	       protocol packet buffer. this vastly simplifies
	       data handling. this does require copying
	       payload in to/out of buffers, which
	       could be eliminated with split read() write()
	       as contemplated.
  14 apr 2015  removed nested ifs in flockToBase(),
               replaced with getPacket(). aargh, missing 
	       parens around mask AND.
  10 apr 2015  added LED stuff, fixed flockToBaseState()
  18 mar 2015  new

*/


#ifndef __SRFLOCK1_H__
#define __SRFLOCK1_H__

#if ARDUINO < 100
#include <WProgram.h>
#else
#include <Arduino.h>
#endif

#include <stddef.h>

#ifdef ARDUINO
#else
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint16_t prog_uint16_t;
#endif

const uint8_t MINCHANNEL = 1;			// Nordic chip hardware limits
const uint8_t MAXCHANNEL = 125;
const uint8_t PACKETSIZE = 32;

// our customary radio pins.
//
const int NRFCEpin = 9;
const int NRFCSNpin = 10;


class SRFlock {

// timers.
//
#define __FLOCKRADIOTIMER 0
#define __FLOCK_RXTIMER 1
#define __FLOCK_RXTOTIMER  2
#define __FLOCK_PEEPTIMER  3
#define __FLOCK_LOOPTIMER  4
#define __FLOCK_NUMTIMERS  5

// LED blink times
//
#define __FLOCK_LEDBLIP  5
#define __FLOCK_LEDBRIEF 10
#define __FLOCK_LEDLONG (600)

#define __FLOCK_MAXRXTO  60

// packet types
//
#define __FLOCK_ACK '!'
#define __FLOCK_HINT '-'
#define __FLOCK_BYE '#'


// transmit queue buffer depth
//
#define __FLOCK_TXQDEPTH 4


public:
	int begin ();
	int begin (uint8_t cepin, uint8_t cspin);
	void setBaseID (char id);
	void setIdentity (char id);
	char getIdentity ();
	void LED (uint8_t pin);
	int packet();
	char peep();
	char message();
	bool connected();
	int status();
	int write ();
	char * getRxBuff();
	char * getTxBuff();


// connection management
//
	void setRxAR (uint8_t sec);
	void setRxAT (uint8_t sec);
	void setAckThresh (uint8_t t);
	void setPromiscuous(bool yup);
	void dynamicChannelEnable();
	void dynamicChannelDisable();
	void nextChannel();
	uint8_t getAckBalance();
	uint8_t * getChanErrorMap ();
	uint8_t getAckThresh();
	int txQueueCount ();
	int getTxQueueDepth ();
	void powerDown ();
	void powerUp ();
	bool powerState();
	void setPeepConnectTime (unsigned n);
	void setPeepUpTime (unsigned n);
	unsigned txMessages();
	unsigned rxMessages();
	void setFlockLoopRate (unsigned n);
	int getRxAR();
	int getRxAT();
	int getPeepConnectTime();
	int getPeepUpTime();

// radio pass-through functions
//
	void setFixedAddress (uint64_t t, uint64_t r);
	void setSelfAddress (uint64_t base, uint8_t t, uint8_t r);
	void setSelfAddress (uint8_t t, uint8_t r);

	void setChannel (uint8_t channel);
	void setMinChannel (uint8_t channel);
	void setMaxChannel (uint8_t channel);
	void setDataRate (uint8_t n);
	void setPALevel (uint8_t n);

	uint8_t getChannel();
	uint8_t getRxPower();
	uint8_t readRegister (uint8_t r);

// Message parser methods, a mix of custom/local and
// passthrough.
//
	void addDispatcher (bool (* func) (int, unsigned));
	int messageBegin ();
	int messageTo (char dest, bool ack);
	int messageAdd (char c, unsigned nnn);
	int messageEnd();

protected:
	void blink (uint8_t n);
	void protoPacket (char dest, char req);
	int getPacket (bool broadcast, bool promisc);
	void copyN (char * d, char * s, uint8_t n);
	uint8_t newChan (uint8_t badChan);
	uint8_t dupCheck (uint8_t r);
	uint8_t crc8 (uint8_t crc, uint8_t data);
	uint8_t crc8buff (char * p, uint8_t n);

	void txDequeue ();
	int packetWrite (uint8_t n);
	int packetWrite (int q, uint8_t n);
	bool heardID (char id);
	void ackBird (char id);
	void nakBird (char id);
	int birdToBit (char id);

private:
	SRRF24 radio;				// radio chip driver
	SRTimer T;				// software timers
	SRCRC8 CRC;				// CRC for data packets
	SRPRNG PRNG;				// minimal PRNG
	SRLED RFLED;				// our RF activity status LED
	SRMessage MSG;				// messaging system
	uint8_t LEDpin;				// optional

	uint8_t cepin;
	uint8_t csnpin;
	byte flockRate;				// how fast Flock runs
	char identity;				// our identity (A..Z)
	char baseID;				// ID of the base (usually @)
	uint8_t state;				// protocol state machine
	uint8_t channel;			// current radio channel
	uint8_t minChannel;			// the SMA radios lousy at
	uint8_t maxChannel;			// low channels...
	uint8_t cstate;				// connect-state report state
	uint8_t pstate;				// power state report state

	char txQueue [__FLOCK_TXQDEPTH] [PACKETSIZE + 1];// TX ring buffer
	int in, out;				// input and output pointers

	char pxBuff[PACKETSIZE + 1];		// protocol packet buffer
	char rxBuff[PACKETSIZE + 1];		// receive packet buffer
	bool connectState;			// true if connected
	bool promisc;				// true if receive any packet
	bool poweroff;				// true if chip/protocol off
	unsigned rxTimeout;			// rx timeout time, decisec
	uint8_t rxPower;			// receive power indication
	bool dynamicChannelMapping;		// enables channel mapping
	uint8_t ackBalance;			// outstanding ack count
	uint8_t ackThresh;			// outstanding ack limit
	uint8_t birdAck[7];			// birds A..Za..z recent ack
	uint8_t badChan[16];			// bad channels, 0..127, bitmap
	uint8_t prevCRC;			// CRC8 of previous payload 
	char seqNumber;				// packet sequence character
	unsigned peepConnectTime;		// allowed time to connect
	unsigned peepUpTime;			// stay up after write()
	unsigned txCount, rxCount;		// packets sent/received
};


/*
  begin/setup Flock protocol.
*/

// use the default pins for the radio and allocate buffers internally.
//
int SRFlock::begin() {

	return begin (NRFCEpin, NRFCSNpin);
}

// allocate buffers internally.
//
int SRFlock::begin (uint8_t cepin, uint8_t csnpin) {

	in= out= 0;
	T.begin (__FLOCK_NUMTIMERS);			// init timers
	MSG.begin();					// ready message parser
	CRC.begin();					// probably does nothing
	PRNG.begin();					// local PRNGs
	LEDpin= 0;					// see LED() for begin()

	setFlockLoopRate (3);				// flock loop time
	state= 0;					// our state machine
	connectState= false;				// no Flock connect
	promisc= false;					// no promiscuous RX
	poweroff= false;				// chip is ready
	dynamicChannelMapping= true;			// on by default
	baseID= '@';					// default base ID

	peepConnectTime= 20000;				// 20 sec Peep connect
	peepUpTime= 877;				// stay up after write()

	int r= radio.begin (cepin, csnpin); 		// set up radio hardware
	radio.setAutoAck (false);			// make sure this is off
	setPALevel (3);					// highest power 
	PRNG.xor16Seed (micros());			// seed the 16 PRNG
	channel= newChan (0);				// for baseToFlock()
	radio.setChannel (channel);
	ackThresh= 10;					// outstanding ack limit
	minChannel= 20;					// set default range
	maxChannel= 120;
	prevCRC= 0;
	seqNumber= '0';					// first packet sequence
	setRxAR (6);					// ack request interval
	setRxAT (31);					// ack req fail interval
	cstate= pstate= 0;				// 
	return r;
}

// define the base ID character, default is already '@'.
//
void SRFlock::setBaseID (char id) {

	baseID= id;
}

// set our identity and protocol.  protocol==Bird 
// unless ID is baseID which means base. 
//
void SRFlock::setIdentity (char id) { 

	identity= id;
	state= (id == baseID ? 4 : 0);
}

// set the loop task time for Flock, Peep and Packet.
//
void SRFlock::setFlockLoopRate (unsigned n) {

	if (n > 100) n= 100;
	T.setTimer (__FLOCK_LOOPTIMER, n);
}

// power management methods used mainly by Peep protocol. 
// after powerDown() is invoked, only powerUp(), powerState(), 
// connectState() are allowed.
//

// power on the radio and reset Flock and Peep protocols.
//
void SRFlock::powerUp () { 

	connectState= false;				// not connected yet
	pstate= cstate= 0;
// changing indices here fouls up messages already
// placed in the queue before power is turned on!
// in= out= 0;					// TX queue cleared
	rxCount= txCount= 0;
	radio.available(); 				// ready the chip
	setIdentity (identity);				// resets state machine
	poweroff= false;
}

// power down the chip and mark everything as disconnected. this specifically
// does not reset the TX queue so that it can be examined by txQueueCount().
//
void SRFlock::powerDown () { 	
	
	connectState= false;
	setIdentity (identity);
	radio.powerDown(); 
	poweroff= true;
}

bool SRFlock::powerState () {

	return ! poweroff;
}

/* 
  Flock message protocol. message() accepts message+address+ack request
  via packet(), strips the header and delivers any messages via the
  dispatcher(s).

  the returned byte count is for the raw payload, in rxBuff. it is
  possible for there to be no messages with a non-zero return, eg.
  ack/ack request packets.

  as a side effect the dispatcher gets the optional protocol
  byte, ! or -, as the first character in the message string.

 */

char SRFlock::message () {
int r;

	r= packet();
	if (r == 0) return '\0';			// no packet

	rxBuff [r]= '\0';				// null terminate it
	MSG.rxString (rxBuff + 2);			// dispatch messages
	return rxBuff[1];				// return sender
}

/* this is the Flock packet machine, which handles both flock-to-base and
base-to-flock. the state variable is initialized by setIdentity -- if @ then
this runs baseToFlock and the state never changes.  the Flock-to-base state
machine changes state accordingly.

  0 no data received. this means either no packet at all, a protocol packet
  (eaten by the state machine)

  1..32 a packet with valid payload received and is now in buff[]. note that
  the first two bytes of the packet are protocol overhead.

  -1 read error, which really means code, driver, or hardware error.
*/

int SRFlock::packet () {

	if (LEDpin) RFLED.LED();			// run LEDs 
	if (! T.timer (__FLOCK_LOOPTIMER)) return 0;	// 

	txDequeue();					// send queued packets
	uint8_t r= 0;
	switch (state) {

// FLOCK TO BASE -------------------------------------------------------------
//
		// search for the operating channel by pinging random
		// channels looking for a response from Base.
		//
		case 0:	
			nextChannel();			// select a new channel
			connectState= false;		// not connected
			state= 1;

		// blindly transmit an ack request, setup to wait
		// for a response.
		//
		case 1:
			protoPacket (baseID, __FLOCK_ACK);// request ack
			T.setTimer (__FLOCKRADIOTIMER, 5);
			state= 2;			// await response
			break;

		// request sent; watch for response (connected on
		// current channel) or timeout (try next channel).
		//
		case 2:	
			if (getPacket (false, false)) {	// if a packet for us,
				unsigned x= __FLOCK_LEDLONG;
				blink (x);
				state= 3;		// we're in sync
				connectState= true;	// we are connected

			} else if (T.timer (__FLOCKRADIOTIMER)) {
				state= 0;
			}
			break;

		// presumably "connected" which simply means we've communicated
		// with the base within the alloted time. poll for incoming
		// packets, respond to ack requests and channel-change hints,
		// deal with timeout, deliver payload to userland.
		//
		case 3:	
			r= getPacket (true, promisc);	// what we got?
			r= dupCheck (r);		// check if duplicate

			// if a valid packet, deliver any payload. watch
			// for ack request and channel-change hint.
			//
			if (r) {
				++rxCount;
				copyN (rxBuff, pxBuff, r);// deliver payload
				if (pxBuff[0] == identity) {
					if (pxBuff[2] == '!') 
						protoPacket (pxBuff[1], 0);
					if (pxBuff[2] == '-') {
						state= 0;
					}
				}
			}

			// if we go long enough without seeing a packet,
			// request ack from the base. this timer is shorter
			// than RXTOTIMER.
			//
			if (rxTimeout && T.timer (__FLOCK_RXTIMER)) 
				protoPacket (baseID, __FLOCK_ACK);

			// if we never hear a response, restart protocol.
			// this timer is longer than RXTIMER.
			//
			if (rxTimeout && T.timer (__FLOCK_RXTOTIMER)) {
				state= 0;
			}
			break;

// BASE TO FLOCK -------------------------------------------------------------
//

		// Base protocol all resides in one state. incoming
		// packets get delivered and ack requests honored.
		// "connect state" is loosely defined as pack success
		// within the specified timeouts, and the number of
		// outstanding ack requests below threshhold.
		// 
		case 4:
			r= radio.available();		// ready/sample RX power
			if (r) {			// 0 else RX power
				rxPower= r;
				r= radio.read (pxBuff, PACKETSIZE);
			}

			// no-packet-received is when we check for timeouts
			// and ack request/reply imbalance. if we reach a
			// timeout, send a broadcast ack request. 
			// if we accumulate too many outstanding requests, 
			// issue a channel-change hint and change
			// channels, forcing birds to re-search for the Base.
			//
			if (r < 2) {
				// if we haven't seen a packet for a while,
				// broadcast an ack request.
				//
				if (rxTimeout && T.timer (__FLOCK_RXTIMER)) 
					protoPacket ('*', __FLOCK_ACK);

				// check ack balance.
				//
				if (ackBalance > ackThresh) { 
					protoPacket ('*', __FLOCK_HINT);
					protoPacket ('*', __FLOCK_HINT);
					nextChannel();
					connectState= false;
					T.trigTimer (__FLOCK_RXTIMER);
				}
				r= 0;			// setup return
				break;
			}

			// brief blip upon receipt of any valid packet.
			//
			blink (__FLOCK_LEDBLIP);

			// if the packet is for us, however, longer blink.
			// if not for us, we're done here.
			//
			if (pxBuff[0] != identity) {
				r= 0;
				break;
			}
			blink (__FLOCK_LEDBRIEF);

			// ack balance counts only packets addressed to us
			// directly since that's how they are sent.
			//
			if (ackBalance) --ackBalance;	

			// good packet, possibly contains payload. deliver
			// payload if present and not a dup. null-terminate
			// it in case it is a string!
			//
			connectState= true;	
			r= dupCheck (r);
			copyN (rxBuff, pxBuff, r);
			rxBuff[r]= 0;

			// honor ack requests.
			//
			if (r > 2 && pxBuff[2] == '!') {
				protoPacket (pxBuff[1], 0);
			}
			break;
	}
	return r;
}

// run the Peep protocol, which is mainly Flock with radio power
// control that is write-driven.
//
char SRFlock::peep() {

	if (powerState() == 0) return 0;	// powered off

	char c= message();			// run the protocol
	if (T.timer (__FLOCK_PEEPTIMER)) {	// power off time has arrived
		powerDown();
	}
	return c;
}


// prepare a message for transmission. the message was
// built in txBuff() which is the end (input) of the queue.
// the Flock sequence character is appended (dup packet
// detection at the receiver).
//
// if the radio is not powered on or not connected, the
// packet remains queued for later transmission. 
//
// if the radio is off, it is powered on and readied.
//
// (the userland caller must check that there is room in 
// the queue; not doing so merrily overwrites the oldest
// unsent message, not an entirely unreasonable thing to do;
// but if doing so advances the in pointer past the out
// pointer re-sends of previously-sent packets will occur.
// all this can be avoided by checking queue depth before
// building a new packet getTxBuff().)
//
int SRFlock::write () {

	// append the sequence character to the message payload.
	//
 	int n= strlen (txQueue [in]);
	if (n < PACKETSIZE - 1) {		// if there is room
		txQueue [in] [n++]= seqNumber;	// append sequence num
		if (++seqNumber > '9') seqNumber= '0';
	}

//	Serial.print ("write ["); Serial.print (in);
//	Serial.print ("] = ");Serial.println (txQueue[in]);

	// if the radio is currently off, turn it on and leave this message
	// queued for later transmission (since it can't happen now)
	// and set Peep's connect-time timer (Flock ignores it).
	//
	if (powerState() == 0) {		// if currently off,
		powerUp();			// enable radio
		T.setTimer (__FLOCK_PEEPTIMER, peepConnectTime); 
		if (++in >= __FLOCK_TXQDEPTH) in= 0;	// enqueue this packet,

//		Serial.print ("write ["); Serial.print (in);
//		Serial.print ("] = ");Serial.println ("powerup queued");

		return 2;
	}

	// radio is up and Peep/Flock connected. write the packet
	// directly. if this succeeds we're done; if it does not
	// successfully send queue it up for later retry. Flock
	// ignores PEEPTIMER.
	//
	if (connected() && txQueueCount() == 0) {
		if (packetWrite (in, n) > 0) {
//	Serial.print ("write ["); Serial.print (in);
//	Serial.print ("] = ");Serial.println ("direct write success");

			T.setTimer (__FLOCK_PEEPTIMER, peepUpTime);
			return 1;		// success
		}
	}
	if (++in >= __FLOCK_TXQDEPTH) in= 0;	// enqueue this packet,
//	Serial.print ("write ["); Serial.print (in);
//	Serial.print ("] = ");Serial.println ("direct write fail queued");

	return 2;				// tx fail, leave queued
}

// if there is a message in the queue transmit it. if
// transmission succeeds, dequeue the message.
//
// NOTE: this needs to check each message dest ID for 
// "heard from that ID", then remove the one from packet write.
// or do it in packet write.
//
void SRFlock::txDequeue () {

	if (in == out) return;			// queue is empty, done

	int n= strlen (txQueue [out]);		// payload length
	if (packetWrite (out, n)) {		// if success,
		if (++out >= __FLOCK_TXQDEPTH) out= 0;
		T.setTimer (__FLOCK_PEEPTIMER, peepUpTime);

//		Serial.print ("dequeue ["); Serial.print (out);
//		Serial.print ("] = "); Serial.print (txQueue[out]); 
//		Serial.println ( " success");
	}
}

// return the number of items in the queue.
//
int SRFlock::txQueueCount () {

	int a= in - out; if (a < 0) a += __FLOCK_TXQDEPTH;
	return a;
}

// return a pointer to the current TX buffer, eg. the oldest queue slot.
//
char * SRFlock::getTxBuff() {
	
	return txQueue[in];
};


// write bytes from the buffer to the radio only
// if we have a connect.
//
int SRFlock::packetWrite (int q, uint8_t n) {

	PRNG.xor16();				// disrupt sequence
	if (! connectState) return 0;		// can't send now

	blink (__FLOCK_LEDBRIEF);
	if (n > 2 && txQueue [q] [2] == '!') {
		++ackBalance;			// require acknowledge
	}
//	Serial.print ("packet write "); 
//	Serial.print (n); Serial.print (" " );
//	Serial.println (txQueue[q]);

	int r= radio.write (txQueue [q], n);
	if (r > 0) ++txCount;
	return r;

}


// returns Flock connection status change. connected() returns current
// state; this tracks changes. the code is a bit of a crock.
//
// 0:    was, is, disconnected
// 1:    was discon, now connected
// 2:    was connected, now disconnected
// 3:    was, is, connected
// 4:    radio was just powered up
// 5:    radio was just powered down
//
//
int SRFlock::status () {

	// calc power state
	//
	pstate <<= 1;
	if (! poweroff) pstate |= 1;
	pstate &= 3;

	// calculate connect state
	//
	cstate <<= 1;					// previous state 
	if (connectState) cstate |= 1;			// new state 
	cstate &= 3;					// this and last, only

	// changes in power state override connect state.
	switch (pstate) {
		case 1: return 4; break;
		case 2: return 5; break;
		default: return cstate; break;
	}
	return cstate;
}

#endif

