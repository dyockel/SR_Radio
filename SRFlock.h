/*

 arduino side flock to base protocol
 
 v3 new packet model; bad-channel stats and
    dynamic channel mapping.

 v2 uses separate rx, tx buffers, declared in userland.

 v1 discovers fixed base channel via
    SCISSORS protocol subset.

  tom jennings
  
 TODO:


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


#ifndef __SRFLOCK_H__
#define __SRFLOCK_H__

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

const uint8_t MINCHANNEL = 1;			// Nordic chip limits
const uint8_t MAXCHANNEL = 125;
const uint8_t PACKETSIZE = 32;

class SRFlock {

private:
	SRRF24 radio;
	SRTimer T;
	SRCRC8 CRC;
	SRLED RFLED;				// our RF activity status LED
	uint8_t LEDpin;				// optional

	uint8_t cepin;
	uint8_t csnpin;
	uint8_t identity;			// our identity (A..Z)
	uint8_t state;				// protocol state machine
	uint8_t channel;			// current radio channel
	uint8_t minChannel;			// the SMA radios lousy at
	uint8_t maxChannel;			// low channels...
	uint8_t * txBuff;			// tx buffer, in userland 
	uint8_t * rxBuff;			// rx buffer, in userland 
	uint8_t pxBuff[PACKETSIZE];		// protocol packet buffer
	bool connectState;			// true if connected
	bool promisc;				// true if receive any packet
	unsigned rxTimeout;			// rx timeout time, decisec
	uint8_t rxPower;			// receive power indication
	bool dynamicChannelMapping;		// enables channel mapping
	uint8_t ackBalance;			// channel error logic
	uint8_t chanThresh;			//
	uint8_t badChan[16];			// bad channels, 0..127, bitmap
	uint8_t prevCRC;			// CRC8 of previous payload 
	char seqNumber;				// transmit packet sequence

// timer stuff.
//
#define __FLOCKRADIOTIMER 0
#define __FLOCK_RXTIMER 1
#define __FLOCK_RXTOTIMER  2
#define __FLOCK_NUMTIMERS  3
#define __FLOCK_LEDBLIP  40
#define __FLOCK_LEDBRIEF 80
#define __FLOCK_LEDLONG (700)
#define __FLOCK_MAXRXTO  60

protected:
	void blink (uint8_t n);
	void ackPacket (char id, bool req);
	int getPacket (bool broadcast, bool promisc);
	void copyN (uint8_t * d, uint8_t * s, uint8_t n);
	uint8_t newChan (uint8_t badChan);
	uint8_t dupCheck (uint8_t r);
	uint8_t crc8 (uint8_t crc, uint8_t data);
	uint8_t crc8buff (uint8_t * p, uint8_t n);

public:
	int begin (uint8_t cepin, uint8_t cspin, uint8_t * rxb, uint8_t * txb);
	void LED (uint8_t pin);
	int baseToFlock ();
	int flockToBase ();
	int flockToBaseState ();
	int write (uint8_t n);
	void setIdentity (char id);

	void setFixedAddress (uint64_t t, uint64_t r);
	void setSelfAddress (uint64_t base, uint8_t t, uint8_t r);
	void setSelfAddress (uint8_t t, uint8_t r);

	void setChannel (uint8_t channel);
	void setMinChannel (uint8_t channel);
	void setMaxChannel (uint8_t channel);
	void setDataRate (uint8_t n);
	void setPALevel (uint8_t n);
	void setRxAR (uint8_t sec);
	void setRxAT (uint8_t sec);
	void setChanThresh (uint8_t t);
	void setPromiscuous(bool yup);

	bool connected();
	uint8_t getChannel();
	uint8_t getRxPower();
	uint8_t readRegister (uint8_t r);

	bool messageDispatcher (char c, unsigned nnn);

	// base to flock only
	//
	void dynamicChannelEnable();
	void dynamicChannelDisable();
	void nextChannel();
	uint8_t getChanError();
	uint8_t * getChanErrorMap ();
	uint8_t getChanThresh();
};

/*
  arduino side flock to base protocol
*/

int SRFlock::begin (uint8_t _ce_pin, uint8_t _csn_pin, 	// radio chip pins
		uint8_t * rxb, uint8_t * txb) {		// RX, TX buffers

	state= 0;
	LEDpin= 0;					// no LED configured yet

	rxBuff= rxb;					// pointers to buffers
	txBuff= txb;
	connectState= false;
	promisc= false;

	uint8_t r= radio.begin (_ce_pin, _csn_pin); 
	radio.setAutoAck (false);
	radio.setPALevel (3);

	chanThresh= 10;					// outstanding ack limit
	minChannel= MINCHANNEL;				// set default range
	maxChannel= MAXCHANNEL;
	channel= random (minChannel, maxChannel + 1);	// setup baseToFlock()
	radio.setChannel (channel);
	dynamicChannelMapping= true;			// on by default
	prevCRC= 0;
	seqNumber= '0';					// first packet sequence

	T.begin (__FLOCK_NUMTIMERS);
	setRxAR (6);					// ack request interval
	setRxAT (31);					// ack req fail interval

	return r;
}

// setup to use an LED for operational status.
//
void SRFlock::LED (uint8_t n) { RFLED.begin (LEDpin= n); }

// if configured, blink the LED for N mS.
//
void SRFlock::blink (uint8_t n) { if (LEDpin) RFLED.blink (n); }


// establish a connection if there is not one (basically, discover
// the channel the Base is listening to). returns 0 when idle
// or negotiating channel, else if a positive value, the number
// of data bytes from the base currently stored in buff.
//
int SRFlock::flockToBase () {

	uint8_t r= 0;
	if (LEDpin) RFLED.LED();			// run LEDs 

	switch (state) {
		// initial setup, or try next channel. 
		//
		case 0:	
			channel= newChan (0);		// select a new channel
			radio.setChannel (channel);
			connectState= false;		// not connected
			state= 1;

		// transmit packets on the selected channel,
		// requesting acknowledgement. 
		//
		case 1:
			ackPacket ('@', true);		// request ack
			T.setTimer (__FLOCKRADIOTIMER, 5);
			state= 2;			// await response
			break;

		// await a response to our request. if it times out
		// pick another channel and try again.
		//
		case 2:	
			if (getPacket (false, false)) {	// if a packet for us,
				unsigned x= __FLOCK_LEDLONG;
				blink (x);		// why can't i cast that #def here?!
				state= 3;		// we're in sync
				connectState= true;	// we are connected

			} else if (T.timer (__FLOCKRADIOTIMER)) state= 0;
			break;

		// look for incoming packets, copy payload to the caller
		// (flagged by the return value, r) and honor any ack
		// request.
		// we have two RX timers; one for "time to request ack"
		// and the other means no packet received; time to renegotiate
		// the channel.
		//
		case 3:	
			r= getPacket (true, promisc);	
			r= dupCheck (r);		// check if duplicate
			if (r) {
				copyN (rxBuff, pxBuff, r);// deliver payload
				if ((pxBuff[0] == identity) &&
				    (pxBuff[1] == '@') &&
				  (pxBuff[2] == '!')) {
					ackPacket (pxBuff[1], false);
				}
			}

			// if quiet too long request ack from the base
			// to ensure that the channel is alive. getPacket()
			// resets the timers if a valid packet is received.
			//
			if (rxTimeout && T.timer (__FLOCK_RXTIMER)) 
				ackPacket ('@', true);
			if (rxTimeout && T.timer (__FLOCK_RXTOTIMER)) 
				state= 0;
			break;
	}
	return r;
}

// send an empty packet as acknowledgement (req == false) or 
// include a single ! as payload (req == true). we up the ackBalance
// counter whenever we request an acknowledge; this is decremented
// upon receipt of any packet from Base.
//
void SRFlock::ackPacket (char dest, bool req) {

	pxBuff[0]= dest;				// set destination
	pxBuff[1]= identity;				// source: us
	uint8_t n= 2;					// packet length
	if (req) {
		pxBuff[n++]= '!';			// optional ack request
		++ackBalance;				// response required
	}
	radio.write (pxBuff, n);			// send packet
	blink (__FLOCK_LEDBLIP);			// blink LED
}

// look for a packet for us. reads any packet available, and checks
// for address and general validity. 
//
// the caller will check if acknowledgement needs to be sent, but 
// to simplify that test, if the packet is short we zero the first
// payload character so the caller doesn't need to check packet length.
//
// returns 0 is not a valid packet or not for us, else it returns
// the packet length. hence, it returns 0, or 2..32.
//
int SRFlock::getPacket (bool broadcast, bool promisc) {

	int r= radio.available();			// ready/sample RX power
	if (r) {					// 0 else RX power
		rxPower= r;
		r= radio.read (pxBuff, PACKETSIZE);	// read available packet
	}
	if (r < 2) return 0;				// invalid length
	blink (__FLOCK_LEDBLIP);			// blip for valid packet
	if (r < 3) pxBuff[2]= 0;			// see comment

	// if we see *any* packet from the base, we assume that
	// we are on the correct channel and that the channel is
	// alive. but is this always true? if we can RX base, can base RX us?
	// are there conditions where this isn't a good assumption? i
	// can't think of any.
	//
	if (pxBuff[1] == '@') {				// if from the base,
		random();				// break the sequence
		T.resetTimer (__FLOCK_RXTIMER);		// channel is alive
		T.resetTimer (__FLOCK_RXTOTIMER);
	}

	// if it's to us, or accept broadcast is enabled, return the
	// packet.
	//
	if ((pxBuff[0] == identity) || (broadcast && (pxBuff[0] == '*'))) {
		if (ackBalance) --ackBalance;		// downcount ack balance
		blink (__FLOCK_LEDBRIEF);
		return r;
	}

	// a valid packet, but not to us. we toss it unless 
	// promiscuous.
	//
	return promisc ? r : 0;
}

// see if the packet in pxBuff of length r is a duplicate. this CRCs
// packets of three or more bytes and compares it against the previous
// CRC; if it's the same we call it a dup and trim it down to just the
// addresses and possible ack request. 
//
// (the sender makes packets CRC uniquely by appending a sequence number;
// this ensures that intentionally identical command strings in contiguous
// packets won't be dropped.)
//
// this returns the corrected packet length.
//
uint8_t SRFlock::dupCheck (uint8_t r) {

	if (r < 3) return r;				// not long enough to check
	int crc= CRC.crc8buff (pxBuff, r);		// CRC the packet
	if (crc == prevCRC) {				// if same as last time
		r= ((pxBuff[2] == '!') ? 3 : 2);	// include ! if present
	}
	prevCRC= crc;
	return r;
}

// return flockToBase() status:
//
// 
//  0:       was, is, disconnected
// -1:       was connected, now disconnected
// -2:       was, is, connected
//  1..127:  was discon, now connected on channel N
//
int SRFlock::flockToBaseState () {

static uint8_t cstate = -1;

	cstate <<= 1;					// previous state 
	if (state == 3) cstate |= 1;			// new state 
	cstate &= 3;					// this and last, only

// cstate is now:
// 0:    was, is, disconnected
// 1:    was discon, now connected
// 2:    was connected, now disconnected
// 3:    was, is, connected
//
	switch (cstate) {
		case 0: return 0;
		case 1: return channel;
		case 2: return -1;
		case 3: return -2;
	}
	return 0;
}

/*

  int status= baseToFlock ();
  
  runs the protocol state machine for the Base end of the
  promiscuous flockToBase protocol.
  
  0     no data received. this means either no packet at all, a protocol
        packet (eaten by the state machine)
    
  1..32 a packet with valid payload received and is now in
        buff[]. note that the first two bytes of the packet are protocol
        overhead.
    
  -1    read error, which really means code, driver, or hardware error.

  this also does channel-switching on high-error-rate channels, by
  upcounting ackBalance on packet transmits that request acknowledgement,
  and down-counting ackBalance upon every packet receive. if the sum of
  this counting exceeds threshhold, we mark the channel as bad and pick
  a new one. 

*/
int SRFlock::baseToFlock () {

	if (LEDpin) RFLED.LED();			// LED state machine

	int r= radio.available();			// ready/sample RX power
	if (r) {					// 0 else RX power
		rxPower= r;
		r= radio.read (pxBuff, PACKETSIZE);	// read available packet
	}

	// if no valid packet, check the ack sent/received counter. if too
	// many outstanding ack requests, change channels and do a broadcast
	// ping.
	//
	if (r < 2) {					// if no packet,
		if (ackBalance > chanThresh) { 
			pxBuff[0]= '*';
			pxBuff[1]= '@';
			pxBuff[2]= '-';			// channel change hint 
			radio.write (pxBuff, 3);	// blind transmit
			radio.write (pxBuff, 3);

			connectState= false;		// fail
			channel= newChan (channel);	// pick a new channel
			radio.setChannel (channel);
			T.trigTimer (__FLOCK_RXTIMER);	// cause immediate ping
		}
		if (rxTimeout && T.timer (__FLOCK_RXTIMER)) 
			ackPacket ('*', true);
		return 0;				// 
	}

	// got packet; blip the LED. if it's for us (base) then longer blink and
	// extract payload, tally ack balance as channel quality.
	//
	blink (__FLOCK_LEDBLIP);			// brief blink
	if (pxBuff[0] != identity) return 0;		// exit if not for us

	if (ackBalance) --ackBalance;			// good packet
	blink (__FLOCK_LEDBRIEF);			// long blink
	connectState= true;				// in some sense, anyway
	r= dupCheck (r);				// see if duplicate
	copyN (rxBuff, pxBuff, r);			// extract payload

	// send an ack packet if one is requested, or if we go long enough
	// without hearing a reply.
	//
	if (r > 2 && pxBuff[2] == '!') {		// if it requests an ack
		ackPacket (pxBuff[1], false);		// ack it
		++ackBalance;				// flag that

	// broadcasting empty packets from base allows the birds to 
	// avoid an ack request.
	//
	} else if (rxTimeout && T.timer (__FLOCK_RXTIMER)) {
		ackPacket ('*', false);
	}

	return r;
}

// transmit a packet, return status.
// if there is room, append the sequence number. in the SR command
// syntax, this appears to be a payload without a command suffix,
// and the command parser will ignore it; but it will make the
// CRC for this packet unique to this packet (and different from
// the CRC of otherwise identical commands, with a different sequence).
//
int SRFlock::write (uint8_t n) {

	random();					// interrupt sequence
	if (! connectState) return -1;			// can't send now

	blink (__FLOCK_LEDBRIEF);
	copyN (pxBuff, txBuff, n);			// copy data in,

	if (n < PACKETSIZE - 1) {			// if there is room
		pxBuff[n++]= seqNumber;			// append sequence num
		if (++seqNumber > '9') seqNumber= '0';
		radio.write (pxBuff, n);		// additional write
		delayMicroseconds (random (311, 1000));	// brief delay
	}

	if (n > 2 && pxBuff[2] == '!') {
		++ackBalance;				// require acknowledge
	}
	return radio.write (pxBuff, n);			// 
}

// selects a new radio channel and returns it, within the channel limits
// and dynamic channel selection. the specified channel, b, is marked as
// bad. this also randomly (5% of the time) marks a random channel as
// good, to both avoid running out of channels and to re-try channels
// once marked bad (environment changes).
//
// channel state is a bit map; 0/1 good/bad.
//
uint8_t SRFlock::newChan (uint8_t b) {

	if (! dynamicChannelMapping) return channel;

	if (b != 0) {
		Serial.print ("MAP BAD ");
		Serial.println (b);
	}

	// mark this channel as bad.
	//
	badChan [b / 8] |= 1 << (b % 8);		// mark bad,

	// select a new, random, channel that is not currently
	// marked as bad. also randomly (5% of the time) un-marks
	// a channel; this prevents entirely running out of channels,
	// and prevents inifnite looping, here. also, "bad" status
	// varies with time since interference varies.
	//
	uint8_t chan, r;
	do { 
		chan= random (minChannel, maxChannel + 1);
		if (random (1, 100) < 5) {		// 5% of the time,
			r= random (minChannel, maxChannel + 1);
			badChan [r / 8] &= ~ (1 << (r % 8)); 
		}

	} while ((badChan [chan % 8] & (1 << (chan % 8)))); // loop while bad

	ackBalance= 0;
	return chan;
}

void SRFlock::setRxAR (uint8_t sec) { 
	if (sec < __FLOCK_MAXRXTO) 
		T.setDeciTimer (__FLOCK_RXTIMER, rxTimeout= sec * 10); 
};

void SRFlock::setRxAT (uint8_t sec) { 
	if (sec < __FLOCK_MAXRXTO) 
		T.setDeciTimer (__FLOCK_RXTOTIMER, sec * 10); 
};


void    SRFlock::setIdentity (char id) 	{ identity= id; }
void    SRFlock::dynamicChannelEnable() { dynamicChannelMapping= true; };
void    SRFlock::dynamicChannelDisable(){ dynamicChannelMapping= false; };
void    SRFlock::nextChannel() 		{ state= 0; radio.setChannel (channel= newChan (0)); }
uint8_t SRFlock::getChannel() 		{ return channel; };
bool SRFlock::connected() 		{ return connectState; };
uint8_t SRFlock::getRxPower() 		{ return rxPower; };
uint8_t SRFlock::getChanError() 	{ return ackBalance; };
uint8_t * SRFlock::getChanErrorMap () 	{ return badChan; };
uint8_t SRFlock::getChanThresh()	{ return chanThresh; };
void    SRFlock::setChanThresh(uint8_t t){ chanThresh= t; };
void    SRFlock::setMinChannel (uint8_t ch) { if (ch >= MINCHANNEL) minChannel= ch; } 
void    SRFlock::setMaxChannel (uint8_t ch) { if (ch <= MAXCHANNEL) maxChannel= ch; }
void    SRFlock::setPromiscuous (bool f) { promisc= f; }

// these are just pass-throughs to the radio object.
//
uint8_t SRFlock::readRegister (uint8_t r){ return radio.read_register (r); }
void SRFlock::setDataRate (uint8_t n) 	{ radio.setDataRate (n); }
void SRFlock::setPALevel (uint8_t n) 	{ radio.setPALevel (n); }
void SRFlock::setChannel (uint8_t ch) 	{ radio.setChannel (channel= ch); }
void SRFlock::setFixedAddress (uint64_t t, uint64_t r) 
					{ radio.setFixedAddress (t, r); }
void SRFlock::setSelfAddress (uint64_t base, uint8_t t, uint8_t r) 
					{ radio.setSelfAddress (base, t, r); }
void SRFlock::setSelfAddress (uint8_t t, uint8_t r) 
					{ radio.setSelfAddress (t, r); }


// local function: copy N bytes, from s to d.
//
void SRFlock::copyN (uint8_t * d, uint8_t * s, uint8_t n) {

	if (n > PACKETSIZE) n= PACKETSIZE;		// don't be a jerk.
	while (n--) *d++= *s++;
}

// this is the message command function dispatcher for Flock.
// return true if the command character c is ours, meaning we
// processed it. return false if it's not ours.
//
bool SRFlock::messageDispatcher (char c, unsigned nnn) {
  
  bool ok= true;

	switch (c) {
		case 'a':  					break;
		case 'b': 					break; 
		case 'c':					break;
		case 'd':					break;
		case 'e': 					break;   
		case 'f': setChannel (nnn);			break;
		case 'g': nextChannel();			break;
		case 'h': setMinChannel (nnn);			break;
		case 'i': setMaxChannel (nnn);			break;
		case 'j': dynamicChannelEnable();		break;
		case 'k': dynamicChannelDisable();		break;
		case 'l': setDataRate (nnn);			break;
		case 'm': setPALevel (nnn);			break;
		case 'n': setRxAR (nnn);			break; 
		case 'o': setRxAT (nnn);			break;
		case 'p': setChanThresh (nnn);			break;
		case 'q': setPromiscuous (nnn);			break;
		case 'r':					break;
		case 's': 					break;
		case 't': 					break;
		case 'u':					break;
		case 'v':					break;
		case 'w':					break;
		case 'x':					break;
		case 'y':					break;
		case 'z':					break;

		default: ok= false; break;
	}
	return ok;
}

// end of #ifndef __SRFLOCK_H__
#endif

