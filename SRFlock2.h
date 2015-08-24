/*

 flock protocol, part 2
 
*/


#ifndef __SRFLOCK2_H__
#define __SRFLOCK2_H__


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
		PRNG.xor16();				// break the sequence
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
	int crc= CRC.crc8buff ((uint8_t *)pxBuff, r);	// CRC the packet
	if (crc == prevCRC) {				// if same as last time
		r= ((pxBuff[2] == '!') ? 3 : 2);	// include ! if present
	}
	prevCRC= crc;
	return r;
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

//	if (b != 0) {
//		Serial.print ("MAP BAD ");
//		Serial.println (b);
//	}

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
		chan= PRNG.random16 (minChannel, maxChannel + 1);
		if (PRNG.random16 (1, 100) < 5) {	// 5% of the time,
			r= PRNG.random16 (minChannel, maxChannel + 1);
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


// setup to use an LED for operational status.
//
void SRFlock::LED (uint8_t n) { RFLED.begin (LEDpin= n); }

// if configured, blink the LED for N mS.
//
void SRFlock::blink (uint8_t n) { if (LEDpin) RFLED.blink (n); }


// "simple" support functions.
//

unsigned SRFlock::rxMessages () { return rxCount; }
unsigned SRFlock::txMessages () { return txCount; }
void	SRFlock::setPeepConnectTime (unsigned n) { peepConnectTime= n; }
void	SRFlock::setPeepUpTime (unsigned n) { peepUpTime= n; }
char    SRFlock::getIdentity() {return identity;};
char *	SRFlock::getRxBuff() {return rxBuff;};
int	SRFlock::getTxQueueDepth() {return __FLOCK_TXQDEPTH;};
void    SRFlock::dynamicChannelEnable() { dynamicChannelMapping= true; };
void    SRFlock::dynamicChannelDisable(){ dynamicChannelMapping= false; };
void    SRFlock::nextChannel() 		{ radio.setChannel (channel= newChan (0)); }
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

// pass-throughs to the radio object.
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
void SRFlock::copyN (char * d, char * s, uint8_t n) {

	if (n > PACKETSIZE) n= PACKETSIZE;		// don't be a jerk.
	while (n--) *d++= *s++;
}

// add a parser, from userland, to the Flock radio system.
//
void SRFlock::addDispatcher (bool (* func) (int, unsigned)) {

	MSG.addDispatcher (func);
}

// the messsage creation methods are a subset of the SRMessage
// methods, and use local default values: txBuff, identity, etc.
// end() is unique to Flock.
//
int SRFlock::messageBegin () {

	MSG.messageBegin (txQueue[in], PACKETSIZE);
	return PACKETSIZE;
}

int SRFlock::messageTo (char dest, boolean ack) {

	MSG.messageChar (dest);		// to
	MSG.messageChar (identity);	// from
	if (ack) MSG.messageChar ('!');	// optional ack request
}

int SRFlock::messageAdd (char c, unsigned nnn) {

	MSG.messageAdd (c, nnn);
	return PACKETSIZE - strlen (txQueue[in]);
}

// end the message, transmitting it. if the message has no payload, add
// ack request. things may not go well if the message is not well-formed.
//
int SRFlock::messageEnd () {

	if (txQueue[in][2] == '\0') {
		txQueue[in][2]= '!';
		txQueue[in][3]= '\0';
	}
	return write ();
}

// end of #ifndef __SRFLOCK_H__
#endif

