/*

 flock protocol, part 2
 
*/


#ifndef __SRFLOCK2_H__
#define __SRFLOCK2_H__


// send an empty packet with optional protocol hint; ACK, HINT, BYE,
// etc. 
//
void SRFlock::protoPacket (char dest, char req) {

	pxBuff[0]= dest;				// set destination
	pxBuff[1]= identity;				// source: us
	pxBuff[2]= req;					// hint, or 0
	radio.write (pxBuff, (req ? 3 : 2));		// 2 if req is 0
	blink (LEDBLIP);				// blink LED
}

// check for available receive packet and return the packet length
// if one is available and matches address requirements. this also checks
// for duplicate packets, and trims those down to the address header
// (eg. no payload).
//
// loose if true means accept broadcast packets, otherwise
// only packets addressed to us.
//
// to simplify ack-request checks this clears the third byte
// (first payload byte) for packet lengths < 3 so that packet length
// doesn't need to be checked.
//
// returns 0 if not a valid packet or not for us, else it returns
// the packet length. hence, it returns 0, or 2..32.
//
int SRFlock::getPacket (bool loose) {

	int r= radio.available();			// ready/sample RX power
	if (r) {					// 0 else RX power
		rxPower= r;				// save for later return
		pxBuff[2]= ' ';				// see comment
		r= radio.read (pxBuff, PACKETSIZE);	// read packet
	}
	if (r < 2) return 0;				// invalid length

	blink (LEDBLIP);				// blip for valid packet

	// if it's not addressed to us, ignore it unless we're
	// in promiscuous mode.
	//
	if (! toUs (loose)) return 0;

	blink (LEDBRIEF);				// for us, longer blink

	// duplicate packets have their payloads truncated.
	//
	int crc= CRC.crc8buff ((uint8_t *)pxBuff, r);	// CRC the packet
	if (crc == prevCRC) {				// if same as last time
		r= ((pxBuff[2] == ACK) ? 3 : 2);	// include ! if present
	}
	prevCRC= crc;					// for next time
	return r;
}

// return true if this packet contains an ACK request.
//
bool SRFlock::isAck () {

	return (pxBuff[2] == ACK);
}

// return true if this packet is a channel-change hint.
//
bool SRFlock::isHint () {

	if (identity == baseID) return false;		// not if we're base!	
	if (pxBuff[1] != baseID) return false;		// only base issues
	return pxBuff[2] == HINT;
}

// return true if the packet is addressed to us, or broadcast and
// we're allowing those.
//
bool SRFlock::toUs (bool loose) {

	if (pxBuff[0] == identity) return true;
	if ((pxBuff[0] == ALL) && loose) return true;
	return false;
}

// mark this bird as heard from (set ACK flag) or
// not heard from (eg. clear it's ACK flag).
//
void SRFlock::nakBird (char id) { 
int b;

	if (b= birdToBit (id) == -1) return;
	birdAck [b / 8] &= 1 << (b % 8); 
}

void SRFlock::ackBird (char id) { 
int b;

	if (b= birdToBit (id) == -1) return;
	birdAck [b / 8] &= 1 << (b % 8); 
}

// return trueif we've heard from this bird within ack time.
//
bool SRFlock::heardID (char id) {
int b;

	if (b= birdToBit (id) == -1) return false;
	return birdAck [b / 8] & 1 << (b % 8); 
}

// turn bird ID in the range of A..Za..z to a number 0..51,
// or -1 if out of that range.
//
int SRFlock::birdToBit (char id) {

	int b= id - 'A'; if (b < 0) return -1;		// < A
	if (b < 26) return b;				// A <= b <= Z

	b= id - 'a'; if (b < 0) return -1;		// < a
	return b < 26 ? b + 26 : -1;			// a <= b <= z else -1
}

//
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


// setup to use an LED for operational status.
//
void SRFlock::LED (uint8_t n) { RFLED.begin (LEDpin= n); }

// if configured, blink the LED for N mS.
//
void SRFlock::blink (uint8_t n) { if (LEDpin) RFLED.blink (n); }


// "simple" support functions.
//

// Peep timers are deciseconds, stored as-is.
//
unsigned SRFlock::getPeepConnectTime()	{ return peepConnectTime; }
unsigned SRFlock::getPeepUpTime()	{ return peepUpTime; }
void	SRFlock::setPeepConnectTime (unsigned n) { peepConnectTime= n; }
void	SRFlock::setPeepUpTime (unsigned n) { peepUpTime= n; }

// AckRequest and AckTimeout are deciseconds.
//
void 	SRFlock::setRxAR (unsigned n) 	{ T.setDeciTimer (RXTIMER, n); };
unsigned SRFlock::getRxAR() 		{ return T.getTimer (RXTIMER) / 100; }
void 	SRFlock::setRxAT (unsigned n) 	{ T.setDeciTimer (RXTOTIMER, n); };
unsigned SRFlock::getRxAT() 		{ return T.getTimer (RXTOTIMER) / 100; }
uint8_t SRFlock::getAckThresh()		{ return ackThresh; };
void    SRFlock::setAckThresh(uint8_t t) { ackThresh= t; };

char    SRFlock::getIdentity() 		{ return identity; }
bool 	SRFlock::connected() 		{ return state == 3; }
unsigned SRFlock::rxMessages ()		{ return rxCount; }
unsigned SRFlock::txMessages ()		{ return txCount; }
char *	SRFlock::getRxBuff() 		{ return rxBuff; }
unsigned SRFlock::getTxQueueDepth()	{ return TXQDEPTH; }
uint8_t SRFlock::getRxPower() 		{ return rxPower; }
uint8_t SRFlock::getAckBalance() 	{ return ackBalance; }
uint8_t * SRFlock::getChanErrorMap () 	{ return badChan; }

void    SRFlock::dynamicChannelEnable() { dynamicChannelMapping= true; }
void    SRFlock::dynamicChannelDisable(){ dynamicChannelMapping= false; }
uint8_t SRFlock::getChannel() 		{ return channel; }
void    SRFlock::nextChannel() 		{ radio.setChannel (channel= newChan (0)); }
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
	*d= '\0';					// and terminate it
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

// end the message, 
//
int SRFlock::messageEnd () {

	return write ();
}

// end of #ifndef __SRFLOCK_H__
#endif

