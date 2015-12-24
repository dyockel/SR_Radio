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

  
  23 dec 2015	added setScanRate().
  05 nov 2015   dispatchers are now type void.
  03 nov 2015	removed addDispatcher level func.
  25 oct 2015	rationalized types and assignments, matches edits
  		to RF24.h to approach MISRA-C. all args are int/unsigned
		in implementation widths unless it is necessary to conform
		to underlying hardware (chip) widths. 
		combined SRFlock1.h and SRFlock2.h. 
  05 oct 2015   added missing addDispatcher ( , level)
  29 SEP 2015	added set/get protocol, folded peep() function
  		into message(), eliminated the Radio "module" object.
  16 sep 2015   reordered init code, slightly smaller but cleaner.
  15 sep 2015	fixed annoying warning about truncated, had blink()
  		arg declaration wrong. clear prevCRC on connect.
  12 sep 2015	added required check for isFromBase() at state 2
  		channel scan.
		there is some bug in roadsterLCDDongle that is failing
		RxAR/RxAT logic.
  05 sep 2015	massive cleanup/minor rewrite. redundancy removed,
  		(some of the) state errors fixed. RxAR, RxAT
		rationalized and much clearer. now works as advertised.
  04 sep 2015   aargh, converted all non-mS timers to deciseconds.
  		RxAR and RxAT were seconds, not fine enough, and
		also a bug returning value off by 10X.
  02 sep 2015   setPeep* times are now deciseconds, not mS
  26 aug 2015	oopsie, missing string terminator added after check
  		character added to enqueued outging payload.
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

//#define DEBUG
//#define DEBUGRX
//#define DEBUGTX

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
#endif

typedef byte uint8_t;

static const unsigned MINCHANNEL = 1;			/* Nordic chip limits */
static const unsigned MAXCHANNEL = 125;
static const unsigned PACKETSIZE = 32;

/* our customary radio pins. */

static const unsigned NRFCEpin = 9;
static const unsigned NRFCSNpin = 10;


class SRFlock {

/* timers. */

static const int RADIOTIMER = 0;
static const int RXTIMER =    1;
static const int RXTOTIMER  = 2;
static const int PEEPTIMER  = 3;
static const int LOOPTIMER  = 4;
static const int NUMTIMERS  = 5;

/* LED blink times */

static const unsigned LEDBLIP = 5;
static const unsigned LEDBRIEF = 10;
static const unsigned LEDLONG = 600;

/* packet stuff */

static const char ACK = '!';		/* ack request */
static const char HINT = '-';		/* channel-change hin */
static const char BYE = '#';		/*  */
static const char ALL = '*';		/* address: all birds */


/* transmit queue buffer depth */

#define TXQDEPTH 4

public:
	int begin (void);
	int begin (unsigned cepin, unsigned cspin);
	void setBaseID (char id);
	void setIdentity (char id);
	char getIdentity (void);
	void setProtocol (int p);
	int getProtocol (void);
	void LED (unsigned pin);
	int packet(void);
	char message(void);
	bool connected(void);
	int status(void);
	int write (void);
	char * getRxBuff(void);
	char * getTxBuff(void);


/* connection management */

	void setScanRate (unsigned n);
	void setRxAR (unsigned n);
	void setRxAT (unsigned n);
	void setAckThresh (unsigned t);
	void setPromiscuous(bool yup);
	void dynamicChannelEnable(void);
	void dynamicChannelDisable(void);
	void nextChannel(void);
	unsigned getAckBalance(void);
	byte * getChanErrorMap (void);
	unsigned getAckThresh(void);
	unsigned txQueueCount (void);
	unsigned getTxQueueDepth (void);
	void powerDown (void);
	void powerUp (void);
	bool poweredOn (void);
	void setPeepConnectTime (unsigned n);
	void setPeepUpTime (unsigned n);
	unsigned txMessages(void);
	unsigned rxMessages(void);
	void setFlockLoopRate (unsigned n);
	unsigned getRxAR(void);
	unsigned getRxAT(void);
	unsigned getPeepConnectTime(void);
	unsigned getPeepUpTime(void);

/* radio pass-through functions */

	void setFixedAddress (uint64_t t, uint64_t r);
	void setSelfAddress (uint64_t base, byte t, byte r);
	void setSelfAddress (byte t, byte r);

	void setChannel (unsigned channel);
	void setMinChannel (unsigned channel);
	void setMaxChannel (unsigned channel);
	void setDataRate (unsigned n);
	void setPALevel (unsigned n);

	unsigned getChannel(void);
	unsigned getRxPower(void);
	byte readRegister (byte r);

/* Message parser methods, a mix of custom/local and */
/* passthrough. */

	void addDispatcher (void (* func) (int, unsigned));
	int messageBegin (void);
	int messageTo (char dest, bool ack);
	int messageAdd (char c, unsigned nnn);
	int messageEnd(void);

protected:
	void blink (unsigned n);
	void protoPacket (char dest, char req);
	int getPacket (bool loose);
	void copyN (char * d, char * s, unsigned n);
	unsigned newChan (unsigned badChan);

private:
	void txDequeue (void);
	int packetWrite (unsigned n);
	int packetWrite (int q, unsigned n);
	bool heardID (char id);
	void ackBird (char id);
	void nakBird (char id);
	int birdToBit (char id);
	bool toUs (bool loose);
	bool isFromBase (void);
	bool isAck (void);
	bool isHint (void);

	SRRF24 radio;				/* radio chip driver */
	SRTimer T;				/* software timers */
	SRCRC8 CRC;				/* CRC for data packets */
	SRPRNG PRNG;				/* minimal PRNG */
	SRLED RFLED;				/* our RF activity status LED */
	SRMessage MSG;				/* messaging system */
	unsigned LEDpin;			/* optional */

	unsigned flockRate;			/* how fast Flock runs */
	unsigned scanRate;			/* how often state 0 scans */
	unsigned protocol;			/* 0=Flock 1=Peep */
	char identity;				/* our identity (A..Z) */
	char baseID;				/* ID of the base (usually @) */
	unsigned state;				/* protocol state machine */
	unsigned channel;			/* current radio channel */
	unsigned minChannel;			/* the SMA radios lousy at */
	unsigned maxChannel;			/* low channels... */
	unsigned cstate;			/* connect-state report state */
	unsigned pstate;			/* power state report state */

	byte txQueue [TXQDEPTH] [PACKETSIZE + 1];/* TX ring buffer */
	int in, out;				/* input and output pointers */

	char pxBuff[PACKETSIZE + 1];		/* protocol packet buffer */
	char rxBuff[PACKETSIZE + 1];		/* receive packet buffer */
	bool promisc;				/* true if receive any packet */
	bool poweroff;				/* true if chip/protocol off */
	int rxPower;				/* receive power indication */
	bool dynamicChannelMapping;		/* enables channel mapping */
	unsigned ackBalance;			/* outstanding ack count */
	unsigned ackThresh;			/* outstanding ack limit */
	byte birdAck[7];			/* birds A..Za..z recent ack */
	byte badChan[16];			/* bad channels, 0..127, bitmap */
	byte prevCRC;				/* CRC8 of previous payload  */
	char seqNumber;				/* packet sequence character */
	unsigned peepConnectTime;		/* allowed time to connect, dS */
	unsigned peepUpTime;			/* stay up after write(), dS */
	unsigned txCount, rxCount;		/* packets sent/received */
};


/*
  begin/setup Flock protocol.
*/

/* use the default pins for the radio and allocate buffers internally. */

int SRFlock::begin() {

	return begin (NRFCEpin, NRFCSNpin);
}

/* allocate buffers internally. */

int SRFlock::begin (unsigned cepin, unsigned csnpin) {

	in= out= 0;
	T.begin (NUMTIMERS);				/* init timers */
	MSG.begin();					/* ready message parser */
	CRC.begin();					/* probably does nothing */
	PRNG.begin();					/* local PRNGs */
	PRNG.xor16Seed (micros());			/* seed the 16 PRNG */
	LEDpin= 0;					/* see LED() for begin() */

	/* intialize the radio.  */

	int r= radio.begin (cepin, csnpin); 		/* set up radio hardware */
	dynamicChannelMapping= true;			/* on by default */
	minChannel= 20;					/* set default range */
	maxChannel= 120;
	nextChannel();					/* select first channel */

	/* default assumes Flock: request ack every 4 seconds, */
	/* connection timeout at 19 seconds. 5 lost acks means  */
	/* bad channel/lost connect. */

	setRxAR (60);					/* ack request interval */
	setRxAT (190);					/* ack req fail interval */
	ackThresh= 5;					/* outstanding ack limit */

	radio.setAutoAck (false);			/* make sure this is off */
	setPALevel (3);					/* highest power  */
	setFlockLoopRate (5);				/* flock loop time */
	promisc= false;					/* no promiscuous RX */
	poweroff= false;				/* chip is ready */
	peepConnectTime= 200;				/* 20 sec Peep connect */
	peepUpTime= 10;					/* stay up after write() */
	baseID= '@';					/* default base ID */

	prevCRC= 0;
	seqNumber= '0';					/* first packet sequence */
	state= 0;					/* our state machine */
	cstate= pstate= 0;				/*  */
	scanRate= 5;					/* default channel scan rate */
	return r;
}

/* define the base ID character, default is already '@'. */

void SRFlock::setBaseID (char id) {

	baseID= id;
}

/* set our identity and protocol.  protocol==Bird  */
/* unless ID is baseID which means base.  */

void SRFlock::setIdentity (char id) { 

	identity= id;
	state= (id == baseID ? 4 : 0);
}

/* set the loop task time for Flock, Peep and Packet. */

void SRFlock::setFlockLoopRate (unsigned n) {

	if (n > 100) n= 100;
	T.setTimer (LOOPTIMER, n);
}

/* set the state 0 channel scan rate. */

void SRFlock::setScanRate (unsigned n) {

	scanRate= n;
}

/* power management methods used mainly by Peep protocol.  */
/* after powerDown() is invoked, only powerUp(), poweredOn() */
/* are allowed. */


/* power on the radio and reset Flock and Peep protocols. */

void SRFlock::powerUp () { 

	radio.available(); 				/* ready the chip */
	rxCount= txCount= 0;				/* quiescent */
	setIdentity (identity);				/* resets state machine */
	pstate= cstate= 0;				/* used by status() */
	poweroff= false;
}

/* power down the chip and mark everything as disconnected. this specifically */
/* does not reset the TX queue so that it can be examined by txQueueCount(). */

void SRFlock::powerDown () { 	
	
	setIdentity (identity);				/* resets state machine */
	radio.powerDown(); 				/* power off the chip */
	poweroff= true;
}

bool SRFlock::poweredOn () {

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

	if (r= packet()) {			/* if we got a packet, */
		rxBuff [r]= '\0';		/* null terminate as string */
		MSG.rxString (rxBuff + 2);	/* dispatch messages */
	}

	/* Peep is the same as Flock, except that it powers down */
	/* when no transmit activity. */

	if (protocol == 1 && T.timer (PEEPTIMER)) {
		powerDown();			/* if Peep, manage power */
	}
	return r ? rxBuff[1] : 0;		/* return sender, if packet */
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
byte r;

	if (! T.timer (LOOPTIMER)) return 0;
	if (LEDpin) RFLED.LED();			/* run LEDs  */
	txDequeue();					/* send queued packets */
	if (! poweredOn()) return 0;			/* we are powered off */

	r= 0;
	switch (state) {

/* FLOCK TO BASE NEGOTIATION ------------------------------------------------------ */

		/* search for the operating channel by pinging random */
		/* channels looking for a response from Base. */

		case 0:	
			nextChannel();			/* select a new channel */
			state= 1;
			break;
			/* FALL THROUGH */

		/* blindly transmit an ack request, setup to wait */
		/* for a response. */

		case 1:
			protoPacket (baseID, ACK);	/* request ack */
			T.setTimer (RADIOTIMER, 3);	/* well tuned, but needs justification */
			state= 2;			/* await response */
			break;

		/* request sent; watch for response (connected on */
		/* current channel) or timeout (try next channel). */

		case 2:	
			if (getPacket (false) && isFromBase()) {/* if a packet for us, */
				blink (LEDLONG);
				T.resetTimer (RXTIMER);	/* reset ack timers */
				T.resetTimer (RXTOTIMER);
				prevCRC= 0;		/* tiny opp for bad... */
				state= 3;		/* we're in sync */

			} else if (T.timer (RADIOTIMER)) {
				state= 0;
			}
			break;

		
		/* channel established; we remain in this state as long as we */
		/* receive or see a packet from the base within the alloted */
		/* time(r)s. if we go too long without an rx packet, or  */
		/* ack balance goes bad, we change channels and try again. */

		case 3:	
			r= getPacket (true);		/* what we got? */
			if (r) {
#ifdef DEBUGRX
				Serial.print ("debug rx3 ");
				Serial.println (pxBuff);
#endif
				/* valid packet addressed to us. count it, */
				/* reset timeout counters, deliver any */
				/* payload, send acknowledgement if requested. */

				if (ackBalance) --ackBalance;	
				blink (LEDBRIEF); 
				++rxCount;
				T.resetTimer (RXTIMER);	
				T.resetTimer (RXTOTIMER);
				copyN (rxBuff, pxBuff, r); /* deliver */

				/* honor ack requests now. */

				if (isAck()) {
#ifdef DEBUG
					Serial.println ("honor ack req");
#endif
					protoPacket (pxBuff[1], 0);
				}

				/* if this is a channel change hint, take */
				/* the hint, but only if we're not the base! */

				if (isHint()) {
#ifdef DEBUG
					Serial.println ("HINT received");
#endif
					setIdentity (identity);
				}

/*
 
   ackBalance has been creeping up; need to examine the state changes
   that lead to incr/decr ackBalance. for now, decr at every packet we
   deliver. likely overeager. this should all get revisited when we do 
   per-address tx queue etc.

				if addressed to us (as opposed to a broadcast) 
				then this packet counts towards our ack 
				balance (decreases outstanding balance). 
				
				if (toUs (false)) {
					if (ackBalance) --ackBalance;	
					blink (LEDBRIEF); // longer blink if ours

				}
*/
				break;			/* no need to run the rest */
			}

			/* handling of timeouts and ack balance is different, */
			/* base vs. bird. */

			if (identity == baseID) {

				/* if we have too many outstanding ack requests */
				/* and we are the base, then ditch this channel and */
				/* try another. */

				if (ackBalance > ackThresh) { 
#ifdef DEBUG
					Serial.println ("ackBalance fail");
#endif
					protoPacket ('*', HINT);
					protoPacket ('*', HINT);
					setIdentity (identity);	/* start over */
				}
			}

			/* for birds, if we go too long without hearing a */
			/* packet from the base, request an ack packet. */
			/* for base, this timer should be set longer than */
			/* RxAT (RXOTIMER). for experimental purposes, if */
			/* used by base it issues a broadcast ack request */
			/* instead; possible utility as "see what birds are */
			/* out there"? */

			if (T.timer (RXTIMER)) {
#ifdef DEBUG
				Serial.println ("rxtimer ack request to base");
#endif
				protoPacket (identity == baseID ? ALL : baseID, ACK);
				++ackBalance;			/* increase the balance */
			}

			/* birds and base: if we never hear a response,  */
			/* restart protocol. this timer is longer than RXTIMER. */

			if (T.timer (RXTOTIMER)) {
#ifdef DEBUG
				Serial.println ("receive timeout");
#endif
				setIdentity (identity);	/* start over */
			}
			break;

/* BASE TO FLOCK NEGOTIATION ----------------------------------------------------- */

		/* pick a new channel, clear the timers and */
		/* await packets. */

		case 4:
			nextChannel();
#ifdef DEBUG
			Serial.print ("new channel "); Serial.println (channel);
#endif
			T.resetTimer (RXTIMER);		/* bird, not base */
			T.resetTimer (RXTOTIMER);	/* base and bird */
			ackBalance= 0;			/* start over */
			state= 3;
			break;
	}
	return r;
}
/* prepare a message for transmission. the message was */
/* built in txBuff() which is the end (input) of the queue. */
/* the Flock sequence character is appended (dup packet */
/* detection at the receiver). */

/* if the radio is not powered on or not connected, the */
/* packet remains queued for later transmission.  */

/* if the radio is off, it is powered on and readied. */

/* (the userland caller must check that there is room in  */
/* the queue; not doing so merrily overwrites the oldest */
/* unsent message, not an entirely unreasonable thing to do; */
/* but if doing so advances the in pointer past the out */
/* pointer re-sends of previously-sent packets will occur. */
/* all this can be avoided by checking queue depth before */
/* building a new packet getTxBuff().) */

int SRFlock::write () {

	/* append the sequence character to the message payload. */

 	int n= strlen ((char *)txQueue [in]);
	if (n < PACKETSIZE - 1) {		/* if there is room */
		txQueue [in] [n++]= seqNumber;	/* append sequence num */
		txQueue [in] [n]= '\0';		/* terminate */
		if (++seqNumber > '9') seqNumber= '0';
	}

	/* if the radio is currently off, turn it on and leave this message */
	/* queued for later transmission (since it can't happen now) */
	/* and set Peep's connect-time timer (Flock ignores it). */

	if (! poweredOn()) {			/* if currently off, */
		powerUp();			/* enable radio */
		T.setDeciTimer (PEEPTIMER, peepConnectTime); 
		if (++in >= TXQDEPTH) in= 0;	/* enqueue this packet, */

		return 2;
	}

	/* radio is up and Peep/Flock connected. write the packet */
	/* directly. if this succeeds we're done; if it does not */
	/* successfully send queue it up for later retry. Flock */
	/* ignores PEEPTIMER. */

	if (connected() && txQueueCount() == 0) {
		if (packetWrite (in, n) > 0) {
			T.setDeciTimer (PEEPTIMER, peepUpTime);
			return 1;		/* success */
		}
	}
	if (++in >= TXQDEPTH) in= 0;	/* enqueue this packet, */
	return 2;				/* tx fail, leave queued */
}

/* if there is a message in the queue transmit it. if */
/* transmission succeeds, dequeue the message. */

/* NOTE: this needs to check each message dest ID for  */
/* "heard from that ID", then remove the one from packet write. */
/* or do it in packet write. */

void SRFlock::txDequeue () {

	if (in == out) return;			/* queue is empty, done */

	int n= strlen ((char *)txQueue [out]);	/* payload length */
	if (packetWrite (out, n)) {		/* if success, */
		if (++out >= TXQDEPTH) out= 0;
		T.setDeciTimer (PEEPTIMER, peepUpTime);
	}
}

/* return the number of items in the queue. */

unsigned SRFlock::txQueueCount () {

	int a= in - out; if (a < 0) a += TXQDEPTH;
	return a;
}

/* return a pointer to the current TX buffer, eg. the oldest queue slot. */

char * SRFlock::getTxBuff() {
	
	return (char *)txQueue[in];
};


/* write bytes from the buffer to the radio only */
/* if we have a connect. */

int SRFlock::packetWrite (int q, unsigned n) {

	PRNG.xor16();				/* disrupt sequence */
	if (! connected()) return 0;		/* can't send now */

#ifdef DEBUGTX
	Serial.print ("debug tx packetwrite ");
	Serial.println ((char *)txQueue [q]);
#endif
	blink (LEDBRIEF);
	if (n > 2 && txQueue [q] [2] == '!') {
		++ackBalance;			/* require acknowledge */
	}
	int r= radio.write (txQueue [q], n);
	if (r > 0) ++txCount;
	return r;

}


/* returns Flock connection status change. connected() returns current */
/* state; this tracks changes. the code is a bit of a crock. */

/* 0:    was, is, disconnected */
/* 1:    was discon, now connected */
/* 2:    was connected, now disconnected */
/* 3:    was, is, connected */
/* 4:    radio was just powered up */
/* 5:    radio was just powered down */


int SRFlock::status () {

	/* calc power state */

	pstate <<= 1;
	if (! poweroff) pstate |= 1;
	pstate &= 3;

	/* calculate connect state */

	cstate <<= 1;					/* previous state  */
	if (state == 3) cstate |= 1;			/* new state  */
	cstate &= 3;					/* this and last, only */

	/* changes in power state override connect state. */
	switch (pstate) {
		case 1: return 4; break;
		case 2: return 5; break;
		default: return cstate; break;
	}
	return cstate;
}

/* send an empty packet with optional protocol hint; ACK, HINT, BYE, */
/* etc.  */

void SRFlock::protoPacket (char dest, char req) {

#ifdef DEBUGTX
	Serial.print ("protoPacket (");
	Serial.write (dest);
	Serial.write (identity);
	Serial.write (req);
	Serial.println (")");
#endif
	pxBuff[0]= dest;				/* set destination */
	pxBuff[1]= identity;				/* source: us */
	pxBuff[2]= req;					/* hint, or 0 */
	radio.write ((byte *)pxBuff, (req ? 3 : 2));	/* 2 if req is 0 */
	blink (LEDBLIP);				/* blink LED */
}

/* check for available receive packet and return the packet length */
/* if one is available and matches address requirements. this also checks */
/* for duplicate packets, and trims those down to the address header */
/* (eg. no payload). */

/* loose if true means accept broadcast packets, otherwise */
/* only packets addressed to us. */

/* to simplify ack-request checks this clears the third byte */
/* (first payload byte) for packet lengths < 3 so that packet length */
/* doesn't need to be checked. */

/* returns 0 if not a valid packet or not for us, else it returns */
/* the packet length. hence, it returns 0, or 2..32. */

int SRFlock::getPacket (bool loose) {

	int r= radio.available();			/* ready/sample RX power */
	if (r) {					/* 0 else RX power */
		rxPower= r;				/* save for later return */
		pxBuff[2]= ' ';				/* see comment */
		r= radio.read ((byte *)pxBuff, PACKETSIZE);/* read packet */
	}
	if (r < 2) return 0;				/* invalid length */

	blink (LEDBLIP);				/* blip for valid packet */

	/* if it's not addressed to us, ignore it unless we're */
	/* in promiscuous mode. */

	if (! toUs (loose)) return 0;

	blink (LEDBRIEF);				/* for us, longer blink */

	/* duplicate packets have their payloads truncated. */
	/* though this is only meaningful when there is an actual */
	/* payload to deliver, it's harmless if for short packets. */

	byte crc= CRC.crc8buff ((uint8_t *)pxBuff, r);	/* CRC the packet */
	if (crc == prevCRC) {				/* if same as last time */
		r= ((pxBuff[2] == ACK) ? 3 : 2);	/* include ! if present */
	}
	prevCRC= crc;					/* for next time */
	return r;
}

/* return true if this packet contains an ACK request. */

bool SRFlock::isAck () {

	return (pxBuff[2] == ACK);
}

/* return true if this packet is a channel-change hint. */

bool SRFlock::isHint () {

	if (identity == baseID) return false;		/* not if we're base!	 */
	if (pxBuff[1] != baseID) return false;		/* only base issues */
	return pxBuff[2] == HINT;
}

/* return true if the packet is addressed to us, or broadcast and */
/* we're allowing those. */

bool SRFlock::toUs (bool loose) {

	if (pxBuff[0] == identity) return true;
	if ((pxBuff[0] == ALL) && loose) return true;
	return false;
}

/* return true if this packet is from our base. */

bool SRFlock::isFromBase () {

	return (pxBuff[1] == baseID);
}

/* mark this bird as heard from (set ACK flag) or */
/* not heard from (eg. clear it's ACK flag). */

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

/* return trueif we've heard from this bird within ack time. */

bool SRFlock::heardID (char id) {
int b;

	if (b= birdToBit (id) == -1) return false;
	return birdAck [b / 8] & 1 << (b % 8); 
}

/* turn bird ID in the range of A..Za..z to a number 0..51, */
/* or -1 if out of that range. */

int SRFlock::birdToBit (char id) {

	int b= id - 'A'; if (b < 0) return -1;		/* < A */
	if (b < 26) return b;				/* A <= b <= Z */

	b= id - 'a'; if (b < 0) return -1;		/* < a */
	return b < 26 ? b + 26 : -1;			/* a <= b <= z else -1 */
}


/* selects a new radio channel and returns it, within the channel limits */
/* and dynamic channel selection. the specified channel, b, is marked as */
/* bad. this also randomly (5% of the time) marks a random channel as */
/* good, to both avoid running out of channels and to re-try channels */
/* once marked bad (environment changes). */

/* channel state is a bit map; 0/1 good/bad. */

unsigned SRFlock::newChan (unsigned b) {

	if (! dynamicChannelMapping) return channel;

/* 	if (b != 0) { */
/* 		Serial.print ("MAP BAD "); */
/* 		Serial.println (b); */
/* 	} */

	/* mark this channel as bad. */

	badChan [b / 8] |= 1 << (b % 8);		/* mark bad, */

	/* select a new, random, channel that is not currently */
	/* marked as bad. also randomly (5% of the time) un-marks */
	/* a channel; this prevents entirely running out of channels, */
	/* and prevents inifnite looping, here. also, "bad" status */
	/* varies with time since interference varies. */

	unsigned chan, r;
	do { 
		chan= PRNG.random16 (minChannel, maxChannel + 1);
		if (PRNG.random16 (1, 100) < 5) {	/* 5% of the time, */
			r= PRNG.random16 (minChannel, maxChannel + 1);
			badChan [r / 8] &= ~ (1 << (r % 8)); 
		}

	} while ((badChan [chan % 8] & (1 << (chan % 8)))); /* loop while bad */

	ackBalance= 0;
	return chan;
}


/* setup to use an LED for operational status. */

void SRFlock::LED (unsigned n) { RFLED.begin (LEDpin= n); }

/* if configured, blink the LED for N mS. */

void SRFlock::blink (unsigned n) { if (LEDpin) RFLED.blink (n); }

/* "simple" support functions. */

void    SRFlock::setMinChannel (unsigned n) { if (n >= MINCHANNEL) minChannel= n; }
void    SRFlock::setMaxChannel (unsigned n) { if (n <= MAXCHANNEL) maxChannel= n; }
void    SRFlock::nextChannel() { radio.setChannel (channel= newChan (0)); }
void    SRFlock::dynamicChannelEnable() { dynamicChannelMapping= true; }
void    SRFlock::dynamicChannelDisable(){ dynamicChannelMapping= false; }
unsigned SRFlock::getChannel() 		{ return channel; }

/* Peep timers are deciseconds, stored as-is. */

unsigned SRFlock::getPeepConnectTime()	{ return peepConnectTime; }
unsigned SRFlock::getPeepUpTime()	{ return peepUpTime; }
void	SRFlock::setPeepConnectTime (unsigned n) { peepConnectTime= n; }
void	SRFlock::setPeepUpTime (unsigned n) { peepUpTime= n; }

/* AckRequest and AckTimeout are deciseconds. */

void 	SRFlock::setRxAR (unsigned n) 	{ T.setDeciTimer (RXTIMER, n); };
unsigned SRFlock::getRxAR() 		{ return T.getTimer (RXTIMER) / 100; }
void 	SRFlock::setRxAT (unsigned n) 	{ T.setDeciTimer (RXTOTIMER, n); };
unsigned SRFlock::getRxAT() 		{ return T.getTimer (RXTOTIMER) / 100; }
unsigned SRFlock::getAckThresh()		{ return ackThresh; };
void    SRFlock::setAckThresh(unsigned t) { ackThresh= t; };
unsigned SRFlock::getAckBalance() 	{ return ackBalance; }

void    SRFlock::setProtocol (int p) 	{ protocol= p; }
int     SRFlock::getProtocol () 	{ return protocol; }
char    SRFlock::getIdentity() 		{ return identity; }
bool 	SRFlock::connected() 		{ return state == 3; }
unsigned SRFlock::rxMessages ()		{ return rxCount; }
unsigned SRFlock::txMessages ()		{ return txCount; }
char *	SRFlock::getRxBuff() 		{ return rxBuff; }
unsigned SRFlock::getTxQueueDepth()	{ return TXQDEPTH; }
unsigned SRFlock::getRxPower() 		{ return rxPower; }
byte * SRFlock::getChanErrorMap () 	{ return badChan; }

void    SRFlock::setPromiscuous (bool f) { promisc= f; }

/* pass-throughs to the radio object. */

byte SRFlock::readRegister (byte r){ return radio.read_register (r); }
void SRFlock::setDataRate (unsigned n) 	{ radio.setDataRate (n); }
void SRFlock::setPALevel (unsigned n) 	{ radio.setPALevel (n); }
void SRFlock::setChannel (unsigned ch) 	{ radio.setChannel (channel= ch); }
void SRFlock::setFixedAddress (uint64_t t, uint64_t r) 
					{ radio.setFixedAddress (t, r); }
void SRFlock::setSelfAddress (uint64_t base, byte t, byte r) 
					{ radio.setSelfAddress (base, t, r); }
void SRFlock::setSelfAddress (byte t, byte r) 
					{ radio.setSelfAddress (t, r); }


/* local function: copy N characters (Flock message), from s to d. */

void SRFlock::copyN (char * d, char * s, unsigned n) {

	if (n > PACKETSIZE) n= PACKETSIZE;		/* don't be a jerk. */
	while (n--) *d++= *s++;
	*d= '\0';					/* and terminate it */
}

/* add a parser, from userland, to the Flock radio system. */

void SRFlock::addDispatcher (void (* func) (int, unsigned)) { 
	MSG.addDispatcher (func); 
}

/* the messsage creation methods are a subset of the SRMessage */
/* methods, and use local default values: txBuff, identity, etc. */
/* end() is unique to Flock. */

int SRFlock::messageBegin () {

	MSG.messageBegin ((char *)txQueue[in], PACKETSIZE);
	return PACKETSIZE;
}

int SRFlock::messageTo (char dest, boolean ack) {

	MSG.messageChar (dest);		/* to */
	MSG.messageChar (identity);	/* from */
	if (ack) MSG.messageChar ('!');	/* optional ack request */
}

int SRFlock::messageAdd (char c, unsigned nnn) {

	MSG.messageAdd (c, nnn);
	return PACKETSIZE - strlen ((char *)txQueue[in]);
}

/* end the message,  */

int SRFlock::messageEnd () {

	return write ();
}

/* end of #ifndef __SRFLOCK_H__ */
#endif

