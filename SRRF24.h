/*

 modified for SR version (Energia and Arduino)

 tom jennings

 25 oct 2015	cleaned up sloppy assignments and source is heading towards
 		MISRA-C.
 07 aug 2015 fixed a couple of compiler warnings.
 28 jun 2015 correct or not, changed buffers from uint8_t's to chars
             for read(), write(). too annoying casting all the time
 06 jun 2015 stray definition of PACKETSIZE deleted.
 05 may 2015 incorrectly cleared RX_DR status. now does so only
             after all RX FIFOs are cleared. (caused RX packets to
	     "pileup" in the FIFO, symptom was hugely delayed
	     delivery.)
 13 apr 2015 set RX address before setting CE high in available(),
             commented out 130uS delay (wait it out in available())
	     and comment edits. 
 18 mar 2015 ditched stupid constructor nonsense, changed class name
 04 mar 2015 version 3
 27 feb 2015 version 2
 12 jan 2015


copyright tom jennings 2015


This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef __SRRF24_H__
#define __SRRF24_H__

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

/* hardware defined object widths. everything
else is implementation width. */

typedef byte uint8_t;


/* simple output pin level changes. */

#define ce(level) digitalWrite(ce_pin,level)
#define csn(level) digitalWrite(csn_pin,level)

/* convert bit number to bit set in byte. */
/* #define _BV(x) (1<<(x)) */

/* this is the default transceiver address, five bytes long.
bytes 4..1 are the base address; byte 0, the LS byte,
is the ID byte, and is set by read() and write(). */

#define DEFAULT_ADDRESS (0xe7e7e7e7e7LL)


/* //////////////////////////////////////////////////////////////////// */

class SRRF24 {

private:
  uint8_t ce_pin; 
  uint8_t csn_pin;
  uint8_t recentRPD;
  bool autoAck;
  bool dynPayload;
  int ourID;
  uint8_t PRXID;
  uint8_t PTXID;
  bool selfAddress;

protected:
  void write_register (byte reg, const byte* buf, unsigned len);
  void write_register (byte reg, byte value);
  void write_register_bits (byte reg, byte bits);
  void flush_rx (void);
  void flush_tx (void);
  void powerUp (void);
  void activateHiddenFeatures (void);
  void setCRCLength (void);
  void setTXID (void);
  void setRXID (void);


public:
  int begin (byte _cepin, byte _cspin);
  int write (byte * buf, unsigned len);
  int available (void);
  int read (byte * buf, unsigned len);
  void powerDown (void);
  
  void setFixedAddress (uint64_t t, uint64_t r);
  void setSelfAddress (uint64_t base, byte t, byte r);
  void setSelfAddress (byte t, byte r);
  void setDataRate (byte n);
  void setPALevel (byte n);
  void setARD (byte n);
  void setARC (byte n);
  void setChannel (byte channel);
  void setAutoAck (bool enable);
  bool setDynamicPayloads (bool enable);

  byte read_register (byte reg, byte* buf, byte len);
  byte read_register (byte reg);
};

/*
 * NSRRF24L01+ driver
 * tom jennings
 *
 * 20 may 2015 added check for compatibility in setDynamicPayload
 *             and begin.
 * 14 mar 2015 write() invokes available() upon completion.
 * 06 mar 2015 minor streamlining, during edits exploring
 *             workarounds to the PTX-to-PTX problem.
 * 03 mar 2015 version 3 (self- and fixed- addressing).
 *             see README.md; simultaneous transmit
 *             with self-addressing has fatal flaw; but
 *             affects only symmetrical peer-to-peer
 *             topologies, and not the chips intended use.
 *             changes to API and addressing model.
 *             implemented stub setDataRate (oversight).
 * 27 feb 2015 version 2 (fixed addressing)
 * 17 feb 2015
 *
 * this driver code was originally derived from J. Coliz' 
 * NSRRF24L01 Arduino library, for which i'm eternally grateful.
 * it was the nicest rational basis i found for achieving 
 * my own ends.
 *
 * i intend this to be a very lean and mean, stripped
 * down but high-performance driver for highest possible
 * reliability. i removed a lot of what i saw as over-
 * generalization (eg. access to each and every possible
 * chip feature) and chose what i see as a solid basis
 * for more complex abstractions to be run over it.
 *
 * by default this driver uses Dynamic Payloads, and
 * therefore the non-PLUS version of the chip is not
 * supported. packet size can vary from 1 to the hardware
 * maximum of 32. the transmitter determines actual
 * packet size. the receiver can chose to read the entire
 * packet or only a portion. see read() for details.
 *
 * Auto-Acknowledge is on by default. it can be turned
 * of if necessary (eg. broadcast type sends).
 *
 * i deemed Nordic's "Multiceiver" feetch to be too
 * application-specific, so this driver supports only
 * one receive and one transmit address ("pipe").
 *
 * an unfortunate side effect of the Nordic design,
 * from a general-purpose-radio POV, is that each
 * end of a bi-directional link must choose
 * RX or TX mode. for many simple applications this
 * isn't a problem, but it's a bit peculiar; eg. you 
 * can't just transmit on channel X and expect to
 * receive packets with only knowing the channel; you
 * must also know the expected RX address (pipe address).
 * however this easy enough to prearrange, and "soon"
 * i will be working on one-to-many and many-to-many
 * topologies (i already have working a frequency-hopping
 * scheme that negotiates this blindly, with no a priori
 * knowledge of endedness).
 *
 *
 */

/*

    NOTES ON TALKING TO THIS CHIP:

   * CE pin low always leads to a stand-by mode; if TX
     in progress, after packet sent/fails.

   * can write to a register (Wreg) only in a Standby
     mode. usually not a problem.

   * per recommended use, the chip ends up in Standby
     mode after a packet is sent (or send fails). this
     code immediately commands it to RX Mode after
     TX Mode, with the assumption that the application
     wants to accept asynchronous recieves. if you
     really want the chip in standby, invoke standby().

   * static 32-byte payload size is default. setting
     dynamic payload allows write() to specify payload
     size. read() returns received payload length in 
     either case. there is no option for fixed payload
     size other than 32.

   * the pipe business can be confusing. the chip was meant
     for fixed-application consumer stuff, not general-
     purpose duplex communication. there is one TX address
     (in register TXADDR) but potentially multiple RX
     addresses. we use only one RX address. however, if
     TX AutoACK is on, *during TX Mode* the RX address
     must be set to the TX address; the receiver (PRX in
     Nordic parlance) rapidly fires off an ACK packet
     intended for reception by PTX. 

     to simplify this, we use the following arrangement:
     TXADDR        is the TX Mode sending address
     RX_ADDR_P0    set to TXADDR; pipe 0, our autoACK address
     RX_ADDR_P1    is the RX Mode receiving address

 
 copyright J. Coliz 2011

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
*/

#include "nRF24L01.h"


/* bring up the radio, and sadly, check for bogus chips. there's a lot of bad 
clones (!) of this inexpensive chip. i do not have a non-+ version to test, so
this may not catch that. i will catch, at least, the duds i got from
AliExpress. (all my good ones came from AliExpress too) */

int SRRF24::begin (byte _cepin, byte _cspin) {

	ce_pin= _cepin;
	csn_pin= _cspin;

	pinMode (ce_pin, OUTPUT);			/* CE pin */
	pinMode (csn_pin, OUTPUT);			/* SPI SS pin */

/* FIXME: arduino-specific, and not friendly to other SPI devices. */

/* Initialize SPI bus 
Minimum ideal SPI bus speed is 2x data rate 
If we assume 2Mbs data rate and 16Mhz clock, a 
divider of 4 is the minimum we want. 
CLK:BUS 8Mhz:2Mhz, 16Mhz:4Mhz, or 20Mhz:5Mhz */

	SPI.begin();
	SPI.setBitOrder(MSBFIRST);
	SPI.setDataMode(SPI_MODE0);
	SPI.setClockDivider(SPI_CLOCK_DIV4);

	ce(LOW);
	csn(HIGH);
	delay (10);				/* paranoia */

	powerDown();				/* clean init */
	write_register (CONFIG, 0);		/* chip may be in wacky state */
	delay (10);				/* after a RESET */
	powerUp();
	activateHiddenFeatures();		/* Nordic "hid" dynamic payloads? */

#define CONFIGBITTEST (_BV(EN_CRC)|_BV(CRCO))
	setCRCLength ();			/* if chip exists, set CRC... */
	if ((read_register (CONFIG) & CONFIGBITTEST) != CONFIGBITTEST)
		return 1;			/* couldn't set CRC? */

	setARD (1);				/* 500 uS retransmission delay */
	setARC (8);				/* 8 retransmission attempts */
	setPALevel (3);				/* highest power output */
	setDataRate (1);			/* 1MBPS bit rate */
	setChannel (80);			/* quiet here in my lab, ymmv */
	setAutoAck (false);			/* auto-acknowledge on */
	setSelfAddress (DEFAULT_ADDRESS, 2, 3);	/* self-address, default base */

	/* setDynamicPayloads(), when enabling, tests for the correct register 
	value, to help detect bad counterfeits. */
	
	if (! setDynamicPayloads (true)) return 2;

	flush_rx();
	flush_tx();
	write_register(STATUS, _BV(TX_DS) | _BV(MAX_RT) );
	ce (HIGH);

	return 0;
}

/* power down the chip. called from userland only. */

void SRRF24::powerDown (void) {

	flush_rx();
	flush_tx();
	write_register (CONFIG, read_register (CONFIG) & ~_BV(PWR_UP));
}

/* if not already powered up, do so and delay for chip wakeup. 
flush buffers, etc. */

void SRRF24::powerUp (void) {

	if (! (read_register (CONFIG) & _BV(PWR_UP))) {
		write_register_bits (CONFIG, _BV(PWR_UP));
		delayMicroseconds (1800);		/* 1.5mS until Standby-1 */
	}
}

/* status= write (buf, len); 
status= write (buf, len, AutoAcknowledge);

transmits the packet, returns transmit status. this blocks 
until completion (but not long). upon completion or error 
we switch back to RX Mode immediately.  

if fixed payload, we always write 32 bytes, padding out the 
packet if len < 32. if dynamic payload, then only 
len bytes are sent. 

upon completion, immediately switches to RX Mode. 

this matches what Appendix A suggests. */

int SRRF24::write (byte * buf, unsigned len) {

	powerUp();					/* wake up chip */
	ce (LOW);					/* stop the receiver */
	write_register (CONFIG, read_register(CONFIG) & ~_BV(PRIM_RX));

/* with CE low and receiver off (PRIM_RX == 0) and TX FIFOs empty, we 
are in Standby-1. set our TX address, clear status... */

	if (selfAddress) setTXID();			/* we are PTX */
	write_register (STATUS, _BV(TX_DS) | _BV(MAX_RT)); /* clear TX statii */

/* calculate how many bytes the TX FIFO needs; static (32) vs. 
dynamic payload (1..32). */

	if (len > 32) len= 32;				/* hardware maximum */
	byte d= dynPayload ? len : 32;

/* calculate how many filler bytes we need to add to 
fill the TX FIFO. */

	d -= len;

/* write the payload into the TX FIFO. */

	csn (LOW);
	SPI.transfer (W_TX_PAYLOAD);
	while (len--) SPI.transfer (*buf++);		/* actual payload */
	while (d--) SPI.transfer (0x55);		/* filler (transitions) */
	csn (HIGH);

/* ...switch to TX Mode begins when CE is high for 10uS and there 
is a poop in the chute. begin the transmission of one packet. */

	ce (HIGH);					/* transition to TX */
	delayMicroseconds (20);				/* (10uS required) */
	ce (LOW);					/* now in "TX settle" */

/* TXTIMEOUT covers only the case of hardware or code failure, to
prevent hangs. it needs to be long enough for  worst-case  
transmit time, which is max. ARD and max. TX retries. 

though technically this code blocks (badness) without tx retries 
it blocks for a couple hundred microseconds; with auto-ack and average- 
worst-case of no recipient, it blocks for tx-time * ARD * ARC. (the TXTIMEOUT 
case means the code is doomed anyway so who cares.) */

	const int TXTIMEOUT = 77;			/* we're doomed */
	byte status = 0;
	uint32_t T= millis() + TXTIMEOUT;
	do {
		status = read_register (STATUS);
		if (status & _BV(TX_DS)) {		/* SUCCESS, why wait */
			int obs = read_register (OBSERVE_TX);
			available();			/* to RX Mode */
			return (obs & 0x0f) + 1;	/* tries it took */
		}
		if (status & _BV(MAX_RT)) break;	/* max tries reached */
	} while (millis() < T);

/* TX failed. flush and return error code. */

	flush_tx();					/* tx failed, flush */
	available();					/* to RX mode */
	if (status & _BV(MAX_RT)) return -1;		/* RX not ack'ed */
	return (-1 & ~255) | status;			/* status in lower 8 */
}

/****************************************************************************/

/* status= available (); 

does two things; checks for a received-packet ready to read,
and polls Received Power Detector. 

returns 0 when there is no packet to read (RX FIFO empty), and 
non-zero when a packet can be read, with 1 meaning low received  
signal strength and 2 meaning high receive signal strength (RPD).  

RPD indicates received RF power in the receiver, up until a packet 
is actually received, at which time it becomes meaningless (rtfm). 
while awaiting a packet, RPD is read and stored; when (if) a 
packet arrives, we return the most-recent RPD. 

powerup and switch to RX Mode if necessary, possibly 
invoking the 130 uS state delay, with FIFOs cleared and all  
that; this occurs when the previous call was to write(). but 
in subsequent/successive available() or read() calls this will 
remain in Receive mode with no delay, and FIFOs and statii 
preserved, allowing for a bit of overlapping I/O i hope. 

FIXME: the Nordic datasheet 1.0 says to check for dynamic payload 
> 32 bytes (via R_RX_PL_WID) but that would need to be checked here 
in available(). don't feel like coding that now. and i thought this 
was all CRC'd? 

this appears to conform to what Appendix A suggests; 'set correct 
payload widths' not done, but that applies to fixed-payload-width 
which this driver does *not* use. */

int SRRF24::available(void) {

	powerUp();
	if (! (read_register (CONFIG) & _BV(PRIM_RX))) { /* to RX Mode */
		write_register_bits (CONFIG, _BV(PRIM_RX));
		if (selfAddress) setRXID();		/* we are PRX */
		ce (HIGH);

		delayMicroseconds (150);		/* RX Settling delay */
		recentRPD= 1;				/* unknown, assume low */
	}

/* if RX ready now, return most-recent RPD reading, 
else update it, return false. */

	if (read_register (STATUS) & _BV(RX_DR)) return recentRPD;

/* no packet to read, so RPD is valid, save it for later return. */

	recentRPD= read_register (RPD) ? 2 : 1;		/* 2 means hi rx power */
/* 	recentRPD= read_register (RPD) + 1;		// update RPD */
	return 0;					/* but not ready yet. */
}

/* n= read (buff, len); 

if a packet has been received, extract the RX FIFO data 
and copy up to len bytes into the passed buffer. the actual 
number of bytes read, n, is returned. if the payload contained  
more than len bytes, they were dropped.  

the radio packet size is 32 if fixed, if dynamic payloads 
enabled, then what the transmitter sent us, up to 32 max. 

n returns 0 if no packet yet received, and < 0 on errors. 

it may take multiple calls to read() to empty the FIFOs 
and clear RC status. */

int SRRF24::read (byte * buff, unsigned len) {

	if (! available()) return 0;			/* no packet, bye bye */

	if (len > 32) len= 32;

/* figure out how many bytes we need to pull from the FIFO. */

	unsigned d= 32;					/* max payload size */
	if (dynPayload)  {				/* if dynamic payloads */
		csn (LOW);
		SPI.transfer (R_RX_PL_WID);		/* use what radio sez */
		d= SPI.transfer (0xff);
		csn (HIGH);
	}
	if (d < len) len= d;				/* FIFO < buff len */

/* figure out how many bytes to drop to not overflow the buffer */

	d -= len;

/* extract the payload from the RX FIFO. */

	csn (LOW);
	SPI.transfer (R_RX_PAYLOAD);			/* extract RX FIFO bytes */
	unsigned n= len;
	while (n--) *buff++= SPI.transfer (0xFF);	/* delivered payload */
	while (d--) SPI.transfer (0xFF);		/* dropped payload */
	csn (HIGH);

/* if all of the RX FIFOs are empty, clear the RX_DR (data ready) status. */

	if (read_register (FIFO_STATUS) & _BV (RX_EMPTY))
		write_register(STATUS, _BV(RX_DR));

	return len;
}

/* set base address, then TX and RX ids for self-addressing. */

void SRRF24::setSelfAddress (uint64_t base, byte t, byte r) {

	setFixedAddress (base, base);			/* 64-bit pipe addresses */
	setSelfAddress (t, r);				/* then store IDs */
}

/* using current base address, set TX and RX IDs. */

void SRRF24::setSelfAddress (byte t, byte r) {

	PTXID= t;
	PRXID= r;
	selfAddress= true;
}

/* set up as PRX or PTX. these are called complementarily, 
in read() and write(). there is a fundamental problem when 
both ends transmit at the same time, or close in time; 
a very large (1 out of 100 or better) chance that 
a colliding transmit-payload packet will appear to the 
other sender as the auto-acknowledge packet, and register 
as a false acknowledge. due to re-transmission, the failure 
window is as large as TX time * ARD * ARC. 

however this suffices for reliable transmission in non- 
symmetrical situations, eg. one sender, one receiver, 
and is more than sufficient for the negotiation of 
PRX/PTX identities. */

/* uint64_t fake = 0xd6d6d6d6d6LL; */


void SRRF24::setRXID (void) {

	write_register (RX_ADDR_P1, PRXID);	/* receive */
	write_register (TX_ADDR,    PTXID);	/* transmit */
	write_register (RX_ADDR_P0, PTXID);	/* autoACK */
}

void SRRF24::setTXID (void) {

	write_register (RX_ADDR_P1, PTXID);	/* receive */
	write_register (TX_ADDR,    PRXID);	/* transmit */
	write_register (RX_ADDR_P0, PRXID);	/* autoACK */
}

/* set the fixed transmit address, the auto-ack address in pipe 0, and the 
receive address in pipe 1. must be called at least once before using 
self addressing, which modifies the LS byte of the address. */

void SRRF24::setFixedAddress (uint64_t t, uint64_t r) {

	write_register (TX_ADDR,    reinterpret_cast<byte*>(&t), 5);
	write_register (RX_ADDR_P0, reinterpret_cast<byte*>(&t), 5);
	write_register (RX_ADDR_P1, reinterpret_cast<byte*>(&r), 5);

	selfAddress= false;
}

/* enable/disable dynamic payloads (else packet size fixed at 32). if enabling, 
this tests for the correct Nordic nRF24L01+ part, by ensuring the dynamic 
stuff is actually enabled. 

there is a counterfeit part that has some ACK bit inverted but this doesn't 
test for that. i dont have one to test. 

this also always enables receive pipes P0 (the auto-ack rx pipe, even if 
auto-ack not used) and P1, our main rx pipe. */

bool SRRF24::setDynamicPayloads (bool enable) {

bool r;

	dynPayload= enable;
	if (dynPayload) {
		write_register (FEATURE, read_register (FEATURE) | _BV(EN_DPL));
		write_register (DYNPD, _BV(DPL_P1) | _BV(DPL_P0));

		/* now check that this actually worked, meaning this is actually */
		/* a Nordic nRF24L01+ chip. */

		r= (read_register (DYNPD) == (_BV(DPL_P1) | _BV(DPL_P0))) &&
		  ((read_register (FEATURE) & _BV(EN_DPL)) == _BV(EN_DPL));

	} else {
		write_register (FEATURE, read_register (FEATURE) & ~_BV(EN_DPL));
		write_register (RX_PW_P0, 32);
		write_register (RX_PW_P1, 32);
		r= true;
	}

	/* in any case, we enable pipes 0 and 1. */

	write_register (EN_RXADDR, read_register (EN_RXADDR) | _BV(ERX_P0) | _BV(ERX_P1));

	return r;
}

/* Nordic "hid" the dynamic payload features. haven't researched why.
dynamic payload must be "activated" but issuing the ACTIVATE command.

FIXME: no mention of this in official Nordic datasheet 1.0. 

http://www.nordicsemi.com/kor/layout/set/print/Nordic-FAQ/Silicon-Products/nSRRF24L01/What-previous-undocumented-features-became-documented-in-revision-2.0-of-the-nSRRF24L01-Product-Specification */

void SRRF24::activateHiddenFeatures (void) {

	csn(LOW);
	SPI.transfer (ACTIVATE);
	SPI.transfer (0x73);
	csn(HIGH);
}

/* globally enable/disable transmit auto-acknowledgement. 
we only use pipes 0 and 1. */

void SRRF24::setAutoAck (bool enable) {

	autoAck= enable;
	write_register (EN_AA, (enable ? B00000011 : 0));
}

/* set transmit power level: 0=lowest, 3=highest */

void SRRF24::setPALevel (byte n) {

#define RFPOWERBITS (_BV(RF_PWR_LOW)|_BV(RF_PWR_HIGH))

	n &= 3;
	n <<= RF_PWR_LOW;
	byte r= read_register (RF_SETUP) & ~RFPOWERBITS;
	write_register (RF_SETUP, r | n);
}

/* set radio data bit rate; 0=250K, 1=1MBPS, 2=2MBPS. */

void SRRF24::setDataRate (byte n) {

	byte r= read_register (RF_SETUP) & ~(_BV(RF_DR_HIGH) | _BV(RF_DR_LOW));
	switch (n) {
		case 0: r |= _BV(RF_DR_LOW); break;	/* low-rate bit (250K) */
		case 1: break;				/* both clear (1M) */
		case 2: r |= _BV(RF_DR_HIGH); break;	/* high-rate bit (2M) */
	}
	write_register (RF_SETUP, r);
}

/* set 16-bit CRC. */

void SRRF24::setCRCLength (void) {

	write_register (CONFIG, read_register (CONFIG) | _BV(EN_CRC) | _BV(CRCO));
}

/* set Auto Retransmission Delay, in n multiples of 250 uS; 0=250, 1=500, */
/* 2=750, 3=1000, up to 15=4000 uS. */

void SRRF24::setARD (byte n) {

	n &= 15;
	n <<= 4;					/* bits 7:4 */
	byte r= read_register (SETUP_RETR) & B00001111;
	write_register (SETUP_RETR, r | n);
}


/* set Auto Retransmission Count, n retries. 0 is no retries, 1, 2, 3, etc up */
/* to 15. */

void SRRF24::setARC (byte n) {

	n &= 15;					/* bits 3:0 */
	byte r= read_register (SETUP_RETR) & B11110000;
	write_register (SETUP_RETR, r | n);
}

/* set radio channel. */

void SRRF24::setChannel (byte channel) {

	write_register (RF_CH, channel & 127);
}


void SRRF24::flush_rx (void) {

	csn (LOW);
	SPI.transfer (FLUSH_RX);
	csn (HIGH);
}

void SRRF24::flush_tx (void) {

	csn (LOW);
	SPI.transfer (FLUSH_TX);
	csn (HIGH);
}

/* return the contents of a register. */

byte SRRF24::read_register (byte reg) {

	csn (LOW);
	SPI.transfer (R_REGISTER | (REGISTER_MASK & reg));
	byte result= SPI.transfer(0xff);
	csn (HIGH);
	return result;
}

/* write bytes to register. */

void SRRF24::write_register (byte reg, const byte* buf, unsigned len) {

	csn (LOW);
	SPI.transfer (W_REGISTER | (REGISTER_MASK & reg));
	while (len--) SPI.transfer (*buf++);
	csn (HIGH);
}

void SRRF24::write_register (byte reg, byte value) {

	csn (LOW);
	SPI.transfer (W_REGISTER | (REGISTER_MASK & reg));
	SPI.transfer (value);
	csn (HIGH);
}

/* OR new bits into a register. */

void SRRF24::write_register_bits (byte reg, byte bits) {

	write_register (reg, read_register (reg) | bits);
}

#endif /* __SRRF24_H__ */

