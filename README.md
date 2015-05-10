###NRF24L01+ driver
#####description and API documentation

#####tom jennings
#####04 march 2015

######REVISION HISTORY

29 mar 2015 corrections to API (pins are specified
in begin). please see the RADIO ADDRESSING section.

20 mar 2015 renamed directories, files and classes
to accommodate the new consistent naming scheme.

04 mar 2015 version 3: expanded radio addressing
(and again changed the API) to accomodate
both fixed (5-byte pipe address for TX, RX) and 
self-addressing (the scheme introduced in version 2).

27 feb 2015 version 2: fully automated (invisible) 
radio addressing.  the 'pipe' addressing scheme of 
the Nordic chip has been automated and effectively 
disappeared. addresses can be set via the API 
if desired.

24 feb 2015 first release.

####DESCRIPTION

the simple API belies the capabilities (and limitations) of this chip.
this low-cost chip can send/receive small (1 - 32 byte)
packets serially in 2400 MHz "wifi" band. though the 
defaults are thorough enough to support usage identical 
to the Arduino Serial object, the API allows more or 
less full control of the radio and will support
rapid switching of up to 128 channels, variable
output power, receive signal strength indicator,
variable packet size, switchable radio speed and bandwidth.
the API supports a not-yet-released frequency-hopping
scheme.

########notes on use of the Nordic NRF24L01+

by default this driver uses Dynamic Payloads, and
therefore the non-PLUS version of the chip is not
supported. packet size can vary from 1 to the hardware
maximum of 32. the transmitter determines actual
packet size. the receiver can chose to read the entire
packet or only a portion. see read() for details.

Auto-Acknowledge is on by default. it can be turned
off if necessary (eg. broadcast type sends).

i deemed Nordic's "Multiceiver" feature to be too
application-specific, so this driver supports only
one receive and one transmit address ("pipe").


####RADIO ADDRESSING

the NRF24L01+ was designed for embedded, "headless" use in
products like wireless keyboards, etc, and specifically not
for use as a general-purpose data radio. this exhibits one 
important limitation: the internal "pipe" addressing.

each receiver and transmitter is required to have an address;
a packet transmitted by radio A, must contain an address 
already programmed into radio B. in other words, you must pre-arrange
for each end of a link to have a unique address. 

this driver overcomes the need for programming addresses, by
using a pair of addresses that are exchanged (swapped) when transmitting
or receiving. this works because when radio A is sending, you can reasonably
assume radio B is listening; and if not, radio protocols that acknowledge
receipt of packets ensures reception. however there is a limitation to this
that is built-in to the NRF24L01+ that cannot be overcome in software,
and that is the way Auto-Acknowledgement is handled. the problem occurs
when both radio A and radio B are transmitting simultaneously, as often
happens in symmetrical, peer-to-peer protocols.

the problem is that Auto-Acknowledgement causes each radio's receiver to
listen for a packet -- any valid packet -- that contains it's own address.
and when both radios are using the same transmit address, "acknowledge" happens
when (1) a true Auto-Acknowledgement packet is received, or (2) an
unexpected, transmitted packet from the other radio is received. the NRF24L01+
cannot tell these two situations apart, by itself.

the short answer is -- in practice this isn't as bad as it sounds, and for
"most" or many applications, the advantages of self-addressing vastly outweigh
this disadvantage.

for cases where you have an "asymmetrical" relationship, where one radio mostly
receives or mostly sends, you will encounter no problems, and the self-addressing
removes the need to configure each radio setup uniquely. even for cases where you have
many radios sharing the same channel -- eg. a "flock" of sensors (sending) or
actuators (receiving) packets, self-addressing works fine.

for peer to peer, then the only reliable choice with the NRF24L01+ is to simply not
use the Auto-Acknowledgement feature, and explicitly acknowledge packets. the
Flock protocol driver uses this. this entirely eliminates the false-ack problem.

of course, if you configure each radio for a unique address, as actually
recommended by the manufacturer (lol) then Auto-Ack works under all circumstances.
this driver supports that very simply and robustly; the "pipe address" has been
rationalized into "BASE ADDRESS" and separate "RX" and "TX" addresses, which
the driver assembles into the "pipe" scheme used by the chip. this is simply an
API view of the chip internals, but it makes for much easier address allocation.


####RF PERFORMANCE

you will find that these radios are very susceptible to their RF environment.
in other words they are lousy radios; but instead of complaining that they
talk to each other poorly, be thankful that a tiny $1/each postage stamp 
can send 2-megabit/sec data at all. WYDSIWYG; the antennas are terrible, power
is low, no grounds, etc. S.N.A.F.U. hence the built-in autoAck and retry
functions.
  
NOTE: some channels can be so busy/noisy as to make it seem  like this
radio isn't working. try different channels.



this driver code was originally derived from J. Coliz' 
NRF24L01 Arduino library, for which i'm eternally grateful.
it was the nicest rational basis i found for achieving 
my own ends.


####NRF24L01+ SR driver API documentation

you talk to this chip with SPI (library provided with Arduino).
the NRF24L01+ board connects to Arduino via five wires (in addition
to power and ground):

```
//             Uno      Mega 2560     TM4C123XL (SPI #2)
// 3.3V                             J1-1
// GND                              J2-1
// CE            9         9                      as desired
// CSN (SS)     10        10        J2-9 (PA_3)   as desired
// SCK          13        52        J1-7          required by the host
// MOSI         11        51        J2-6          required by the host
// MISO         12        50        J2-7          required by the host
```

SCK, MISO, MOSI pin numbers are predestined; consult the appropriate library
documentation but the above seemed correct as of this writing.

you select CE and CSN pins. my personal choices are listed above. ymmv.


####INVOKATION/INCANTATION

```
// instantiate the driver object.
//
#include <SRRF24.h>
SRRF24 radio;
```



####FUNCTION REFERENCE


```
	void begin (CE_pin, CSN_pin);				// 
```

initialize the driver object. this sets up the driver, initializes the
chip, sets default settings, etc. there is no return value and no indication that
you've wired the thing wrong and that it is now on fire.

the settings are now:

```
	channel 80
	2 MBPS data bit rate
	lowest transmitter power
	auto-acknowledge ENABLED
	dynamic payloads ENABLED
	16-bit CRC
	transmission retry delay 500 uS
	retry 8 times on transmission error
```

receive and transmit addresses ARE NOT SET and they must be set
before you can use the radio.



```
	void powerDown ();
```

puts the chip in Power Down Mode. the chip is now quiescent and will
not receive packets. 

any call to read(), available(), write(), will power on the chip as necessary
before performing it's function. the chip will remain powered up until explicitly
powered down with this function.



```
	int status= write (buffer, length);
```

transmits a packet containing the contents of the given buffer as payload, returns
transmission status. 

buffer is assumed to be an array of uint8_t's, length from 1 to 32 (the hardware
maximum). if dynamic payloads are enabled (default) 'length' bytes will be sent.
if fixed payload size, always 32 bytes are sent, and any shortfall in 
'length' will be padded with 0x55's. under no circumstances will
more than 32 bytes be sent.

immediately upon completion (success or failure) the chip is put into RX Mode
(you don't need to know this) so that it will accept any incoming packets. the
assumption is that you are alternately sending and receiving... if your 
application is send-only (eg. a remote sensor or whatnot) then invoke 
powerDown() to put the chip in power saving mode.

positive return values (1..16) are the number of packet-sends it took to actually
deliver the packet; assuming auto-acknowledge is enabled (default) then this is
the number of transmissions it took before the packet was acknowledged. the 
maximum returned-successfully value is the current transmit-retry count (see
setARC()). if auto-acknowledge is disabled, then write() returns 1.

negative return values indicate errors; -1 means the packet was not acknowledged
(assuming auto-acknowledge is enabled (default)), -2 means the driver code is 
buggy or the chip died. other values (less than -500) are actually the contents 
of the STATUS register in the lower byte and all 1's in the upper byte.

packet-send-time is somewhat complicated; rtfm the datasheet. very roughly speaking,
transmission time is well under 1 millisecond when it sends without a retry. each retry
takes an additional delay, called ARD, which defaults to half a millisecond; so again
very roughly, assume an additional millisecond for each retry. ARD (auto retry delay)
can be changed with the setARD() function.

strictly speaking, write() "blocks", eg. internally it loops until transmission
is completed. however, each transmit phase is very brief (see above). average
worst case is when auto-acknowledge is enabled and the packet is not received
(also, see above), worst worst case is when code or hardware fails, then write()
times out, approximately 37mS, but in that case you're screwed anyway so who cares.
at least i checked.



```
	int status= available();
```

available() does a number of things; puts the chip into RX Mode,
if it isn't already, checks for a received-packet ready to read,
and polls the Received Power Detector (RPD). 

returns 0 when there is no packet to read (RX FIFO empty) and
returns a non-zero number indicating signal strength when a packet is
actually received. this value can be ignored, and available() tested
for zero/non-zero, in which case it works like Serial.available() and
other such functions.

however RPD gives a vague idea of how strong the radio signal was. 
the idea is that if the signal is "strong" you can tell the transmitter 
that it can use a lower power setting to transmit. time will tell if 
this feetch works out as planned... 


```
	int n= read (buffer, length);
```

if a packet has been by the chip (it will do so asynchronously when
in RX Mode), extract the payload from the chip and store it in the 
given buffer.  see below for discussion on buffer and payload length.

the return value is 0 when there is actually no packet to receive
(eg. it internally calls available() to determine this) otherwise it
is the number of bytes actualy copied into the buffer (eg. there
was a packet in the radio, and now you have it).

PAYLOAD LENGTH: the buffer given should be large enough to hold
'length' bytes of payload data. Bad Things will happen if length
is larger than the actual buffer size. this is entirely Your Fault
when your program misbehaves. the largest possible payload is
32 bytes under any circumstance, so a 32 uint8_t array always works.

read() will return the smallest of: the actual payload size, or length.
if length == 32 but there are only 11 bytes in the payload, read()
returns 11. if length == 32 and there are 32 bytes in the payload, 
read() returns 32. the above assertion assumes that dynamic payloads
are enabled (default).

this works even if fixed payloads are used. with fixed payload size,
the radios are always flinging 32-byte packets around, though you can 
specify a shorter length for both write() and read(). but with fixed
payload size YOU, NOT THE API! needs to keep track payload size -- 
while it is possible for the sender to send 11 bytes, and the receiver
to read 32, 21 of those bytes will be "filler" added by the sender.

confused? don't be. either use dynamic payloads (default) or hard-code
the payload size at each end. generally 32 works.

why would you ever want to use shorter payloads? because it makes the
on-air radio time shorter, less exposure to interference and packet
loss and it speeds things up too. these radios suck, and need all the
help they can get.



```
	void setDynamicPayloads (bool enable);
```

enable/disable dynamic payloads. 
see the discussions under write() and read() for details. the default is enabled.


```
	void setAutoAck (bool enable);
```

enable/disable automatic acknowledgement of transmitted (write()) packets.
see the discussions under write() and read() for details. the default is enabled.

```
	void setPALevel (uint8_t n);
```
sets transmit output power, from low to microscopic. 0 is lowest, 3 is highest.
lower power obviously will consume less energy.


```
	void setDataRate (uint8_t n);
```
sets the radio data bit rate: 0=250K bits/sec, 1=1M bits/sec, 2=2M bits/sec.
the bit rate choice affects the available channel selection: at 1M bits/sec
and below, there are 128 channels, each using 1 MHz of radio bandwidth. 
at 2M bits/sec each channel requires 2 MHz of radio bandwidth, so only
every-other channel can be used. 


```
	void  setARD (uint8_t n);
```
set Auto Retransmission Delay, in n multiples of 250 uS; 0=250, 1=500,
2=750, 3=1000, up to 15=4000 uS. for a two-radio peer-to-peer link this
should be set to 1 (500 uS). the main reason that this option exists is to
allow forcing multiple transmitters attempting to talk to one receiver
to stagger their re-transmissions in case of packet collision. this is a
non-trivial setting, best left alone unless you feel like reading the
datasheet on the subject. 



```
	void  setARC (uint8_t n);
```
sets the Automatic Retry Count, for re-transmission of unacknowledged
write() packets, to n. the default is 8, for no particular reason. 
the maximum is 15, and the minimum can be 0 (no retries). the practical
difference between 8 and 15 seems nil, generally speaking, but 15 takes
more time. ymmv. 

there are times when you want to disable auto-acknowledge, eg. "broadcast"
packets. for that case it's better to do the broadcasts with setAutoAck (false)
but functionally it probably doesn't matter.

```
	void setChannel (uint8_t channel);
```

sets the radio channel number to talk to the other radio in. this sets
both the receive and transmit frequencies simultaneously. i hope it is
abundantly obvious that both the transmitter and receiver must be using
the same channel.

the bit rate choice affects the available channel selection: at 1M bits/sec
and below, there are 125 channels, each using 1 MHz of radio bandwidth. 
at 2M bits/sec each channel requires 2 MHz of radio bandwidth. 

note that 2400 MHz is the very busy and noisy ISM band, occupied by
wifi, microwave ovens, etc and you will find that some channels are so
noisy as to be unusable (and make it seem like your radio isn't working).
try different channels. the original code's default channel 76 was unusable
here in my lab, but 80 is quiet. ymmv.

the radio frequency is 2400 MHz + (channel * channel width). 



```
	void setSelfAddress (uint8_t txid, uint8_t rxid);
	void setSelfAddress (uint32_t baseAddress, uint8_t txid, uint8_t rxid);
```

the NRF24L01+'s proprietary radio packet addressing scheme is now
handled automatically; the default settings are fine for nearly any
application. 

the chip's 5-byte address has been defined as two components here: 
a 4-byte base address, and a one-byte ID for each direction, send and
receive: TXID and RXID. the one-byte ID is appended to the 4-byte
base address to create the pipe address and TX address.

Nordic designates one end of a link PTX (transmitter) and the other end
PRX (receiver), and assigns a pair of 5-byte addresses to each end;
call those A and B. the end designated PTX uses A as transmit address and
B as receive; the PRX does the opposite. this fairly inconvenient for
general-purpose bidirectional radio communication, especially 
with multiple receivers and/or transmitters. this feature was intended
for very-low-overhead dedicated applications suck as wireless keyboards
and mice.

my scheme sets the 4-byte base address portion in all three address
registers (TX_ADDR, RX_ADDR_P0, RX_ADDR_P1), and assigns the LS byte
of those three to either TXID (transmitting, eg. write()) or RXID (receiving,
eg. read()). a nice feature of the chip allows for setting only the LS
byte of the address. the reasonable assumption is that when one end
is sending, it implies (requires!) that the other end is receiving.
done and done.

even though base address and ID bytes allow for any thinkable combination
of addresses, i can see no real use for it at this time.


########base address limitations

there are some physical limitations on base addresses and IDs.
radio serial clock recovery precludes long runs of bits without
transitions; base addresses like 00000, FFFFF, etc are likely to
increase error rates. ensure a 1/0 transition or two per byte 
if possible.

########ID limitations

ID values 0, 1, 254 and 255 are reserved. please do not use these.
the default base address is 0xe7e7e7e7. the default TXID is 2,
the default RXID is 3. the full 5-byte address is assembled from
base address and ID; the upper 4 bytes are base address the 
lower byte is ID.

```
	void setFixedAddress (uint64_t txAddr, uint64_t rxAddr);
```

this sets a fixed address for transmitting and receiving, as per
most everyone else's addressing schemes for this chip. this is in
essence the manufacturer's recommendation. however, each
end of a link must be told which it is, and each must be
the complement of the other. for example:

```
  if (recv) {
    Serial.println ("i am the receiver");
    radio.setFixedAddress (0xe7e7e7e7e7LL, 0xc3c3c3c3c3LL);
    
  } else {
    Serial.println ("i am the transmitter");
    radio.setFixedAddress (0xc3c3c3c3c3LL, 0xe7e7e7e7e7LL);
  }
```

-- end





