This is firmware for a TTGO T-Beam device, to make it into a HAB LoRa receiver.

The receiver can be connected to a phone or tablet or PC, using Bluetooth or BLE (Bluetooth Low Power) or USB Serial so in order to view the received telemetry, upload to habhub etc.  It supports multiple simulateneous connections.

See my github account and blog for suitable host software for Windows, Mac, iPad, iPhone, Android phone or tablet.



Serial Protocol
===============

The same protocol is used over USB, Bluetooth and BLE.  It accepts commands of the form

something=value<CR><LF>

The somethings are, currently:

	- CurrentRSSI=<RSSI>
	- Message=<telemetry>
	- Hex=<hex packet e.g. SSDV>
	- FreqErr=<error in kHz>
	- PacketRSSI=<RSSI>
	- PacketSNR=<SNR>

The serial interface accepts a few commands, each of the form

~<command><value><CR>

(a trailing <LF> can be sent but is ignored).  Accepted commands are responded to with an OK (* <CR> <LF>) and rejected commands (unknown command, or invalid command value) with a WTF (? <CR> <LF>)

The current commands are:

	- F<frequency in MHz>
	- M<mode>
	- B<bandwidth>
	- E<error coding from 5 to 8>
	- S<spreading factor from 6 to 11>
	- I<1=implicit mode, 0=explicit mode>
	- L(low data rate optimisation: 1=enable, 0=disable)
	
The supported modes are:

0 = (normal for telemetry)  Explicit mode, Error coding 4:8, Bandwidth 20.8kHz, SF 11, Low data rate optimize on
1 = (normal for SSDV)       Implicit mode, Error coding 4:5, Bandwidth 20.8kHz,  SF 6, Low data rate optimize off
2 = (normal for repeater)   Explicit mode, Error coding 4:8, Bandwidth 62.5kHz,  SF 8, Low data rate optimize off	

Bandwidth value strings can be 7K8, 10K4, 15K6, 20K8, 31K25, 41K7, 62K5, 125K, 250K, 500K

History
=======

23/09/2016	V1.1	- Added Hex=... message for any packet that is not ASCII telemetry
					- Added LoRa modes 3-7K8
					- Increased baud rate to 57,600 (from 9,600) so handle high LoRa data rates esp. with SSDV
					
30/06/2016	V1.0	- First version
