This is the repository for a homebrew GPS Rocket Tracker built around Arduino feather M0 units with embedded LORA radios. Observed successful range over flat territory (when the rocket is laying on the ground) is 1.2km. Longer range has been achieved when the target is airborne, but I'm not sure yet what the maximum is. It likely depends on hardware (antenna).

Comments/improvements/etc. welcome. Please email:

gregory.a.huber@outlook.com with the subject "Arduino Lora Tracker"

==============================

This project requires two units:
(1) A transmitter, the unit that goes in the rocket, which sends its GPS information every second
(2) A receiver/base unit, which you hold onto, which tracks its own GPS location and receives information from the transmitter.

Manifest:

Parts lists for each unit, available from AdaFruit, are in the PartsList folder

Shape files (stls) for the case that holds the receiver are in the STLFilesBaseUnitCase folder

Photos of the assembled transmitter and receiver are in the Photos folder

Code: Feather_PairedGPSSendTrack_publicversion.ino

==============================

Brief overview of functionality:

The transmitter and receiver need to share a frequency (probably 915mhz) and a 2 digit hex "secret code." The 2 digit code allows the receiver to know what unit it should be talking to.

*** You must set the Mhz and secret code to be the same between your units, and should change them from the defaults so you don't conflict with someone else

You upload the code, using the Arduino IDE, to both the transmitter and the receiver/base unit. 
You have to set two flags in the code:

#define UseDisplay false
must be set to true for the receiver, or else it cannot display any info. Use of a display is optional for the transmitter.

#define ReceiveNotTransmitLocation false
must be set to true for the receiver and false for the transmitter

Details about functionality

The receiver has three information screens, selected by pressing the top (A), middle (B), or bottom (C) button on the display.

Screen A displays:
A <Pk> flag appears whenever a new packet is received from the transmitter.
The receiver's status, "R: {F=Valid current own GPS Fix;SF(#)=# of seconds since last valid own GPS Fix;NF=Never had a valid own GPS Fix}"
The transmitter's status, "T: {F=Valid current target GPS Fix;SF(#)=# of seconds since last valid target GPS Fix;Never Fix=Never had a valid target GPS Fix; Never Sync=Never received a packet from target}"
If the receiver has ever gotten a valid fix from the transmitter, it will display that Lat., Long., and Alt.

Screen B (default) displays
The receiver's status, "R: {F=Valid current own GPS Fix;SF(#)=# of seconds since last valid own GPS Fix;NF=Never had a valid own GPS Fix}"
If the receiver has ever gotten a valid own fix, it will display its Lat., Long., and Alt., Speed, and Bearing

Screen C is the navigation screens. It displays
A <Pk> flag appears whenever a new packet is received from the transmitter.
If either the receiver or transmitter information is stale, it will show (RS) or (TS) as appropriate
Assuming it has valid ever fixes for the transmitter and receiver, it displays the distance and bearing from the transmitter to the receiver and the RSSI (signal strength) of the last packet. It will walk you to your rocket.
If is missing its own fix (Missing own fix) or both fixes (Missing both fix) it will display that error. It will also display (Missing both fix) if it is just missing the target.
