This is the repository for a homebrew GPS Rocket Tracker built around Arduino feather M0 units with embedded LORA radios. Observed successful range over flat territory (when the rocket is laying on the ground) is 1.2km. Longer range has been achieved when the target is airborne, but I'm not sure yet what the maximum is. It likely depends on the antenna.

Comments/improvements/etc. welcome. Please email:

gregory.a.huber@outlook.com with the subject "Arduino Lora Tracker"

==============================

This project requires two units:
(1) A transmitter, the unit that goes in the rocket, which sends its GPS information every second
(2) A receiver/base unit, which you hold onto, which tracks its own GPS location, receives information from the transmitter, and can route you to the transmitter.

Manifest:

Parts lists for each unit, available from AdaFruit, are in the PartsList folder

Shape files (stls) for the case that holds the receiver are in the STLFilesBaseUnitCase folder

Photos of the assembled transmitter and receiver are in the Photos folder

Code: 

Feather_GPS_Receiver_publicversion.ino in the folder "Feather_GPS_Receiver_publicversion" is the code for the receiver unit.

Feather_GPS_Transmitter_publicversion.ino in the folder "Feather_GPS_Transmitter_publicversion" is the code for the transmitter unit.

==============================

Brief overview of functionality:

The transmitter and receiver need to share a frequency (probably 915mhz, but in the US it can be anything in the range of 902 to 928 MHz if your hardware supports it. See https://www.everythingrf.com/community/lora-frequency-bands-in-north-america) and a 2 digit hex "secret code." The 2 digit code allows the receiver to know what unit it should be talking to.

*** You must set the Mhz and secret code to be the same between your transmitter and receiver units, and should change them from the defaults so you don't conflict with someone else. Even with different codes, units operating on the same frequency can conflict.

You upload the relevant code, using the Arduino IDE, to both the transmitter and the receiver/base unit. 

In the transmitter codebase, you have to set one flag:
#define UseDisplay false
is the default, which means the unit doesn't have a display. If set to true, the transmitter also displays its own address.

Details about functionality

The receiver has three information screens, selected by pressing the top (A), middle (B), or bottom (C) button on the display.

Screen A displays information about the transmiter:
A <Pk> flag appears whenever a new packet is received from the transmitter.
The receiver's status, "R: {F=Valid current own GPS Fix;SF(#)=Stale fix, # of seconds since last valid own GPS Fix;NF=Never had a valid own GPS Fix}"
The transmitter's status, "T: {F=Valid current target GPS Fix;SF(#)=Stale fix, # of seconds since last valid target GPS Fix;Never Fix=Never had a valid target GPS Fix; Never Sync=Never received a packet from target}"
If the receiver has ever gotten a valid fix from the transmitter, it will display that Lat., Long., and Alt.

Screen B (default) displays information about the receiver (base unit):
The receiver's status, "R: {F=Valid current own GPS Fix;SF(#)=Stale fix, # of seconds since last valid own GPS Fix;NF=Never had a valid own GPS Fix}"
If the receiver has ever gotten a valid own fix, it will display its Lat., Long., Alt., Speed, and Bearing.

Screen C is the navigation screens:
It displays a <Pk> flag appears whenever a new packet is received from the transmitter.
If either the receiver or transmitter information is stale, it will show (RS) or (TS) as appropriate.
Assuming it has valid ever fixes for the transmitter and receiver, it displays the distance and bearing from the transmitter to the receiver and the RSSI (signal strength) of the last packet. It will walk you to your rocket.
If is missing its own fix (Never own fix), the transmitter's fix (Never target fix), or both fixes (Never either fix) it will display that error.
