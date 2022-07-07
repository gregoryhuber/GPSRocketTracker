https://github.com/gregoryhuber/GPSRocketTracker

This is the repository for a homebrew GPS Rocket Tracker built around Arduino feather M0 units with embedded LORA radios. Observed successful range over flat territory (when the rocket is laying on the ground) is 1.2km. Longer range has been achieved when the target is airborne, but I'm not sure yet what the maximum is. It likely depends on the antenna.

Comments/improvements/etc. welcome. Please email:

gregory.a.huber@outlook.com with the subject "Arduino Lora Tracker"

==============================

This project requires two units:
(1) A transmitter, the unit that goes in the rocket, which sends its GPS information every second
(2) A receiver/base unit, which you hold onto, which tracks its own GPS location, receives information from the transmitter, and can route you to the transmitter in certain configurations.

Manifest:

Parts lists for each unit, available from AdaFruit, are in the PartsList folder

Shape files (stls) are in the STLfiles folder. There are separate folders for the case that holds the receiver/base unit and transmitter in the two stacked vertical feathers orientation that fits in a 24mm body tube.

Photos of the assembled transmitter and receiver are in the Photos folder.

(Note that there are not photos of the transmitter with an SD Card unit included, but that you can use an SD card on the transmitter.)

Code: 

Recevier.ino in the folder "Receiver" is the code for the receiver/base unit.

Transmitter.ino in the folder "Transmitter" is the code for the transmitter unit.

==============================

Brief overview of code edits/decisions:

The transmitter and receiver need to share a frequency (probably 915mhz, but in the US it can be anything in the range of 902 to 928 MHz if your hardware supports it. See https://www.everythingrf.com/community/lora-frequency-bands-in-north-america) and a 3 digit "MagicNumber." The 3 digit code allows the receiver to know what unit it should be talking to.

See these lines in both files:
#define RF95_FREQ 915.00
#define MagicNumber 123
*** You must set the Mhz and MagicNumber to be the same between your transmitter and receiver units, and should change them from the defaults so you don't conflict with someone else. Even with different codes, units operating on the same frequency may conflict.

You upload the relevant code, using the Arduino IDE, to both the transmitter and the receiver/base unit. 

In the TRANSMITTER codebase, you have to set three flags:

#define UseRadio true
Only turn this to false if you just want to display or capture to the SD Card GPS data0

#define UseSDCardWriter false
You can turn this to true if you install an SD card module and want to record/log data to it.

#define UseDisplay false
Only turn this to true if you have a display installed and you want the transmitter to display its own address.

In the RECEIVER codebase, you have to set two flags

#define UseDisplay true
If you don't have a display (and are only teathered to a PC) you can set to false.

#define PrintDataToSerialForLogging true
This outputs the data received to the serial port, so you can plot it or save it in real time.

PLEASE NOTE: There are also numerous debugging flags in the code, which you can use to diagnose problems (like bad wiring to the GPS or SDCard module/etc.). Leave them as FALSE unless you need to fix things. If you leave them on and try to record data from the receiver on a PC it will screw up the data stream.

==============================

Details about functionality

The receiver has three information screens, selected by pressing the top (A), middle (B), or bottom (C) button on the display.

Screen A displays information about the transmitter.
Screen B displays information about the base station.
Screen C is a navigation screen that guides you from the base to the transmitter.

On all three screens, the top row is the same:

{A,B,C}]*B:XX T:XX P=XX
Where A/B/C is which screen you are on.
The * appears whenever a new packet is received from the transmitter
B: XX indicates the status of the base's GPS fix: F=Valid current Fix;SF(#)=Stale fix, # of seconds since last valid GPS Fix;NF=Never had a valid GPS Fix
T: XX indicates the status of the transmitter's GPS fix: F=Valid current Fix;SF(#)=Stale fix, # of seconds since last valid GPS Fix;NF=Never had a valid GPS Fix;NS Never synced with transmitter
P = XX is the signal strength/RSSI, if synced, of last packed received from transmitter

On Screen A, the next rows will display a summary of the TRANSMITTER's GPS data:
Never Target Sync
or
Never Target Fix + the Transmitter's read battery voltage
or
Lat, Long, Altitude (in meters), Velocity (MPH)+ the Transmitter's read battery voltage

On Screen B, the next rows will display a summary of the BASE's GPS data:
Never Target Fix + the BASE's read battery voltage
or
Lat, Long, Altitude (in meters), Velocity (MPH), Bearing (Degrees) + the Transmitter's read battery voltage

On Screen C, the next rows display
"Can't navigate. Missing something." if either the TRANSMITTER or BASE GPS is missing
or
Distance (feet) and Bearing (degrees) from the BASE to the transmitter
Plus Own bearing, Own Battery Voltage, and Transmitter Battery Voltage
