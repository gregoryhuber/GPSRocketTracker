//////////////////////////////////////////////////////////////////////////
// RFM95 Lora Arduino GPS tracking system
// Version 1.2
// 2021/08/18
// Gregory Huber (gregory.a.huber@outlook.com)
//
// This is the code for the RECEIVER
//
//////////////////////////////////////////////////////////////////////////

// This is the codebase for an Arduino feather-based GPS tracking system suitable for rocket tracking
// The physical setup is comprised of a base station (receiver) and a tracker (transmitter)
// For the tracker/transmitter, the display unit is optional, configured using UseDisplay

// The base station is built around the following minimal equipment:
// Adafruit Feather M0 with RFM95 LoRa Radio - 900MHz – RadioFruit https://www.adafruit.com/product/3178
// Feather GPS receiver https://www.adafruit.com/product/3133
// Feather Display Unit/Control Buttons (with headers) https://www.adafruit.com/product/3045
// FeatherWing Triple (base unit) https://www.adafruit.com/product/3417  
// uFL SMT Connector https://www.adafruit.com/product/1661
// 900 Mhz Antenna https://www.adafruit.com/product/3340
// 2000mAh LIPO battery https://www.adafruit.com/product/2011

// The (smallest) tracking unit is built around an the following equipment:
// Adafruit Feather M0 with RFM95 LoRa Radio - 900MHz – RadioFruit https://www.adafruit.com/product/3178
// GPS receiver https://www.adafruit.com/product/746
// Quarter Wavelength Wire antenna (3.12inches) http://www.csgnetwork.com/antennagenericfreqlencalc.html
// 400mah LIPO battery https://www.adafruit.com/product/3898

// This code draws on the following resouces and references. Thank you.
// This is a fully fleshed out example from which many of the transmission details were copied https://github.com/mloudon/electrosew/blob/master/feather_send/feather_send.ino
// https://gis.stackexchange.com/questions/252672/calculate-bearing-between-two-decimal-gps-coordinates-arduino-c
// https://community.particle.io/t/tinygps-using-distancebetween/28233/3
// http://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF95.html
// https://www.thethingsnetwork.org/forum/t/best-practices-when-sending-gps-location-data/1242/40

#include <Adafruit_GPS.h>
#include <math.h>;
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RH_RF95.h>

//////////////////////////////////////////////////////////
// DebugDebugging mode
// Enable each of these flags to enable diffent parts of the DebugDebugging code
#define DebuggingButtons false
#define DebuggingDisplay false
#define DebuggingOwnGPS false
#define DebuggingTargetGPS false
#define DebuggingRadio false

// for target GPS, either grab via radio or set to fixed static location for testing purposes
#define TargetGPSUseStatic false
// New Haven Green
#define TargetStaticLat 41.308200 
#define TargetStaticLong -72.926100
#define TargetStaticAlt 100

//////////////////////////////////////////////////////////
// Read battery voltage
// For reasons I don't understand, reading the battery voltage causes the remaining code to fail. It is disabled below.
#define VBATPIN A7

//////////////////////////////////////////////////////////
// Setup GPS

// what's the name of the hardware serial port for the GPS?
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

//////////////////////////////////////////////////////////
// Setup OLED display

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);
  
// OLED FeatherWing buttons map to different pins depending on board:
#define BUTTON_A  9
#define BUTTON_B  6
#define BUTTON_C  5

//////////////////////////////////////////////////////////
// Setup LORA Radio
#define LED 13

#define RFM95_CS      8
#define RFM95_INT     3
#define RFM95_RST     4

// This is important. This defines the frequency the units will use. Both base and transmitter need to be on the same frequency
// I do not not know if you can provide more specificity here (say 915.2)
#define RF95_FREQ 915.0

// The magic number is a unique two digit code that is used to verify the pair between tracker and base unit
// Please change it to something different form this so that your tracker doesn't conflict with someone else's!
#define MAGIC_NUMBER_LEN 2
// Magic numbers are two hex digits; I've set this to 2,13 for now, but change it to something else!
const uint8_t MAGIC_NUMBER[MAGIC_NUMBER_LEN] = {0x02, 0x0D};

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

//////////////////////////////////////////////////////////
// set update intervals for getting own and target GPS locations and displaying to OLED
// These are in milliseconds, so 1000 = 1 second
#define OwnGPSUpdateInterval 1000
#define TargetGPSUpdateInterval 1000
#define DisplayUpdateInterval 750
#define ButtonUpdateInterval 500

// Whether ever received a valid GPS fix
bool OwnEverValidGPS = false;
bool TargetEverSync = false;
bool TargetEverValidGPS = false;

// Timers used to figure out last target GPS update, last local GPS update, and Last display
uint32_t LastOwnLocationUpdate = millis();
uint32_t LastTargetLocationUpdate= millis();
uint32_t LastDisplayUpdate = millis();
uint32_t LastButtonUpdate = millis();
uint32_t LastSendViaRadio = millis();

// Storage for GPS info
double OwnLat, OwnLong;
int OwnGSPLastQuality, OwnAlt, OwnSpeed, OwnBearing;
uint32_t OwnLastUpdateMillis, OwnLastUpdateTime;
bool OwnStaleLocation = false;

double TargetLat, TargetLong;
int TargetAlt;
uint32_t TargetLastUpdateMillis, TargetLastUpdateTime;
bool TargetStaleLocation = false;
bool DisplayNewPacketFlag = false;

// store last button pushed on diplay, default to 'B'
char LastButtonPushed = 'B';

void setup()
{

  pinMode(LED, OUTPUT);

  //////////////////////////////////////////////////////////////////////////
  // Prepare serial display
  // connect to the serial terminal at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);

  //////////////////////////////////////////////////////////////////////////
  // Prepare OLED display
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
	display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32

	// Setup parameters for talking to OLED featherwing with buttons
	pinMode(BUTTON_A, INPUT_PULLUP);
	pinMode(BUTTON_B, INPUT_PULLUP);
	pinMode(BUTTON_C, INPUT_PULLUP);

	// Show image buffer on the display hardware.
	// Since the buffer is intialized with an Adafruit splashscreen
	// internally, this will display the splashscreen.
	display.display();
	delay(500);

	// Clear the buffer.
	display.clearDisplay();
	display.display();

	// text display tests
	display.setTextSize(1);
	display.setTextColor(WHITE);
	display.setCursor(0,0);
	display.println("Init. GPS Receiver");
	display.print("Freq.: ");
	display.println(RF95_FREQ);
	display.print("Magic Num.: ");
	display.print(MAGIC_NUMBER[0]);
	display.print(",");
	display.println(MAGIC_NUMBER[1]);
	display.print("Be patient...\n");
	display.setCursor(0,0);
	display.display(); // actually display all of the above
	delay(5000);
  
  //////////////////////////////////////////////////////////////////////////
  // Setup GPS, including parameters for talking to GPS
  
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
     
  // Request updates on antenna status, comment out to keep quiet
  //GPS.sendCommand(PGCMD_ANTENNA);

  delay(5000);
  
  // Ask for firmware version
  //GPSSerial.println(PMTK_Q_RELEASE);

  //////////////////////////////////////////////////////////////////////////
  // Setup LORA Radio

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
	  display.clearDisplay();
	  display.print("LoRa radio init failed");
	  display.display();
	  while (1);
  }

  if (!rf95.setFrequency(RF95_FREQ)) {
	  display.clearDisplay();
	  display.print("setFrequency failed");
	  display.display();
	  while (1);
  }

  // This is a very useful post for understanding the tradeoffs in LORA settings: https://forum.arduino.cc/t/what-is-lora/595381
  // Reference documents for LORA chipset are here: https://www.semtech.com/products/wireless-rf/lora-core/sx1276
  
  // See https://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF95.html#af82a441a5946b538f60f31e727009750
  // Search for ModemConfigChoice
  // Bw31_25Cr48Sf512 is listed as Bw = 31.25 kHz, Cr = 4/8, Sf = 512chipschips/symbol, CRC on. Slow+long range. 
  // This has been used successfully for up to 1.6KM communication.
  
  // Trying instead this slightly larger spreading factor to increase range.
  // This configuration is:
  //   31.25KHZ BW
  //   4/8 coding rate
  //   Spreading factor 10 (1024)
  //   and the last parameter (0x04) turns the AGC on, because this packet takes a bit to send and we are worried about doppler shift.
  //RH_RF95::ModemConfig myconfig =  {RH_RF95_BW_31_25KHZ | RH_RF95_CODING_RATE_4_8, RH_RF95_SPREADING_FACTOR_1024CPS, 0x04};
  //rf95.setModemRegisters(&myconfig);
  
  // Use commented out code if using preset per above
  if (!rf95.setModemConfig(RH_RF95::Bw31_25Cr48Sf512 )) {
    display.clearDisplay();
    display.print("setModemConfig failed");
    display.display();
    while (1);
  }
   
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
  // rf95.printRegisters(); 
}

///////////////////////////////////////////////////////////////////////////////////////////
//
// Begin code bundle edited from other person's tracker

// These are additional declarations. I'm not sure what they do but they relate to the radio
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
int lastRSSI;

// processRecv processes an incoming packet
// Returns true if packet is valid and matches magic number, else returns false
bool processRecv() {

  bool TempStaleGPS;
  bool TempEverGPSFix;
  int32_t TempRecLat;
  int32_t TempRecLong;
  int32_t TempAlt;
    
  // verify magic number matches. If this fails, return false
  for (int i = 0; i < MAGIC_NUMBER_LEN; i++) {
    if (MAGIC_NUMBER[i] != buf[i]) {
      return(false);
    }
  }

  // If we get this far, we know we've gotten a packet
  TargetEverSync = true;

  // Now take apart packet into its pieces
  // This is stored in global variables
  void* p = buf + MAGIC_NUMBER_LEN;
  TempEverGPSFix= *(bool*)p;
  p = (bool*)p + 1;
  TempStaleGPS= *(bool*)p;
  p = (bool*)p + 1;
  TempRecLat = *(int32_t*)p;
  p = (int32_t*)p + 1;
  TempRecLong = *(int32_t*)p;
  p = (int32_t*)p + 1;
  TempAlt = *(int32_t*)p;

  // Has the transmitter ever gotten a valid fix? If not, simply return true that we've gotten a packet
  if (TempEverGPSFix==0) {
      return(true);
  }
  // If we get here, we know we've ever gotten a valid GPS signal. Set flag
  TargetEverValidGPS = true;

  // If we've gotten a non-stale GPS packet, update LastTargetLocationUpdate time and TargetStaleLocation flag
  if (TempStaleGPS==0) {
    LastTargetLocationUpdate=millis();
    TargetStaleLocation=false;
  }   

  // You might ask why are we processing a stale GPS signal from the tranceiver? Well suppose the tracker loses power and you reboot it
  // You want to make sure that your tracker will parse the stale GPS coordinates, because it is still better than nothing
  
  // Note that this force division to be in doubles to recover decimal correctly
  TargetLat=(double)TempRecLat/100000;
  TargetLong=(double)TempRecLong/100000;
  TargetAlt=TempAlt;
  
  if (DebuggingRadio) {
    Serial.print("Received data: ");
    Serial.print(TempStaleGPS);
    Serial.print(",");
    Serial.print(TempRecLat);
    Serial.print(",");
    Serial.print(TempRecLong);
    Serial.print(",");
    Serial.println(TempAlt);
    Serial.print("Decoded data: ");
    Serial.print(TargetLat,6);
    Serial.print(",");
    Serial.print(TargetLong,6);
    Serial.print(",");
    Serial.println(TargetAlt,6);    
  }  
  return(true);
}

// Both the distanceBetween and courtTo functions are from TinyGPS++ library, copied entirely and edited to return in feet rather than meters
double distanceBetween(double lat1, double long1, double lat2, double long2)
{
  // My edit is to return distance in feet, rather than meters
  
  // returns distance in meters between two positions, both specified
  // as signed decimal-degrees latitude and longitude. Uses great-circle
  // distance computation for hypothetical sphere of radius 6372795 meters.
  // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
  // Courtesy of Maarten Lamers
  double delta = radians(long1-long2);
  double sdlong = sin(delta);
  double cdlong = cos(delta);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double slat1 = sin(lat1);
  double clat1 = cos(lat1);
  double slat2 = sin(lat2);
  double clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
  delta = sq(delta);
  delta += sq(clat2 * sdlong);
  delta = sqrt(delta);
  double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
  delta = atan2(delta, denom);
  // multipling by 3.281 converts meters to feet
  return delta * 6372795*3.281;
}

double courseTo(double lat1, double long1, double lat2, double long2)
{
  // returns course in degrees (North=0, West=270) from position 1 to position 2,
  // both specified as signed decimal-degrees latitude and longitude.
  // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
  // Courtesy of Maarten Lamers
  double dlon = radians(long2-long1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double a1 = sin(dlon) * cos(lat2);
  double a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  return degrees(a2);
}

// This is the main loop which runs over and over
void loop() // run over and over again
{

  //////////////////////////////////////////////////////////////////////////
  // in receive mode, grab any packet that has come over the radio
    if (rf95.available()) {
      uint8_t len = sizeof(buf);
      if (rf95.recv(buf, &len)) {
        lastRSSI = rf95.lastRssi();
        digitalWrite(LED, HIGH);
        digitalWrite(LED, LOW);
        if (processRecv()) {
          TargetLastUpdateMillis=millis();
          DisplayNewPacketFlag=true;
          if (DebuggingRadio) {
            Serial.print("Last RSSI: ");
            Serial.println(lastRSSI);
            Serial.print("Decoded data: ");
            Serial.print(TargetEverValidGPS);
            Serial.print(",");
            Serial.print(TargetStaleLocation);
            Serial.print(",");
            Serial.print(TargetLat);
            Serial.print(",");
            Serial.print(TargetLong);
            Serial.print(",");
            Serial.println(TargetAlt);
          }  
        }
        else if (DebuggingRadio) {
          Serial.println("Unable to parse received packet");
        }
      }
    if ((LastTargetLocationUpdate+30000)<millis()) TargetStaleLocation = true;
  } 
  
  //////////////////////////////////////////////////////////////////////////
  // Query Button Status every ButtonUpdateInterval
    if ((LastButtonUpdate+ButtonUpdateInterval)<millis()) {
  
      LastButtonUpdate=millis();
     
      // Record which button is being pushed. Note if multiple buttons, records C
      if(!digitalRead(BUTTON_C)) {
        LastButtonPushed = 'C';
      }
      else if(!digitalRead(BUTTON_B)) {
        LastButtonPushed = 'B';
      }
      else if(!digitalRead(BUTTON_A)) {
        LastButtonPushed = 'A';
      }
      
      // In Debugging mode, serial print last button pushed
      if (DebuggingButtons) {
        Serial.print("Last Button Pushed: ");
        Serial.println(LastButtonPushed);
      }
    
    }

  //////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////////
  // If Receiving, check whether using Static GPS for testing.
    if ((TargetGPSUseStatic==true)) {
      TargetEverValidGPS = true;
      TargetLat = TargetStaticLat;
      TargetLong = TargetStaticLong;
      TargetAlt = TargetStaticAlt;
      if (DebuggingTargetGPS) {
        delay(10000);
        Serial.print("Set Static Target GPS Lat/Long/Alt: ");
        Serial.print(TargetLat,6);
        Serial.print(", ");
        Serial.print(TargetLong,6);
        Serial.print(", ");
        Serial.println(TargetAlt);
       }
    }
  //////////////////////////////////////////////////////////////////////////


  //////////////////////////////////////////////////////////////////////////
  // Update OLED display screen every ButtonUpdateInterval
    if ((LastDisplayUpdate+DisplayUpdateInterval )<millis()) {
  
    LastDisplayUpdate=millis();

    display.clearDisplay();
    display.display();
    display.setCursor(0,0);
	  
	// This is what gets displayed if this is a RECEIVER with a display

       // In Debugging mode, serial print which loop triggered
        if (DebuggingDisplay) {
          Serial.print("Last Button Pushed in OLED Loop: ");
          Serial.println(LastButtonPushed);
        }
                 
        // Button A is just basic status
        if (LastButtonPushed == 'A') {
		      //No longer printing battery voltage because it causes unit to reboot. Don't know why.
          //float  measuredvbat  =  (analogRead ( VBATPIN ));
          //measuredvbat *= 2;
          //measuredvbat *= 3.3;
          //measuredvbat /= 1024;
          display.print("A:");
          //display.println(measuredvbat);
          if (DisplayNewPacketFlag) {
            display.print("<Pk>");
            DisplayNewPacketFlag=false;
          }
          display.print(" ");

          //Display receiver status
          display.print("R:");
          if(OwnEverValidGPS & !OwnStaleLocation) {
            display.print("F");
          } else if(OwnEverValidGPS) {
            display.print("SF(" + String(min(99,int((millis()-LastOwnLocationUpdate)/1000))) + ")");          
          } else {
            display.print("NF");
          }

          //Display target status
          display.print(" T:");
          if(TargetEverSync) {
           
            if (TargetEverValidGPS & !TargetStaleLocation) {
              display.print("F\n");
              }
            if (TargetEverValidGPS & TargetStaleLocation) {
              display.print("SF(" + String(min(99,int((millis()-LastTargetLocationUpdate)/1000))) + ")\n");          
             }
            
            //If we have ever had a valid fix to the target, print Lat/Long/Alt
            if (TargetEverValidGPS) {
              display.print(String("TLt: " +  String(int( TargetLat  ) , DEC ) + "." + String( int(abs(round( ((TargetLat -int(TargetLat)) * 100000)))), DEC ) + "\n"));
              display.print(String("TLg: " + String(int( TargetLong ) , DEC ) + "." + String( int(abs(round( ((TargetLong-int(TargetLong)) * 100000)))), DEC ) + "\n"));
              display.print(String("TAlt: " + String(int( TargetAlt ) , DEC ) + "\n"));
            }           
            if (!TargetEverValidGPS) {
              display.print("NeverFix\n");
            }
            } else {
              display.print("NeverSync\n");
          }       
        }
        // Button B is own receiver location
        else if (LastButtonPushed == 'B') {
          display.print("B:");
          if (DisplayNewPacketFlag) {
            display.print("<Pk> ");
            DisplayNewPacketFlag=false;
          }
          display.print(" ");
          if(OwnEverValidGPS) {
            if (OwnStaleLocation) {
              display.print("SF(" + String(min(99,int((millis()-LastOwnLocationUpdate)/1000))) + ") ");          
             }
            display.print(String("RLt: " +  String(int( OwnLat  ) , DEC ) + "." + String( int(abs(round( ((OwnLat -int(OwnLat)) * 100000)))), DEC ) + "\n"));
            display.print(String("RLg: " + String(int( OwnLong ) , DEC ) + "." + String( int(abs(round( ((OwnLong-int(OwnLong)) * 100000)))), DEC ) + "\n"));
            display.print(String("RAlt: " + String(int( OwnAlt ) , DEC )));
            display.print(String(" | Spd: " + String(OwnSpeed, DEC)));
            display.print(String(" | Bear: " + String(OwnBearing, DEC)) + "\n");
          }
          else {
               display.print("Never GPS Fix");
          }
        }
        // If not A or B, assume it is C
        else {
          display.print("C:");
          if (DisplayNewPacketFlag) {
            display.print("<Pk>");
            DisplayNewPacketFlag=false;
          }
          display.print(" ");
          
          if(OwnEverValidGPS & TargetEverValidGPS) {
            if (OwnStaleLocation) display.print("(RS) ");
            if (TargetStaleLocation) display.print("(TS) ");
            display.print("\n");
            display.print("RSSI=");
            display.println(lastRSSI);
            display.print(String("Distance: " + String(round(distanceBetween(OwnLat, OwnLong, TargetLat, TargetLong)), 0)) + " ft.\n");
            display.print(String("Bearing: " + String(round(courseTo(OwnLat, OwnLong, TargetLat, TargetLong)), 0)) + " deg.\n");
          }
          else if(!OwnEverValidGPS & !TargetEverValidGPS) {
            display.print("\n");
            display.println("Never either fix");
            if (TargetEverSync) {
             display.print("RSSI=");
             display.println(lastRSSI);             
            }
          }
          else if (TargetEverValidGPS) {
            display.print("\n");
            display.println("Never own fix");
            display.print("RSSI=");
            display.println(lastRSSI);
          }
          else {
            display.print("\n");
            display.println("Never target fix");
            if (TargetEverSync) {
             display.print("RSSI=");
             display.println(lastRSSI);             
            }
          }
        }
     
      display.display(); // actually display all of the above
  
    }

  //////////////////////////////////////////////////////////////////////////
  
  //////////////////////////////////////////////////////////////////////////
  // Read GPS data. This is unedited code from the Arduino example
  char c = GPS.read();
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
    //if (DebuggingOwnGPS) {
    //  Serial.print("Last Latitude from GPS: ");
    //  Serial.println(GPS.latitudeDegrees);
    //}
  }

  //////////////////////////////////////////////////////////////////////////
  // Update Own GPS calculations every OwnGPSUpdateInterval
  if ((LastOwnLocationUpdate+OwnGPSUpdateInterval )<millis()) {

    LastOwnLocationUpdate=millis();

    // If never gotten valid own GPS fix, update flag
    if (!OwnEverValidGPS) OwnEverValidGPS = GPS.fix;

    if (GPS.fix) {
       OwnLat =  GPS.latitudeDegrees;
       OwnLong = GPS.longitudeDegrees;
       // translating to feet
       OwnAlt = GPS.altitude*3.28084;
       OwnSpeed = GPS.speed;
       OwnBearing = GPS.angle;
       OwnLastUpdateTime = GPS.hour*10000+GPS.minute*100+GPS.seconds;
       OwnLastUpdateMillis = millis();
       OwnStaleLocation = false;
    }

    // If no GPS update in last 60 seconds, declare own location stale and set flag
    if ((OwnLastUpdateMillis+60000)<millis()) OwnStaleLocation = true;

    if (DebuggingOwnGPS) {
      Serial.print("Time: ");
      Serial.print(GPS.hour, DEC); Serial.print(':');
      Serial.print(GPS.minute, DEC); Serial.print(':');
      Serial.print(GPS.seconds, DEC); Serial.print('.');
      Serial.println(GPS.milliseconds);
      Serial.print("Date: ");
      Serial.print(GPS.day, DEC); Serial.print('/');
      Serial.print(GPS.month, DEC); Serial.print("/20");
      Serial.println(GPS.year, DEC);
      Serial.print("Fix: "); Serial.print((int)GPS.fix);
      Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
      if (GPS.fix) {
        Serial.print("Location: ");
        //Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
        //Serial.print(", ");
        //Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
        //Serial.print("Converted internally: ");
        Serial.print(GPS.latitudeDegrees,6);
        Serial.print(", ");
        Serial.println(GPS.longitudeDegrees,6);  
        Serial.print("Altitude: "); Serial.println(OwnAlt);
        Serial.print("Speed (knots): "); Serial.println(GPS.speed);
        Serial.print("Angle: "); Serial.println(GPS.angle);
        Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      }
        if (OwnStaleLocation) Serial.print("(Own Stale!) ");
        Serial.print("\n");
    }
  }

  // if millis() or timer wraps around, we'll just reset it
  if (LastOwnLocationUpdate > millis()) {
    LastOwnLocationUpdate = millis();
    LastDisplayUpdate = millis();
    LastButtonUpdate= millis();
    LastSendViaRadio= millis();
    OwnLastUpdateMillis=millis();
  }
}
