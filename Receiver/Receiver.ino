//////////////////////////////////////////////////////////////////////////
// RFM95 Lora Arduino GPS tracking system
// Version 2.1
// 2022/07/01
// Gregory Huber (gregory.a.huber@outlook.com)
//
// This is the code for the RECEIVER
//
//////////////////////////////////////////////////////////////////////////

// This is the receiver half of the codebase for an Arduino feather-based GPS tracking system suitable for rocket tracking
// The physical setup is comprised of a base station (receiver) and a tracker (transmitter)

// The base station is built around the following minimal equipment:
// Adafruit Feather M0 with RFM95 LoRa Radio - 900MHz – RadioFruit https://www.adafruit.com/product/3178

// The following parts are optional if you just want to capture data from the transmitter directly to a PC

// Necessary to dispay incoming GPS data and to use free navigation (without connection to a PC)
// Feather Display Unit/Control Buttons (with headers) https://www.adafruit.com/product/3045
// Set UseDisplay below to true to use

// Necessary to use free navigation (without connection to a PC)
// Feather GPS receiver https://www.adafruit.com/product/3133
// Even if no GPS is present, code will still run

// These are hardware options to hold everything as well as a good antenna option. I find soldering the uFL connector hard so I tend to use simple wire antennas
// FeatherWing Triple (base unit) https://www.adafruit.com/product/3417  
// uFL SMT Connector https://www.adafruit.com/product/1661
// 900 Mhz Antenna https://www.adafruit.com/product/3340
// 2000mAh LIPO battery https://www.adafruit.com/product/2011

// This code draws on the following resouces and references. Thank you.
// This is a fully fleshed out example from which many of the transmission details were copied https://github.com/mloudon/electrosew/blob/master/feather_send/feather_send.ino
// https://gis.stackexchange.com/questions/252672/calculate-bearing-between-two-decimal-gps-coordinates-arduino-c
// https://community.particle.io/t/tinygps-using-distancebetween/28233/3
// http://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF95.html
// https://www.thethingsnetwork.org/forum/t/best-practices-when-sending-gps-location-data/1242/40

// Version 2.5 (2022/07):
// Introduced data structures, which allowed much more efficiently sending more data in the LORA packet
// On the transmitter:
    // Read and send battery voltage
    // Added support for a data logger
    // Refined display code to fix what is displayed on transmitter
// On the receiver/base unit:
    // Implement serial logging to connect to PC
    // Improve option for running without display
    // Read and display battery voltages
    // Improved handling of stale packets/stale GPS data
    // More data/info now displayed on screen

#include <Adafruit_GPS.h>
#include <math.h>;
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RH_RF95.h>

////////////////////////////////////////////////////////////////////////////////////////////
//
// Radio constants
//
////////////////////////////////////////////////////////////////////////////////////////////
  
  // This is important. This defines the frequency the units will use. Both base and transmitter need to be on the same frequency
  // LORA frequency range in the US is 902 to 928 MHz https://www.everythingrf.com/community/lora-frequency-bands-in-north-america
  // I do not not know if you can provide more specificity here (say 915.2)
  #define RF95_FREQ 915
  
  // The magic number is a unique three digit code that is used to verify the pair between tracker and base unit.
  // Valid ranges are 000 to 999
  // If these numbers don't match, any incoming packet is ignored
  // Please change it to something different form this so that your tracker doesn't conflict with someone else's!
  #define MagicNumber 123

////////////////////////////////////////////////////////////////////////////////////////////
//
// Hardware setup options
//
////////////////////////////////////////////////////////////////////////////////////////////

  #define UseDisplay true

////////////////////////////////////////////////////////////////////////////////////////////
//
// Connection to PC options
//
////////////////////////////////////////////////////////////////////////////////////////////

  // For real time-tracking using PC, enable printing of target GPS to serial port for plotting
  #define PrintDataToSerialForLogging true
  // Note that if this is enabled, other debugging flags below should be set to false 
  // or else reading serial port will likely crash

////////////////////////////////////////////////////////////////////////////////////////////
//
// Timing constants
//
////////////////////////////////////////////////////////////////////////////////////////////

  // Update intervals for getting own GPS locations
  // These are in milliseconds, so 1000 = 1 second
  #define OwnGPSUpdateInterval 500
  #define DisplayUpdateInterval 250
  #define ButtonUpdateInterval 500
  #define SerialPortUpdateInterval 500

////////////////////////////////////////////////////////////////////////////////////////////
//
// Debugging
//
////////////////////////////////////////////////////////////////////////////////////////////

  // Enable each of these flags to enable diffent parts of the DebugDebugging code
  #define DebuggingOwnGPS false        // DebuggingOwnGPS prints to serial own gps data
  #define DebuggingIncoming false      // DebuggingIncoming prints to serial the incoming packet data
  #define DebuggingOwnRadio false      // DebuggingOwnRadio prints to serial configuration for the LORA radio

  // Fake data used for static GPS testing
  // Lat/Long is the New Haven, CT Green. Other data is made up.
  #define UseStaticData false
  #define StaticLat 41.308231
  #define StaticLong -72.926173
  #define StaticBearing 317
  #define StaticVelocity 987
  #define StaticAlt 3611
  #define StaticNumSatelites 8
  #define StaticSecondsSinceUpdate 27
  #define StaticEverValidFix 1
  #define StaticVBAT 4.10  

////////////////////////////////////////////////////////////////////////////////////////////
//
// Define pins/hardware
//
////////////////////////////////////////////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////////////////////////
  // Battery voltage
  // Battery voltage is measured on A7 on the M0
  #define VBATPIN A7

  //////////////////////////////////////////////////////////////////////////////////////////
  // GPS
  // hardware serial port for the GPS
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

  //////////////////////////////////////////////////////////////////////////////////////////
  // Setup LORA Radio
  #define LED           13
  #define RFM95_CS      8
  #define RFM95_INT     3
  #define RFM95_RST     4

  // Create singleton instance of the radio driver
  RH_RF95 rf95(RFM95_CS, RFM95_INT);

////////////////////////////////////////////////////////////////////////////////////////////
//
// Structures
//
////////////////////////////////////////////////////////////////////////////////////////////

  // uint8_t is 0 to 255
  // uint16_t is 0 to 65535 
  // uint32_t 0 to 4,294,967,295
  
  // This is the structure we use to store our own GPS data at all times
  struct __attribute__ ((packed)) processed_gps_t {
    bool evervalidfix;              // whether we've ever had a valid gps fix
    uint32_t lastupdatemillis;      // last gps update in millis format
    uint32_t lastupdatetime;        // last gps update in hhmmss integer format
    uint32_t secondsincelastupdate; // last gps update in millis format
    bool stalelocation;             // whether last gps update was more than 60 seconds ago (1=yes)
    
    int numsatellites;              // number of satellites in last fix
  
    double latitude;          // in decimal format
    double longitude;         // in decimal format
    int32_t altitude_meters;  // in meters
    uint32_t velocity_mph;    // in mph
    uint16_t  bearing;        // in degrees (0-360)
    
    double bvolt;             // battery voltage [Range 3.0 to 4.2]
  } ;
  
  // declaring a new type "radiopacket_gps_t" as a structure for the info we want to send/receive
  // there is a fair bit of being creative here to keep the packet as small as is possible
  struct __attribute__ ((packed)) radiopacket_gps_t {
  
    //This is 2-bit unsigned integer, which ranges from 0 to 65535 
    uint16_t vbat_code;               // 2 digits are vbat * 10 and 3 digits are unique code for pairing
    
    //Thse are 4-bit unsigned integers, which ranges in value from 0 to 4,294,967,295 = 10 digits to play with
    //With 5 digits of decimal precision we still have two additional place for storing other data
    uint32_t bear_veloc_alt;          // format is XXX bearing, XXX velocity capped at 999, and XXXX altitude in meters +100 capped at 9999
    uint32_t latitude_sats_ever;      // lat with +180 to ensure positive and 5 digits precision uses 8 digits, 1 digit for # satellites seen, 1 digit for whether ever valid fix
    uint32_t longitude_sincelastfix;  // long with +180 to ensure positive and 5 digits precision uses 8 digits, 2 digits for seconds since last fix, capped at 99
  } ;
  
////////////////////////////////////////////////////////////////////////////////////////////
//
// Globabl Variables
//
////////////////////////////////////////////////////////////////////////////////////////////

  // Own GPS Data
  processed_gps_t OwnGPSData = {
    .evervalidfix=0,
    .lastupdatemillis=0,
    .lastupdatetime=0,
    .secondsincelastupdate=0,
    .stalelocation=0,
    .numsatellites=0,
    .latitude=0,
    .longitude=0,
    .altitude_meters=0,
    .velocity_mph=0,
    .bearing=0,
    .bvolt=0
  };

  // Own GPS Data
  processed_gps_t TargetGPSData = {
    .evervalidfix=0,
    .lastupdatemillis=0,
    .lastupdatetime=0,
    .secondsincelastupdate=0,
    .stalelocation=0,
    .numsatellites=0,
    .latitude=0,
    .longitude=0,
    .altitude_meters=0,
    .velocity_mph=0,
    .bearing=0,
    .bvolt=0
  };

  // Radio packet to receive
  radiopacket_gps_t SentDataToProcess;

  // Whether a new valid GPS location has arrived that needs to be parsed/stored
  boolean NewValidGPSDataArrived = 0;
  boolean BaseEverValidGPS = 0;  

  // Information about Target
  boolean TargetEverSync = 0;
  boolean TargetEverValidGPS = 0;  
  boolean TargetStaleLocation = 0;
  boolean GotNewLoraPacket = 0;
  boolean DisplayNewPacketFlag = 0;
  uint16_t SecondsSinceIncomingPacket = 0;
  uint16_t SecondsSinceIncomingGPS = 0;
  
  // Last RSSI from radio
  int lastRSSI;

  // store last button pushed on diplay, default to 'A'
  char LastButtonPushed = 'A';

  // Counter for packets in logfile

  // Timers
  uint32_t LastFixFromOwnGPS = 0;         // Last millis for which we got a Valid GPS Fix
  uint32_t LastUpdateOwnGPSPacket = 0;    // Last millis at which we updated the GPS Packet
  uint32_t LastLoraPacket = 0;            // Last millis for which we got a Valid GPS Fix
  uint32_t LastDisplayUpdate = 0;         // Last millis at which display was updated.
  uint32_t LastButtonUpdate = 0;          // Last millis at which button status was updated.
  uint32_t LastSerialPortUpdate = 0;      // Last millis at which sent logging packet to serial port.
  uint32_t EndLoopMillis = 0;             // Last millis at which we finished the main loop.
                                          // Used solely to deal with looping in Millis.
 
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

  //////////////////////////////////////////////////////////////////////////////////////////
  // Define LED as output
  pinMode(LED, OUTPUT);

  //////////////////////////////////////////////////////////////////////////
  // connect to the serial terminal at 115200 so we can read the GPS fast enough and echo without dropping chars
  Serial.begin(115200);

  //////////////////////////////////////////////////////////////////////////
  // Setup GPS, including parameters for talking to GPS
  
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  
  // Set the update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
     
  // Request updates on antenna status, comment out to keep quiet
  //GPS.sendCommand(PGCMD_ANTENNA);
  
  // Ask for firmware version
  //GPSSerial.println(PMTK_Q_RELEASE);

  if (UseDisplay) {
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
    display.println("Init. Receiver");
    display.print("Freq.: ");
    display.println(RF95_FREQ);
    display.print("Magic Num.: ");
    display.println(MagicNumber);
    display.print("Be patient...\n");
    display.setCursor(0,0);
    display.display(); // actually display all of the above
  }

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
    while (1);
  }

  if (!rf95.setFrequency(RF95_FREQ)) {
    while (1);
  }

  // This is a very useful post for understanding the tradeoffs in LORA settings: https://forum.arduino.cc/t/what-is-lora/595381
  // Reference documents for LORA chipset are here: https://www.semtech.com/products/wireless-rf/lora-core/sx1276
  
  // See https://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF95.html#ab9605810c11c025758ea91b2813666e3
  // Search for ModemConfigChoice
  
  // Bw125Cr45Sf128   
  // Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range.
  // This is used by other people and is MUCH faster
  
  // Bw500Cr45Sf128  
  // Bw = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range.

  // Bw31_25Cr48Sf512  
  // Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range.
  // This has been used successfully for up to 1.6KM communication.

  // Bw125Cr48Sf4096   
  // Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, low data rate, CRC on. Slow+long range.

  // Bw125Cr45Sf2048   
  // Bw = 125 kHz, Cr = 4/5, Sf = 2048chips/symbol, CRC on. Slow+long range. 

  // Note that this must match between transmitter and receiver
  if (!rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128 )) {
    while (1);
  }
   
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  // Waiting 5 seconds for LORA to boot
  delay(5000);

  if (DebuggingOwnRadio) {
    rf95.printRegisters();
  } 

}

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
// receiveLoraPacket receives and processes an incoming LORA packet
bool receiveLoraPacket() {

  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  // These are temporary variables to store data while decoding
  bool evervalidfix;              // whether we've ever had a valid gps fix
  uint32_t secondsincelastupdate; // last gps update in millis format
  int numsatellites;              // number of satellites in last fix
  double latitude;          // in decimal format
  double longitude;         // in decimal format
  int32_t altitude_meters;  // in meters
  uint32_t velocity_mph;    // in mph
  uint16_t  bearing;        // in degrees (0-360)
  double bvolt;             // battery voltage [Range 3.0 to 4.2]
  int code;
  
  // get the message
  if (rf95.recv(buf, &len)) {
    if (len >= sizeof(radiopacket_gps_t)) {
      
      // Copy the received buffer values directly into our Observation data structure
      memcpy(&SentDataToProcess, buf, sizeof(radiopacket_gps_t));
      bvolt=int(SentDataToProcess.vbat_code/1000);
      code = SentDataToProcess.vbat_code - bvolt*1000;
      bvolt=bvolt/10;
      
      if (DebuggingIncoming) {
          Serial.println("\nDebugging Income Radio. Got Incoming Packet-->");
          Serial.print("     VVCCC:      ");
          Serial.println(SentDataToProcess.vbat_code);
          Serial.print("BBBVVVAAAA: ");
          Serial.println(SentDataToProcess.bear_veloc_alt);
          Serial.print("LLLLLLLL#E: ");
          Serial.println(SentDataToProcess.latitude_sats_ever);
          Serial.print("LLLLLLLLSS: ");
          Serial.println(SentDataToProcess.longitude_sincelastfix);
      }

      // Only proceed with the rest of this if the magic codes match
      if (code==MagicNumber) {

        // At this point we know we've gotten a valid packet that matches the magic number

        bearing = int(SentDataToProcess.bear_veloc_alt/10000000);
        velocity_mph = int((SentDataToProcess.bear_veloc_alt-(bearing*10000000))/10000);
        altitude_meters = (SentDataToProcess.bear_veloc_alt - (bearing*10000000) - (velocity_mph*10000)) - 100;
  
        latitude = int(SentDataToProcess.latitude_sats_ever/100);
        numsatellites  = int((SentDataToProcess.latitude_sats_ever - (latitude*100))/10);
        evervalidfix = SentDataToProcess.latitude_sats_ever - (latitude*100) - (numsatellites*10);
        latitude = (latitude/100000) - 180;
  
        longitude = int(SentDataToProcess.longitude_sincelastfix/100);
        secondsincelastupdate = SentDataToProcess.longitude_sincelastfix - (longitude*100);
        longitude = (longitude/100000) - 180;

        TargetGPSData.bvolt=bvolt;
       
        if (evervalidfix==1) {
          TargetGPSData.evervalidfix=evervalidfix;
          TargetGPSData.bearing=bearing;
          TargetGPSData.velocity_mph=velocity_mph;
          TargetGPSData.altitude_meters=altitude_meters;
          TargetGPSData.latitude=latitude;
          TargetGPSData.numsatellites=numsatellites;
          TargetGPSData.longitude=longitude;
          TargetGPSData.lastupdatemillis=millis();
        }
        TargetGPSData.secondsincelastupdate=secondsincelastupdate;

        if (DebuggingIncoming) {
          Serial.print("\Decoded Data-> ");
          Serial.print("Vbat: ");
          Serial.print(bvolt);
          Serial.print(" Magic Number: ");
          Serial.println(code);
        
          Serial.print("Bearing: ");
          Serial.print(bearing);
          Serial.print(" Velocity: ");
          Serial.print(velocity_mph);
          Serial.print(" Altitude: ");
          Serial.println(altitude_meters);
          
          Serial.print("Latitude: ");
          Serial.print(latitude, 6);
          Serial.print(" Longitude: ");
          Serial.println(longitude, 6);
          
          Serial.print("Ever Fix: ");
          Serial.print(evervalidfix);
          Serial.print(" #Satelites: ");
          Serial.print(numsatellites);
          Serial.print(" Seconds since GPS update: ");
          Serial.println(secondsincelastupdate);
        }
        return(true);
      }
    }
  }
  return(false);
}

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
// The distanceBetween function is from the TinyGPS++ library, copied entirely and edited to return in feet rather than meters
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

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
// The courseTo function is from the TinyGPS++ library, copied entirely
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

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
// Main Loop, runs over and over
void loop() {

  //////////////////////////////////////////////////////////////////////////
  // Each time through the loop, check for a LORA packet
  //
  
  // Turn on communication to LORA. Keeping it off the rest of the time stops this from interfering with other parts of code.
  digitalWrite (RFM95_CS, LOW);
  
  if (rf95.available()) {
    GotNewLoraPacket = receiveLoraPacket();
    if (GotNewLoraPacket) {
      TargetEverSync = true;
      if (TargetGPSData.evervalidfix==1) {
        TargetEverValidGPS = true;
      }
      lastRSSI = rf95.lastRssi();
      LastLoraPacket=millis();
      DisplayNewPacketFlag=true;
    }
  }
  
  // Turn off communication to LORA. Keeping it off stops this from interfering with other parts of code.
  digitalWrite (RFM95_CS, HIGH);

  //////////////////////////////////////////////////////////////////////////
  // Each time through the loop, attempt to read GPS data.
  // This code doesn't try to read from GPS unless it is available.
  // Additionally, if no new data is available, it just skips through.
  if (GPSSerial.available() > 0) {
    char c = GPS.read();
    if (GPS.newNMEAreceived()) {
      if (GPS.parse(GPS.lastNMEA())) {
        if (GPS.fix==1) {
          NewValidGPSDataArrived = 1;
          BaseEverValidGPS = 1;
          LastFixFromOwnGPS = millis();
        }
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////
  // Update Own GPS calculations every OwnGPSUpdateInterval
  if ((LastUpdateOwnGPSPacket+OwnGPSUpdateInterval )<millis()) {
    LastUpdateOwnGPSPacket=millis();

    // This block of conditional code just sets the GPS data to either the fixed stata data for testing purposes or the actual GPS data
    if (UseStaticData) {
      // Fixed data for testing purpuses
      OwnGPSData.evervalidfix=StaticEverValidFix;
      OwnGPSData.lastupdatemillis=millis();
      OwnGPSData.lastupdatetime=234233;
      OwnGPSData.secondsincelastupdate=(OwnGPSData.lastupdatemillis-(millis()-(1000*StaticSecondsSinceUpdate)))/1000;
      OwnGPSData.stalelocation=(OwnGPSData.secondsincelastupdate>60);
      OwnGPSData.numsatellites=StaticNumSatelites;
      OwnGPSData.latitude=StaticLat;
      OwnGPSData.longitude=StaticLong;
      OwnGPSData.altitude_meters=StaticAlt;
      OwnGPSData.velocity_mph=StaticVelocity;
      OwnGPSData.bearing=StaticBearing;
      OwnGPSData.bvolt = StaticVBAT;
    }
    else {
      // Actual data
      OwnGPSData.bvolt = (2*3.3*analogRead(VBATPIN))/1024;
      //OwnGPSData.bvolt = 4.2;
      // NewValidGPSDataArrived: This flag lets us know if in the intervening period a new GPS packet with a valid fix has been parsed
      if (NewValidGPSDataArrived) {
        // Do this if new data has arrived
        NewValidGPSDataArrived = 0;
        OwnGPSData.evervalidfix=1;
        OwnGPSData.lastupdatemillis=LastFixFromOwnGPS;
        OwnGPSData.lastupdatetime=GPS.hour*10000+GPS.minute*100+GPS.seconds;
        OwnGPSData.secondsincelastupdate=0;
        OwnGPSData.numsatellites=(int)GPS.satellites;
        OwnGPSData.latitude=GPS.latitudeDegrees;
        OwnGPSData.longitude=GPS.longitudeDegrees;
        OwnGPSData.altitude_meters=GPS.altitude;
        OwnGPSData.velocity_mph=GPS.speed*1.15078;
        OwnGPSData.bearing=(int)GPS.angle;
        OwnGPSData.stalelocation=0;
      }
      else {
        // Do this if new data has NOT arrived
        OwnGPSData.secondsincelastupdate=(millis()-LastFixFromOwnGPS)/1000;
        OwnGPSData.stalelocation=(OwnGPSData.secondsincelastupdate>60);
        if (OwnGPSData.stalelocation) {
          OwnGPSData.numsatellites=0;
        }
      }
      
    }

    if (DebuggingOwnGPS) {
      Serial.print("\nDebugging Own GPS-> ");
      Serial.print("Vbat: ");
      Serial.print(OwnGPSData.bvolt);
      Serial.print(" Magic Number: ");
      Serial.println(MagicNumber);
      
      Serial.print("Bearing: ");
      Serial.print(OwnGPSData.bearing);
      Serial.print(" Velocity: ");
      Serial.print(OwnGPSData.velocity_mph);
      Serial.print(" Altitude: ");
      Serial.println(OwnGPSData.altitude_meters);
      
      Serial.print("Latitude: ");
      Serial.print(OwnGPSData.latitude, 6);
      Serial.print(" Longitude: ");
      Serial.println(OwnGPSData.longitude, 6);
      
      Serial.print("Ever Fix: ");
      Serial.print(OwnGPSData.evervalidfix);
      Serial.print(" #Satelites: ");
      Serial.print(OwnGPSData.numsatellites);
      Serial.print(" Seconds since update: ");
      Serial.println(OwnGPSData.secondsincelastupdate);
    }
  }

  //Serial.print("Old button: ");
  //Serial.println(LastButtonPushed);
  //Serial.print("Button reads: A:");
  //Serial.print(analogRead(BUTTON_A));
  //Serial.print(" B: ");
  //Serial.print(digitalRead(BUTTON_B));
  //Serial.print(" C: ");
  //Serial.println(digitalRead(BUTTON_C));

  //////////////////////////////////////////////////////////////////////////
  // Query Button Status every ButtonUpdateInterval
  if ((LastButtonUpdate+ButtonUpdateInterval)<millis() & UseDisplay) {

    LastButtonUpdate=millis();
       
    // Record which button is being pushed. Note if multiple buttons, records C
    if(!digitalRead(BUTTON_C)) {
      LastButtonPushed = 'C';
    }
    else if(!digitalRead(BUTTON_B)) {
      LastButtonPushed = 'B';
    }
    else if(analogRead(BUTTON_A)<100) {
      // Note: The analog read is because when reading battery voltage, this digital read is no longer reliable.
      // So instead of using digitalRead, setting a specific threshold with analogRead
      LastButtonPushed = 'A';
    }
  }

  //Serial.print("New button: ");
  //Serial.println(LastButtonPushed);
  //delay(1000);
  
  //////////////////////////////////////////////////////////////////////////
  // Update OLED display screen every ButtonUpdateInterval
  if ((LastDisplayUpdate+DisplayUpdateInterval )<millis()) {

    LastDisplayUpdate=millis();

    // To lessen computational load, we only make these calculations when relevant, which is inside of display update loop
    if(TargetEverSync) {
      SecondsSinceIncomingPacket = int((millis()-LastLoraPacket)/1000);
      if(TargetEverValidGPS) {  
        SecondsSinceIncomingGPS = SecondsSinceIncomingPacket + TargetGPSData.secondsincelastupdate;
        if(SecondsSinceIncomingGPS>60) {
          TargetStaleLocation=1;
          } else {
          TargetStaleLocation=0; 
          }
      }
    }

    if(UseDisplay) {
  
      display.clearDisplay();
      // disabling the below line stops flickering. we don't actually need to draw the blank screen with new text until the bottom of this conditional
      // display.display();
      display.setCursor(0,0);
  
      // Print appropriate header
      if (LastButtonPushed == 'A') {
        display.print("A]");
      } else if (LastButtonPushed == 'B') {
        display.print("B]");
      } else {
        display.print("C]");
      }
  
      // Common code for all screens
  
      // New Packet flag
      if (DisplayNewPacketFlag) {
        display.print("*");
        DisplayNewPacketFlag=false;
      } else {
        display.print(" ");
      }
  
      // Base status
      display.print("B:");
      if(BaseEverValidGPS & !OwnGPSData.stalelocation) {
        display.print("F");
      } else if(BaseEverValidGPS) {
        display.print("S=" + String(min(99,int(OwnGPSData.secondsincelastupdate))));          
      } else {
        display.print("NF");
      }
      display.print(" ");
  
      //Display target status
      display.print("T:");
      if(!TargetEverSync) {
        display.print("NS");
      } else if (TargetEverValidGPS & !TargetStaleLocation) {
        display.print("F");
      } else if (TargetEverValidGPS & TargetStaleLocation) {
        display.print("S=" + String(min(99,SecondsSinceIncomingGPS)));
      } else {
        display.print("NF");
      }
  
      //If ever sync, last RSSI
      if(TargetEverSync) {
        display.print(" P=");
        display.print(lastRSSI);
      }      
      display.print("\n");  
  
      //This is the Target (transmitter) information screen
      if (LastButtonPushed == 'A') {
        if (TargetEverValidGPS) {
            display.print(String("TLt: " +  String(int( TargetGPSData.latitude  ) , DEC ) + "." + String( int(abs(round( ((TargetGPSData.latitude -int(TargetGPSData.latitude)) * 100000)))), DEC ) + "\n"));
            display.print(String("Lg: " + String(int( TargetGPSData.longitude ) , DEC ) + "." + String( int(abs(round( ((TargetGPSData.longitude-int(TargetGPSData.longitude)) * 100000)))), DEC ) + "\n"));
            display.print(String("Al:" + String(int( TargetGPSData.altitude_meters ) , DEC ) + " Vl:" + String(int( TargetGPSData.velocity_mph ) , DEC ) + " v:" + String(TargetGPSData.bvolt, 1 ) + "\n"));
        } else if (!TargetEverSync) {
            display.print("Never Target Sync\n");
        } else {
            display.print("Never Target Fix\n");
            display.print(String("Vbolt:" + String(TargetGPSData.bvolt, 1 ) + "\n"));
        }
      }
  
      //This is the Base (receiver) information screen
      if (LastButtonPushed == 'B') {
        if (BaseEverValidGPS) {
            display.print(String("BLt: " +  String(int( OwnGPSData.latitude  ) , DEC ) + "." + String( int(abs(round( ((OwnGPSData.latitude -int(OwnGPSData.latitude)) * 100000)))), DEC ) + " v:" + String(OwnGPSData.bvolt, 1 ) + "\n"));
            display.print(String("Lg: " + String(int( OwnGPSData.longitude ) , DEC ) + "." + String( int(abs(round( ((OwnGPSData.longitude-int(OwnGPSData.longitude)) * 100000)))), DEC ) + "\n"));
            display.print(String("Al:" + String(int( OwnGPSData.altitude_meters ) , DEC ) + " Vl:" + String(int( OwnGPSData.velocity_mph ) , DEC ) + " Br:" + String(int( OwnGPSData.bearing ) , DEC ) + "\n"));
        } else {
            display.print("Never Base Fix\n");
            display.print(String("v:" + String(OwnGPSData.bvolt, 1 ) + "\n"));     
        }
      }

      //This is the Navigation (base to receiver) information screen
      if (LastButtonPushed == 'C') {
        if (BaseEverValidGPS & TargetEverValidGPS) {
            display.print(String("Dist:" + String(round(distanceBetween(OwnGPSData.latitude, OwnGPSData.longitude, TargetGPSData.latitude, TargetGPSData.longitude)), 0)) + " ft.\n");
            display.print(String("Bear:" + String(round(courseTo(OwnGPSData.latitude, OwnGPSData.longitude, TargetGPSData.latitude, TargetGPSData.longitude)), 0)) + " deg.\n");
            display.print(String("OB:" + String(int( OwnGPSData.bearing ) , DEC ) + " Ov:" + String(OwnGPSData.bvolt, 1 ) + " Tv:" + String(TargetGPSData.bvolt, 1 ) + "\n"));
        } else {
            display.print("Can't navigate.\n");
            display.print("Missing something.\n");
        }
      }
      
      display.display(); // actually display all of the above
    }
  }

  //////////////////////////////////////////////////////////////////////////
  // Every SerialPortUpdateInterval send data from the Arduino to the Serial Port
  if ((LastSerialPortUpdate+SerialPortUpdateInterval )<millis()) {

    LastSerialPortUpdate=millis();

    if (PrintDataToSerialForLogging) {
        // millis()
        Serial.print(int(millis()/1000));
        Serial.print(",");
  
        // OwnGPSData.lastupdatemillis
        Serial.print(",");
  
        // OwnGPSData.lastupdatetime
        Serial.print(",");
  
        // OwnGPSData.date
        Serial.print(",");
  
        // OwnGPSData.evervalidfix
        Serial.print(int(TargetEverValidGPS));
        Serial.print(",");
  
        // OwnGPSData.secondsincelastupdate
        Serial.print(int(SecondsSinceIncomingGPS));
        Serial.print(",");
  
        // OwnGPSData.numsatellites
        Serial.print(TargetGPSData.numsatellites);
        Serial.print(",");
  
        // OwnGPSData.latitude
        Serial.print(TargetGPSData.latitude ,5);
        Serial.print(",");
  
        // OwnGPSData.longitude
        Serial.print(TargetGPSData.longitude,5);
        Serial.print(",");
  
        // OwnGPSData.altitude_meters
        Serial.print(TargetGPSData.altitude_meters);
        Serial.print(",");
  
        // OwnGPSData.velocity_mph
        Serial.print(TargetGPSData.velocity_mph);
        Serial.print(",");
  
        // OwnGPSData.bearing
        Serial.print(TargetGPSData.bearing);
        Serial.print(",");
  
        // OwnGPSData.bvolt
        Serial.print(TargetGPSData.bvolt);
        Serial.print("\n");
    }
  }

  //////////////////////////////////////////////////////////////////////////
  // if timers wraps around (because unit is left on for a long time), we'll just reset all of them to 0
  if (EndLoopMillis > millis()) {
    LastFixFromOwnGPS = 0;         // Last millis for which we got a Valid GPS Fix
    LastUpdateOwnGPSPacket = 0;    // Last millis at which we updated the GPS Packet
    LastLoraPacket = 0;            // Last millis for which we got a Valid GPS Fix
    LastDisplayUpdate = 0;
    LastButtonUpdate = 0;
    LastSerialPortUpdate = 0;
    }
  
  EndLoopMillis=millis();

}
