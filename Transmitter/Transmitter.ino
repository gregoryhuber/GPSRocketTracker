//////////////////////////////////////////////////////////////////////////
// RFM95 Lora Arduino GPS tracking system
// Version 2.1
// 2022/07/01
// Gregory Huber (gregory.a.huber@outlook.com)
//
// This is the code for the TRANSMITTER
//
//////////////////////////////////////////////////////////////////////////

// This is the transmitter half of the codebase for an Arduino feather-based GPS tracking system suitable for rocket tracking
// The physical setup is comprised of a base station (receiver) and a tracker (transmitter)

// The (smallest) transmitter is built around an the following equipment:
// Adafruit Feather M0 with RFM95 LoRa Radio - 900MHz – RadioFruit https://www.adafruit.com/product/3178
// GPS receiver https://www.adafruit.com/product/746
// Quarter Wavelength Wire antenna (3.12inches) http://www.csgnetwork.com/antennagenericfreqlencalc.html
// 400mah LIPO battery https://www.adafruit.com/product/3898
// Wiring:
//  TX on M0 module connects to RX on GPS
//  RX on M0 module connects to TX on GPS
//  PWR on M0 module connects to PWR on GPS
//  GND on M0 module connects to GND on GPS

// This code draws on the following resouces and references. Thank you.
// This is a fully fleshed out example from which many of the transmission details were copied https://github.com/mloudon/electrosew/blob/master/feather_send/feather_send.ino
// https://gis.stackexchange.com/questions/252672/calculate-bearing-between-two-decimal-gps-coordinates-arduino-c
// https://community.particle.io/t/tinygps-using-distancebetween/28233/3
// http://www.airspayce.com/mikem/arduino/RadioHead/classRH__RF95.html
// https://www.thethingsnetwork.org/forum/t/best-practices-when-sending-gps-location-data/1242/40

// Version 2.0/2.1 (2022/01):
// Introduced data structures, which allowed much more efficiently sending more data in the LORA packet
// On the transmitter:
    // Read and send battery voltage
    // Added support for a micro SD data logger
    // Refined display code to fix what is displayed on transmitter when it has a display
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
#include <SD.h>

////////////////////////////////////////////////////////////////////////////////////////////
//
// Radio constants
//
////////////////////////////////////////////////////////////////////////////////////////////
  
  // This is important. This defines the frequency the units will use. Both base and transmitter need to be on the same frequency
  // LORA frequency range in the US is 902 to 928 MHz https://www.everythingrf.com/community/lora-frequency-bands-in-north-america
  #define RF95_FREQ 915.00
  
  // The magic number is a unique three digit code that is used to verify the pair between tracker and base unit.
  // Valid ranges are 000 to 999
  // If these numbers don't match, any incoming packet is ignored
  // Please change it to something different form this so that your tracker doesn't conflict with someone else's!
  #define MagicNumber 123

////////////////////////////////////////////////////////////////////////////////////////////
//
// Hardware constants
//
////////////////////////////////////////////////////////////////////////////////////////////

  #define UseRadio true
  #define UseSDCardWriter false
  #define UseDisplay false

////////////////////////////////////////////////////////////////////////////////////////////
//
// Timing constants
//
////////////////////////////////////////////////////////////////////////////////////////////

  // Update intervals for getting own GPS locations and sending it
  // These are in milliseconds, so 1000 = 1 second
  #define OwnGPSUpdateInterval 100
  #define DisplayUpdateInterval 250
  // SendViaRadioInterval must be longer than the amount of time it takes a packet to actually send or
  // the radio will crash into itself by resending before the prior packet is done.
  // With the settings currently in use here, it takes .05 seconds to send a packet, so I set it to 200 milliseconds, or 5 packets per second.
  #define SendViaRadioInterval 200
  // Write data to SD card every #/1000 seconds
  #define WriteSDInterval 1000

////////////////////////////////////////////////////////////////////////////////////////////
//
// Debugging
//
////////////////////////////////////////////////////////////////////////////////////////////

  // Enable each of these flags to enable diffent parts of the DebugDebugging code
  // DebuggingOwnGPS prints to serial own gps data
  // DebuggingRadio prints to serial the radio packet data
  #define DebuggingOwnGPS false
  #define DebuggingRadio false
  #define DebuggingSD false
  
  // Fake data used for static testing
  #define UseStaticData false

  #This is the New Haven, CT Green
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
  // Setup for StickBug 1.0
  // #define VBATPIN A7

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

  //////////////////////////////////////////////////////////////////////////////////////////
  // Setup SD Card Featherwing
  #define SD_CS         10
  #define SD_CardDetect 12

  // Setup for StickBug 1.0
  //#define SD_CS         11
  //#define SD_CardDetect 12
  
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
    bool evervalidfix;              // whether we've ever had a valid gps fix; 1 =yes
    uint32_t lastupdatemillis;      // last gps update in millis format
    uint32_t lastupdatetime;        // last gps update in hhmmss integer format
    uint32_t date;                  // last gps data in YYYYMMSS format
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
    .date=0,
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

  // Radio packet to send
  radiopacket_gps_t OwnDataToSend;

  // Whether a new valid GPS location has arrived that needs to be parsed/stored
  boolean NewValidGPSDataArrived = 0;
  // Flag for display when new packet sent
  boolean SentNewPacketFlag = 0;

  // Timers
  uint32_t LastSendViaRadio = 0;    // Last millis for which we strated a LORA packet send
  uint32_t LastFixFromGPS = 0;      // Last millis for which we got a Valid GPS Fix
  uint32_t LastUpdateGPSPacket = 0; // Last millis at which we updated the GPS Packet
  uint32_t LastDisplayUpdate = 0;   // Last millis at which display was updated.
  uint32_t LastSDWrite = 0;         // Last millis at which we wrote to SD card
  uint32_t EndLoopMillis = 0;       // Last millis at which we finished the main loop.
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
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); // 10 Hz update rate
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
     
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
    display.println("Init. Transmitter");
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
  if (UseRadio) {
  
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
    // I've adoped it because it is faster, has good range, and does not seem as vulnerable to doppler shift from moving rocket
    
    // Bw500Cr45Sf128  
    // Bw = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range.
  
    // Bw31_25Cr48Sf512  
    // Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range.
    // This has been used successfully for up to 1.6KM communication.
    // However because it is such a long packet send, it seems vulnerable to doppler shift from a moving object.
  
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
  
    if (DebuggingRadio) {
      rf95.printRegisters();
    } 
  }
  
  // Now we turn off chip select for the LORA module so it doesn't interfere with anything else
  digitalWrite (RFM95_CS, HIGH);  

  //////////////////////////////////////////////////////////////////////////
  // Setup SD Card writer

  if (UseSDCardWriter) {

    // attempt to initalize SD card
    if (!SD.begin(SD_CS)) {
      Serial.println("SD init failed");
    } else {
      Serial.println("SD init worked");
    } 
  
    // just double checking that the sd card is in
    if ( digitalRead(SD_CardDetect) == LOW ) {
      String header = "CurrentMillis,LastUpdateMillis,LastUpdateTime,Date,EverValidFix,SecondsSinceLastUpdate,NumberSatellites,Latitude,Longitude,Altitude_meters,Velocity_mph,Bearing,Battery_volts";
      File dataFile = SD.open("DataLog.csv", FILE_WRITE );
      dataFile.println(header);
      dataFile.flush();
      dataFile.close();
      Serial.println("SD header write worked");
    }
    else {
      Serial.println("SD header write failed");
    }
    
  }  
}

// transmitData sends an outgoing packet
void sendLoraPacket() {

  // Note that in this codeblock, we are using mins to make sure that we don't overflow any of our data types
  // (For example, vbat can only be two digis, magicnumber can be only 3)
  OwnDataToSend.vbat_code = min(65,int(OwnGPSData.bvolt*10))*1000 + min(999,MagicNumber);
  OwnDataToSend.bear_veloc_alt = ((round(OwnGPSData.bearing))*10000000) + (min(999,round(OwnGPSData.velocity_mph))*10000) + min(9999,max(0,(round(OwnGPSData.altitude_meters)+100)));
  OwnDataToSend.latitude_sats_ever = (100*round((OwnGPSData.latitude+180)*100000)) + (min(9,round(OwnGPSData.numsatellites))*10) + OwnGPSData.evervalidfix;
  OwnDataToSend.longitude_sincelastfix = (100*round((OwnGPSData.longitude+180)*100000)) + min(99,round(OwnGPSData.secondsincelastupdate));
  
  if (DebuggingRadio) {
    Serial.print("\nDebugging Radio. Packet to send-> Size:");
    Serial.println(sizeof(OwnDataToSend));
    Serial.print("VVCCC: ");
    Serial.println(OwnDataToSend.vbat_code);
    Serial.print("BBBVVVAAAA: ");
    Serial.println(OwnDataToSend.bear_veloc_alt);
    Serial.print("LLLLLLLL#E: ");
    Serial.println(OwnDataToSend.latitude_sats_ever);
    Serial.print("LLLLLLLLSS: ");
    Serial.println(OwnDataToSend.longitude_sincelastfix);
    Serial.print("Current millis before trying to send packet: ");
    Serial.println(millis());
  }
  
  bool trysending = rf95.send((uint8_t*)&OwnDataToSend, sizeof(OwnDataToSend)); 
  // I tried commenting out the below line so it wouldn't wait for the packet to send before moving on
  // But this caused the whole thing to crash after about 40 seconds whenever the SD card was writing.
  // Experiment at your own risk
    // OLD: // We aren't waiting here for the packet to send. Instead, we are making sure using SendViaRadioInterval that we give the
            // packet time to send before sending another one. This make the LORA send non-blocking.
  rf95.waitPacketSent();

  if (DebuggingRadio) {
    Serial.print("Return value of trying to send packet: ");
    Serial.println(trysending);
    Serial.print("Current millis after sending packet: ");
    Serial.println(millis());
  }
}

// log_to_sd logs current GPS data to the SD Card
void log_to_sd() 
{
  if ( digitalRead(SD_CardDetect) == LOW ) {
    String SD_DATA = "";
    SD_DATA += String(millis(),DEC) + ",";
    SD_DATA += String(OwnGPSData.lastupdatemillis,DEC) + ",";
    SD_DATA += String(OwnGPSData.lastupdatetime,DEC) + ",";
    SD_DATA += String(OwnGPSData.date,DEC) + ",";
    SD_DATA += String(OwnGPSData.evervalidfix,DEC) + ",";
    SD_DATA += String(OwnGPSData.secondsincelastupdate,DEC) + ",";
    SD_DATA += String(OwnGPSData.numsatellites,DEC) + ",";
    SD_DATA += String(OwnGPSData.latitude,5) + ",";
    SD_DATA += String(OwnGPSData.longitude,5) + ",";
    SD_DATA += String(OwnGPSData.altitude_meters,DEC) + ",";
    SD_DATA += String(OwnGPSData.velocity_mph,DEC) + ",";
    SD_DATA += String(OwnGPSData.bearing,DEC) + ",";
    SD_DATA += String(OwnGPSData.bvolt,1);
    File dataFile = SD.open("DataLog.csv", FILE_WRITE );
    if (DebuggingSD) {
      Serial.println("Opening SD Card File");
    }
    dataFile.println(SD_DATA);
    if (DebuggingSD) {
      Serial.println("Done printing data to SD Card");
    }
    dataFile.flush();
    if (DebuggingSD) {
      Serial.println("Done flusing data to SD Card");
    }
    dataFile.close();
    if (DebuggingSD) {
      Serial.println("Closed SD Card File");
      Serial.println(SD_DATA);
    }
  } else {
    if (DebuggingSD) {
      Serial.println("No SD Card Detected");
    }
  }
}

// Main loops, runs over and over
void loop() {

  //////////////////////////////////////////////////////////////////////////
  // Each time through the loop, send Lora Packet if it has been at least
  // SendViaRadioInterval millis since last packet was sent.
  //
  // Note that this keeps sending packets even if GPS location was stale or non-existent.
  //
  // Even non-exisitent GPS data is useful for figuring out if the LORA is working and
  // at the extreme signal strength could be used to find a missing tracker.
  //
  // Additionally, even if we can't get an updated GPS location, a transmission of old
  // GPS data is still useful for finding a lost tracker.

  if (UseRadio) {

    // Turn on communication to LORA. Keeps this from interfering with other parts of code.
    digitalWrite (RFM95_CS, LOW);
    
    if ((LastSendViaRadio+SendViaRadioInterval)<millis()) {
      LastSendViaRadio=millis();
      sendLoraPacket();
      SentNewPacketFlag=1;
      }
  
    // Turn off communication to LORA. Keeps this from interfering with other parts of code.
    digitalWrite (RFM95_CS, HIGH);  
  }

  //////////////////////////////////////////////////////////////////////////
  // Each time through the loop, attempt to read GPS data.
  // This code doesn't try to read from GPS unless it is available.
  // Additionally, if no new data is available, it just skips through.
  if (GPSSerial.available() > 0) {
    char c = GPS.read();
    if (GPS.newNMEAreceived()) {
      if (GPS.parse(GPS.lastNMEA())) {
        if (DebuggingOwnGPS) {
          Serial.println(GPS.lastNMEA());
        }
        if (GPS.fix==1) {
          NewValidGPSDataArrived = 1;
          LastFixFromGPS = millis();
          }
        }
      }
  }

  //////////////////////////////////////////////////////////////////////////
  // Update Own GPS calculations every OwnGPSUpdateInterval
  if ((LastUpdateGPSPacket+OwnGPSUpdateInterval )<millis()) {
    LastUpdateGPSPacket=millis();

    // This block of conditional code just sets the GPS data to either the fixed stata data for testing purposes or the actual GPS data
    if (UseStaticData) {
      // Fixed data for testing purpuses
      OwnGPSData.evervalidfix=StaticEverValidFix;
      OwnGPSData.lastupdatemillis=millis();
      OwnGPSData.lastupdatetime=234233;
      OwnGPSData.date=101112;
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
      // NewValidGPSDataArrived: This flag lets us know if in the intervening period a new GPS packet with a valid fix has been parsed
      if (NewValidGPSDataArrived) {
        // Do this if new data has arrived
        NewValidGPSDataArrived = 0;
        OwnGPSData.evervalidfix=1;
        OwnGPSData.lastupdatemillis=LastFixFromGPS;
        OwnGPSData.lastupdatetime=GPS.hour*10000+GPS.minute*100+GPS.seconds;
        OwnGPSData.date=GPS.year*10000 + GPS.month*100 + GPS.day;
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
        OwnGPSData.secondsincelastupdate=(millis()-LastFixFromGPS)/1000;
        OwnGPSData.stalelocation=(OwnGPSData.secondsincelastupdate>60);
        if (OwnGPSData.stalelocation) {
          OwnGPSData.numsatellites=0;
        }
      }
    }

    if (DebuggingOwnGPS) {
      Serial.println("\nDebugging Own GPS. GPS Data-> ");
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

  //////////////////////////////////////////////////////////////////////////
  // Display own status on display if using display
  if ((UseDisplay) && ((LastDisplayUpdate+DisplayUpdateInterval )<millis())) {

    LastDisplayUpdate=millis();

    display.clearDisplay();
    // disabling the below line stops flickering. we don't actually need to draw the blank screen with new text until the bottom of this conditional
    // display.display();
    display.setCursor(0,0);
    if (SentNewPacketFlag==1) {
      display.print("*");
      SentNewPacketFlag=0;
    } else {
      display.print(" ");
    }

    if(OwnGPSData.evervalidfix & !OwnGPSData.stalelocation) {
      display.print("F");
    } else if(OwnGPSData.evervalidfix) {
      display.print("S=" + String(min(99,int(OwnGPSData.secondsincelastupdate))));          
    } else {
      display.print("NF");
    }
    display.print(" VB:");
    display.print(String(OwnGPSData.bvolt, 1 ));
    display.print(" SAT:");
    display.println(String(OwnGPSData.numsatellites));

    if (OwnGPSData.evervalidfix ) {
        display.print(String("BLt: " +  String(int( OwnGPSData.latitude  ) , DEC ) + "." + String( int(abs(round( ((OwnGPSData.latitude -int(OwnGPSData.latitude)) * 100000)))), DEC ) + "\n"));
        display.print(String("Lg: " + String(int( OwnGPSData.longitude ) , DEC ) + "." + String( int(abs(round( ((OwnGPSData.longitude-int(OwnGPSData.longitude)) * 100000)))), DEC ) + "\n"));
        display.print(String("Al:" + String(int( OwnGPSData.altitude_meters ) , DEC ) + " Vl:" + String(int( OwnGPSData.velocity_mph ) , DEC ) + " Br:" + String(int( OwnGPSData.bearing ) , DEC ) + "\n"));
    }

    display.display(); // actually display all of the above
  }

  //////////////////////////////////////////////////////////////////////////
  // Write data to SD Card every WriteSDInterval
  if (UseSDCardWriter) {
    if ((LastSDWrite+WriteSDInterval)<millis()) {
      LastSDWrite=millis();
      log_to_sd();
    }
  }
  
  // if millis() or timer wraps around, we'll just reset it
  if (EndLoopMillis > millis()) {
    LastSendViaRadio = 0;
    LastFixFromGPS = 0;
    LastUpdateGPSPacket = 0;
    LastSDWrite = 0;
    LastDisplayUpdate = 0;
  }

  EndLoopMillis=millis();
}
