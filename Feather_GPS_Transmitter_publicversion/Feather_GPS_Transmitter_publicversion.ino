//////////////////////////////////////////////////////////////////////////
// RFM95 Lora Arduino GPS tracking system
// Version 1.2
// 2021/08/18
// Gregory Huber (gregory.a.huber@outlook.com)
//
// This is the code for the TRANSMITTER
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
// Specify mode and hardware

// Set to true to enable display. Set to false to disable. 
#define UseDisplay false

//////////////////////////////////////////////////////////
// DebugDebugging mode
// Enable each of these flags to enable diffent parts of the DebugDebugging code
#define DebuggingOwnGPS false
#define DebuggingRadio false

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
// Setup LORFA Radio
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
#define DisplayUpdateInterval 750
#define SendViaRadioInterval 1000

// Whether ever received a valid GPS fix
bool OwnEverValidGPS = false;

// Timers used to figure out last target GPS update, last local GPS update, and Last display
uint32_t LastOwnLocationUpdate = millis();
uint32_t LastDisplayUpdate = millis();
uint32_t LastButtonUpdate = millis();
uint32_t LastSendViaRadio = millis();

// Storage for GPS info
double OwnLat, OwnLong;
int OwnGSPLastQuality, OwnAlt, OwnSpeed, OwnBearing;
uint32_t OwnLastUpdateMillis, OwnLastUpdateTime;
bool OwnStaleLocation = false;

bool SendingPacket = false;

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
  if (UseDisplay) {
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
    display.println("Init. GPS Transmitter");
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
  }
  
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
    if (UseDisplay) {
      display.clearDisplay();
      display.print("LoRa radio init failed");
      display.display();
    }
    while (1);
  }

  if (!rf95.setFrequency(RF95_FREQ)) {
    if (UseDisplay) {
      display.clearDisplay();
      display.print("setFrequency failed");
      display.display();
    }
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
    if (UseDisplay) {
      display.clearDisplay();
      display.print("setModemConfig failed");
      display.display();
      }
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

// transmitData sends an outgoing packet
void transmitData() {

  int32_t TempLat;
  int32_t TempLong;
  int32_t TempAlt;
  bool sending = false;

  TempLat = int(OwnLat)*100000 + round( (OwnLat- int(OwnLat)) * 100000 );
  TempLong = int(OwnLong)*100000 + round( (OwnLong- int(OwnLong)) * 100000 );
  TempAlt = int(OwnAlt);
  
  if (DebuggingRadio) {
    Serial.print("Original data: ");
    Serial.print(OwnEverValidGPS);   
    Serial.print(",");    
    Serial.print(OwnStaleLocation);   
    Serial.print(",");
    Serial.print(OwnLat);
    Serial.print(",");
    Serial.print(OwnLong);
    Serial.print(",");
    Serial.println(OwnAlt);
      
    Serial.print("Data to send: ");
    Serial.print(TempLat);
    Serial.print(",");
    Serial.print(TempLong);
    Serial.print(",");
    Serial.println(TempAlt);  }
  
  // Define the packet size--two booleans, 3 int_32, and then the magic numbers
  // Packet is composed of:
  //   binary flag for whether ever a valid GPS fix
  //   binary flag for whether GPS fix is stale
  //   Latitude (without decimal)
  //   Longitude (without decimal)
  //   Altitude (in feet)
  
  uint8_t len = 2 * sizeof(bool) + + 3 * sizeof(int32_t) + + MAGIC_NUMBER_LEN + 1;
  uint8_t radiopacket[len];
  for (int i = 0; i < MAGIC_NUMBER_LEN; i++) {
    radiopacket[i] = MAGIC_NUMBER[i];
  }  
  void* p = radiopacket + MAGIC_NUMBER_LEN;
  *(bool*)p = OwnEverValidGPS;
  p = (bool*)p + 1;
  *(bool*)p = OwnStaleLocation;
  p = (bool*)p + 1;
  *(int32_t*)p = TempLat;
  p = (int32_t*)p + 1;
  *(int32_t*)p = TempLong;
  p = (int32_t*)p + 1;
  *(int32_t*)p = TempAlt;
  radiopacket[len - 1] = '\0';

  sending = true;
  rf95.send((uint8_t *)radiopacket, len);
  rf95.waitPacketSent();
  sending = false;

  if (DebuggingRadio) {
    Serial.print("Radio Packet:");
    for(int i=0;i<len;i++){
      Serial.print(radiopacket[i]);
    }
    Serial.print("\n");
  }

}

// This is the main loop which runs over and over
void loop() // run over and over again
{

  //////////////////////////////////////////////////////////////////////////
  // If in transmit mode, send location via radio every SendViaRadioInterval
  // Note I edited code to keep sending packets even if GPS location was stale,
  // because a transmission signal is still useful for finding a lost tracker
  // that can't get an updated GPS location. Additionally, this sends even if there
  // is not valid GPS data, because at least then you can check for a sync
  if ((LastSendViaRadio+SendViaRadioInterval)<millis()) {
    LastSendViaRadio=millis();
    SendingPacket = true;
	transmitData();
    }

  //////////////////////////////////////////////////////////////////////////
  // Update OLED display screen every ButtonUpdateInterval
  if (UseDisplay) {
    if ((LastDisplayUpdate+DisplayUpdateInterval )<millis()) {
  
    LastDisplayUpdate=millis();

    display.clearDisplay();
    display.display();
    display.setCursor(0,0);
	  
	// This is what gets displayed if this is a transmitter with a display

  display.print("Send: ");
	//display.print("Send: VBAT=");
	//float  measuredvbat  =  (analogRead ( VBATPIN ));
	//measuredvbat *= 2;
	//measuredvbat *= 3.3;
	//measuredvbat /= 1024;
	//display.print(measuredvbat);
	if(OwnEverValidGPS) {
	  if (OwnStaleLocation) display.print("(Stale GPS!) ");
	  if (SendingPacket) {
		  display.print("(*) ");
		  SendingPacket = false;
	  }
    display.print("\n");
    display.print(String("Lat: " +  String(int( OwnLat  ) , DEC ) + "." + String( int(abs(round( ((OwnLat -int(OwnLat)) * 100000)))), DEC ) + "\n"));
    display.print(String("Long: " + String(int( OwnLong ) , DEC ) + "." + String( int(abs(round( ((OwnLong-int(OwnLong)) * 100000)))), DEC ) + "\n"));
    display.print(String("Alt: " + String(int( OwnAlt ) , DEC ) + "\n"));
	}
	else {
		 display.print("Never Fix");
	}
  
	display.display(); // actually display all of the above
  
    }
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
