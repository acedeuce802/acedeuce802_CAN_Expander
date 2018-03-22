/*
  Although heavily modified by now to work with the Teensy and with newer Megasquirt features
  (CAN broadcasting), much of this code originally came from https://github.com/kckr/MSCan_Gauge
  Many thanks to that project, as I would not have gotten started without it.
*/

// User configuerable variables
const int myCANid = 10; // CAN ID of this unit
const int msCANid = 0; // CAN ID of the Megasquirt (should almost always be 0)
const int REVLIMIT = 6800; // soft rev limit at which to start blinking the tach gauge

#include <Metro.h>
#include <FlexCAN.h>
#include <kinetis_flexcan.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Time.h>
#include <TinyGPS++.h>
TinyGPSPlus GPS;

#include <elapsedMillis.h>

#include <FastLED.h>
#define LEDPIN 5
#define NUMLEDS 16
CRGB leds[NUMLEDS];

#include <Encoder.h>
Encoder myEnc(7, 6); // interrupts on pin6, 7
#define RBUTTON_INT 15 // pin 15
const unsigned long debouncing_time = 150; //Debouncing Time - 150 is good, 200 is better, 250 seems worse
volatile unsigned long last_millis; //switch debouncing

#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// OLED display
/* Software SPI
#define OLED_MOSI   9
#define OLED_CLK   10
#define OLED_DC    11
#define OLED_CS    12
#define OLED_RESET 8
Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
*/

// Hardware SPI
#define OLED_DC     10
#define OLED_CS     9
#define OLED_RESET  8
Adafruit_SSD1306 display(OLED_DC, OLED_RESET, OLED_CS);

#if (SSD1306_LCDHEIGHT != 64)
//#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

// Metro ticks are milliseconds
Metro gpsTimer = Metro(1000); // display GPS status
Metro commTimer = Metro(1000); // display an error message if no CAN data during this interval
Metro displayTimer = Metro(100); // refresh the display at this interval
Metro ledTimer = Metro(1); // how long to flash the led upon CAN frame receive/transmit
Metro gaugeBlinkTimer = Metro(100); // blink the led ring pixels during certain conditions
boolean connectionState = false;
boolean gpsLogo = false;
boolean gaugeBlink = false;

int led = 13;
FlexCAN CANbus(500000);
static CAN_message_t txmsg,rxmsg;
static uint8_t hex[17] = "0123456789abcdef";

int rpm,ect,afr;
unsigned int txTimer,rxTimer;

//MS data vars
byte indicator[7]; // where to store indicator data
unsigned int RPM, CLT, MAP, MAT, SPKADV, BATTV, TPS, Knock, Baro, EGOc, IAC, dwell, bstduty, idle_tar, oilpres, fuelpres, oiltemp, oiltempgauge, waterpres, sensor4, AFRtgterr1, AFRtgterr2, AFR1, AFR2, sensor3, gear;
int AFR, AFRtgt;
unsigned int MAP_HI, Knock_HI, RPM_HI, CLT_HI, MAT_HI;
int AFR_HI, AFR_LO;
int RPM_0_var, RPM_black, RPM_green_1, RPM_green_2, RPM_green_3, RPM_yellow_1, RPM_yellow_2, RPM_red, flash_int, GPSspeed;

//static char sprintfbuffer[2];

struct ledval {
  byte r0;
  byte g0;
  byte b0;
  byte r1;
  byte g1;
  byte b1;
  byte r2;
  byte g2;
  byte b2;
  byte r3;
  byte g3;
  byte b3;
};


const unsigned char gps [] = {
  0x00, 0x10, 0x00, 0x38, 0x00, 0x7C, 0x00, 0xF8, 0x01, 0xF0, 0x05, 0xE0, 0x00, 0xC0, 0x0C, 0x00,
  0x1E, 0x90, 0x3E, 0x12, 0x7C, 0x24, 0xF8, 0xC4, 0x70, 0x08, 0x20, 0x30, 0x00, 0x40, 0x00, 0x00
};

struct MSDataObject {
  char name[10]; // length tbd
  byte block; // max val 32?
  unsigned int offset; // max val?
  byte reqbytes; // max val 8
  byte mult; // does this need to be * 0.1 ?
};

const MSDataObject MSData[] PROGMEM = {
// string,  block, offset, reqbytes, mult, div
  {"RPM",     7,   6,  2,    0   }, // 0
  {"AFR",     7,  252, 1,    0   }, // 1
  {"CLT",     7,  22,  2,    1   }, // 2
  {"MAP",     7,  18,  2,    1   }, // 3
  {"MAT",     7,  20,  2,    1   }, // 4
  {"SPKADV",  7,   8,  2,    1   }, // 5
  {"BATTV",   7,  26,  2,    1   }, // 6
  {"TPS",     7,  24,  2,    1   }, // 7
  {"Knock",   7,  32,  2,    1   }, // 8
  {"Baro",    7,  16,  2,    1   }, // 9
  {"EGOc",    7,  34,  2,    1   }, // 10
  {"IAC",     7,  54,  2,    0   }, // 11 -- this was GFC's to 49 / 125
  {"dwell",   7,  62,  2,    1   }, // 12
  {"bstduty", 7,  39,  1,    0   }, // 13 boost duty cycle
  {"idletar", 7, 380,  2,    0   }, // 14
  {"AFRtgt",  7,  12,  1,    1   }, // 15
};

struct MSDataBinaryObject {
  char name[14];
  byte sbyte;
  byte bitp;
};

const MSDataBinaryObject MSDataBin[] PROGMEM = {
// "1234567890"
//"name", indicator byte, bit position
//0 engine - block 7 offset 11
  {"Ready",            0,  0 },
  {"Crank",            0,  1 },
  {"ASE",              0,  2 },
  {"WUE",              0,  3 },
  {"TPS acc",          0,  4 },
  {"TPS dec",          0,  5 },
  {"MAP acc",          0,  6 },
  {"MAP dec",          0,  7 },
//1 status1 - block7 offset 78
  {"Need Burn",        1,  0,},
  {"Data Lost",        1,  1 },
  {"Config Error",     1,  2 },
  {"Synced",           1,  3 },
  {"VE1/2",            1,  5 },
  {"SPK1/2",           1,  6 },
  {"Full-sync",        1,  7 },
//2 status2 - block 7 offset 79
  {"N2O 1",            2,  0 },
  {"N2O 2",            2,  1 },
  {"Rev lim",          2,  2 },
  {"Launch",           2,  2 },
  {"Flat shift",       2,  4 },
  {"Spark cut",        2,  5 },
  {"Over boost",       2,  6 },
  {"CL Idle",          2,  7 },
//3 status3 - block 7 offset 80
  {"Fuel cut",         3,  0 },
//{"T-log",            3,  1 },
//{"3 step",           3,  2 },
//{"Test mode",        3,  3 },
//{"3 step",           3,  4 },
  {"Soft limit",       3,  5 },
//{"Bike shift",       3,  6 },
  {"Launch",           3,  7 },
//4 check engine lamps - block 7 offset 425
  {"cel_map",          4,  0 },
  {"cel_mat",          4,  1 },
  {"cel_clt",          4,  2 },
  {"cel_tps",          4,  3 },
  {"cel_batt",         4,  4 },
  {"cel_afr0",         4,  5 },
  {"cel_sync",         4,  6 },
  {"cel_egt",          4,  7 },
//5 port status - block 7 offset 70
  {"port0",            5,  0 },
  {"port1",            5,  1 },
  {"port2",            5,  2 },
  {"port3",            5,  3 },
  {"port4",            5,  4 },
  {"port5",            5,  5 },
  {"port6",            5,  6 },
//6 status6 - block 7 offset 233
  {"EGT warn",         6,  0 },
  {"EGT shutdown",     6,  1 },
  {"AFR warn",         6,  2 },
  {"AFR shutdown",     6,  3 },
  {"Idle VE",          6,  4 },
  {"Idle VE",          6,  5 },
  {"Fan",              6,  6 },
//7 status7 - block 7 offset 351
  {"Knock",            7,  4 },
  {"AC",               7,  5 },
};

long R_index = 0; // for rotary encoder
byte B_index = 0; // Button increment
byte M_index = 0; // Menu increment
byte S_index = 0; // Select increment

byte neo_brightness = 4;
byte g_textsize = 1;
char tempchars[11];

// -------------------------------------------------------------
static void hexDump(uint8_t dumpLen, uint8_t *bytePtr) {
  uint8_t working;
  while( dumpLen-- ) {
    working = *bytePtr++;
    Serial.write( hex[ working>>4 ] );
    Serial.write( hex[ working&15 ] );
  }
  //Serial.write('\r');
  //Serial.write('\n');
}

static void binDump(char working) {
  int i;
  for (i=7; i>=0; i--) {
    (bitRead(working,i) == 1) ? Serial.print("1") : Serial.print("0");
  }
}

static void ledBlink() {
  ledTimer.reset();
  digitalWrite(led, 1);
}

// pack/unpack the Megasquirt extended message format header
typedef struct msg_packed_int {
  unsigned char b0;
  unsigned char b1;
  unsigned char b2;
  unsigned char b3;
} msg_packed_int;

typedef struct msg_bit_info {
  unsigned int spare:2;
  unsigned int block_h:1;
  unsigned int block:4;
  unsigned int to_id:4;
  unsigned int from_id:4;
  unsigned int msg_type:3;
  unsigned int offset:11;
} msg_bit_info;

typedef union {
  unsigned int i;
  msg_packed_int b;
  msg_bit_info values;
} msg_packed;

msg_packed rxmsg_id,txmsg_id;

// unpack the vars from the payload of a MSG_REQ packet
typedef struct msg_req_data_packed_int {
  unsigned char b2;
  unsigned char b1;
  unsigned char b0;
} msg_req_data_packed_int;

typedef struct msq_req_data_bit_info {
  unsigned int varbyt:4;
  unsigned int spare:1;
  unsigned int varoffset:11;
  unsigned int varblk:4;
} msg_req_data_bit_info;

typedef union {
  msg_req_data_packed_int bytes;
  msg_req_data_bit_info values;
} msg_req_data_raw;

msg_req_data_raw msg_req_data;

unsigned long validity_window; // for hi/low + histogram window update
unsigned long validity_window2;

byte histogram[64]; // 512 memory usage
byte histogram_index;

unsigned int accel_x,accel_y,accel_z;

// touch "buttons"
int btnA,btnB,btnC;
int btnA_init,btnB_init,btnC_init;
int btnA_last,btnB_last,btnC_last;

struct btn {
  unsigned int last:1;
  int init;
  int value;
};

btn buttons[3];
// -------------------------------------------------------------
void setup(void) {  
  digitalWrite(led, 1);
  SPI.setSCK(14); // alternate clock pin so we can still use the LED
  //setTime(1415515398);
  CANbus.begin();
  pinMode(led, OUTPUT);
  digitalWrite(led, 1);

  Serial2.begin(115200);

  // Init GPS on Serial1
  Serial1.begin(9600);

  //switch the GPS baud rate to 38400
  Serial1.write("$PMTK251,38400*27\r\n");
  //change baud rate of serial port to 38400
  Serial1.flush();
  delay(10);
  Serial1.end();
  Serial1.begin(38400);
  //This command turns off all NMEA messages except VTG, GGA and GSA. GSA is only sent once every five transmissions.
  Serial1.write("$PMTK314,0,1,1,1,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0*2C\r\n");
  //After sending those commands to the GPS, it is possible to turn on 10Hz update rate.
  Serial1.write("$PMTK220,100*2F\r\n");
  //And this time you’ll get
  //$PMTK001,220,3*30

  // set encoder pins as input with internal pull-up resistors enabled
  pinMode(RBUTTON_INT, INPUT);
  digitalWrite(RBUTTON_INT, HIGH);
 // attachInterrupt(RBUTTON_INT, ISR_debounce, FALLING);


  FastLED.show();
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC);
  // init done
  display.clearDisplay();
//  display.drawBitmap(0,0, ms_logo, 128, 64, 1);
  display.display(); // show splashscreen

  accel.begin();
  accel.setRange(ADXL345_RANGE_4_G); // 2,4,8,16g are valid

  /*
    // calibrate touch "buttons"
    for (int i=0; i<10; i++) {
      for (int j=0; j<3; j++) {
        buttons[j].init += touchRead(15 + j);
      }
      //btnA_init += touchRead(15);
      //btnB_init += touchRead(16);
      //btnC_init += touchRead(17);
      delay(100);
    }
    buttons[0].init /= 10;
    buttons[1].init /= 10;
    buttons[2].init /= 10;
    */

  FastLED.addLeds<NEOPIXEL, LEDPIN>(leds, NUMLEDS);
  // For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one.
  //fill_rainbow(leds, NUMLEDS, 0, 255/16);
  for(int i=0; i<16; i++) {
    leds[i].setRGB(16,16,16); // initialize led ring
  }
  FastLED.show();
  delay(1000);
  //Serial.println(F("Hello Teensy 3.1 CAN Test."));
  digitalWrite(led, 0);
  //commTimer.reset();
}

// -------------------------------------------------------------
void loop(void) {

  oiltemp = analogRead(A0);
  
  if (ledTimer.check() && digitalRead(led)) {
    digitalWrite(led, 0);
    ledTimer.reset();
  }
  if (gaugeBlinkTimer.check()) {
    gaugeBlink = gaugeBlink ^ 1;
    gaugeBlinkTimer.reset();
  }

  // Handle GPS data

  bool nmeaReceived = false;
  while (!nmeaReceived && (Serial1.available() > 0)) {
    nmeaReceived = GPS.encode(Serial1.read());
  }

  if (GPS.location.isUpdated()) { // set local clock from GPS
    if (GPS.time.age() < 500) {
//      setTime(GPS.time.hour(), GPS.time.minute(), GPS.time.second(), GPS.date.day(), GPS.date.month(), GPS.date.year());
//      adjustTime(-8 * SECS_PER_HOUR);
    }
  }
  if (commTimer.check()) { // see if we have gotten any CAN messages in the last second. display an error if not
//    clear();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,56);
    display.println("Waiting for data...");
    display.setCursor(0,0);
    display.println(GPS.time.age());
    display.println(GPS.location.isUpdated());
    display.println(GPS.location.lat(),5);
    display.println(GPS.location.lng(),5);
    display.display();
    for(int i=0; i<16; i++) {
      leds[i].setRGB(0,0,0); // initialize led ring
    }
    FastLED.show();
    commTimer.reset();
    connectionState = false;
  }
  if (gpsTimer.check()) {
    if (GPS.satellites.value()) {
      gpsLogo = true;
    } else {
      gpsLogo = gpsLogo ^ 1;
    }
    gpsTimer.reset();
  }
  /* touchRead() can take up to 5ms to do its thing, so this is useless..
  if (0 && displayTimer.check()) { // read the buttons
    for (int i=0; i<3; i++) {
      if (touchRead(15 + i) - buttons[i].init > 1000) { // button is pressed
        if (buttons[i].last == 0) { // button was not pressed during last loop, register a "click"
          switch (i) {
            case 0: R_index++; break;
            case 1: ISR_debounce(); break;
            case 2: R_index--; break;
          }
        }
        buttons[i].last = 1;
      } else {
        buttons[i].last = 0;
      }
    }
  }
  */
  if (connectionState && displayTimer.check()) { // main display routine
    R_index=myEnc.read()/4;
//    if (! value_oob() ) {
      switch (B_index) {
      case 0:
//        gauge_vitals();
        break;
      case 1:
//        gauge_single();
        break;
      case 2:
 //       gauge_histogram();
        break;
      case 3:
  //      gauge_debug();
        break;
      //case 4:
      //  gauge_danger();
      //  break;
      case 4:
  //      gauge_menu();
        break;
        //default: write_gauge_3(); break;
      }
//      write_neopixel();

    } else {
 //     gauge_warning();
      FastLED.show();
    }
//    display.display();
 //   displayTimer.reset();
  

  // handle received CAN frames
  if ( CANbus.read(rxmsg) ) {
    commTimer.reset();
    connectionState = true;
    ledBlink();
    switch (rxmsg.id) { // ID's 1520+ are Megasquirt CAN broadcast frames
    case 1520: // 0
      RPM=(int)(word(rxmsg.buf[6], rxmsg.buf[7]));
      break;
    case 1521: // 1
      SPKADV=(int)(word(rxmsg.buf[0], rxmsg.buf[1]));
      indicator[0]=rxmsg.buf[3]; // engine
      AFRtgt=(int)(word(0x00, rxmsg.buf[4]));
      break;
    case 1522: // 2
      Baro=(int)(word(rxmsg.buf[0], rxmsg.buf[1]));
      MAP=(int)(word(rxmsg.buf[2], rxmsg.buf[3]));
      MAT=(int)(word(rxmsg.buf[4], rxmsg.buf[5]));
      CLT=(int)(word(rxmsg.buf[6], rxmsg.buf[7]));
      break;
    case 1523: // 3
      TPS=(int)(word(rxmsg.buf[0], rxmsg.buf[1]));
      BATTV=(int)(word(rxmsg.buf[2], rxmsg.buf[3]));
      break;
    case 1524: // 4
      Knock=(int)(word(rxmsg.buf[0], rxmsg.buf[1]));
      EGOc=(int)(word(rxmsg.buf[2], rxmsg.buf[3]));
      break;
    case 1526: // 6
      IAC=(int)(word(rxmsg.buf[6], rxmsg.buf[7])); //IAC = (IAC * 49) / 125;
    case 1529: // 9
      dwell=(int)(word(rxmsg.buf[4], rxmsg.buf[5]));
      break;
    case 1530: // 10
      indicator[1]=rxmsg.buf[0]; // status 1
      indicator[2]=rxmsg.buf[1]; // status 2
      indicator[3]=rxmsg.buf[2]; // status 3
      indicator[6]=rxmsg.buf[6]; // status 6
      indicator[7]=rxmsg.buf[7]; // status 7
      break;
    case 1533: // Group 13
        oilpres = (float)(word(rxmsg.buf[0], rxmsg.buf[1]));
        waterpres = (float)(word(rxmsg.buf[2], rxmsg.buf[3]));
        oiltempgauge = (float)(word(rxmsg.buf[4], rxmsg.buf[5]));
        sensor4 = (float)(word(rxmsg.buf[6], rxmsg.buf[7]));
    case 1537: // 17
      bstduty=(int)(word(rxmsg.buf[4], rxmsg.buf[5]));
      break;
    case 1548: // 28
      idle_tar=(int)(word(rxmsg.buf[0], rxmsg.buf[1]));
      break;
    case 1551: // 31
      AFR1=(int)(word(0x00, rxmsg.buf[0]));
      AFR2=(int)(word(0x00, rxmsg.buf[1]));
      //afr = rxmsg.buf[0];
      break;
      case 1553: // Group 33
        gear = rxmsg.buf[6];
    case 1574: // 54
      indicator[4]=rxmsg.buf[2]; // cel
      break;
    default: // not a broadcast packet
      if (rxmsg.ext) { //assume this is a normal Megasquirt CAN protocol packet and decode the header
        rxmsg_id.i = rxmsg.id;
        if (rxmsg_id.values.to_id == myCANid) { // is this being sent to us?
          switch (rxmsg_id.values.msg_type) {
          case 1: // MSG_REQ - request data
            // the data required for the MSG_RSP header is packed into the first 3 data bytes
            msg_req_data.bytes.b0 = rxmsg.buf[0];
            msg_req_data.bytes.b1 = rxmsg.buf[1];
            msg_req_data.bytes.b2 = rxmsg.buf[2];
            // Create the tx packet header
            txmsg_id.values.msg_type = 2; // MSG_RSP
            txmsg_id.values.to_id = msCANid; // Megasquirt CAN ID should normally be 0
            txmsg_id.values.from_id = myCANid;
            txmsg_id.values.block = msg_req_data.values.varblk;
            txmsg_id.values.offset = msg_req_data.values.varoffset;
            txmsg.ext = 1;
            txmsg.id = txmsg_id.i;
            txmsg.len = 8;
            // Use the same block and offset as JBPerf IO expander board for compatibility reasons
            // Docs at http://www.jbperf.com/io_extender/firmware/0_1_2/io_extender.ini (or latest version)
            if (rxmsg_id.values.block == 7 && rxmsg_id.values.offset == 110) { // realtime clock
              /*
                 rtc_sec          = scalar, U08,  110, "", 1,0
                 rtc_min          = scalar, U08,  111, "", 1,0
                 rtc_hour         = scalar, U08,  112, "", 1,0
                 rtc_day          = scalar, U08,  113, "", 1,0 // not sure what "day" means. seems to be ignored...
                 rtc_date         = scalar, U08,  114, "", 1,0
                 rtc_month        = scalar, U08,  115, "", 1,0
                 rtc_year         = scalar, U16,  116, "", 1,0
              */
//              if (timeStatus() == timeSet) { // only return clock info if the local clock has actually been set (via GPS or RTC)
//                txmsg.buf[0] = second();
//                txmsg.buf[1] = minute();
//                txmsg.buf[2] = hour();
//                txmsg.buf[3] = 0;
//                txmsg.buf[4] = day();
//                txmsg.buf[5] = month();
//                txmsg.buf[6] = year() / 256;
//                txmsg.buf[7] = year() % 256;
                // send the message!
                CANbus.write(txmsg);
              }
             else if (rxmsg_id.values.block == 7 && rxmsg_id.values.offset == 128) { // gps1
              /*
                 gps_latdeg       = scalar, S08,  128, "", 1,0
                 gps_latmin       = scalar, U08,  129, "", 1,0
                 gps_latmmin      = scalar, U16,  130, "", 1,0
                 gps_londeg       = scalar, U08,  132, "", 1,0
                 gps_lonmin       = scalar, U08,  133, "", 1,0
                 gps_lonmmin      = scalar, U16,  134, "", 1,0
              */
              signed char latdeg;
              unsigned char latmin,lonmin,londeg;
              unsigned int latmmin,lonmmin;
              double intpart;
              latdeg, londeg, latmin, lonmin, latmmin, lonmmin = 0;
              latdeg = GPS.location.rawLat().deg;
              latmin = (GPS.location.lat()-GPS.location.rawLat().deg)*60;
              latmmin = ((GPS.location.rawLat().billionths * 3 / 50000) - latmin * 1000) * 10;
              londeg = GPS.location.rawLng().deg;
              lonmin = modf(abs(GPS.location.lng()),&intpart)*60,3;
              lonmmin = ((GPS.location.rawLng().billionths * 3 / 50000) - lonmin * 1000) * 10;
              txmsg.buf[0] = latdeg;
              txmsg.buf[1] = latmin;
              txmsg.buf[2] = latmmin / 256;
              txmsg.buf[3] = latmmin % 256;
              txmsg.buf[4] = londeg;
              txmsg.buf[5] = lonmin;
              txmsg.buf[6] = lonmmin / 256;
              txmsg.buf[7] = lonmmin % 256;
              // send the message!
              CANbus.write(txmsg);
            } else if (rxmsg_id.values.block == 7 && rxmsg_id.values.offset == 136) { // gps2
              /*
                 gps_lonEW        = scalar, U08,  136, "", 1,0 // bit 0 = E/W
                 gps_altk         = scalar, S08,  137, "", 1,0
                 gps_altm         = scalar, S16,  138, "", 0.1,0
                 gps_speedkm      = scalar, U16,  140, "", 0.1,0
                 gps_course       = scalar, U16,  142, "", 0.1,0
              */
              txmsg.buf[0] = GPS.location.rawLng().negative ? 1 : 0;
              txmsg.buf[1] = GPS.altitude.kilometers();
              txmsg.buf[2] = GPS.altitude.meters() * 10 / 256;
              txmsg.buf[3] = (int)GPS.altitude.meters() * 10 % 256;
              txmsg.buf[4] = GPS.speed.kmph() / 256;
              txmsg.buf[5] = (int)GPS.speed.kmph() % 256;
              txmsg.buf[6] = GPS.course.deg() / 256;
              txmsg.buf[7] = (int)GPS.course.deg() % 256;
              // send the message!
              CANbus.write(txmsg);
            } else if (rxmsg_id.values.block == 7 && rxmsg_id.values.offset == 2) { // ADC 1-4 - accelerometer
              sensors_event_t event;
              accel.getEvent(&event);
              // normalize +/- 4G to a 12 bit unsigned int value
              accel_x = (event.acceleration.x / 9.8 * 1023) + 2047;
              accel_y = (event.acceleration.y / 9.8 * 1023) + 2047;
              accel_z = (event.acceleration.z / 9.8 * 1023) + 2047;
              txmsg.buf[0] = accel_x / 256;
              txmsg.buf[1] = accel_x % 256;
              txmsg.buf[2] = accel_y / 256;
              txmsg.buf[3] = accel_y % 256;
              txmsg.buf[4] = accel_z / 256;
              txmsg.buf[5] = accel_z % 256;
              txmsg.buf[6] = 0;
              txmsg.buf[7] = 0;
              // send the message!
              CANbus.write(txmsg);
            } else if (rxmsg_id.values.block == 7 && rxmsg_id.values.offset == 10) { // ADC 1-4 - accelerometer
              txmsg.buf[0] = oiltemp / 256;
              txmsg.buf[1] = oiltemp % 256;
              // send the message!
              CANbus.write(txmsg);
            }
            break;
          }
        }
      } else {
        Serial.write("ID: ");
        Serial.print(rxmsg.id);
      }
//    case 70: indicator[5]=databuffer[0]; break; // port status
// this was in the original code, but i can't find it's equivalent in http://www.msextra.com/doc/general/files/Megasquirt_CAN_broadcast_2014-03-10.pdf
// and it was never used regardless.
    }
  }
  if (Serial.available()) {
    byte incomingByte = Serial.read();
    switch(incomingByte) {
    case '1':
      // increment encoder
      R_index--;
      break;
    case '2':
      // decrement encoder
      R_index++;
      break;
    case '3':
      // press button
      //void ISR_debounce () {
//if((long)(millis() - last_millis) >= (debouncing_time * 10)) {
      clear();
      if (S_index != 0) {
        S_index=0; // deselect brightness
        clear();
        return;
      }
      if (B_index < 5) {
        B_index++;
        M_index=0;
        R_index=0;
        myEnc.write(0);
      }
      if (B_index == 5) {
        //menu settings
        B_index=5;
        if (R_index >= 3) {
          //save selected - return to main menu
          M_index=0;
          B_index=0;
          R_index=0;
          myEnc.write(0);
          S_index=0;
        }
        if (R_index == 1) {
          S_index=1; // select brightness
        }
        if (R_index == 2) {
          S_index=2; // select text size, though not implemented
        }
      } // end B_index5
//} else {return; }//end button
// last_millis = millis();
//}//end debounce
      break;
    }
    Serial.print(B_index);
    Serial.print(" ");
    Serial.print(R_index);
    Serial.print(" ");
    Serial.print(M_index);
    Serial.print(" ");
    Serial.println(S_index);
  }

  gauge_display();
}

  elapsedMillis DisplayTime; //Establish a timer to prevent unnecessary screen rewrites
  elapsedMillis timeElapsed; //start timer for flash RPM

  void gauge_display() {

  if ( DisplayTime < 50 ) return;
  DisplayTime = 0;

  AFRtgterr1 = abs (AFR1 - AFRtgt);
  AFRtgterr2 = abs (AFR2 - AFRtgt);

  //Serial.println(rxmsg.id); Any type of data you want to go to Serial Monitor place here

  GPSspeed = GPS.speed.kmph() * 0.621371;
  
  Serial2.print("t2.txt="); //Oil Pressure readout
  Serial2.write(0x22);
  Serial2.print(oilpres);
  Serial2.write(0x22);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("t4.txt="); //Water temp readout
  Serial2.write(0x22);
  Serial2.print(CLT / 10);
  Serial2.write(0x22);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("t3.txt="); //Oil temp readout
  Serial2.write(0x22);
  Serial2.print(oiltempgauge);
  Serial2.write(0x22);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("t5.txt=4"); //Gear indicator
  Serial2.write(0x22);
  Serial2.print(gear);
  Serial2.write(0x22);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("t1.txt="); //AFR1 readout
  Serial2.write(0x22);
  Serial2.print(AFR1);
  Serial2.write(0x22);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("t0.txt="); //AFR2 readout
  Serial2.write(0x22);
  Serial2.print(AFR2);
  Serial2.write(0x22);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("t6.txt="); //Speed readout
  Serial2.write(0x22);
  Serial2.print(GPSspeed);
  Serial2.write(0x22);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  if (AFRtgterr1 < 0.5)
    AFR1_gauge_green();

  if ((AFRtgterr1 < 1.0) && (AFRtgterr1 >= 0.5))
    AFR1_gauge_yellow();

  if (AFRtgterr1 >= 1.0)
    AFR1_gauge_red();

  if (AFRtgterr2 < 0.5)
    AFR2_gauge_green();

  if ((AFRtgterr2 < 1.0) && (AFRtgterr1 >= 0.5))
    AFR2_gauge_yellow();

  if (AFRtgterr2 >= 1.0)
    AFR2_gauge_red();

 // Thresholds for picture selection for Engine RPM

  if (RPM <= 4000)
    RPM_green_1 = 1,
    RPM_green_2 = 1,
    RPM_green_3 = 1,
    RPM_yellow_1 = 1,
    RPM_yellow_2 = 1,
    RPM_red = 1,
    RPM_solid();

  if ((RPM > 4000) && (RPM <= 4750))
    RPM_green_1 = 2,
    RPM_green_2 = 1,
    RPM_green_3 = 1,
    RPM_yellow_1 = 1,
    RPM_yellow_2 = 1,
    RPM_red = 1,
    RPM_solid();

  if ((RPM > 4750) && (RPM <= 5250))
    RPM_green_1 = 2,
    RPM_green_2 = 2,
    RPM_green_3 = 1,
    RPM_yellow_1 = 1,
    RPM_yellow_2 = 1,
    RPM_red = 1,
    RPM_solid();

  if ((RPM > 5250) && (RPM <= 5500))
    RPM_green_1 = 2,
    RPM_green_2 = 2,
    RPM_green_3 = 2,
    RPM_yellow_1 = 1,
    RPM_yellow_2 = 1,
    RPM_red = 1,
    RPM_solid();

  if ((RPM > 5500) && (RPM <= 5750))
    RPM_green_1 = 2,
    RPM_green_2 = 2,
    RPM_green_3 = 2,
    RPM_yellow_1 = 3,
    RPM_yellow_2 = 1,
    RPM_red = 1,
    RPM_solid();

  if ((RPM > 5750) && (RPM <= 6000))
    RPM_green_1 = 2,
    RPM_green_2 = 2,
    RPM_green_3 = 2,
    RPM_yellow_1 = 3,
    RPM_yellow_2 = 3,
    RPM_red = 1,
    RPM_solid();
    
  if ((RPM > 6000) && (RPM <= 6250))
    RPM_green_1 = 2,
    RPM_green_2 = 2,
    RPM_green_3 = 2,
    RPM_yellow_1 = 3,
    RPM_yellow_2 = 3,
    RPM_red = 4,
    RPM_solid();

  if ((RPM > 6250) && ((flash_int % 2) == 0))
    RPM_flash_on();

  if ((RPM > 6250) && ((flash_int % 2) != 0))
    RPM_flash_off();
  }

  void RPM_solid() {

  Serial2.print("p0a.pic="); //Water temp readout
  Serial2.print(RPM_green_1);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p0b.pic=");
  Serial2.print(RPM_green_1);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p1a.pic=");
  Serial2.print(RPM_green_2);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p1b.pic=");
  Serial2.print(RPM_green_2);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);
  
  Serial2.print("p2a.pic=");
  Serial2.print(RPM_green_3);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p2b.pic=");
  Serial2.print(RPM_green_3);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p3a.pic=");
  Serial2.print(RPM_yellow_1);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p3b.pic=");
  Serial2.print(RPM_yellow_1);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);
  
  Serial2.print("p4a.pic=");
  Serial2.print(RPM_yellow_2);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p4b.pic=");
  Serial2.print(RPM_yellow_2);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p5a.pic=");
  Serial2.print(RPM_red);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p5b.pic=");
  Serial2.print(RPM_red);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  }

  void RPM_flash_on() {

  if ( timeElapsed < 100 ) return;
   timeElapsed = 0;

  Serial2.print("p0a.pic=4");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p0b.pic=4");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p1a.pic=4");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p1b.pic=4");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);
  
  Serial2.print("p2a.pic=4");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p2b.pic=4");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p3a.pic=4");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p3b.pic=4");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);
  
  Serial2.print("p4a.pic=4");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p4b.pic=4");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p5a.pic=4");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p5b.pic=4");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  flash_int = flash_int + 1;

  timeElapsed = 0;
  }

  void RPM_flash_off(){

  if ( timeElapsed < 100 ) return;
   timeElapsed = 0;
  
  Serial2.print("p0a.pic=1");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p0b.pic=1");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p1a.pic=1");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p1b.pic=1");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);
  
  Serial2.print("p2a.pic=1");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p2b.pic=1");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p3a.pic=1");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p3b.pic=1");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);
  
  Serial2.print("p4a.pic=1");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p4b.pic=1");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p5a.pic=1");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("p5b.pic=1");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);


  flash_int = flash_int + 1;

  }

  void AFR1_gauge_green() {

  Serial2.print("t1.pco=1024");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  }

  void AFR1_gauge_yellow() {

  Serial2.print("t1.pco=65504");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  }

  void AFR1_gauge_red() {

  Serial2.print("t1.pco=63488");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  }

  void AFR2_gauge_green() {

  Serial2.print("t0.pco=1024");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  }

  void AFR2_gauge_yellow() {

  Serial2.print("t0.pco=65504");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  }

  void AFR2_gauge_red() {

  Serial2.print("t0.pco=63488");
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  }


void clear() {
  // used where display.clearDisplay() would normally be called so that we can make sure certain status
  // indicators are always shown (GPS, stc)
  display.clearDisplay();
  if (B_index == 0) { // there is no room for the GPS status on other than the main screen
    display.setCursor(116,0);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.print(GPS.satellites.value());
    if (gpsLogo)
      display.drawBitmap(100,0, gps, 16, 16, 1);
  }
}

void divby10(int val) {
  byte length;

  itoa(val, tempchars, 10);
  length=strlen(tempchars);

  tempchars[length + 1]=tempchars[length]; // null shift right
  tempchars[length]=tempchars[length - 1]; //
  tempchars[length - 1]='.';
}

void ISR_debounce () { // interrupt handler for the encoder button
  if((long)(millis() - last_millis) >= (debouncing_time * 10)) {
    clear();
    if (S_index != 0) {
      S_index=0; // deselect brightness
      clear();
      return;
    }
    if (B_index < 4) {
      B_index++;
      M_index=0;
      R_index=0;
      myEnc.write(0);
    }
    if (B_index == 4) {
      //menu settings
      if (R_index >= 3) {
        //save selected - return to main menu
        M_index=0;
        B_index=0;
        R_index=0;
        myEnc.write(0);
        S_index=0;
      }
      if (R_index == 1) {
        S_index=1; // select brightness
      }
      if (R_index == 2) {
        S_index=2; // select text size, though not implemented
      }
    } // end B_index5
  } else {
    return;  //end button
  }
  last_millis = millis();
}//end debounce

void gauge_histogram() {
  byte val;

  val = AFR - 100; // temporary

  if (millis() > (validity_window + 80)) { // 10hz update time
    clear();

    if (R_index > 2 || R_index < 0) {
      R_index=0;
      myEnc.write(0);
    }
    switch (R_index) {
    // 0-50 value normalization
    case 0:
      val = (AFR - 100) / 2; // real rough estimation here here of afr on a 0-50 scale
      if (val > 50) {
        val=50;
      }
      break;
    case 1:
      val = ((MAP/10) - 30) / 4;
      if (val > 50) {
        val = 50;
      }
      break;
    case 2:
      val = (MAT/10) / 4;
      if (val > 50) {
        val = 50;
      }
      break;
    }

    histogram_index++;
    if (histogram_index >=64) {
      histogram_index=0;
    }
    histogram[histogram_index]=val;

    for (byte i = 0; i < 64; i++) {
      int x = histogram_index - i;
      if ( x < 0) {
        x = 64 + histogram_index - i;
      }
      display.drawFastVLine((128 - (i * 2)), (64 - histogram[x]), 64, WHITE);
      display.drawFastVLine((127 - (i * 2)), (64 - histogram[x]), 64, WHITE);
    }

    display.setCursor(8,0);
    display.setTextSize(2);
    display.setTextColor(WHITE);

    switch (R_index) {
    case 0:
      display.print("AFR ");
      divby10(AFR);
      display.print(tempchars);
      display.drawFastHLine(0, 40, 128, WHITE); // stoich 14.7 line
      for (byte x=1; x < 128; x = x + 2) {
        display.drawPixel(x, 40, BLACK);
      }
      break;
    case 1:
      display.print("MAP ");
      display.print(MAP/10);
      display.drawFastHLine(0, 47, 128, WHITE); // Baro line.. roughly 98kpa
      for (byte x=1; x < 128; x = x + 2) {
        display.drawPixel(x, 47, BLACK);
      }
      break;
    case 2:
      display.print("MAT ");
      display.print(MAT / 10);
      break;
    }

    /*  refresh rate debug
      display.setCursor(50, 0);
      display.setTextSize(1);
      display.setTextColor(WHITE);
      display.print(" t");
      display.print((millis() - validity_window));
      display.print(" v");
      display.print(val);
      */
    validity_window=millis();
//  display.display();
  }//end millis

}

boolean value_oob() {
  if (RPM > 100) {
    if ((CLT/10) > 260) return 1;
//    if (OILP < 7 ) return 1;
//    if (RPM > 7600 ) return 1;
//    if (EGT > 1550 ) return 1;
    //if (indicator[4] != 0) return 1;
  } else {
    return false;
  }
  if ( bitRead(indicator[2],6) == 1) return true; // overboost
  //if (RPM > 6800) return true;
  return false;
}

void gauge_warning() {
  byte dlength, llength;
  int midpos;

  clear();

  if (RPM > 6800) {
    dlength=4;
    llength=3;
    midpos=(63 - ((dlength * 23) / 2));
    display.setTextColor(WHITE);
    display.setCursor(midpos,0);
    display.setTextSize(4);
    display.print(RPM);

    display.setTextSize(2);
    display.setCursor(8, (63 - 15));
    display.print("RPM");

    for (byte i = 0; i < 16; i++) {
      leds[i].setRGB(0, 0, 0);
    } // zero out

    byte i = ((RPM - 6800) / 50);

    for (byte p=0; p < i; p++) {
      if (i <= 2) {
        leds[p+14].setRGB(((255 * neo_brightness) / 16), 0, 0);
      } else {
        leds[14].setRGB(((255 * neo_brightness) / 16), 0, 0);
        leds[15].setRGB(((255 * neo_brightness) / 16), 0, 0);
        leds[p-2].setRGB(((255 * neo_brightness) / 16), 0, 0);
      }
    }

  }

  if ((CLT/10) > 260) {
    dlength=3;
    llength=3;
    midpos=(63 - ((dlength * 23) / 2));
    display.setTextColor(WHITE);
    display.setCursor(midpos,0);
    display.setTextSize(4);
    display.print(CLT/10);

    display.setTextSize(2);
    display.setCursor(8, (63 - 15));
    display.print("CLT");
    for (byte i=0; i < 16; i++) {
      neogauge(999, i, 0);
    }
  }

  if (bitRead(indicator[4], 0)) {
    display.setTextColor(WHITE);
    dlength=3;
    midpos=(63 - ((dlength * 23) / 2));
    display.setCursor(29, 0);
    display.setTextSize(4);
    display.print("MAP");
    display.setTextSize(2);
    display.setCursor(8, 48);
    display.print("Error");
    for (byte i=0; i < 16; i++) {
      neogauge(999, i, 0);
    }
  }

  if (bitRead(indicator[4], 1)) {
    display.setTextColor(WHITE);
    dlength=3;
    midpos=(63 - ((dlength * 23) / 2));
    display.setCursor(29, 0);
    display.setTextSize(4);
    display.print("MAT");
    display.setTextSize(2);
    display.setCursor(8, 48);
    display.print("Error");
    for (byte i=0; i < 16; i++) {
      neogauge(999, i, 0);
    }

  }

  if (bitRead(indicator[4], 2)) {
    display.setTextColor(WHITE);
    dlength=3;
    midpos=(63 - ((dlength * 23) / 2));
    display.setCursor(29, 0);
    display.setTextSize(4);
    display.print("CLT");
    display.setTextSize(2);
    display.setCursor(8, 48);
    display.print("Error");
    for (byte i=0; i < 16; i++) {
      neogauge(999, i, 0);
    }
  }

  if (bitRead(indicator[4], 3)) {
    display.setTextColor(WHITE);
    dlength=3;
    midpos=(63 - ((dlength * 23) / 2));
    display.setCursor(29, 0);
    display.setTextSize(4);
    display.print("TPS");
    display.setTextSize(2);
    display.setCursor(8, 48);
    display.print("Error");
    for (byte i=0; i < 16; i++) {
      neogauge(999, i, 0);
    }

  }

  if (bitRead(indicator[4], 4)) {
    display.setTextColor(WHITE);
    dlength=3;
    midpos=(63 - ((dlength * 23) / 2));
    display.setCursor(29, 0);
    display.setTextSize(4);
    display.print("BATT");
    display.setTextSize(2);
    display.setCursor(8, 48);
    display.print("Error");
    for (byte i=0; i < 16; i++) {
      neogauge(999, i, 0);
    }
  }

  if (bitRead(indicator[4], 5)) {
    display.setTextColor(WHITE);
    dlength=3;
    midpos=(63 - ((dlength * 23) / 2));
    display.setCursor(29, 0);
    display.setTextSize(4);
    display.print("AFR");
    display.setTextSize(2);
    display.setCursor(8, 48);
    display.print("Error");
    for (byte i=0; i < 16; i++) {
      neogauge(999, i, 0);
    }
  }

  if (bitRead(indicator[4], 6)) {
    display.setTextColor(WHITE);
    dlength=3;
    midpos=(63 - ((dlength * 23) / 2));
    display.setCursor(29, 0);
    display.setTextSize(4);
    display.print("Sync");
    display.setTextSize(2);
    display.setCursor(8, 48);
    display.print("Error");
    for (byte i=0; i < 16; i++) {
      neogauge(999, i, 0);
    }
  }

  if (bitRead(indicator[4], 7)) {
    display.setTextColor(WHITE);
    dlength=3;
    midpos=(63 - ((dlength * 23) / 2));
    display.setCursor(29, 0);
    display.setTextSize(4);
    display.print("EGT");
    display.setTextSize(2);
    display.setCursor(8, 48);
    display.print("Error");
    for (byte i=0; i < 16; i++) {
      neogauge(999, i, 0);
    }
  }

  if ( bitRead(indicator[2],6) == 1) {
    gauge_danger();
  }

}


void gauge_debug() {
  clear();
  display.setCursor(32,0);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.print("m");
  display.print(M_index);
  display.print("r");
  display.print(R_index);
  display.print("s");
  display.print(S_index);
  display.print("b");
  display.println(B_index);
  signed char latdeg;
  unsigned char latmin,lonmin,londeg;
  unsigned int latmmin,lonmmin;
  double intpart;
  latdeg, londeg, latmin, lonmin, latmmin, lonmmin = 0;
  latdeg = GPS.location.rawLat().negative ? 0 - GPS.location.rawLat().deg : GPS.location.rawLat().deg;
  latmin = (GPS.location.lat()-GPS.location.rawLat().deg)*60;
  latmmin = (GPS.location.rawLat().billionths * 3 / 50000) - latmin * 1000;
  londeg = GPS.location.rawLng().deg;
  lonmin = modf(abs(GPS.location.lng()),&intpart)*60,3;
  lonmmin = (GPS.location.rawLng().billionths * 3 / 50000) - lonmin * 1000;
  display.println(GPS.location.lng(),5);
  display.println(modf(abs(GPS.location.lng()),&intpart)*60,3);
  display.println(lonmin);
  display.println(lonmmin);
  display.setTextSize(2);
}

void gauge_vitals() {
  //hard coded for style
  // fonts are 5x7 * textsize
  // size 1 .. 5 x 7
  // size 2 .. 10 x 14
  //Vitals - AFR, RPM, MAP,
  clear();

//  display.drawLine(63, 0, 63, 55, WHITE); //vert centerline
//  display.drawLine(0, 31, 127, 31, WHITE); //horiz centerline

  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,0);

  display.setCursor(41, 0); // 4 char normally - 4 * 10 = 40, - 128 = 88, /2 = 44
  display.println(RPM);
  display.setTextSize(1);
  display.setCursor(21, 7);
  display.print("RPM");

  //line2
  display.setCursor(2, 26);
  display.setTextSize(1);
  display.print("AFR");
  display.setCursor(20, 19);
  display.setTextSize(2);
  divby10(AFR);
  display.print(tempchars);
  /*
  display.setCursor(69,19);
  display.setTextSize(1);
  display.print("AFR");
  display.drawTriangle(72, 32, 77, 27, 82, 32, WHITE);
  divby10(AFR - AFRtgt);
  display.setTextSize(1);
  display.setCursor(88,26);
  display.print(tempchars);
  display.setCursor(88,18);
  divby10(AFRtgt);
  display.print(tempchars);
  */
  display.setCursor(71, 25);
  display.setTextSize(1);
  display.print("CLT");
  display.setCursor(88, 18);
  display.setTextSize(2);
  display.print(CLT/10);

  //line3
  display.setCursor(0, 40);
  display.setTextSize(1);
  display.print("MAP");
  display.setCursor(20, 40);
  display.setTextSize(2);
//  divby10(MAP); doesn't need single point resolution
  display.print(MAP/10);

  if ( bitRead(indicator[2],7) == 1) { // contextual gauge - if idle on, show IAC%
    display.setCursor(72, 47);
    display.setTextSize(1);
    display.print("IAC");
    display.setCursor(92, 40);
    display.setTextSize(2);
    display.print(IAC);
  } else if (MAP > Baro) {
    int psi;
    display.setCursor(72, 38);
    display.setTextSize(1);
    display.print("PSI");
    display.setCursor(92, 38);
    display.setTextSize(1);
    // 6.895kpa = 1psi
    psi = MAP - Baro;
    psi=(psi * 200) / 1379;
    divby10(psi);
    display.print(tempchars);

    display.setCursor(72, 47);
    display.setTextSize(1);
    display.print("MAT");
    display.setCursor(92, 47);
    display.print(MAT/10);

  }  else {
    display.setCursor(72, 47);
    display.setTextSize(1);
    display.print("MAT");
    display.setCursor(92,40);
    display.setTextSize(2);
//    divby10(MAT);
    display.print(MAT/10);
  }
  //
  //display.display();
  gauge_bottom();
  //Serial.println("End gauge display");
//  display.display();
} // end gauge_vitals

void gauge_bottom() {

  display.setTextSize(1);
  display.drawFastHLine(1, (63 - 7), 126, WHITE);
  display.setCursor(0, 57);
  display.setTextColor(BLACK, WHITE);
//display.print("CELIDLFANKNKBSTAFR");
  display.setTextColor(BLACK, WHITE);

//CEL
  if ( indicator[4] != 0 ) {
    display.setTextColor(BLACK, WHITE);
    display.drawFastVLine(2, 57, 8, WHITE);
  } else {
    display.setTextColor(WHITE);
  }
  display.setCursor(3, 57);
  display.print("CEL");
  display.drawFastVLine(1, 57, 8, WHITE);

//FAN
  if ( bitRead(indicator[6],6) == 1) {
    display.setTextColor(BLACK, WHITE);
    display.drawFastVLine(23, 57, 8, WHITE);
    display.drawFastVLine(22, 57, 8, WHITE);
    display.drawFastVLine(42, 57, 8, WHITE);
  } else {
    display.setTextColor(WHITE);
  }
  display.setCursor(24, 57);
  display.print("FAN");
  display.drawFastVLine(21, 57, 8, WHITE);

//Idle
  if ( bitRead(indicator[2],7) == 1) {
    display.setTextColor(BLACK, WHITE);
    display.drawFastVLine(44, 57, 8, WHITE);
  } else {
    display.setTextColor(WHITE);
  }
  display.setCursor(45, 57);
  display.print("Idl");
  display.drawFastVLine(43, 57, 8, WHITE);

//Knock
  if ( bitRead(indicator[7],4) == 1) {
    display.setTextColor(BLACK, WHITE);
    display.drawFastVLine(65, 57, 8, WHITE);
    display.drawFastVLine(64, 57, 8, WHITE);
    display.drawFastVLine(84, 57, 8, WHITE);
  } else {
    display.setTextColor(WHITE);
  }
  display.setCursor(66, 57);
  display.print("KnK");
  display.drawFastVLine(63, 57, 8, WHITE);

//Overboost
  if ( bitRead(indicator[2],6) == 1) {
    display.setTextColor(BLACK, WHITE);
    display.drawFastVLine(87, 57, 8, WHITE);
    display.drawFastVLine(86, 57, 8, WHITE);
  } else {
    display.setTextColor(WHITE);
  }
  display.setCursor(88, 57);
  display.print("Bst");
  display.drawFastVLine(85, 57, 8, WHITE);

//WUE
  if ( bitRead(indicator[0],3) == 1) {
    display.setTextColor(BLACK, WHITE);
    display.drawFastVLine(107, 57, 8, WHITE);
  } else {
    display.setTextColor(WHITE);
  }
  display.setCursor(108, 57);
  display.print("WUE");
  display.drawFastVLine(106, 57, 8, WHITE);
  display.drawFastVLine(126, 57, 8, WHITE);

//Serial.println(".");
//FAN, WUE, ASE, CEL, Idl, Knk, over boost
//CEL - Idl - FAN - KnK - BST - AFR
  display.display();
}

void gauge_single() {
  byte mult_test;
  char data[10];
  String label;
  byte temp_index;
  clear();
//  unsigned int RPM, CLT, MAP, MAT, SPKADV, BATTV, TPS, Knock, Baro, EGOc, IAC, dwell, bstduty, idle_tar;
//  int AFR, AFRtgt;
  if (R_index < 0) {
    myEnc.write(0);
    R_index=0;
  }

  if (R_index <= 15) {
    switch (R_index) {
    case 0:
      label="RPM";
      itoa(RPM, data, 10);
      break;
    case 1:
      label="AFR";
      divby10(AFR);
      strcpy(data, tempchars);
      break;
    case 2:
      label="Coolant";
      divby10(CLT);
      strcpy(data, tempchars);
      break;
    case 3:
      label="MAP";
      divby10(MAP);
      strcpy(data, tempchars);
      break;
    case 4:
      label="MAT";
      divby10(MAT);
      strcpy(data, tempchars);
      break;
    case 5:
      label="Timing";
      divby10(SPKADV);
      strcpy(data, tempchars);
      break;
    case 6:
      label="Voltage";
      divby10(BATTV);
      strcpy(data, tempchars);
      break;
    case 7:
      label="TPS";
      divby10(TPS);
      strcpy(data, tempchars);
      break;
    case 8:
      label="Knock";
      divby10(Knock);
      strcpy(data, tempchars);
      break;
    case 9:
      label="Barometer";
      divby10(Baro);
      strcpy(data, tempchars);
      break;
    case 10:
      label="EGO Corr";
      divby10(EGOc);
      strcpy(data, tempchars);
      break;
    case 11:
      label="IAC";
      itoa(IAC, data, 10);
      break;
    case 12:
      label="Spark Dwell";
      divby10(dwell);
      strcpy(data, tempchars);
      break;
    case 13:
      label="Boost Duty";
      itoa(bstduty, data, 10);
      break;
    case 14:
      label="Idle Target";
      itoa(idle_tar, data, 10);
      break;
    case 15:
      label="AFR Target";
      divby10(AFRtgt);
      strcpy(data, tempchars);
      break;
    }
  } else {
    temp_index = R_index - 15;
    char temporary[15];
    byte sbyte, bitp, dbit;
    strcpy_P(temporary, MSDataBin[temp_index].name);
    label=temporary;

    sbyte=pgm_read_byte(&MSDataBin[temp_index].sbyte);
    bitp=pgm_read_byte(&MSDataBin[temp_index].bitp);
    dbit=bitRead(indicator[sbyte], bitp);
    if ( dbit == 1 ) {
      data[0]='O';
      data[1]='n';
      data[2]='\0';
    } else {
      data[0]='O';
      data[1]='f';
      data[2]='f';
      data[3]='\0';
    }

  }

  byte dlength=strlen(data);
  byte llength=label.length();
  int midpos;

//dlength * (width of font) / 2 -1
//size 2 = 11
//size 3 = 17
//size 4 = 23

  midpos = (63 - ((dlength * 23)/ 2));

  display.setTextColor(WHITE);
  display.setCursor(midpos,0);
  display.setTextSize(4);
  display.print(data);

  display.setTextSize(2);
  display.setCursor(8, (63 - 15));
  display.print(label);

//unsigned int MAP_HI, Knock_HI, RPM_HI, CLT_HI, MAT_HI;
//int AFR_HI, AFR_LO;

//Additional Hi-Lo's for niftiness
  if (R_index == 0) {
    if (millis() > (validity_window + 30000)) {
      //after 30 seconds from latest high, set new high
      RPM_HI = RPM;
      validity_window=millis();
    }
    if (RPM > RPM_HI) {
      RPM_HI = RPM;
      validity_window=millis();
    }
    display.setTextSize(2);
    display.setCursor((127 - 48), 31);
    display.print(RPM_HI);
  }

  if (R_index == 1) {
    if (millis() > (validity_window + 30000)) {
      //after 30 seconds from latest high, set new high
      AFR_HI = AFR;
      validity_window=millis();
    }
    if (millis() > (validity_window2 + 30000)) {
      //after 30 seconds from latest high, set new high
      AFR_LO = AFR;
      validity_window2=millis();
    }
    if (AFR > AFR_HI) {
      AFR_HI = AFR;
      validity_window=millis();
    }
    if (AFR < AFR_LO) {
      AFR_LO = AFR;
      validity_window2=millis();
    }
    display.setTextSize(2);
    display.setCursor(0, 31);
    divby10(AFR_LO);
    display.print(tempchars);
    display.setCursor((127 - 48), 31);
    divby10(AFR_HI);
    display.print(tempchars);
  }

  if (R_index == 2) {
    if (millis() > (validity_window + 30000)) {
      //after 30 seconds from latest high, set new high
      CLT_HI = CLT;
      validity_window=millis();
    }
    if (CLT > CLT_HI) {
      CLT_HI = CLT;
      validity_window=millis();
    }
    display.setTextSize(2);
    display.setCursor((127 - 60), 31);
    divby10(CLT_HI);
    display.print(tempchars);
  }

  if (R_index == 3) {
    if (millis() > (validity_window + 30000)) {
      //after 30 seconds from latest high, set new high
      MAP_HI = MAP;
      validity_window=millis();
    }
    if (MAP > MAP_HI) {
      MAP_HI = MAP;
      validity_window=millis();
    }
    display.setTextSize(2);
    display.setCursor((127 - 48), 31);
    divby10(MAP_HI);
    display.print(tempchars);
  }

  if (R_index == 4) {
    if (millis() > (validity_window + 30000)) {
      //after 30 seconds from latest high, set new high
      MAT_HI = MAT;
      validity_window=millis();
    }
    if (MAT > MAT_HI) {
      MAT_HI = MAT;
      validity_window=millis();
    }
    display.setTextSize(2);
    display.setCursor((127 - 48), 31);
    divby10(MAT_HI);
    display.print(tempchars);
  }

  if (R_index == 8) {
    if (millis() > (validity_window + 30000)) {
      //after 30 seconds from latest high, set new high
      Knock_HI = Knock;
      validity_window=millis();
    }
    if (Knock > Knock_HI) {
      Knock_HI = Knock;
      validity_window=millis();
    }
    display.setTextSize(2);
    display.setCursor((127 - 48), 31);
    divby10(Knock_HI);
    display.print(tempchars);
  }
// display.display();
}//end gauge_single

void gauge_menu() {
//display.setTextSize(1);display.setTextColor(WHITE);display.print("m");display.print(M_index); display.print("r"); display.print(R_index);display.print("s"); display.print(S_index);display.print("b");display.println(B_index);display.setTextSize(2);

  if (R_index < 0) {
    R_index = 0;
  }

  display.setTextColor(WHITE);
  clear();
  display.setCursor(0,0);
  display.setTextSize(2);

  if (S_index == 0) {
    if (R_index > 3) {
      R_index = 3;
    }
    switch (R_index) {

    case 0:
      //line1
      display.setTextColor(BLACK, WHITE);
      display.println("_Menu");
      //line2
      display.setTextColor(WHITE);
      display.print("Lum: ");
      display.println(neo_brightness);
      //line3
      display.print("Text: ");
      display.println(g_textsize);
      //line4
      display.print("Save");
      display.display();
      break;

    case 1: //brightness selected
      //line1
      display.setTextColor(WHITE);
      display.println("_Menu");
      //line2
      display.setTextColor(BLACK, WHITE);
      display.print("Lum: ");
      display.println(neo_brightness);
      //line3
      display.setTextColor(WHITE);
      display.print("Text: ");
      display.println(g_textsize);
      //line4
      display.print("Save");
      display.display();
      break;

    case 2: //text size selected
      //line1
      display.setTextColor(WHITE);
      display.println("_Menu");
      //line2
      display.print("Lum: ");
      display.println(neo_brightness);
      //line3
      display.setTextColor(BLACK, WHITE);
      display.print("Text: ");
      display.println(g_textsize);
      //line4
      display.setTextColor(WHITE);
      display.print("Save");
      display.display();
      break;

    case 3: //save selected
      //line1
      display.setTextColor(WHITE);
      display.println("_Menu");
      //line2
      display.print("Lum: ");
      display.println(neo_brightness);
      //line3
      display.setTextColor(WHITE);
      display.print("Text: ");
      display.println(g_textsize);
      //line4
      display.setTextColor(BLACK, WHITE);
      display.print("Save");
      display.display();
      break;
    } //end switch
  } // end S_index 0

  if (S_index == 1) {
    neo_brightness=R_index;
    clear();
    display.setCursor(0,0);
    if (R_index > 8) {
      R_index = 8;
      myEnc.write(8*4);
      neo_brightness = 8;
    }
    if (R_index < 1) {
      R_index = 1;
      myEnc.write(1);
      neo_brightness=1;
    }
    //line1
    display.setTextColor(WHITE);
    display.println("_Menu");
    //line2
    display.setTextColor(BLACK, WHITE);
    display.print("Lum: ");
    display.println(neo_brightness);
    //line3
    display.setTextColor(WHITE);
    display.print("Text: ");
    display.println(g_textsize);
    //line4
    display.print("Save");
    display.display();
  }// brightness selection, end S_index 1

  if (S_index == 2) {
//  temp=M_index;
    g_textsize=R_index;
    /*  if (R_index > 4) {
        R_index = 4;
        myEnc.write(16);
        g_textsize = 4;
        }*/
    if (R_index < 1) {
      R_index = 1;
      myEnc.write(4);
      g_textsize=1;
    }
    //line1
    display.setTextColor(WHITE);
    display.println("_Menu");
    //line2
    display.print("Lum: ");
    display.println(neo_brightness);
    //line3
    display.setTextColor(BLACK, WHITE);
    display.print("Text: ");
    display.println(g_textsize);
    //line4
    display.setTextColor(WHITE);
    display.print("Save");
    display.display();
  }// text size selection S_index 2
} // end gauge_menu

void gauge_danger() {
  display.setTextSize(2);
  display.setTextColor(WHITE);
  clear();
  display.setCursor(0,0);

  display.setCursor(4,0);
  display.setTextSize(2);
  display.print("Warning");
  display.print("!");
  display.print("!");
  display.print("!");

  display.setCursor(10,28);
  display.print("Danger to");
  display.setCursor(12,45);
  display.println("Manifold");
//display.display();
}

int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

void bytePrint(byte victim) {
  boolean temp;
  Serial.print("b");
  for (int x = 7; x >=0; x--) {
    temp=bitRead(victim,x);
    Serial.print(temp,BIN);
  }
}


void neogauge(int val, byte led, byte enable_warning) {
  unsigned int red, green, blue;
  val = val/2;

  if ( val > 500 ) {
    if (enable_warning > 0) {
      leds[led].setRGB(0, 0, 0);
      FastLED.show();
      delay(50);
      red = (255 * neo_brightness) / 16;
      leds[led].setRGB(red, 0, 0);
      FastLED.show();
    } else {
      red = (255 * neo_brightness) / 16;
      leds[led].setRGB(red, 0, 0);
      FastLED.show();
    }
  } else if ( val < 0 ) {
    if (enable_warning > 0) {
      leds[led].setRGB(0, 0, 0);
      FastLED.show();
      delay(50);
      blue = (255 * neo_brightness) / 16;
      leds[led].setRGB(0, 0, blue);
      FastLED.show();
    } else {
      blue = (255 * neo_brightness) / 16;
      leds[led].setRGB(0, 0, blue);
      FastLED.show();
    }
  } else if ((val >= 0) && (val <= 500)) {
   // red =   pgm_read_byte (&ledarray[val].r0);
   // green = pgm_read_byte (&ledarray[val].g0);
   // blue =  pgm_read_byte (&ledarray[val].b0);
    red = (red * neo_brightness) / 16;
    green = (green * neo_brightness) / 16;
    blue = (blue * neo_brightness) / 16;
    leds[led].setRGB(red, green, blue);
//    FastLED.show();
  }
}

void neogauge4led(int val, byte led0, byte led1, byte led2, byte led3, byte enable_warning) {
  unsigned int red, green, blue;
  val = val/2;

  if ( val > 500 ) {
    if (enable_warning > 0 && !gaugeBlink) {
      leds[led0].setRGB(0, 0, 0);
      leds[led1].setRGB(0, 0, 0);
      leds[led2].setRGB(0, 0, 0);
      leds[led3].setRGB(0, 0, 0);
    } else {
      red = (255 * neo_brightness) / 16;
      leds[led0].setRGB(red, 0, 0);
      leds[led1].setRGB(red, 0, 0);
      leds[led2].setRGB(red, 0, 0);
      leds[led3].setRGB(red, 0, 0);
    }

  } else if ( val < 0 ) {
    if (enable_warning > 0 && !gaugeBlink) {
      leds[led0].setRGB(0, 0, 0);
      leds[led1].setRGB(0, 0, 0);
      leds[led2].setRGB(0, 0, 0);
      leds[led3].setRGB(0, 0, 0);
    } else {
      blue = (255 * neo_brightness) / 16;
      leds[led0].setRGB(0, 0, blue);
      leds[led1].setRGB(0, 0, blue);
      leds[led2].setRGB(0, 0, blue);
      leds[led3].setRGB(0, 0, blue);
    }
  } else {
  //  red   = pgm_read_byte (&ledarray[(val)].r0);
  //  green = pgm_read_byte (&ledarray[(val)].g0);
  //  blue  = pgm_read_byte (&ledarray[(val)].b0);
    red = (red * neo_brightness) / 16;
    green = (green * neo_brightness) / 16;
    blue = (blue * neo_brightness) / 16;
    leds[led0].setRGB(red, green, blue);

   // red   = pgm_read_byte (&ledarray[(val)].r1);
   // green = pgm_read_byte (&ledarray[(val)].g1);
   // blue  = pgm_read_byte (&ledarray[(val)].b1);
    red = (red * neo_brightness) / 16;
    green = (green * neo_brightness) / 16;
    blue = (blue * neo_brightness) / 16;
    leds[led1].setRGB(red, green, blue);

   // red   = pgm_read_byte (&ledarray[(val)].r2);
   // green = pgm_read_byte (&ledarray[(val)].g2);
  //  blue  = pgm_read_byte (&ledarray[(val)].b2);
    red = (red * neo_brightness) / 16;
    green = (green * neo_brightness) / 16;
    blue = (blue * neo_brightness) / 16;
    leds[led2].setRGB(red, green, blue);

   // red   = pgm_read_byte (&ledarray[(val)].r3);
   // green = pgm_read_byte (&ledarray[(val)].g3);
   // blue  = pgm_read_byte (&ledarray[(val)].b3);
    red = (red * neo_brightness) / 16;
    green = (green * neo_brightness) / 16;
    blue = (blue * neo_brightness) / 16;
    leds[led3].setRGB(red, green, blue);
  }
}

void write_neopixel() {
  long temp;
//  unsigned int RPM, AFR, CLT, MAP, MAT, SPKADV, BATTV, TPS, Knock, Baro, EGOc, IAC, dwell, bstduty, idle_tar, AFRtgt;
//  void neogauge4led(int val, byte led0, byte led1, byte led2, byte led3)
//  void neogauge(int val, byte led)

  temp = (RPM * 1000) / REVLIMIT;
  neogauge4led(temp, 1, 0, 15, 14, 1); // RPM min 0 max REVLIMIT

  temp = ((AFR * 2) * 100) / 59;
  if (AFR <= 147) {
    temp = (pow((AFR - 147),3) / 150) + 500;
  } else if (AFR > 147) {
    temp = (pow((AFR - 147),3) / 20) + 500;
  }
  neogauge4led(temp, 9, 10, 11, 12, 0); // AFR

  temp=TPS;
  neogauge(temp, 2, 0); //TPS - min 0 max 1000

  temp=(CLT * 5) / 12; //CLT - min ? mid 120 max 240
  neogauge(temp, 4, 1);


  temp=(MAT * 5) / 7; //MAT - min ? mid 70 max 140
  neogauge(temp, 6, 0);

  temp=MAP/2;
  neogauge(temp, 8, 0); //MAP - min impossible mid 100kpa max 200kpa

// will need to play with this some, 50 looks reasonable though
  temp=((AFR - AFRtgt) * 50) + 500;
  neogauge(temp, 13, 0);

  leds[3].setRGB(0, 0, 0); // unallocated
  leds[5].setRGB(0, 0, 0);
  leds[7].setRGB(0, 0, 0);

  FastLED.show();

//todo: oil temp, oil pressure, EGT
//todo: rearrange LED's into something nicer
//todo: might be faster to do a final FastLED.show here instead of inside the neogauge functions
}
