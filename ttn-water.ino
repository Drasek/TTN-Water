/*******************************************************************************
 * ttn-mapper-feather for Adafruit Feather M0 LoRa + Ultimate GPS FeatherWing
 * 
 * Code adapted from the Node Building Workshop using a modified LoraTracker board
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * 
 * This uses OTAA (Over-the-air activation), in the ttn_secrets.h a DevEUI,
 * a AppEUI and a AppKEY is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 * 
 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey. Do not forget to adjust the payload decoder function.
 * 
 * This sketch will send Battery Voltage (in mV), the location (latitude, 
 * longitude and altitude) and the hdop using the lora-serialization library 
 * matching setttings have to be added to the payload decoder funtion in the
 * The Things Network console/backend.
 * 
 * In the payload function change the decode function, by adding the code from
 * https://github.com/thesolarnomad/lora-serialization/blob/master/src/decoder.js
 * to the function right below the "function Decoder(bytes, port) {" and delete
 * everything below exept the last "}". Right before the last line add this code
 * switch(port) {    
 *   case 1:
 *     loraserial = decode(bytes, [uint16, uint16, latLng, uint16], ['vcc', 'geoAlt', 'geoLoc', 'hdop']);   
 *     values = {         
 *       lat: loraserial["geoLoc"][0],         
 *       lon: loraserial["geoLoc"][1],         
 *       alt: loraserial["geoAlt"],         
 *       hdop: loraserial["hdop"]/1000,         
 *       battery: loraserial['vcc']       
 *     };       
 *     return values;     
 *   default:       
 *     return bytes;
 * and you get a json containing the stats for lat, lon, alt, hdop and battery
 * 
 * Licence:
 * GNU Affero General Public License v3.0
 * 
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *******************************************************************************/
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <LoraEncoder.h>
#include <LoraMessage.h>
#include "LowPower.h"
#include <SoftwareSerial.h>
#include "ttn_secrets.h"

#define DEBUG  // activate to get debug info on the serial interface (115200 Baud)

// Serial for sensor
static const int waterRXPin = 2, waterTXPin = 9;
static const uint32_t waterBaud = 9600;
static const int waterLEN = 6;
static const int waterSleepPin = 3;

SoftwareSerial waterSerial(waterRXPin, waterTXPin, true);

// LoRaWAN Sleep / Join variables
int port = 1;
int sleepCycles = 1;
int sleepCyclesSlow = 112; // every sleepcycle will last 8 secs, total sleeptime will be sleepcycles * 8 sec
int sleepCyclesFast = 37; // every sleepcycle will last 8 secs, total sleeptime will be sleepcycles * 8 sec
int waterDistanceLast[] = { 0, 0, 0, 0 };
bool joined = false;
bool sleeping = false;

// LoRaWAN keys
static const u1_t app_eui[8]  = SECRET_APP_EUI;
static const u1_t dev_eui[8]  = SECRET_DEV_EUI;
static const u1_t app_key[16] = SECRET_APP_KEY;

// Getters for LMIC
void os_getArtEui (u1_t* buf) 
{
  memcpy(buf, app_eui, 8);
}

void os_getDevEui (u1_t* buf)
{
  memcpy(buf, dev_eui, 8);
}

void os_getDevKey (u1_t* buf)
{
  memcpy(buf, app_key, 16);
}

// Pin mapping
// The Feather M0 LoRa does not map RFM95 DIO1 to an M0 port. LMIC needs this signal 
// in LoRa mode, so you need to bridge IO1 to an available port -- I have bridged it to 
// Digital pin #11
// We do not need DIO2 for LoRa communication.
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LMIC_UNUSED_PIN,
  .dio = {4, 5, 7},
};

static osjob_t sendjob;
static osjob_t initjob;

// Init job -- Actual message message loop will be initiated when join completes
void initfunc (osjob_t* j)
{
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // Allow 1% error margin on clock
  // Note: this might not be necessary, never had clock problem...
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  // start joining
  LMIC_startJoining();
}

// Reads battery voltage

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else // for ATmega328(P) & ATmega168(P)
    ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (0<<MUX0);
  #endif  

  delay(50); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1092274L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  //result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000 - standard value for calibration
  return result; // Vcc in millivolts
}


int waterLoop() {
  boolean stringComplete = false;
  int waterDistance;
  int i = 0;
  char buf[waterLEN];
  digitalWrite(waterSleepPin, HIGH);
  while (stringComplete == false) {
    //Serial.print("reading ");    //debug line
    if (waterSerial.available()) {
      char incomingByte = waterSerial.read();
      if(incomingByte == 'R') {
        //Serial.println("rByte set");
        while (i < (waterLEN-1)) {
          if (waterSerial.available()) {
            buf[i] = waterSerial.read(); 
            //Serial.println(buf[i]);
            i++;
          }  
        }
        buf[i] = 0x00;
      }
      incomingByte = 0;
      i = 0;
      waterDistance = atoi(buf);
      if(waterDistance != 0 && waterDistance != 500) {
        stringComplete = true;
      }
    }
  }
  digitalWrite(waterSleepPin, LOW);
  return waterDistance;
}


// Send job
static void do_send(osjob_t* j)
{
  // get Battery Voltage
  int vccValue = readVcc();
  int waterDistance = waterLoop();

  waterDistanceLast[0] = waterDistanceLast[1];
  waterDistanceLast[1] = waterDistanceLast[2];
  waterDistanceLast[2] = waterDistanceLast[3];
  waterDistanceLast[3] = waterDistance;
  
  // compress the data into a few bytes
  LoraMessage message;
  message
    .addUint16(vccValue)
    .addUint16(waterDistance);
    
  // Check if there is not a current TX/RX job running   
  if (LMIC.opmode & OP_TXRXPEND) {
    #ifdef DEBUG
      Serial.println(F("OP_TXRXPEND, not sending"));
    #endif
  } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(port, message.getBytes(), message.getLength(), 0);
    #ifdef DEBUG
      Serial.println(F("Sending: "));
    #endif
  }
}

// LoRa event handler
// We look at more events than needed, to track potential issues
void onEvent (ev_t ev) {
  switch(ev) {
    case EV_SCAN_TIMEOUT:
      #ifdef DEBUG
        Serial.println(F("EV_SCAN_TIMEOUT"));
      #endif
      break;
    case EV_BEACON_FOUND:
      #ifdef DEBUG
        Serial.println(F("EV_BEACON_FOUND"));
      #endif
      break;
    case EV_BEACON_MISSED:
      #ifdef DEBUG
        Serial.println(F("EV_BEACON_MISSED"));
      #endif
      break;
    case EV_BEACON_TRACKED:
      #ifdef DEBUG
        Serial.println(F("EV_BEACON_TRACKED"));
      #endif
      break;
    case EV_JOINING:
      #ifdef DEBUG
        Serial.println(F("EV_JOINING"));
      #endif
      break;
    case EV_JOINED:
      #ifdef DEBUG
        Serial.println(F("EV_JOINED"));
      #endif
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      joined = true;
      LMIC_setDrTxpow(DR_SF7,14);
      break;
    case EV_RFU1:
      #ifdef DEBUG
        Serial.println(F("EV_RFU1"));
      #endif
      break;
    case EV_JOIN_FAILED:
      #ifdef DEBUG
        Serial.println(F("EV_JOIN_FAILED"));
      #endif
      break;
    case EV_REJOIN_FAILED:
      #ifdef DEBUG
        Serial.println(F("EV_REJOIN_FAILED"));
      #endif
      os_setCallback(&initjob, initfunc);
      break;
    case EV_TXCOMPLETE:
      sleeping = true;
      #ifdef DEBUG
        if (LMIC.dataLen) {
          Serial.print(F("Data Received: "));
          Serial.println(LMIC.frame[LMIC.dataBeg],HEX);
        }
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      #endif
      break;
    case EV_LOST_TSYNC:
      #ifdef DEBUG
        Serial.println(F("EV_LOST_TSYNC"));
      #endif
      break;
    case EV_RESET:
      #ifdef DEBUG
        Serial.println(F("EV_RESET"));
      #endif
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      #ifdef DEBUG
        Serial.println(F("EV_RXCOMPLETE"));
      #endif
      break;
    case EV_LINK_DEAD:
      #ifdef DEBUG
        Serial.println(F("EV_LINK_DEAD"));
      #endif
      break;
    case EV_LINK_ALIVE:
      #ifdef DEBUG
        Serial.println(F("EV_LINK_ALIVE"));
      #endif
      break;
    default:
      #ifdef DEBUG
        Serial.println(F("Unknown event"));
      #endif
      break;
  }
}

void setup() {
  #ifdef DEBUG
    Serial.begin(115200);
    delay(250);
    Serial.println(F("Starting"));
  #endif
  // initialize the scheduler
  os_init();

  // Initialize radio
  os_setCallback(&initjob, initfunc);
  //LMIC_reset();

  waterSerial.begin(waterBaud);
  pinMode(waterSleepPin, OUTPUT);
  digitalWrite(waterSleepPin, LOW);
}

void loop() {
  if (joined==false) {
    os_runloop_once();
  }
  else {
    do_send(&sendjob);    // Sent sensor values
    while(sleeping == false) {
      os_runloop_once();
    }
    sleeping = false;
    if(abs((waterDistanceLast[0]+waterDistanceLast[1]+waterDistanceLast[2])/3-waterDistanceLast[3]) <= 50) {
      sleepCycles = sleepCyclesSlow;
    } else {
      sleepCycles = sleepCyclesFast;
    }
    for (int i=0; i<sleepCycles; i++) {
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);    //sleep 8 seconds per sleepcycle
    }
  }
}
