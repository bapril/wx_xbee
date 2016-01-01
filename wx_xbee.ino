//This is weather 3 with sleep added.

#include <XBee.h>
#include <Wire.h>
#include <OneWire.h>
#include <DS2423.h>
#include <Adafruit_BMP085.h>
#include "Adafruit_SI1145.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <SoftwareSerial.h>
#include "Adafruit_HTU21DF.h"
#include <avr/sleep.h>
#include <avr/power.h>

#include "wx_xbee_config.h"

#define OWPIN 1
#define SLPPIN 9 //Xbee sleep pin. Low for wake, high for sleep.

// SoftSerial RX: connect Arduino digitial 8 to the TX of of usb-serial device.  note: I'm using Modern Device's USB BUB (set to 5V).  You can use a 3.3V usb-serial with a voltage divider on RX (TX does not require since Arduino is 3.3V tolerant)
uint8_t ssRX = 15;
// SoftSerial TX: connect Arduino digital 9 to RX of usb-serial device
uint8_t ssTX = 20;

int cycle_count;
int error_count;

bool rh_enable = false;
bool bmp_enable = false;
bool uv_enable = false;
bool mag_enable = false;

OneWire  ow(OWPIN);  // on pin 10 (a 4.7K resistor is necessary)
DS2423 ds2423(&ow, DS2423_address);
Adafruit_BMP085 bmp;
Adafruit_SI1145 uv = Adafruit_SI1145();
XBee xbee = XBee();

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12346);
Adafruit_HTU21DF htu = Adafruit_HTU21DF();
uint8_t const max_payload_size = 9;
uint8_t payload[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

uint8_t state = 0;

//Address Coordinator, whatever address they may have.
XBeeAddress64 addr64 = XBeeAddress64(0x0013a200, 0x40B798F8);
ZBTxRequest zbTx = ZBTxRequest(addr64, payload,sizeof(payload));
ZBRxResponse rx = ZBRxResponse();
ModemStatusResponse msr = ModemStatusResponse();
SoftwareSerial nss(ssRX, ssTX);

//Notes,
//Must run at < 16Mhz

void setup()
{
  analogReference(DEFAULT);
  cycle_count = 0;
  error_count = 0;
  //Pins and their use:
  //
  // 0
  pinMode(0,OUTPUT);
  digitalWrite(0,LOW);
  // 1        OWPIN
  // 2
  pinMode(2,OUTPUT);
  digitalWrite(2,LOW);
  // 3
  pinMode(3,OUTPUT);
  digitalWrite(3,LOW);
  // 4
  pinMode(4,OUTPUT);
  digitalWrite(4,LOW);
  // 5        I2C SCL
  // 6        I2C SDA
  // 7        Serial RX
  // 8        Serial TX
  // 9        XBEE sleep
  // 10
  pinMode(10,OUTPUT);
  digitalWrite(10,LOW);
  // 11 A10 (LED)
  pinMode(11,OUTPUT);
  digitalWrite(11,LOW);
  //12
  pinMode(12,OUTPUT);
  digitalWrite(12,LOW);
  //13
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
  //14
  pinMode(14,OUTPUT);
  digitalWrite(14,LOW);
  // 15 A6 <--- Soft Serial RX.
  // 16 A5
  pinMode(16,OUTPUT);
  digitalWrite(16,LOW);
  // 17 A4
  pinMode(17,OUTPUT);
  digitalWrite(17,LOW);
  // 18 A3
  pinMode(18,OUTPUT);
  digitalWrite(18,LOW);
  // 19 A2
  pinMode(19,OUTPUT);
  digitalWrite(19,LOW);
  // 20 A1  XBEE Serial TX
  // 22
  pinMode(22,OUTPUT);
  digitalWrite(22,LOW);
  // 23
  pinMode(23,OUTPUT);
  digitalWrite(23,LOW);
  // 24
  pinMode(24,OUTPUT);
  digitalWrite(24,LOW);

  Serial.begin(9600);  // start serial for output
  nss.begin(9600);
  xbee.setSerial(nss);
  pinMode(SLPPIN,OUTPUT);
  digitalWrite(SLPPIN,0); //Wake up

  Serial.println("Serial Startup");
  delay(30000);


  if (! uv.begin()) {
    Serial.println("UV:NX");
  } else {
    Serial.println("UV Sensor on-line.");
    uv_enable = true;
  }

  if(!mag.begin()) {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("WD:NX");
  } else {
    Serial.println("WD Sensor on-line.");
    mag_enable = true;
  }

  if (!bmp.begin()) {
    Serial.println("BP:Couldn't find sensor!");
  } else {
    Serial.println("BP on-line");
    bmp_enable = true;
  }

  if (!htu.begin()) {
    Serial.println("RH:Couldn't find sensor!");
  } else {
    Serial.println("RH Sensor on-line.");
    rh_enable = true;
  }
  ds2423.begin(DS2423_COUNTER_A|DS2423_COUNTER_B);

  Serial.println("Drviers loaded, starting up");
  wait_for_assoc();
  Serial.println("Starting");
}

int payload_size = 0, batt_status = 0, value = 0;

void loop()
{
  sensors_event_t event;
  float val, heading;
  unsigned long vall;
  byte * a;
  payload[0] = 0x57; //W
  payload[1] = 0x58; //X
  payload[2] = Device_id;    // Device 0
  payload[3] = state; // Current State.

  switch (state) {
    case 0: //BP
      if (bmp_enable) {
        val = bmp.readTemperature();
        a = (byte *) &val;
        payload[4]  = a[0];
        payload[5]  = a[1];
        payload[6]  = a[2];
        payload[7]  = a[3];
        val = bmp.readPressure();
        a = (byte *) &val;
        payload[8]  = a[0];
        payload[9]  = a[1];
        payload[10]  = a[2];
        payload[11]  = a[3];
        payload_size = 12;
        tx_packet();
      }
      state = 1;
      break;
    case 1:
      if (rh_enable) {
        val = htu.readTemperature();
        a = (byte *) &val;
        payload[4]  = a[0];
        payload[5]  = a[1];
        payload[6]  = a[2];
        payload[7]  = a[3];
        val = htu.readHumidity();
        a = (byte *) &val;
        payload[8]  = a[0];
        payload[9]  = a[1];
        payload[10]  = a[2];
        payload[11]  = a[3];
        payload_size = 12;
        tx_packet();
      }
      state = 2;
      break;
    case 2:
      if (uv_enable) {
        val = uv.readVisible();
        a = (byte *) &val;
        payload[4]  = a[0];
        payload[5]  = a[1];
        payload[6]  = a[2];
        payload[7]  = a[3];
        val = uv.readIR();
        a = (byte *) &val;
        payload[8]  = a[0];
        payload[9]  = a[1];
        payload[10]  = a[2];
        payload[11]  = a[3];
        val = uv.readUV();
        a = (byte *) &val;
        payload[12]  = a[0];
        payload[13]  = a[1];
        payload[14]  = a[2];
        payload[15]  = a[3];
        payload_size = 16;
        tx_packet();
      }
      state = 3;
      break;
    case 3: //MAG
      if (mag_enable){
        mag.getEvent(&event);
        val = event.magnetic.x;
        a = (byte *) &val;
        payload[4]  = a[0];
        payload[5]  = a[1];
        payload[6]  = a[2];
        payload[7]  = a[3];
        val = event.magnetic.y;
        a = (byte *) &val;
        payload[8]  = a[0];
        payload[9]  = a[1];
        payload[10]  = a[2];
        payload[11]  = a[3];
        val = event.magnetic.z;
        a = (byte *) &val;
        payload[12]  = a[0];
        payload[13]  = a[1];
        payload[14]  = a[2];
        payload[15]  = a[3];
        payload_size = 16;
        tx_packet();
      }
      state = 4;
      break;
    case 4: //Counter1
      vall = ds2423.getCount(DS2423_COUNTER_A);
      a = (byte *) &vall;
      payload[4]  = a[0];
      payload[5]  = a[1];
      payload[6]  = a[2];
      payload[7]  = a[3];
      vall = ds2423.getCount(DS2423_COUNTER_B);
      a = (byte *) &vall;
      payload[8]  = a[0];
      payload[9]  = a[1];
      payload[10]  = a[2];
      payload[11]  = a[3];
      payload_size = 12;
      tx_packet();
      state = 5;
      break;
    case 5:
      if (mag_enable){
        //My Declination is -14° 46'
        //-14.766667 degrees from http://transition.fcc.gov/mb/audio/bickel/DDDMMSS-decimal.html
        //-0.2577269587 Radians = .25

        //Find yours here: http://www.magnetic-declination.com/

        mag.getEvent(&event);
        heading = atan2(event.magnetic.x,event.magnetic.z);
        heading += DECLINATION;
        if(heading < 0)
          heading += 2*PI;
        if(heading > 2*PI)
          heading -= 2*PI;
        val = heading * 180/M_PI;
        a = (byte *) &val;
        payload[4]  = a[0];
        payload[5]  = a[1];
        payload[6]  = a[2];
        payload[7]  = a[3];
        heading = atan2(event.magnetic.y,event.magnetic.z);
        heading += DECLINATION;
        if(heading < 0)
          heading += 2*PI;
        if(heading > 2*PI)
          heading -= 2*PI;
        val = heading * 180/M_PI;
        a = (byte *) &val;
        payload[8]  = a[0];
        payload[9]  = a[1];
        payload[10]  = a[2];
        payload[11]  = a[3];
        heading = atan2(event.magnetic.x,event.magnetic.y);
        heading += DECLINATION;
        if(heading < 0)
          heading += 2*PI;
        if(heading > 2*PI)
          heading -= 2*PI;
        val = heading * 180/M_PI;
        a = (byte *) &val;
        payload[12]  = a[0];
        payload[13]  = a[1];
        payload[14]  = a[2];
        payload[15]  = a[3];
        payload_size = 16;
        tx_packet();
      }
      state = 6;
      break;
    case 6:
      a = (byte *) &cycle_count;
      payload[4]  = a[0];
      payload[5]  = a[1];
      payload[6]  = a[2];
      payload[7]  = a[3];
      a = (byte *) &Error_count;
      payload[5]  = a[0];
      payload[6]  = a[1];
      payload[7]  = a[2];
      payload[8]  = a[3];
      payload_size = 8;
      tx_packet();
      cycle_count++;
      //Serial.println("Napping");
      //digitalWrite(SLPPIN,1); //Put radio to sleep.
      //delay(29000);
      //digitalWrite(SLPPIN,0 ); //Wake up
      delay(INTER_LOOP_DELAY);
      //wait_for_assoc();
      state = 0;
      Serial.println("Nap Over, back to work!");
      break;
  }
}

void tx_packet(){
  Serial.println("Start TX packet");
  ZBTxStatusResponse txStatus = ZBTxStatusResponse();
  boolean need_to_send = true;

  while(need_to_send){
    xbee.send(zbTx);
    if (xbee.readPacket(500)) {
      // got a response!

      // should be a znet tx status
      if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
        xbee.getResponse().getZBTxStatusResponse(txStatus);
        // get the delivery status, the fifth byte
        if (txStatus.getDeliveryStatus() == SUCCESS) {
          need_to_send = false;
          Serial.println("Sent");
          // success.  time to celebrate
        } else {
          Serial.println("Fail1");
          error_count++;
          delay(INTER_PACKET_DELAY);
        }
      } else if (xbee.getResponse().isError()) {
         error_count++;
         Serial.println("Fail2");
         delay(INTER_PACKET_DELAY);
      } else {
        Serial.println("Fail3");
        error_count++;
        delay(INTER_PACKET_DELAY);
      }
    } else {
      error_count++;
      Serial.println("Fail4");
    }
  }
  Serial.println("Packet Sent");
  delay(INTER_PACKET_DELAY);
}

void wait_for_assoc(){
  Serial.println("Waiting for xbee assoc");
   boolean need_assoc = true;
   while(need_assoc){
      xbee.readPacket(250);
      if (xbee.getResponse().isAvailable()) {
        // got something
        if (xbee.getResponse().getApiId() == ZB_RX_RESPONSE) {
          // got a zb rx packet
          Serial.println("Got RX_RESPONSE");
          // now fill our zb rx class
          xbee.getResponse().getZBRxResponse(rx);
        } else if (xbee.getResponse().getApiId() == MODEM_STATUS_RESPONSE) {
          xbee.getResponse().getModemStatusResponse(msr);
          // the local XBee sends this response on certain events, like association/dissociation
          if (msr.getStatus() == ASSOCIATED) {
            // yay this is great.  flash led
            Serial.println("Have Assoc!");
            need_assoc = false;
          } else {
             Serial.println("Not assoc packet");
          }
        } else {
          Serial.println("No packet");
        }

      } else if (xbee.getResponse().isError()) {
        Serial.print("Error reading packet.  Error code: ");
        Serial.println(xbee.getResponse().getErrorCode());
      } else {
        Serial.println("No Packet found.");
      }
    delay(INTER_PACKET_DELAY);
   }
}
