# wx_xbee
My Xbee connected Teensy 2.0 base weather station.

# The platform consists of the following sensors:
* One-Wire Bus
 * DS2423 - Dallas Semi, One Wire dual counter. (Rain tip counter/Windspeed)
* I2C Bus
 * BMP085 - Bosch Barometric Pressure sensor.
 * SI1145 - Silicon Labs IR/UV/Visible light sensor.
 * HMC5883_U - Triple Axis compass (wind direction)
 * HTU21DF - RH sensor

# Setup
* Copy wx_xbee_config.h.dist to wx_xbee_config.h
* Edit wx_xbee_config.h with appropriate values.
* Setup the Pair of Xbee radios such that they peer when powered-up and not in sleep mode.
* Wire everything up as per the pinout below.
* Program the Teensy.

# Teensy Setup.
The teensy 2.0 must be run at 16Mhz to enable soft-serial to work. At the moment I'm powering it via +5VDC over USB to the teensy, I had grand plans to Power it via solar, but that didn't work as well as I had hoped. You can monitor activity on the teensy via the usb-serial monitor.

# The following Teensy 2.0 pins are used:
* 0  - N/U
* 1  - One-Wire pin w/ 4.7K pull-up resistor.
* 2  - N/U
* 3  - N/U
* 4  - N/U
* 5  - I2C SCL
* 6  - I2C SDA
* 7  - Serial RX
* 8  - Serial TX
* 9  - Sleep pin for Xbee Radio, Low for wake, high for sleep.
* 10 - N/U
* 11 - N/U
* 12 - N/U
* 13 - N/U
* 14 - N/U
* 15 - Xbee Serial RX
* 16 - N/U
* 17 - N/U
* 18 - N/U
* 19 - N/U
* 20 - Xbee Serial TX
* 21 - N/U
* 22 - N/U
* 23 - N/U
* 24 - N/U

# Output
The Radio output payload is sent as a set of messages. All frames start with a magic value "WX" (0x57,0x58). Next we have a Device_id , one byte (I use zero for this device). The Device_id is followed by the message type identifier, also one byte, this identifies what to expect in the payload.

The existing payload types are as follows:

* 0 - BMP
  * 32-bit float little-endian Temperature from BMP sensor.
  * 32-bit float little-endian Pressure from BMP sensor.
* 1 - RH
  * 32-bit float little-endian Temperature from RH sensor.
  * 32-bit float little-endian Humidity from RH sensor.
* 2 - Light
  * 32-bit float little-endian Viz from light sensor.
  * 32-bit float little-endian IR from light sensor.
  * 32-bit float little-endian UV from light sensor.
* 3 - Compass
  * 32-bit float little-endian Mag from compass X sensor.
  * 32-bit float little-endian Mag from compass Y sensor.
  * 32-bit float little-endian Mag from compass Z sensor.
* 4 - DS223 Counter
  * 32-bit little-endian integer Counter 1
  * 32-bit little-endian integer Counter 2
* 5 - Tilt and declination adjusted headings.
  * 32-bit float little-endian Mag from XZ.
  * 32-bit float little-endian Mag from YZ.
  * 32-bit float little-endian Mag from XY.
* 6 - Errors/Cycles
  * 32-bit little-endian integer Cycle count.
  * 32-bit little-endian integer TX Error count.

# TODO
* Schematic/wiring diagram + pictures.
* Add soil moisture/temp
* Add Visibility
* Add Snow depth
* Add Giger counter
* Add Seismometer.
* Add Lightning counter
