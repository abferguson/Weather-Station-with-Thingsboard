# Weather-Station-with-Thingsboard
A simple weather station that periodically wakes and sends data to a Thingsboard server database for action and historical record keeping.

I run a Thingsboard server for storing a host of temperatures, humidities, light levels, etc inside a building. I'm most interested in understanding just how warm or cool it gets and how humid or dry it gets so that I can avoid problems.

Like so many others, I put together a weather station.  I use it to complement the data from inside the building.

Hopefully, something in this project might help or inspire others in their projects.  I was inspired by the hackaday project below and give credit to the author
https://hackaday.io/project/165061-solar-powered-wifi-weather-station-v20

This project uses Arduino IDE, Thingsboard, SPIFFS, JSON and ezTime.  Sensors are a BME280 and TSL2591.  My project is powered by a Lipo battery charged with a solarcell through a TP4056 control unit.  The microcontroller is a D1 mini pro.

Also included are images of the finished project along with Fritzing and CAD files.

Partial BOM of purchased parts:
D1 Mini Pro microcontroller
Adafruit TSL2591 High Dynamic Range Digital Light Sensor
YDL 3.7V 4000mAh 606090 Lipo battery
Micro USB 5V 1A 18650 TP4056 Lithium Battery Charging Module
2.5W 5V/500mAh Solar Panel (130 x 150 mm)
BME280 Digital Sensor
8dBi WiFi RP-SMA male Antenna
PMMA to cover TSL2591 sensor

Notes:
The red LED is mounted on the top of my unit beneath the solar cell.  The CAD files do not include this opening.
  - All electronics are coated with sealant.  I use a silicone modified conformal coating from Amazon (MG Chemicals 422B).
  - All penetrations are sealed with silicone.
  - The light sensor is mounted off the top of the solar cell support.  A small piece of PMMA or Acrylic is cut to cover the top      rectangular hole.  The entire light sensor assembly is sealed with silicone.
  - The Lipo battery is mummy wrapped with a self-sealing silicone tape to keep all moisture out. https://www.homedepot.com/p/Nashua-Tape-1-in-x-3-33-yd-Stretch-and-Seal-Self-Fusing-Silicone-Tape-in-Clear-1210364/203534911
  - I used PETG filament for the station pieces.  All pieces were spray painted white.  We'll see if this prevents excessive weathering.
  - The bottom_mount part is designed to connect the weather station to a flat corner brace. If you choose to use a brace, you may need to either drill holes in the brace or redesign the part to match hole spacings.
  
