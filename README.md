# Weather-Station-with-Thingsboard
A simple weather station that periodically wakes and sends data to a Thingsboard server

I run a Thingsboard server for storing temperatures, humidities, light levels, etc in my house. I'm most interested in understanding just how warm or cool it gets and how humid or dry it gets so that I can avoid problems.

Like so many others, I put together a weather station to capture outside weather conditions.  Hopefully, something in this project might help or inspire others in their projects.

I was inspired by the hackaday project below and give credit to the author
https://hackaday.io/project/165061-solar-powered-wifi-weather-station-v20

This project uses Arduino IDE, Thingsboard, SPIFFS, JSON and ezTime.  Sensors are a BME280 and TSL2591.  My project is powered by a Lipo battery charged with a solarcell through a TP4056 control unit.  The microcontroller is a D1 mini pro.
