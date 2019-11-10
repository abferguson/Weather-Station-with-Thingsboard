/* Weather Station Unit
 * Hardware: BME280 pressure/termperature/humidity sensor
 * Adafruit TSL2591 digital light sensor
 * TP4056 lithium battery charging module
 * WEMOS D1 mini pro microcontroller with external antenna
 * Charging (red) and fully charged (blue) LEDs on LiPo battery charging module
 *
 * Compile using Board: LOLIN(Wemos) D1 R2 & Mini
 *           File Size: 4M(3M SPIFFS)
 *         Erase Flash: All Flash Contents
 * Let it cycle through more than once as it will fail to open SPIFFS file
 * and will then save initial pressure data, reboot then take another reading
 * 
 * Note that OTA is not used in this example although some code is their to support its use.
 *         
*/
#include "ThingsBoard.h"
#include <ESP8266WiFi.h>
#include <BME280I2C.h>
#include <EnvironmentCalculations.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2591.h>
#include <ezTime.h>
#include <FS.h>
#include <Arduino.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoJson.h>

#ifdef ESP32
#pragma message(THIS EXAMPLE IS FOR ESP8266 ONLY!)
#error Select ESP8266 board.
#endif

#define ssid "your_ssid"
#define WIFI_PASSWORD "your_ssid_password"
#define node_name "weather_sta"

#define TOKEN "your Thingsboard API token" // Weather Station
#define THINGSBOARD_SERVER  "your Thingboard server IP address"
String server = "http://192.168.1.107:80"; // modifiy with your server information

String WeatherLEDlink = server+"/api/v1/"+TOKEN+"/attributes?sharedKeys=sleepTimesec,OTAupdate,ledOnsec,ledIntensity,publishTimeseries";

#define LEDred D6
int sleepTimesec = 600;
bool OTAupdate = false;
float ledOnsec = 2;
int ledIntensity = 30;
bool publishTimeseries=false;

int sw_version = 7;
int powerPin = A0;  // battery voltage
int readcount = 0;

// Baud rate for debug serial
#define SERIAL_DEBUG_BAUD   115200

// Initialize ThingsBoard client
WiFiClient espClient;
//ThingsBoard tb(espClient); // work with fewer attributes fields
ThingsBoardSized<128, 32> tb(espClient); // needed for more than 8 fields

long rssi;
String ipaddrstr;

// the Wifi radio's status
int status = WL_IDLE_STATUS;
bool subscribed = false;

float tempDewpoint;
float tempDrybulb;
float relHumidity;
float tempHeatindex;
float pressure;
float seaLevelPres;
float absHumidity;
int rel_pressure_rounded;
bool bmefail = false;

uint16_t lightVisible;
uint16_t lightIR;
uint16_t lightFullSpec;
float lightLUX;
bool tsl2591fail = false;

float volt;

String msgText = " ";

// FORECAST CALCULATION
unsigned long current_timestamp;    // Actual timestamp read from NTPtime_t now;
unsigned long thingsboard_timestamp;    // Actual timestamp read from NTPtime_t now;
unsigned long saved_timestamp;      // Timestamp stored in SPIFFS

float pressure_value[12];           // Array for the historical pressure values (6 hours, all 30 mins)
float pressure_difference[12];      // Array to calculate trend with pressure differences

// FORECAST RESULT
int accuracy;                       // Counter, if enough values for accurate forecasting
String ZambrettisWords;             // Final statement about weather forecast
String trend_in_words;              // Trend in words

void(* resetFunc) (void) = 0;       // declare reset function @ address 0

Timezone myTZ;
String nowtime;

const char TEXT_RISING_FAST[]       = "rising quickly";
const char TEXT_RISING[]            = "rising";
const char TEXT_RISING_SLOW[]       = "rising slowly";
const char TEXT_STEADY[]            = "steady";
const char TEXT_FALLING_SLOW[]      = "falling slowly";
const char TEXT_FALLING[]           = "falling";
const char TEXT_FALLING_FAST[]      = "falling quickly";

const char TEXT_ZAMBRETTI_A[]       = "Settled, Fine Weather";
const char TEXT_ZAMBRETTI_B[]       = "Fine Weather";
const char TEXT_ZAMBRETTI_C[]       = "Becoming Fine";
const char TEXT_ZAMBRETTI_D[]       = "Fine, Becoming Less Settled";
const char TEXT_ZAMBRETTI_E[]       = "Fine, Possibly showers";
const char TEXT_ZAMBRETTI_F[]       = "Fairly Fine, Improving";
const char TEXT_ZAMBRETTI_G[]       = "Fairly Fine, Possibly showers early";
const char TEXT_ZAMBRETTI_H[]       = "Fairly Fine, Showers Later";
const char TEXT_ZAMBRETTI_I[]       = "Showery Early, Improving";
const char TEXT_ZAMBRETTI_J[]       = "Changeable, Improving";
const char TEXT_ZAMBRETTI_K[]       = "Fairly Fine, Showers likely";
const char TEXT_ZAMBRETTI_L[]       = "Rather Unsettled, Clearing Later";
const char TEXT_ZAMBRETTI_M[]       = "Unsettled, Probably Improving";
const char TEXT_ZAMBRETTI_N[]       = "Showery Bright Intervals";
const char TEXT_ZAMBRETTI_O[]       = "Showery Becoming Unsettled";
const char TEXT_ZAMBRETTI_P[]       = "Changeable, some rain";
const char TEXT_ZAMBRETTI_Q[]       = "Unsettled, short fine Intervals";
const char TEXT_ZAMBRETTI_R[]       = "Unsettled, Rain later";
const char TEXT_ZAMBRETTI_S[]       = "Unsettled, rain at times";
const char TEXT_ZAMBRETTI_T[]       = "Very Unsettled, Finer at times";
const char TEXT_ZAMBRETTI_U[]       = "Rain at times, Worse later";
const char TEXT_ZAMBRETTI_V[]       = "Rain at times, becoming very unsettled";
const char TEXT_ZAMBRETTI_W[]       = "Rain at Frequent Intervals";
const char TEXT_ZAMBRETTI_X[]       = "Very Unsettled, Rain";
const char TEXT_ZAMBRETTI_Y[]       = "Stormy, possibly improving";
const char TEXT_ZAMBRETTI_Z[]       = "Stormy, much rain";
const char TEXT_ZAMBRETTI_DEFAULT[] = "Sorry, no forecast for the moment";

BME280I2C::Settings settings(
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::OSR_X1,
   BME280::Mode_Forced,
   BME280::StandbyTime_1000ms,
   BME280::Filter_16,
   BME280::SpiEnable_False,
   BME280I2C::I2CAddr_0x76
);

 BME280I2C bme(settings);    // Default : forced mode, standby time = 1000 ms
//BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
                    // Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,

Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)

void setup() {
  Serial.begin(115200);
  Wire.begin(D2,D1);  //sda, scl
  Wire.setClock(100000);  // speed up i2c wire speed. it works
  WiFi.hostname(node_name);
  InitWiFi();
  InitNTP();

  subscribeThingsboard();

  Serial.println("Checking SPIFFS state: ");
  boolean mounted = SPIFFS.begin();               // load config if it exists. Otherwise use defaults.
  if (!mounted) {
    Serial.println("FS not formatted. Doing that now... (can last up to 30 sec).");
    SPIFFS.format();
    Serial.println("FS formatted...");
    SPIFFS.begin();
  }
  else {
    Serial.println("SPIFFS FS is already formatted.");
  }

  
  Serial.println("Turning on BME280 sensor.");
  readcount = 0;
  while(!bme.begin() && readcount < 11) {
    Serial.println("Could not find BME280 sensor!");
    delay(500);
    readcount++;
  }
  if(readcount < 11) {
    Serial.println(F("BME280 sensor Begin successful"));
    bmefail = false;
  }
  else {
    Serial.println("BME280 sensor Begin not successful");
    msgText = msgText+"BME280 Begin fail. ";
    bmefail = true;
  }

  switch(bme.chipModel()) {
    case BME280::ChipModel_BME280:
      Serial.println("Found BME280 sensor ");
      break;
    case BME280::ChipModel_BMP280:
      Serial.println("Found BMP280 sensor.");
      break;
    default:
      Serial.println("Found UNKNOWN sensor. Error!");
  }
  delay(2000);
  Serial.print("BME280 chip model: ");
  Serial.println(bme.chipModel());

  displaySensorDetails();
  configureTSL2591();
   
  Serial.println("Turning on TSL2591 sensor");
  readcount = 0;
  while (!tsl.begin() && readcount < 11 ) {
    Serial.println(F("TSL2591 sensor Begin unsuccessful. "));
    delay(1000);
    readcount++;
  }
  if(readcount < 11) {
    Serial.println(F("Found TSL2591 sensor"));
    tsl2591fail = false;
  }
  else {
    Serial.println("TSL2591 sensor Begin not successful. ");
    msgText = msgText+"TSL2591 Begin fail. ";
    tsl2591fail = true;
  }
  
  if (WiFi.status() != WL_CONNECTED) {
    reconnect();
  }

  rssi = WiFi.RSSI();
  readBatVoltage();

  if (!bmefail) {
    bmefail = true;
    readcount = 0;
    while ( bmefail && readcount < 11 ) {
      BME280Read();
      delay(1000);
     readcount++;
    }
    if(readcount < 11) {
      Serial.println(F("BME280 sensor Read successful"));
      bmefail = false;
    }
    else {
      Serial.println("BME280 sensor Read not successful");
      msgText = msgText+"BME280 Read fail. ";
      bmefail = true;
    }
  } // if !bmefail

  if (!tsl2591fail) {
    TSL2591Read();  // the read function for the tsl2591 sensor does not throw errors when executed. So no Read error trap possible.
  }  // if !tsl2591fail

  if(!tsl2591fail && !bmefail) {
    msgText = "Sensors OK";
  }
  getparseData(WeatherLEDlink);  // parse JSON string from HTTP request

  Serial.println("Zambretti, Timeseries and attribute update");
  Spiffs_Zambretti_update();
  AttributeUpdate();
  if(publishTimeseries) {
    TimeseriesUpdate();
  }

  tb.loop();
  
  WiFi.disconnect();
  goToSleep();                //over and out

  WiFi.disconnect();

  if(ledIntensity > 99) {
    ledIntensity = 99;
  }
  pinMode(LEDred, OUTPUT);
  analogWrite(LEDred, int(ledIntensity/100.0*1000.0)); // turn on LEDred at ledBright intensity
  delay(ledOnsec*1000);  // on time for LEDred
  digitalWrite(LEDred, LOW);  // turn off LEDred
  
  goToSleep();
   
} // void setup

void loop() {
  // not used
}


// set value on switch 1

void subscribeThingsboard() {
  if (!tb.connected()) {
    subscribed = false;
    Serial.print("Connecting to ThingsBoard server: ");
    Serial.print(THINGSBOARD_SERVER);
    Serial.print(" with token ");
    Serial.println(TOKEN);
    if (!tb.connect(THINGSBOARD_SERVER, TOKEN)) {
      Serial.println("Failed to connect. retrying...");
      delay(1000);
      return;
    }
  }
} // void subscribeThingsboard

void AttributeUpdate() {
Serial.print("msgText: ");
Serial.println(msgText);
const char * messageText = msgText.c_str();
Attribute attributes0[2] = {
  { "sw_version", sw_version },
  { "currenttimestamp", thingsboard_timestamp },
};
Attribute attributes1[1] = {
  { "messageText", messageText },
};
  Attribute attributes2[2] = {
    { "bmestate", bmefail },
    { "tsl2591state", tsl2591fail },
  };
  Attribute attributes3[2] = {
    { "rssi", rssi },
    { "voltage", volt },
  };
  const char * ipaddr = ipaddrstr.c_str();
  Attribute attributes4[1] = {
    { "ipaddr", ipaddr },
  };
  Attribute attributes5[2] = {
    { "tempdrybulb", tempDrybulb },
    { "tempdewpoint", tempDewpoint },
  };
  Attribute attributes6[2] = {
    { "relhumidity", relHumidity },
    { "tempheatindex", tempHeatindex },
  };
  Attribute attributes7[2] = {
    { "pressure", pressure },
    { "sealevelpress", seaLevelPres },
  };
  Attribute attributes8[1] = {
    { "abshumidity", absHumidity },
  };
  Attribute attributes9[2] = {
    { "lightVisible", lightVisible },
    { "lightIR", lightIR },
  };
  Attribute attributes10[2] = {
    { "lightFullSpec", lightFullSpec },
    { "lightLUX", lightLUX },
  };
  const char * ZWords = ZambrettisWords.c_str();
  Attribute attributes11[1] = {
    { "zwords", ZWords },
  };
  tb.sendAttributes(attributes0, 2);
  tb.sendAttributes(attributes1, 1);
  tb.sendAttributes(attributes2, 2);
  tb.sendAttributes(attributes3, 2);
  tb.sendAttributes(attributes4, 1);
  delay(500);  // need delays for attributes to make it
  if(!bmefail) {
    tb.sendAttributes(attributes5, 2);
    tb.sendAttributes(attributes6, 2);
    tb.sendAttributes(attributes7, 2);
    tb.sendAttributes(attributes8, 1);
  }
  delay(500);
  if(!tsl2591fail) {
    tb.sendAttributes(attributes9, 2);
    tb.sendAttributes(attributes10, 2);
  }
  tb.sendAttributes(attributes11, 1);

  Serial.println("finished attribute update");
  Serial.print("ZWords: ");
  Serial.println(ZWords);
} // void AttributeUpdate

void TimeseriesUpdate() {
  if(!bmefail) {
    tb.sendTelemetryFloat("drytemperature", tempDrybulb);
    tb.sendTelemetryFloat("wettemperature", tempDewpoint);
    tb.sendTelemetryFloat("humidity", relHumidity);
    tb.sendTelemetryFloat("heatindex", tempHeatindex);
    tb.sendTelemetryFloat("pressure", seaLevelPres);
    tb.sendTelemetryFloat("abshumidity", absHumidity);
  }
  delay(500);  // need delays for timeseries to make it
  if(!tsl2591fail) {
    tb.sendTelemetryFloat("lightvisible", lightVisible);
    tb.sendTelemetryFloat("lightir", lightIR);
    tb.sendTelemetryFloat("lightfullspec", lightFullSpec);
    tb.sendTelemetryFloat("lightlux", lightLUX);
  }
  delay(500);
  tb.sendTelemetryFloat("battvolt", volt);

  Serial.println("Finished timeseries update");
} // void TimeseriesUpdate

void Spiffs_Zambretti_update() {
  ReadFromSPIFFS();              //read stored values and update data if more recent data is available
  Serial.print("Timestamp difference: ");
  Serial.println(current_timestamp - saved_timestamp);

  if (current_timestamp - saved_timestamp > 21600){    // last save older than 6 hours -> re-initialize values
    FirstTimeRun();
  }
  else if (current_timestamp - saved_timestamp > 1800){ // it is time for pressure update (1800 sec = 30 min)
    
    for (int i = 11; i >= 1; i = i -1) {
      pressure_value[i] = pressure_value[i-1];          // shifting values one to the right
  }
   
  pressure_value[0] = rel_pressure_rounded;             // updating with acutal rel pressure (newest value)
  
  if (accuracy < 12) {
    accuracy = accuracy + 1;                            // one value more -> accuracy rises (up to 12 = 100%)
    }
    WriteToSPIFFS(current_timestamp);                   // update timestamp on storage
  }
  else {         
    WriteToSPIFFS(saved_timestamp);                     // do not update timestamp on storage
  }

  int accuracy_in_percent = accuracy*94/12;            // 94% is the max predicion accuracy of Zambretti

  ZambrettisWords = ZambrettiSays(char(ZambrettiLetter()));
  
  Serial.print("Zambretti says: ");
  Serial.print(ZambrettisWords);
  Serial.print(", ");
  Serial.println(trend_in_words);
  Serial.print("Prediction accuracy: ");
  Serial.print(accuracy_in_percent);
  Serial.println("%");
  if (accuracy < 12){
    Serial.println("Not enough weather data yet.");
    Serial.print("We need ");
    Serial.print((12 - accuracy) / 2);
    Serial.println(" hours more to get sufficient data.");
  }
  Serial.println("SPIFFS data for Zambetti algorithm saved");
} // void Spiffs_Sambretti_update

void InitWiFi()
{
  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println("WiFi: Connecting to Fergnetw ...");
  // attempt to connect to WiFi network
  WiFi.begin(ssid, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected to Fergnetw");
 
  ipaddrstr = WiFi.localIP().toString();
  Serial.print("IP Address: ");
  Serial.println(ipaddrstr);

  rssi = WiFi.RSSI();
  Serial.print("signal strength: ");
  Serial.println(rssi);
} // void InitWiFi

void InitNTP() {
  Serial.println("NTP: Connecting to NTP server...");
//  setDebug(INFO);
//  setServer("0.us.pool.ntp.org");
  uint16_t timeout = 5;
  waitForSync(timeout);  // ezTime library
//  Serial.print("pre location set Current timestamp myTZ.now: ");
//  current_timestamp_utc = myTZ.now();
//  Serial.println(current_timestamp);
  myTZ.setLocation(F("America/Chicago"));
//  myTZ.setDefault();
  Serial.print("myTZ.dateTime: ");
  Serial.println(myTZ.dateTime());
  nowtime = myTZ.dateTime("g:i:s");
  Serial.print("myTZ.dateTime g:i:s= ");
  Serial.println(nowtime);
  Serial.print("Current timestamp myTZ.now: ");
  current_timestamp = myTZ.now(); // gives local value sing location was set above
  Serial.println(current_timestamp);
  thingsboard_timestamp = myTZ.tzTime(myTZ.now(),LOCAL_TIME);  // UTC value for Thingsboard
}  // void InitNTP

void reconnect() {
  // Loop until we're reconnected
  status = WiFi.status();
  if ( status != WL_CONNECTED) {
    Serial.println("Reconnecting to Fergnet AP");
    WiFi.begin(ssid, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("Reconnected to Fergnet AP");
  }
} // void reconnect


void BME280Read() {

  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_hPa);

  bme.read(pressure, tempDrybulb, relHumidity, tempUnit, presUnit);

  float referencePressure = 1013.25;  // hPa local QFF (official meteor-station reading)
  float outdoorTemp = tempDrybulb;
  float barometerAltitude = 283.5;  // meters ... map readings + barometer position
  EnvironmentCalculations::AltitudeUnit envAltUnit  =  EnvironmentCalculations::AltitudeUnit_Meters;
  EnvironmentCalculations::TempUnit     envTempUnit =  EnvironmentCalculations::TempUnit_Celsius;

  /// To get correct local altitude/height (QNE) the reference Pressure
  ///    should be taken from meteorologic messages (QNH or QFF)
  float altitude = EnvironmentCalculations::Altitude(pressure, envAltUnit, referencePressure, tempDrybulb, envTempUnit);

  tempDewpoint = EnvironmentCalculations::DewPoint(tempDrybulb, relHumidity, envTempUnit);

  /// To get correct seaLevel pressure (QNH, QFF)
  ///    the altitude value should be independent on measured pressure.
  /// It is necessary to use fixed altitude point e.g. the altitude of barometer read in a map
  
  seaLevelPres = EnvironmentCalculations::EquivalentSeaLevelPressure(barometerAltitude, tempDrybulb, pressure, envAltUnit, envTempUnit);
  absHumidity = EnvironmentCalculations::AbsoluteHumidity(tempDrybulb, relHumidity, envTempUnit);

  tempHeatindex = EnvironmentCalculations::HeatIndex(tempDrybulb, relHumidity, envTempUnit);

  if (isnan(tempDrybulb) || isnan(pressure) || isnan(relHumidity)) {
    Serial.println("BME280 malfunction");
    bmefail = true;
  }
  else {
    bmefail = false;
    Serial.println("BME280:");
    Serial.print("Temperature: ");
    Serial.print(tempDrybulb); Serial.print(String(tempUnit == BME280::TempUnit_Celsius ? " C  " :" F  "));
    tempDrybulb=tempDrybulb*9/5+32;
    Serial.print(tempDrybulb); Serial.println(" F");
    Serial.print("DP Temperature: ");
    Serial.print(tempDewpoint); Serial.print(String(tempUnit == BME280::TempUnit_Celsius ? " C  " :" F  "));
    tempDewpoint=tempDewpoint*9/5+32;
    Serial.print(tempDewpoint); Serial.println(" F");
    Serial.print("Humidity: ");
    Serial.print(relHumidity); Serial.println(" %");
    Serial.print("Heat index: ");
    Serial.print(tempHeatindex); Serial.print(String(tempUnit == BME280::TempUnit_Celsius ? " C  " :" F  "));
    tempHeatindex=tempHeatindex*9/5+32;
    Serial.print(tempHeatindex); Serial.println(" F");
    Serial.print("Pressure: ");
    Serial.print(pressure); Serial.print(String(presUnit == BME280::PresUnit_hPa ? " hPa  " : " Pa  "));
    Serial.print(pressure*= 0.02953); Serial.println(" inHg");
    rel_pressure_rounded=(int)(seaLevelPres);
    Serial.print("rel pressure_rounded: ");
    Serial.print(rel_pressure_rounded); Serial.println(" hPa");
    Serial.print("Altitude: ");
    Serial.print(altitude); Serial.print((envAltUnit == EnvironmentCalculations::AltitudeUnit_Meters ? " m  " : " ft  "));
    altitude=altitude*3.28084;
    Serial.print(altitude); Serial.println(" ft");
    Serial.print("Sea Level Pressure: ");
    Serial.print(seaLevelPres); Serial.println(String(presUnit == BME280::PresUnit_hPa ? " hPa" : " Pa"));
    Serial.print("Absolute Humidity: ");
    Serial.print(absHumidity); Serial.println(" g/cm^3");
  }
} // void BME280Read

void TSL2591Read(void)
{
  // Simple data read example. Just read the infrared, fullspectrum diode 
  // or 'visible' (difference between the two) channels.
  // This can take 100-600 milliseconds! Uncomment whichever of the following you want to read
  uint16_t x = tsl.getLuminosity(TSL2591_VISIBLE);
  //uint16_t x = tsl.getLuminosity(TSL2591_FULLSPECTRUM);
  //uint16_t x = tsl.getLuminosity(TSL2591_INFRARED);
  Serial.print(F("[ ")); Serial.print(millis()); Serial.print(F(" ms ] "));
  Serial.print(F("Luminosity: "));
  Serial.println(x, DEC);
 
 // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;   // high word, ch1
  full = lum & 0xFFFF; // low, ch0
  Serial.print(F("[ ")); Serial.print(millis()); Serial.print(F(" ms ] "));
  Serial.print(F("IR: ")); Serial.print(ir);  Serial.print(F("  "));
  Serial.print(F("Full: ")); Serial.print(full); Serial.print(F("  "));
  Serial.print(F("Visible: ")); Serial.print(full - ir); Serial.print(F("  "));
  Serial.print(F("Lux: ")); Serial.println(tsl.calculateLux(full, ir), 6);

  uint16_t ch0 = tsl.getLuminosity(TSL2591_FULLSPECTRUM);
  uint16_t ch1 = tsl.getLuminosity(TSL2591_INFRARED); 
  Serial.print(ch0);
  Serial.print(" ;");
  Serial.println(ch1);
  float wh_ch0 = ch0 / 6024.0 *10000 / 1e6;
  float wh_ch1 = ch1/ 1003.0 *10000 / 1e6;
  float ir_ch0 = ch0 / 5338.0 *10000 / 1e6;
  float ir_ch1 = ch1 / 3474.0 *10000 / 1e6;
  Serial.print("white ch0: "); Serial.println(wh_ch0, 6);
  Serial.print("white ch1: "); Serial.println(wh_ch1, 6);
  Serial.print("lamp ch0: "); Serial.println(ir_ch0, 6);
  Serial.print("lamp ch1: "); Serial.println(ir_ch1, 5);

  lightIR = ir;
  lightFullSpec = full;
  lightVisible = full-ir;
  lightLUX = tsl.calculateLux(full, ir);
  if(isnan(lightLUX)) {
    lightLUX = 0;
  }
}  // void TSL2591Read

void readBatVoltage() {
  //******Battery Voltage Monitoring*********************************************
  
  // Voltage divider R1 = 220k+100k+220k =540k and R2=100k
  float calib_factor = 5.2378; // change this value to calibrate the battery voltage
  float raw = analogRead(A0);
  Serial.print("raw a0 output: ");
  Serial.println(raw);
  volt = raw * calib_factor/1023; 
  
  Serial.print( "Voltage = ");
  Serial.print(volt, 2); // print with 2 decimal places
  Serial.println (" V");

}
void configureTSL2591(void)
{
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
   tsl.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)

  // Display the gain and integration time for reference sake //  
  Serial.print  (F("Gain:         "));
  tsl2591Gain_t gain = tsl.getGain();
  switch(gain)
  {
    case TSL2591_GAIN_LOW:
      Serial.println(F("1x (Low)"));
      break;
    case TSL2591_GAIN_MED:
      Serial.println(F("25x (Medium)"));
      break;
    case TSL2591_GAIN_HIGH:
      Serial.println(F("428x (High)"));
      break;
    case TSL2591_GAIN_MAX:
      Serial.println(F("9876x (Max)"));
      break;
  }
  Serial.print  (F("Timing:       "));
  Serial.print((tsl.getTiming() + 1) * 100, DEC); 
  Serial.println(F(" ms"));
} // void configureTSL2591

void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println("TSL2591 sensor characteristics driver");
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" lux"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" lux"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution, 4); Serial.println(F(" lux"));  
  delay(500);
} // void displaySensorDetails

void ReadFromSPIFFS() {
  char filename [] = "/data.txt";
  File myDataFile = SPIFFS.open(filename, "r");       // Open file for reading
  if (!myDataFile) {
    Serial.println("readspiffs: Failed to open file");
    FirstTimeRun();                                   // no file there -> initializing
  }
  
  Serial.println("Reading from SPIFFS");
  
  String temp_data;
  
  temp_data = myDataFile.readStringUntil('\n');  
  saved_timestamp = temp_data.toInt();
  Serial.print("Timestamp from SPIFFS: ");  Serial.println(saved_timestamp);
  
  temp_data = myDataFile.readStringUntil('\n');  
  accuracy = temp_data.toInt();
  Serial.print("Accuracy value read from SPIFFS: ");  Serial.println(accuracy);

  Serial.print("Last 12 saved SPIFFS pressure values: ");
  for (int i = 0; i <= 11; i++) {
    temp_data = myDataFile.readStringUntil('\n');
    pressure_value[i] = temp_data.toInt();
    Serial.print(pressure_value[i]);
    Serial.print("; ");
  }
  myDataFile.close();
  Serial.println();
} // void ReadFromSPIFFS

void WriteToSPIFFS(int write_timestamp) {
  char filename [] = "/data.txt";
  File myDataFile = SPIFFS.open(filename, "w");        // Open file for writing (appending)
  if (!myDataFile) {
    Serial.println("writespiffs: Failed to open file");
  }
  
  Serial.println("Writing to SPIFFS");
  
  myDataFile.println(write_timestamp);                 // Saving timestamp to /data.txt
  myDataFile.println(accuracy);                        // Saving accuracy value to /data.txt
  
  for ( int i = 0; i <= 11; i++) {
    myDataFile.println(pressure_value[i]);             // Filling pressure array with updated values
 }
  myDataFile.close();
  
  Serial.println("File written. Now reading file again.");
  myDataFile = SPIFFS.open(filename, "r");             // Open file for reading
  Serial.print("Found in /data.txt = "); 
  while (myDataFile.available()) { 
    Serial.print(myDataFile.readStringUntil('\n'));
    Serial.print("; ");
  }
  Serial.println();
  myDataFile.close();
} // void WriteToSPIFFS

void FirstTimeRun() {
  Serial.println("Starting SPIFFS initializing process.");
  accuracy = 1;
  char filename [] = "/data.txt";
  File myDataFile = SPIFFS.open(filename, "w");            // Open a file for writing
  if (!myDataFile) {
    Serial.println("firstimerun: Failed to open file");
    Serial.println("Stopping process - maybe flash size not set (SPIFFS).");
    exit(0);
  }
  myDataFile.println(current_timestamp);                   // Saving timestamp to /data.txt
  myDataFile.println(accuracy);                            // Saving accuracy value to /data.txt
  for ( int i = 0; i < 12; i++) {
    myDataFile.println(rel_pressure_rounded);              // Filling pressure array with current pressure
  }
  Serial.println("** Saved initial pressure data. **");
  myDataFile.close();
  Serial.println("Doing a system reset now.");
  resetFunc();                                              //call reset
}  // void FirstTimeRun

int CalculateTrend(){
  int trend;                                    // -1 falling; 0 steady; 1 raising
  Serial.println("Calculating Zambretti trend");
  
  //--> giving the most recent pressure reads more weight
  pressure_difference[0] = (pressure_value[0] - pressure_value[1])   * 1.5;
  pressure_difference[1] = (pressure_value[0] - pressure_value[2]);
  pressure_difference[2] = (pressure_value[0] - pressure_value[3])   / 1.5;
  pressure_difference[3] = (pressure_value[0] - pressure_value[4])   / 2;
  pressure_difference[4] = (pressure_value[0] - pressure_value[5])   / 2.5;
  pressure_difference[5] = (pressure_value[0] - pressure_value[6])   / 3;
  pressure_difference[6] = (pressure_value[0] - pressure_value[7])   / 3.5;
  pressure_difference[7] = (pressure_value[0] - pressure_value[8])   / 4;
  pressure_difference[8] = (pressure_value[0] - pressure_value[9])   / 4.5;
  pressure_difference[9] = (pressure_value[0] - pressure_value[10])  / 5;
  pressure_difference[10] = (pressure_value[0] - pressure_value[11]) / 5.5;
  
  //--> calculating the average and storing it into [11]
  pressure_difference[11] = (  pressure_difference[0]
                             + pressure_difference[1]
                             + pressure_difference[2]
                             + pressure_difference[3]
                             + pressure_difference[4]
                             + pressure_difference[5]
                             + pressure_difference[6]
                             + pressure_difference[7]
                             + pressure_difference[8]
                             + pressure_difference[9]
                             + pressure_difference[10]) / 11;
  
  Serial.print("Current Zambretti trend: ");
  Serial.println(pressure_difference[11]);

  if      (pressure_difference[11] > 3.5) {
    trend_in_words = TEXT_RISING_FAST;
    trend = 1;}
  else if (pressure_difference[11] > 1.5   && pressure_difference[11] <= 3.5)  {
    trend_in_words = TEXT_RISING;
    trend = 1;
  }
  else if (pressure_difference[11] > 0.25  && pressure_difference[11] <= 1.5)  {
    trend_in_words = TEXT_RISING_SLOW;
    trend = 1;
  }
  else if (pressure_difference[11] > -0.25 && pressure_difference[11] < 0.25)  {
    trend_in_words = TEXT_STEADY;
    trend = 0;
  }
  else if (pressure_difference[11] >= -1.5 && pressure_difference[11] < -0.25) {
    trend_in_words = TEXT_FALLING_SLOW;
    trend = -1;
  }
  else if (pressure_difference[11] >= -3.5 && pressure_difference[11] < -1.5)  {
    trend_in_words = TEXT_FALLING;
    trend = -1;
  }
  else if (pressure_difference[11] <= -3.5) {
    trend_in_words = TEXT_FALLING_FAST;
    trend = -1;
  }

  Serial.println(trend_in_words);
  return trend;
}  // int(CalculateTrend()

char ZambrettiLetter() {
  Serial.println("Calculating Zambretti letter");
  char z_letter;
  int(z_trend) = CalculateTrend();
  // Case trend is falling
  if (z_trend == -1) {
    float zambretti = 0.0009746 * rel_pressure_rounded * rel_pressure_rounded - 2.1068 * rel_pressure_rounded + 1138.7019; 
    if (month(current_timestamp) < 4 || month(current_timestamp) > 9) zambretti = zambretti + 1;
    Serial.print("Calculated and rounded Zambretti in numbers: ");
    Serial.println(round(zambretti));
    switch (int(round(zambretti))) {
      case 0:  z_letter = 'A'; break;       //Settled Fine
      case 1:  z_letter = 'A'; break;       //Settled Fine
      case 2:  z_letter = 'B'; break;       //Fine Weather
      case 3:  z_letter = 'D'; break;       //Fine Becoming Less Settled
      case 4:  z_letter = 'H'; break;       //Fairly Fine Showers Later
      case 5:  z_letter = 'O'; break;       //Showery Becoming unsettled
      case 6:  z_letter = 'R'; break;       //Unsettled, Rain later
      case 7:  z_letter = 'U'; break;       //Rain at times, worse later
      case 8:  z_letter = 'V'; break;       //Rain at times, becoming very unsettled
      case 9:  z_letter = 'X'; break;       //Very Unsettled, Rain
    }
  }
  // Case trend is steady
  if (z_trend == 0) {
    float zambretti = 138.24 - 0.133 * rel_pressure_rounded;
    Serial.print("Calculated and rounded Zambretti in numbers: ");
    Serial.println(round(zambretti));
    switch (int(round(zambretti))) {
      case 0:  z_letter = 'A'; break;       //Settled Fine
      case 1:  z_letter = 'A'; break;       //Settled Fine
      case 2:  z_letter = 'B'; break;       //Fine Weather
      case 3:  z_letter = 'E'; break;       //Fine, Possibly showers
      case 4:  z_letter = 'K'; break;       //Fairly Fine, Showers likely
      case 5:  z_letter = 'N'; break;       //Showery Bright Intervals
      case 6:  z_letter = 'P'; break;       //Changeable some rain
      case 7:  z_letter = 'S'; break;       //Unsettled, rain at times
      case 8:  z_letter = 'W'; break;       //Rain at Frequent Intervals
      case 9:  z_letter = 'X'; break;       //Very Unsettled, Rain
      case 10: z_letter = 'Z'; break;       //Stormy, much rain
    }
  }
  // Case trend is rising
  if (z_trend == 1) {
    float zambretti = 142.57 - 0.1376 * rel_pressure_rounded;
    //A Summer rising, improves the prospects by 1 unit over a Winter rising
    if (month(current_timestamp) < 4 || month(current_timestamp) > 9) zambretti = zambretti + 1;
    Serial.print("Calculated and rounded Zambretti in numbers: ");
    Serial.println(round(zambretti));
    switch (int(round(zambretti))) {
      case 0:  z_letter = 'A'; break;       //Settled Fine
      case 1:  z_letter = 'A'; break;       //Settled Fine
      case 2:  z_letter = 'B'; break;       //Fine Weather
      case 3:  z_letter = 'C'; break;       //Becoming Fine
      case 4:  z_letter = 'F'; break;       //Fairly Fine, Improving
      case 5:  z_letter = 'G'; break;       //Fairly Fine, Possibly showers, early
      case 6:  z_letter = 'I'; break;       //Showery Early, Improving
      case 7:  z_letter = 'J'; break;       //Changeable, Improving
      case 8:  z_letter = 'L'; break;       //Rather Unsettled Clearing Later
      case 9:  z_letter = 'M'; break;       //Unsettled, Probably Improving
      case 10: z_letter = 'Q'; break;       //Unsettled, short fine Intervals
      case 11: z_letter = 'T'; break;       //Very Unsettled, Finer at times
      case 12: z_letter = 'Y'; break;       //Stormy, possibly improving
      case 13: z_letter = 'Z'; break;;      //Stormy, much rain
    }
  }
  Serial.print("This is Zambretti's famous letter: ");
  Serial.println(z_letter);
  return z_letter;
} // char ZambrettiLetter()

String ZambrettiSays(char code){
  String zambrettis_words = "";
  switch (code) {
  case 'A': zambrettis_words = TEXT_ZAMBRETTI_A; break;  //see Tranlation.h
  case 'B': zambrettis_words = TEXT_ZAMBRETTI_B; break;
  case 'C': zambrettis_words = TEXT_ZAMBRETTI_C; break;
  case 'D': zambrettis_words = TEXT_ZAMBRETTI_D; break;
  case 'E': zambrettis_words = TEXT_ZAMBRETTI_E; break;
  case 'F': zambrettis_words = TEXT_ZAMBRETTI_F; break;
  case 'G': zambrettis_words = TEXT_ZAMBRETTI_G; break;
  case 'H': zambrettis_words = TEXT_ZAMBRETTI_H; break;
  case 'I': zambrettis_words = TEXT_ZAMBRETTI_I; break;
  case 'J': zambrettis_words = TEXT_ZAMBRETTI_J; break;
  case 'K': zambrettis_words = TEXT_ZAMBRETTI_K; break;
  case 'L': zambrettis_words = TEXT_ZAMBRETTI_L; break;
  case 'M': zambrettis_words = TEXT_ZAMBRETTI_M; break;
  case 'N': zambrettis_words = TEXT_ZAMBRETTI_N; break;
  case 'O': zambrettis_words = TEXT_ZAMBRETTI_O; break;
  case 'P': zambrettis_words = TEXT_ZAMBRETTI_P; break; 
  case 'Q': zambrettis_words = TEXT_ZAMBRETTI_Q; break;
  case 'R': zambrettis_words = TEXT_ZAMBRETTI_R; break;
  case 'S': zambrettis_words = TEXT_ZAMBRETTI_S; break;
  case 'T': zambrettis_words = TEXT_ZAMBRETTI_T; break;
  case 'U': zambrettis_words = TEXT_ZAMBRETTI_U; break;
  case 'V': zambrettis_words = TEXT_ZAMBRETTI_V; break;
  case 'W': zambrettis_words = TEXT_ZAMBRETTI_W; break;
  case 'X': zambrettis_words = TEXT_ZAMBRETTI_X; break;
  case 'Y': zambrettis_words = TEXT_ZAMBRETTI_Y; break;
  case 'Z': zambrettis_words = TEXT_ZAMBRETTI_Z; break;
   default: zambrettis_words = TEXT_ZAMBRETTI_DEFAULT; break;
  }
  return zambrettis_words;
} // String ZambrettiSays

void getparseData(String location) {
  HTTPClient http;  //Declare an object of class HTTPClient
  http.begin(location);  //Specify request destination
  int httpCode = http.GET();  //Send the request
  Serial.println("httpCode");
  Serial.println(location);

  if (httpCode > 0) { //Check the returning code
    
// from https://arduinojson.org/v6/assistant/ just paste JSON results from GET request and you get the parse code
// enumerate the location string, paste it into browser, enter, then cut and past JSON or Raw tabs data into
// arduinojson.org webpage.  Use parsing program output.
  const size_t capacity = JSON_OBJECT_SIZE(1) + JSON_OBJECT_SIZE(5) + 80;
  DynamicJsonDocument doc(capacity);
  const char* json = http.getString().c_str();
  deserializeJson(doc, json);
  sleepTimesec = doc["shared"]["sleepTimesec"];
  OTAupdate = doc["shared"]["OTAupdate"]; // not used in this example
  ledOnsec = doc["shared"]["ledOnsec"];
  ledIntensity = doc["shared"]["ledIntensity"];
  publishTimeseries = doc["shared"]["publishTimeseries"];

    Serial.println("results from json parsing: sleepTime, OTAupdate, ledOnsec, ledIntensity and publishTimeseries");
    Serial.print(sleepTimesec);
    Serial.print("; ");
    Serial.print(OTAupdate);
    Serial.print("; ");
    Serial.print(ledOnsec);
    Serial.print("; ");
    Serial.print(ledIntensity);
    Serial.print(""); 
    Serial.print(publishTimeseries);
    Serial.println(""); 
  } // end if httpCode > 0
} // void getparseData

void goToSleep() {
  Serial.print ("Going to sleep now for ");
  Serial.print (sleepTimesec);
  Serial.println (" seconds.");
  
  ESP.deepSleep(sleepTimesec * 1000000); // convert to microseconds
} // void goToSleep
