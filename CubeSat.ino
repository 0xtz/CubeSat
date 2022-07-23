// imports 
#include <WiFi101.h>
#include <Wire.h>
#include <ArduCAM.h>
#include "memorysaver.h"
#include <Arduino_MKRENV.h>
#include "RTClib.h"
#include <SPI.h>
#include <SD.h>
#include <Seeed_HM330X.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

File myFile;
RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
// ----------------------- WEBSERVER 
char ssid[] = "SSID";        
char pass[] = "NETWORK_KEY";     
int keyIndex = 0;                      
int status = WL_IDLE_STATUS;          //connection status
WiFiServer server(80);                //server socket port 

WiFiClient client = server.available();

// ----------------------- CAMERA
#if !(defined (OV5640_MINI_5MP_PLUS)||defined (OV5642_MINI_5MP_PLUS))
#error Please select the hardware platform and camera module in the ../libraries/ArduCAM/memorysaver.h file
#endif
#define   FRAMES_NUM    0x00
const int CS = 1;
#define SD_CS 0
bool is_header = false;
int total_time = 0;
#if defined (OV5640_MINI_5MP_PLUS)
  ArduCAM myCAM(OV5640, CS);
#else
  ArduCAM myCAM(OV5642, CS);
#endif
uint8_t read_fifo_burst(ArduCAM myCAM);
//---------------------- Grove Particle


#ifdef  ARDUINO_SAMD_VARIANT_COMPLIANCE
    #define SERIAL_OUTPUT SerialUSB
#else
    #define SERIAL_OUTPUT Serial
#endif

HM330X sensor;
uint8_t buf[30];


const char* str[] = {"sensor num: ", "PM1.0 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM2.5 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM10 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                     "PM1.0 concentration(Atmospheric environment,unit:ug/m3): ",
                     "PM2.5 concentration(Atmospheric environment,unit:ug/m3): ",
                     "PM10 concentration(Atmospheric environment,unit:ug/m3): ",
                    };

HM330XErrorCode print_result(const char* str, uint16_t value) {
    if (NULL == str) {
        return ERROR_PARAM;
    }
    SERIAL_OUTPUT.print(str);
    SERIAL_OUTPUT.println(value);
    return NO_ERROR;
}

HM330XErrorCode parse_result(uint8_t* data) {
    uint16_t value = 0;
    if (NULL == data) {
        return ERROR_PARAM;
    }
    for (int i = 1; i < 8; i++) {
        value = (uint16_t) data[i * 2] << 8 | data[i * 2 + 1];
        print_result(str[i - 1], value);

    }

    return NO_ERROR;
}
String airQuality;


//------------------ Sound Detector
String soundLevel;


void setup() 
{
    // eable wifi 
    enable_WiFi();
    connect_WiFi();  
    //ENV Shield
    if (!ENV.begin()) {
    while (1);
  }


  //Temperature Setup
   sensors.begin(); 


  //RTC Setup
    if (! rtc.begin()) {
    Serial.flush();
    abort();
  }

//SD Setup
if (!SD.begin(0)) {
    while (1);
  }


  //ArduCam Setup
  
  uint8_t vid, pid;
uint8_t temp;
#if defined(__SAM3X8E__)
  Wire1.begin();
#else
  Wire.begin();
#endif

pinMode(CS, OUTPUT);
digitalWrite(CS, HIGH);

SPI.begin();
myCAM.write_reg(0x07, 0x80);
delay(100);
myCAM.write_reg(0x07, 0x00);
delay(100); 
  
while(1){
  myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
  temp = myCAM.read_reg(ARDUCHIP_TEST1);
  if(temp != 0x55)
  {
    Serial.println(F("SPI interface Error!"));
    delay(1000);continue;
  }else{
    Serial.println(F("SPI interface OK."));break;
  }
}
#if defined (OV5640_MINI_5MP_PLUS)
while(1){
  //Check if the camera module type is OV5640
  myCAM.rdSensorReg16_8(OV5640_CHIPID_HIGH, &vid);
  myCAM.rdSensorReg16_8(OV5640_CHIPID_LOW, &pid);
  if ((vid != 0x56) || (pid != 0x40)){
    Serial.println(F("Can't find OV5640 module!"));
    delay(1000); continue;
  }else{
    Serial.println(F("OV5640 detected."));break;      
  }
}
#else
while(1){
  //Check if the camera module type is OV5642
  myCAM.rdSensorReg16_8(OV5642_CHIPID_HIGH, &vid);
  myCAM.rdSensorReg16_8(OV5642_CHIPID_LOW, &pid);
  if ((vid != 0x56) || (pid != 0x42)){
    Serial.println(F("Can't find OV5642 module!"));
    delay(1000);continue;
  }else{
    Serial.println(F("OV5642 detected."));break;      
  }
}
#endif
//Initialize SD Card
while(!SD.begin(SD_CS))
{
  Serial.println(F("SD Card Error!"));delay(1000);
}
Serial.println(F("SD Card detected."));


//Change to JPEG capture mode and initialize the OV5640 module
myCAM.set_format(JPEG);
myCAM.InitCAM();
myCAM.set_bit(ARDUCHIP_TIM, VSYNC_LEVEL_MASK);
myCAM.clear_fifo_flag();
myCAM.write_reg(ARDUCHIP_FRAMES, FRAMES_NUM);

}
// ---------------------- LOOP FUNC 
void loop() 
{
  //Real Time Clock

   if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  } 
  
    DateTime now = rtc.now();
    
  //Temperature Sensor
  sensors.requestTemperatures();
  
  String tepreture_value = (sensors.getTempCByIndex(0); 
    
  //ENV Shield
  float temperature = ENV.readTemperature();
  float humidity    = ENV.readHumidity();
  float pressure    = ENV.readPressure();
  float illuminance = ENV.readIlluminance();
 //UV Sensor
  float sensorVoltage; 
  float sensorValue;
  float uVindex;
  sensorValue = analogRead(A6);
  sensorVoltage = sensorValue/1024*5;
  uVindex = sensorVoltage/0.1;

  String uV_value = uVindex;

//Sound detector 
   int value;
  value = analogRead(A5);
  Serial.print("Sound Status: ");
  if(value <= 100){
    soundLevel = "Quiet";
  }else if( (value > 100) && (value <= 160) ) {
    soundLevel = "Moderate";
  }else if( (value > 160) && ( value <= 220) ) {
    soundLevel = "Loud";
  }else if(value > 220) {
    soundLevel = "Very Loud";
  }
  String sound_value = soundLevel;
  //Grove Particle Sensor
      if (sensor.read_sensor_value(buf, 29)) {
        SERIAL_OUTPUT.println("HM330X read result failed!!!");
    }
   if ((buf[7]) < 12.0) {
    airQuality = "Good";
   }else if ((buf[7]) < 35.4) {
    airQuality = "Moderate";
   }else if ((buf[7]) < 55.4) {
    airQuality = "Unhealthy for Sensitive Groups";
   }else if ((buf[7]) < 150.4) {
    airQuality = "Unhealthy";
   }else if ((buf[7]) < 250.4) {
    airQuality = "Very Unhealthy";
   }else {
    airQuality = "Hazardous";
   }
   String airQuality_value = airQuality;

    //SD Card
  myFile = SD.open("info.txt", FILE_WRITE);

  if (myFile) {
    Serial.println("Writing to SD card");
        
    myFile.print(now.year(), DEC);
    myFile.print('/');
    myFile.print(now.month(), DEC);
    myFile.print('/');
    myFile.print(now.day(), DEC);
    myFile.print(" (");
    myFile.print(daysOfTheWeek[now.dayOfTheWeek()]);
    myFile.print(") ");
    myFile.print(now.hour(), DEC);
    myFile.print(':');
    myFile.print(now.minute(), DEC);
    myFile.print(':');
    myFile.println(now.second(), DEC);
  myFile.print("DS18B20 Temperature: ");
  myFile.print(sensors.getTempCByIndex(0)); 
  myFile.print(" C, ");
  myFile.print(sensors.getTempFByIndex(0));
  myFile.println(" F");
  myFile.print("ENV Shield Temperature = ");
  myFile.print(temperature);
  myFile.println(" C");
  myFile.print("Humidity    = ");
  myFile.print(humidity);
  myFile.println(" %");
  myFile.print("Pressure    = ");
  myFile.print(pressure);
  myFile.println(" kPa");
  myFile.print("Illuminance = ");
  myFile.print(illuminance);
  myFile.println(" lx");
    myFile.print("UV index    = ");
    myFile.println(uVindex);
  myFile.print("Sound Status: ");
  myFile.println(soundLevel);
   myFile.print("Air Quality: ");
   myFile.println(airQuality);
   myFile.print("PM1.0 Concentration: ");
   myFile.print(buf[5]);
   myFile.println(" ug/m3");
   myFile.print("PM2.5 Concentration: ");
   myFile.print(buf[7]);
   myFile.println(" ug/m3");
   myFile.print("PM10 Concentration: ");
   myFile.print(buf[9]);
   myFile.println(" ug/m3");
   myFile.println("----------------------------------------------------------------------");
   myFile.println();

    myFile.close();
  } else {
    // if the file didn't open, print an error:

  }

  
    //ArduCam

    myCAM.flush_fifo();
myCAM.clear_fifo_flag();
#if defined (OV5640_MINI_5MP_PLUS)
  myCAM.OV5640_set_JPEG_size(OV5640_320x240);delay(1000);
#else
  myCAM.OV5642_set_JPEG_size(OV5642_320x240);delay(1000);
#endif
//Start capture
myCAM.start_capture();

total_time = millis();
while ( !myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)); 

total_time = millis() - total_time;

total_time = millis();
read_fifo_burst(myCAM);
total_time = millis() - total_time;

//Clear the capture done flag
myCAM.clear_fifo_flag();

delay(5000);
}
uint8_t read_fifo_burst(ArduCAM myCAM)
{
uint8_t temp = 0, temp_last = 0;
uint32_t length = 0;
static int i = 0;
static int k = 0;
char str[8];
File outFile;
byte buf[256]; 
length = myCAM.read_fifo_length();

if (length >= MAX_FIFO_SIZE) //8M
{
  return 0;
}
if (length == 0 ) //0 kb
{
  return 0;
} 
myCAM.CS_LOW();
myCAM.set_fifo_burst();//Set fifo burst mode
i = 0;
while ( length-- )
{
  temp_last = temp;
  temp =  SPI.transfer(0x00);
  //Read JPEG data from FIFO
  if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
  {
    buf[i++] = temp;  //save the last  0XD9     
    //Write the remain bytes in the buffer
    myCAM.CS_HIGH();
    outFile.write(buf, i);    
    //Close the file
    outFile.close();
    is_header = false;
    myCAM.CS_LOW();
    myCAM.set_fifo_burst();
    i = 0;
  }  
  if (is_header == true)
  { 
    //Write image data to buffer if not full
    if (i < 256)
     buf[i++] = temp;
    else
    {
      //Write 256 bytes image data to file
      myCAM.CS_HIGH();
      outFile.write(buf, 256);
      i = 0;
      buf[i++] = temp;
      myCAM.CS_LOW();
      myCAM.set_fifo_burst();
    }        
  }
  else if ((temp == 0xD8) & (temp_last == 0xFF))
  {
    is_header = true;
    myCAM.CS_HIGH();
    //Create a avi file
    k = k + 1;
    itoa(k, str, 10);
    strcat(str, ".jpg");
    //Open the new file
    outFile = SD.open(str, O_WRITE | O_CREAT | O_TRUNC);
    if (! outFile)
    {
      while (1);
    }
    myCAM.CS_LOW();
    myCAM.set_fifo_burst();   
    buf[i++] = temp_last;
    buf[i++] = temp;   
  }
}
  myCAM.CS_HIGH();
  return 1;
  client = server.available();
  
  if (client) {
   printWEB();
  }
  
}


void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");

  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}

void enable_WiFi() {

  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") {
    Serial.println("Please upgrade the firmware");
  }
}

void connect_WiFi() {
  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
}

void printWEB() {
  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        if (c == '\n') {                    // if the byte is a newline character

          if (currentLine.length() == 0) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html"); 
            htmlPage( client );

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          }
          else {      // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }

      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}

void htmlPage( WiFiClient client ) {
  String value = "$$$$";
  client.print("<!doctype html><html lang=\"fr\">");
  client.print("<head> <meta charset=\"utf-8\"> <title> CubeSat </title> </head>");
  client.print("<body><style></style> </head>");
  client.print("<body style=\"margin: 0;font-family: monospace;\">");
  client.print("<nav style=\"width: 100vw;background-color: #D6EFFF;\"><ul style=\"margin: 0;padding: 0;\"><li style=\"list-style-type: none;display: inline;margin: 0 5px;font-size: 20px;\">");
  client.print("<a style=\"text-decoration: none;color: black;padding: 2vw;\"> CubeSat Team Morocco || FirstGlobal </a></li></ul></nav>");
  
  //   -----------------------------------------------------------------------------------------
  client.print("    <br> <p style=\"text-align:center; font-size: 20px;\"> 1. Temperature : "+ tepreture_value +" °C</p><br>");
  client.print("    <p style=\"text-align:center; font-size: 20px;\"> 2.  Sound Status : "+ sound_value +"</p> <br>");
  client.print("    <p style=\"text-align:center; font-size: 20px;\"> 3. UV index  "+ uV_value +"  </p><br> ");
  client.print("    <p style=\"text-align:center; font-size: 20px;\"> 4. Air Quality  "+ value +" </p> <br>");
  // ---------------------------------------------------------------------------------------------
  
  client.print("<script> setTimeout(function(){window.location.reload(1);}, 3000); ");
  client.print("var today = new Date();");
  client.print("var date = today.getFullYear()+'-'+(today.getMonth()+1)+'-'+today.getDate();");
  client.print("var time = today.getHours() + \":\" + today.getMinutes() + \":\" + today.getSeconds();");
  client.print("var dateTime = 'Date '+date+' '+time;");
  client.print("document.getElementById(\"Date\").innerHTML = dateTime ;</script> </body> </html>");
}
