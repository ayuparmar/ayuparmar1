#include <MiCS6814-I2C.h>

#ifdef ESP32
#error ESP32 does not work with SoftSerial, use HardwareSerial example instead
#endif

#include "ThingSpeak.h"
#include <ESP8266WiFi.h>
#include "secrets.h"
#include <SDS011.h>
MiCS6814 sensor;
bool sensorConnected;

#define SENSOR_ADDR     0X04        // default to 0x04

#include "DHT.h"
#define DHTPIN D3                 //DHT pin declaration

char ssid[] = SECRET_SSID;   // your network SSID (name)
char pass[] = SECRET_PASS;   // your network password
WiFiClient  client;

unsigned long myChannelNumber = SECRET_CH_ID;  // Thingspeak channel
const char * myWriteAPIKey = SECRET_WRITE_APIKEY;  // Thingspeak channel write key

#define DHTTYPE DHT22
float p10, p25;
int error;
double temp;
double h;
SDS011 my_sds;   //sds instance
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);  // Initialize serial
  Serial.println("Node Initialising");
  my_sds.begin(D5, D6); //RX, TX
 
  
  WiFi.mode(WIFI_STA);
  dht.begin(); 
  ThingSpeak.begin(client);  // Initialize ThingSpeak
  Serial.begin(115200);
sensorConnected = sensor.begin(SENSOR_ADDR);

  if (sensorConnected == true) {
    // Print status message
    Serial.println("Connected to MiCS-6814 sensor");

    // Turn heater element on
    sensor.powerOn();
    
    // Print header for live values
    Serial.println("Current concentrations:");
    Serial.println("CO\tNO2\tNH3\tC3H8\tC4H10\tCH4\tH2\tC2H5OH");
  } else {
    // Print error message on failed connection
    Serial.println("Couldn't connect to MiCS-6814 sensor");
  }
}


void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(SECRET_SSID);
    while (WiFi.status() != WL_CONNECTED) {
      WiFi.begin(ssid, pass);  // Connect to WPA/WPA2 network. Change this line if using open or WEP network
      Serial.print(".");
      delay(5000);
    }

    Serial.println("\nConnected.");
  }
  
    Serial.println("\nConnected.");
  h = dht.readHumidity();
  temp = dht.readTemperature();
 
 
  error = my_sds.read(&p25, &p10);
    Serial.println("P2.5: " + String(p25));
    Serial.println("P10:  " + String(p10));
    Serial.println(temp);
    Serial.println(h);
    Serial.println(sensor.measureCO());
    Serial.println(sensor.measureNO2());
    Serial.println(sensor.measureNH3());
    ThingSpeak.setField(1, (float)temp);
    ThingSpeak.setField(2, (float)h);
    ThingSpeak.setField(3, (float)p25);
    ThingSpeak.setField(4, (float)p10);
    ThingSpeak.setField(5, sensor.measureCO());
    ThingSpeak.setField(6, sensor.measureNO2());
    ThingSpeak.setField(7, sensor.measureNH3());
    delay(1000);
    // write to the ThingSpeak channel
    int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    if (x == 200) {
      Serial.println("Channel update successful.");
    }
    \
    else {
      Serial.println("Problem updating channel. HTTP error code " + String(x));
    }
  

  delay(15000);
}
