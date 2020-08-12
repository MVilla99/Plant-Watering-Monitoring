/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/mauri/Documents/IoTc2/git_hub/l14-soilmoisture-MVilla99/PlantWatering/src/PlantWatering.ino"
/*
 * Project PlantWatering
 * Description: for room/plant monitoring 
 * Author: Mauricio Villa
 * Date: 9 - August - 2020
 */
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MQTT.h>
#include <Air_quality_Sensor.h>

#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"

void setup();
void loop();
void  MQTT_connect();
void bmeValues();
void oledPrint();
void pump();
void dustSensor();
void airQualitySensor();
#line 18 "c:/Users/mauri/Documents/IoTc2/git_hub/l14-soilmoisture-MVilla99/PlantWatering/src/PlantWatering.ino"
#define AIO_SERVER "io.adafruit.com"
#define AIO_SERVERPORT 1883
#define AIO_USERNAME "mauriciov99"
#define AIO_KEY "aio_tzOm50B3ppnXKjLlCYWlCvwf8vdK"
#define OLED_RESET A0

TCPClient TheClient;
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY);

Adafruit_BME280 bme;
Adafruit_SSD1306 display(OLED_RESET);
AirQualitySensor AQsensor(A2); 

Adafruit_MQTT_Subscribe SubData = Adafruit_MQTT_Subscribe(&mqtt,AIO_USERNAME "/feeds/pumping");
Adafruit_MQTT_Publish PubProbeVal = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/probeValue");
Adafruit_MQTT_Publish PubTemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/roomTemp");
Adafruit_MQTT_Publish PubAQ = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/seeedAQSensor");
Adafruit_MQTT_Publish PubDust = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/seeedDustSensor");

unsigned long last;
float value;
int probeVal;
int probeRead;
float temp;
float hum;
int relayPin = A5;

int dustPin = A3;
unsigned long dustDuration;
unsigned long dustStartTime;
unsigned long ReplaceThisTime = 30000;
unsigned long lowPulseOccupancy = 0;
float ratio = 0;
float concentration = 0;

int quality;
int AQval;
int AQCodeVal;

void setup() {
mqtt.subscribe(&SubData);
Wire.begin();
display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
display.display();
display.clearDisplay();
display.display();
bme.begin(0x76);
AQsensor.init();
pinMode(relayPin, OUTPUT); 
pinMode(dustPin, INPUT);
dustStartTime = millis();
Serial.begin(9600);
delay(100);

}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  // The core of your code will likely live here.
  probeRead = analogRead(A1);
  probeVal = probeRead;
 // if(probeVal>=2450){
 //   pump();
 // }
  bmeValues();
  dustSensor();
  airQualitySensor();
  oledPrint();  
  MQTT_connect();

  if((millis()-last)>120000){
    Serial.printf("pinging MQTT\n");
    if(!mqtt.ping()){
      Serial.printf("disconnecting \n");
      mqtt.disconnect();
    }
    last = millis();
    }
/*      for subscribing       */
  Adafruit_MQTT_Subscribe *subscription;
  while((subscription = mqtt.readSubscription(5000))){
    if(subscription == &SubData){
      value = atof((char *)SubData.lastread);
      if(value ==1){
        pump();
      }
    }
  }
  /*      for publishing       */
  if((millis()-last>15000)){
    if(mqtt.Update()){
      PubProbeVal.publish(probeVal);
      PubTemp.publish(temp);
      PubAQ.publish(AQval);
      PubDust.publish(concentration);
    }
  last = millis();
  }

  
}

void  MQTT_connect(){
  int8_t ret;

  if(mqtt.connected()){
    return;
  }

  Serial.print("connecting to MQTT..");
  while((ret = mqtt.connect()) != 0){
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("retrying MQTT connectioon in 5 seconds...");
    mqtt.disconnect();
    delay(5000);
  }
  Serial.println("MQTT connected!");
}
void bmeValues(){ // function for reading the bme280 sensor
  temp = (bme.readTemperature() * 9/5)+32;
  hum = bme.readHumidity();
}
void oledPrint(){ // function for printing values to the oled screen
  display.clearDisplay(); // for displaying BME readings.
  display.display();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.printf("temperature is: %0.02f\n",temp);
  display.display();
  Serial.println("display working");
  display.setCursor(0,10);
  display.printf("soil value is: %i\n",probeVal);

  display.setCursor(0,20); // for display dust sensor values
  display.printf("concentration = %0.02f\n",concentration);
  display.printf("pcs/0.01cf \n");
  display.display(); 

  display.setCursor(0,25); // for displaying air quality code value.
  display.printf("AQ code: %i\n",AQCodeVal);
  display.display();
}
void pump(){ // function for using the relay to engage the pump and water for half a second
  unsigned long startMillis;
    startMillis = millis();
    while(millis()-startMillis<=500){
      digitalWrite(relayPin, HIGH);
      Serial.println("pumping");
    }
  digitalWrite(relayPin,LOW);
  Serial.println("not pumped");
}

void dustSensor(){ // function for measuring dust sensor values
  dustDuration = pulseIn(dustPin, LOW);
  lowPulseOccupancy = lowPulseOccupancy+dustDuration;

  if((millis()-dustStartTime)>ReplaceThisTime){
    ratio = lowPulseOccupancy/(ReplaceThisTime*10.0);
    concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62;
    Serial.println("dust sensor engaged");
    lowPulseOccupancy = 0;
    dustStartTime = millis();
  }
}
void airQualitySensor(){ // function for measuring air quality
  quality = AQsensor.slope();
  AQval = AQsensor.getValue();
  AQCodeVal = 0;
  if (quality == AirQualitySensor::FORCE_SIGNAL) {
    AQCodeVal = AirQualitySensor::FORCE_SIGNAL;
  }
  else if (quality == AirQualitySensor::HIGH_POLLUTION) {
    AQCodeVal = AirQualitySensor::HIGH_POLLUTION;
  }
  else if (quality == AirQualitySensor::LOW_POLLUTION) {
    AQCodeVal = AirQualitySensor::LOW_POLLUTION;
  }
  else if (quality == AirQualitySensor::FRESH_AIR) {
    AQCodeVal = AirQualitySensor::FRESH_AIR;
  }
  Serial.println(AQval);
  Serial.println(AQCodeVal);
}