#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>
#include <freertos/xtensa_timer.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <WiFi.h>

const char* ssid ="Hynes57"; //replace this with your wifi  name
const char* password ="Donaldduck"; //replace with your wifi password

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
WiFiClient wifiClient;
PubSubClient client(wifiClient);
int status = WL_IDLE_STATUS;
char hostname[] ="54.205.9.137"; //replace this with IP address of machine 
//on which broker is installed
#define TOKEN "esp32board"
#define TOPIC "esp32board"

#define SENSOR_PIN 27

#define WATT_CONVERSION_CONSTANT 3600000
unsigned long last_read;
int calc_power;
int watt_hr_current_hr;
//TODO: price_current_hr should be updated hourly to calculate the price for that hour 
int ore_per_Kwhr = 114; //DKK øre!/Kwr 
float paid_this_hr; // Øre!!!
// Send updated meassurement flag
bool transmit_value = false;


//just a dummy value
volatile unsigned int dummy;

void IRAM_ATTR flash_handler();
void print_wakeup_reason();
void reconnect();
void MQTTPOST();

void setup() {
  // put your setup code here, to run once:
	Serial.begin(115200);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_27, 0);
  
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi\n");
  Serial.println("ESP32 AS PUBLISHER\n");
  client.setServer(hostname, 1883 ); //default port for mqtt is 1883
  timeClient.update();
  if ( !client.connected() )
  {
    reconnect();
  }

  last_read = millis(); //Give it an initial value
  Serial.printf("started the program\n");

	pinMode(SENSOR_PIN, INPUT_PULLUP);
	attachInterrupt(SENSOR_PIN, flash_handler, FALLING);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (transmit_value) // Did we receive a request for updated values
    {
        paid_this_hr = ore_per_Kwhr * watt_hr_current_hr / 1000.0;
        Serial.printf("Whr this hour:%d Whr\n",watt_hr_current_hr);
        Serial.printf("paid this hour:%f øre DKK\n",paid_this_hr);
        Serial.printf("Current power drain:%d Watts\n",calc_power);
        String values = String(calc_power);
        //Particle.publish("power", values);
        if ( !client.connected() )
        {
          reconnect();
        }
        MQTTPOST();
        delay(1);
        transmit_value = false;
    }

  /*if(digitalRead(SENSOR_PIN) & !transmit_value){
    Serial.printf("entering light sleep\n");
    delayMicroseconds(500);
    dummy = !dummy;
    esp_light_sleep_start();
    Serial.printf("leaving light sleep\n");
    status = WiFi.status();
    if ( WiFi.status() != WL_CONNECTED){
      reconnect();
    }
    print_wakeup_reason();
  }*/
  
}

void IRAM_ATTR flash_handler() {
    unsigned long delta;
    unsigned long current_reading = millis();
    
    if ((delta = current_reading-last_read) > 200)
    {
        watt_hr_current_hr += 1;
        calc_power = WATT_CONVERSION_CONSTANT / delta;
        last_read = current_reading;
        transmit_value = true;
    }
}
//price_current_per_Kwhr = 1.14; //Dkk pr Kwr

void MQTTPOST()
{
//payload formation begins here
 

 String payload ="{";
 payload +="\"Topic\":\""; payload +=TOPIC; payload +="\",";
 payload +="\"watt_hr_current_hr\":"; payload +=watt_hr_current_hr; payload +=",";
 payload +="\"paid_this_hr\":"; payload +=paid_this_hr; payload +=",";
 payload +="\"calc_power\":"; payload +=calc_power; payload +=",";
 payload +="\"Timestamp\":"; payload +=timeClient.getEpochTime(); 
 payload +="}";

 char attributes[1000];
 payload.toCharArray( attributes, 1000 );
 client.publish(TOPIC, attributes); //topic="test" MQTT data post command.
 Serial.println( attributes );
 delayMicroseconds(1000);
 //esp_light_sleep_start();
}

/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO\n"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL\n"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer\n"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad\n"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program\n"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void reconnect() 
{
  while (!client.connected()) {
    status = WiFi.status();
    if ( status != WL_CONNECTED) 
    {
      WiFi.begin(ssid, password);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }

      Serial.println("Connected to AP");
    }

    Serial.print("Connecting to Broker …");
    Serial.print("hostname");

    if ( client.connect("Test", TOKEN, NULL) )
    {
      Serial.println("[DONE]" );
    }
    else {
      Serial.println( " : retrying in 5 seconds]" );
      delay( 5000 );
    }
  }
}