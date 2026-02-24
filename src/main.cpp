#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <ScioSense_ENS16x.h>
#include "ESP8266_ISR_Servo.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "secrets.h"

#define SCL_PIN PIN_WIRE_SCL //5
#define SDA_PIN PIN_WIRE_SDA //4

/****AHT21 sensor****/
#define AHT21_I2C_ADDRESS 0x38
Adafruit_AHTX0 aht;
sensors_event_t hum_event, temp_event;
float humidity_float, temp_float;

/****ENS160 sensor****/
#define ENS160_I2C_ADDRESS 0x52
ENS160 ens160;
//humidity and temp reformatted to format required for ENS160 calibration
uint16_t humidity_ens, temp_ens;

/****Birb servo****/
#define SERVO_MIN_MICROS      544
#define SERVO_MAX_MICROS      2450
#define SERVO_PIN             2
int servo_idx;
int birb_position;

/****MQTT****/
/* Required states / events:
- Publish temp (float)
- Publish hum (float)
- Publish eco2 (int)
- Publish tvoc (int)
- Publish AQI-UBA (int)
- Publish bird state (bool)

- Subscribe home assistant status (bool)
- Subscribe HA-generated birb status override (bool)
- Substribe HA-generated temp values?
*/
#define MQTT_CLIENT_ID "Birb-Mqtt-Client"
#define HA_DISCOVERY_TOPIC "homeassistant/device/birb/config"
WiFiClient espClient;
PubSubClient mqtt_client(espClient);


/****Other sensors and outputs****/
bool led_state;

void print_ens160_compensation_registers() {
  uint8_t rdata[4];
  uint16_t temp_raw, hum_raw;
  float temp_parsed, hum_parsed;

  ens160.read(ENS16X_REGISTER_ADDRESS_DATA_T, rdata, 4);
  temp_raw = rdata[1];
  temp_raw <<= 8;
  temp_raw |= rdata[0];
  hum_raw = rdata[3];
  hum_raw <<= 8;
  hum_raw |= rdata[2];

  temp_parsed = ((float) temp_raw / 64) - 273.15;
  hum_parsed = ((float) hum_raw) / 512;

  
  Serial.printf("ENS160 temp comp: raw=%04x, converted=%2.2f\n", temp_raw, temp_parsed);
  Serial.printf("ENS160 hum  comp: raw=%04x, converted=%2.2f\n", hum_raw, hum_parsed);
}

/** Enable WiFi connection. Blocks until WiFi-connection is established */
void setup_wifi() {
  delay(10);
  Serial.printf("Conneting to wifi %s\n", WIFI_SSID_NAME);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID_NAME, WIFI_PASSWORD);

  //TODO: Fallback to run without WiFi if not connected
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());
  Serial.println("WiFi connected");
  Serial.printf("IP address: %p", WiFi.localIP());
}

void publish_mqtt_ha_sensors() {
  JsonDocument json;
}

/**Establish MQTT connection, creating initial broadcast message of all sensors */
void mqtt_connect() {
  while(!mqtt_client.connected()) {
    Serial.print("Attempting MQTT connection ...");
    String clientID = "Birb-Client-";
    clientID += String(random(0xffff), HEX);

    //TODO pretty sure we're using username and password as well
    if (mqtt_client.connect(MQTT_CLIENT_ID)) {
      Serial.println("MQTT connection established");
      mqtt_client.publish("mqtt_test/outTopic", "hello world");
      mqtt_client.subscribe("mqtt_test/inTopic"); 
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}



void setup() {
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);

  //Setup AHT sensor on I2C bus
  Serial.println("Starting AHT21");
  aht.begin(&Wire);

  //Setup ENS160 sensor on I2C bus
  Serial.println("Starting ENS160");
  ens160.enableDebugging(Serial);
  ens160.begin(&Wire);
  while (!ens160.init()) {
    Serial.print(".");
    delay(1000);
  }
  ens160.startStandardMeasure(); //Set to STANDARD mode for performing operations
  Serial.println("\nENS160 online");

  //Setup servo motor for bird
  // servo_idx = ISR_Servo.setupServo(SERVO_PIN, SERVO_MIN_MICROS, SERVO_MAX_MICROS);
  // birb_position = 0;
  // Serial.printf("Servo attached to pin %d\n", servo_idx);
  // ISR_Servo.setPosition(servo_idx, birb_position);

  led_state = true;
  pinMode(LED_BUILTIN, OUTPUT);
}
//8b = 1000_1011

void loop() {
  Result ens160_result;

  //Read temp and RH, use for calibration of ENS160
  aht.getEvent(&hum_event, &temp_event);
  humidity_float = hum_event.relative_humidity;
  temp_float = temp_event.temperature;
  Serial.printf(">Temp:%2.2f\n", temp_float);
  Serial.printf(">Humidity:%2.2f\n", humidity_float);

  //Write compensation data
  temp_ens = (uint16_t) ((temp_float + 273.15) * 64);
  humidity_ens = (uint16_t) (humidity_float * 512);
  ens160.writeCompensation(temp_ens, humidity_ens);

  ens160.wait();
  ens160_result = ens160.update();
  if (ens160_result == RESULT_OK) {
    if (ens160.hasNewData()) {
      Serial.printf(">AQI-UBA:%d\n", (uint8_t) ens160.getAirQualityIndex_UBA());
      Serial.printf(">TVOC:%d\n", ens160.getTvoc());
      Serial.printf(">eCO2:%d\n", ens160.getEco2());
    }

    // if (ens160.hasNewGeneralPurposeData()) {
    //   Serial.printf("ENS160 GPR read");
    //   Serial.printf("RS0=%0x, RS1=%0x, RS2=%0x, RS3=%0x\n", ens160.getRs0(), ens160.getRs1(), ens160.getRs2(), ens160.getRs3());
    // }
  } else {
    Serial.printf("ENS160 was not RESULT_OK. Was %02x\n", (uint8_t) ens160_result);
  }

  // birb_position += 10;
  // if (birb_position > 180) {
  //   birb_position = 0;
  // }
  // ISR_Servo.setPosition(servo_idx, birb_position);

  //Debug outputs
  Serial.println("DEBUG OUTPUTS");
  print_ens160_compensation_registers();

  delay(10000);
  digitalWrite(LED_BUILTIN, led_state);
  led_state = !led_state;
}
