#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <ScioSense_ENS16x.h>
#include "ESP8266_ISR_Servo.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "secrets.h"

#define SCL_PIN PIN_WIRE_SCL //5
#define SDA_PIN PIN_WIRE_SDA //4

#define ABS(x)  ((x) < 0 ? -(x) : (x))
#define SIGN(x) ((x) < 0 ? -1 : 1)

#define WIFI_CONNECT_NUM_ATTEMPTS 10
#define MQTT_CONNECT_NUM_ATTEMPTS 10
#define SAMPLE_PERIOD_MS 10000
#define AVERAGE_SAMPLE_COUNT 6
#define MQTT_LOOP_POLL_MS 100

/****AHT21 sensor****/
#define AHT21_I2C_ADDRESS 0x38
Adafruit_AHTX0 aht;
sensors_event_t hum_event, temp_event;
float humidity_float, temp_float;

/****ENS160 sensor****/
#define ENS160_I2C_ADDRESS 0x52
#define AQI_LIMIT          AQI_UBA_POOR //4 out of 5
ENS160 ens160;
//humidity and temp reformatted to format required for ENS160 calibration
uint16_t humidity_ens, temp_ens;
uint8_t aqi_uba;
uint16_t tvoc;
uint16_t eco2;

// Averaged sensor values (published to MQTT)
float avg_temp;
float avg_humidity;
uint16_t avg_eco2;
uint16_t avg_tvoc;
uint8_t avg_aqi;

// Aggregation state for averaging N samples before publishing.
float temp_sum;
float humidity_sum;
uint32_t eco2_sum;
uint32_t tvoc_sum;
float aqi_sum;
uint8_t measurement_sample_cnt;


/****Birb servo****/
#define SERVO_MIN_MICROS      544
#define SERVO_MAX_MICROS      2400
#define SERVO_PIN             2
#define BIRB_STEPS_PER_SECOND 20
#define BIRB_STEP_PERIOD_MS   (1000 / BIRB_STEPS_PER_SECOND)
int servo_idx;
int birb_position;
bool birb_is_dead;

/****MQTT****/
/* Required states / events:
- Publish temp (float)
- Publish hum (float)
- Publish eco2 (int)
- Publish tvoc (int)
- Publish AQI-UBA (int)
- Publish bird state (bool)

- Subscribe home assistant status (bool)
- Subscribe HA-generated birb status override (enum)
*/
#define MQTT_CLIENT_ID "Birb-Mqtt-Client"
#define MQTT_DISCOVERY_TOPIC "homeassistant/device/birb/config"
#define MQTT_STATE_TOPIC "homeassistant/device/birb/state"
#define MQTT_HA_STATUS_TOPIC "homeassistant/status"
#define MQTT_STATE_OVERRIDE_TOPIC "homeassistant/device/birb/state_override"
WiFiClient espClient;
PubSubClient mqtt_client(espClient);
bool should_send_discovery_message;
bool should_send_mqtt_updates;
const char* MQTT_HA_DISCOVERY_PAYLOAD = "{\"dev\":{\"identifiers\":\"birb_0001\",\"name\":\"Birb\"},\"cmps\":{\"birb_0001_state\":{\"p\":\"binary_sensor\",\"name\":\"State\",\"device_class\":\"problem\",\"payload_on\":\"ON\",\"payload_off\":\"OFF\",\"value_template\":\" {{ value_json.state }} \",\"unique_id\":\"birb_0001_state\"},\"birb_0001_t\":{\"p\":\"sensor\",\"name\":\"Temperature\",\"device_class\":\"temperature\",\"unit_of_measurement\":\"°C\",\"value_template\":\" {{ value_json.temp }} \",\"unique_id\":\"birb_0001_t\"},\"birb_0001_h\":{\"p\":\"sensor\",\"name\":\"Humidity\",\"device_class\":\"humidity\",\"unit_of_measurement\":\"%\",\"value_template\":\" {{ value_json.hum }} \",\"unique_id\":\"birb_0001_h\"},\"birb_0001_eco2\":{\"p\":\"sensor\",\"name\":\"eCO2\",\"device_class\":\"carbon_dioxide\",\"unit_of_measurement\":\"ppm\",\"value_template\":\" {{ value_json.eco2 }} \",\"unique_id\":\"birb_0001_eco2\"},\"birb_0001_tvoc\":{\"p\":\"sensor\",\"name\":\"TVOC\",\"device_class\":\"volatile_organic_compounds_parts\",\"unit_of_measurement\":\"ppb\",\"value_template\":\" {{ value_json.tvoc }} \",\"unique_id\":\"birb_0001_tvoc\"},\"birb_0001_aqi\":{\"p\":\"sensor\",\"name\":\"AQI\",\"device_class\":\"aqi\",\"value_template\":\" {{ value_json.aqi }} \",\"unique_id\":\"birb_0001_aqi\"},\"birb_0001_override\":{\"p\":\"select\",\"name\":\"Override\",\"command_topic\":\"homeassistant/device/birb/state_override\",\"options\":[\"OFF\",\"ALIVE\",\"DEAD\"],\"qos\": 1,\"unique_id\":\"birb_0001_override\"}},\"o\":{\"name\":\"KasperHesse/Birb\",\"sw\":\"1.0\",\"url\":\"https://github.com/KasperHesse/Birb\"},\"qos\":0,\"state_topic\":\"homeassistant/device/birb/state\"}";

typedef enum {
  OFF, DEAD, ALIVE
} birb_state_override_t;
birb_state_override_t birb_state_override;

/*
General flow:
- On power up: Attempt to connect to MQTT (try_connect_mqtt_broker == true)
  - If not connected after 5 attempts
    - Set timer for 5 minutes, try again, exit early
    - Set should_send_discovery_message false
    - Set should_send_mqtt_updates false
  - If connected:
    - Set should_send_discovery_message == true
     -Set should_send_mqtt_updates true
     -Set try_connect_mqtt_broker false
- Every N seconds (TBD)
  - Send discovery if not already sent, reset flag (include random delay)
  - Make measurements, store in relevant variables
  - Publish measurements over MQTT
  - Change birb position if required (true == dead, false == live)
  - Delay

- Subscribe to topics
  - HA discovery topic
    - If message: Enable/disable flags for sending MQTT updates and sending discovery message
  - HA birb state override (topic TBD)
    - If message: Set override value

- Interrupts
  - N minute timer if mqtt not connected
    - Set try_connect_mqtt_broker true


TO BE CONSIDERED: How are mqtt messages handled while in deep sleep??
*/

bool wifi_is_connected() {
  return WiFi.status() == WL_CONNECTED;
}

bool mqtt_is_connected() {
  return mqtt_client.connected();
}

void start_wifi_mqtt_reconnect_timer() {
  //TODO: Implement using the Ticker library (add `Ticker` to lib_deps and `#include <Ticker.h>`):
  //  static Ticker reconnect_timer;
  //  reconnect_timer.once_ms(300000, []() { ensure_mqtt_connection(); });
  //  For increasing backoff, double the delay on each failed attempt up to a max (e.g. 5 min).
}

/** Enable WiFi connection.
 *
 * Performs WIFI_CONNECT_NUM_ATTEMPTS attempts with a 1-second delay to establish connection
 */
void setup_wifi() {
  int num_attempts;
  delay(10);
  Serial.printf("Connecting to wifi %s\n", WIFI_SSID_NAME);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID_NAME, WIFI_PASSWORD);

  //TODO: Fallback to run without WiFi if not connected
  for(num_attempts=0; num_attempts<WIFI_CONNECT_NUM_ATTEMPTS; num_attempts++) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("WiFi connected");
      Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());
      break;
    }
    delay(1000);
    Serial.print(".");
  }
  if (!wifi_is_connected()) {
    Serial.printf("WiFi was NOT connected. Status=%d\n", WiFi.status());
    start_wifi_mqtt_reconnect_timer();
    should_send_discovery_message = false;
    should_send_mqtt_updates = false;
  }
  randomSeed(micros()); //Unsure of reason, in MQTT example
}

/**Establish MQTT connection, creating initial broadcast message of all sensors
 *
 * Performs MQTT_CONNECT_NUM_ATTEMPTS attempts with a 1-second delay to establish connection
 */
void mqtt_connect() {
  uint8_t num_attempts;

  for(num_attempts=0; num_attempts<MQTT_CONNECT_NUM_ATTEMPTS; num_attempts++) {
    Serial.print("Attempting MQTT connection, attempt ");
    Serial.println(num_attempts);

    if (mqtt_client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD, 0, 0, 0, 0, 0)) {
      Serial.println("MQTT connection established. Subscribing to topics");
      mqtt_client.subscribe(MQTT_HA_STATUS_TOPIC, 1);
      mqtt_client.subscribe(MQTT_STATE_OVERRIDE_TOPIC, 1);
      should_send_mqtt_updates = true;
      break;
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" trying again in 3 seconds");
      delay(3000);
    }
  }
}

/** Ensure connection to the MQTT broker. Performs setup of WiFi and MQTT client if they are not connected */
bool ensure_mqtt_connection() {
  if (!mqtt_is_connected()) {
    should_send_mqtt_updates = false;
    if (!wifi_is_connected()) {
      setup_wifi();
    }
    if (wifi_is_connected()) {
      mqtt_connect();
      //mqtt_connect sets flags high if connection works
    }
  }

  return mqtt_is_connected();
  //TODO: In all cases where WiFi / mqtt fail, use increasing timeouts (30sec, 1min, 3min, 5min)
}

/** MQTT callback function for handling messages received on subscribed topics */
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.printf("MQTT message received on [%s]\n", topic);
  Serial.print("  '");
  for(unsigned int i=0; i<length; i++) {
    Serial.print( (char) payload[i]);
  }
  Serial.println("'");

  //Handle writes to birb override topic
  if (strcmp(topic, MQTT_HA_STATUS_TOPIC) == 0) {
    Serial.println("Received HA status message");
    if (strncmp((char*) payload, "online", 6) == 0) {
      Serial.println("Received 'online' message, sending discovery message next time around");
      should_send_discovery_message = true;
      should_send_mqtt_updates = false;
    } else if (strncmp((char*) payload, "offline", 7) == 0) {
      Serial.println("Received 'offline' message, disabling mqtt updates");
      should_send_mqtt_updates = false;
    } else {
      Serial.println("Unknown payload for HA status topic");
    }
    return;
  }

  //Handle writes to birb override topic
  if (strcmp(topic, MQTT_STATE_OVERRIDE_TOPIC) == 0) {
    Serial.println("Received bird override message");
    if (strncmp((char*) payload, "OFF", 3) == 0) {
      Serial.println("Disabling birb state override");
      birb_state_override = OFF;
      birb_is_dead = (avg_aqi >= AQI_LIMIT);
    } else if (strncmp((char*) payload, "ALIVE", 5) == 0) {
      Serial.println("Overriding birb state to alive");
      birb_state_override = ALIVE;
      birb_is_dead = false;
    } else if (strncmp((char*) payload, "DEAD", 4) == 0) {
      Serial.println("Overriding birb state to dead");
      birb_state_override = DEAD;
      birb_is_dead = true;
    } else {
      Serial.println("Did not identify command for setting birb state override");
    }
    return;
  }
}

/** Read attached sensors (AHT21 and ENS160), storing values in global variables */
void read_sensors() {
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

  //Read ENS160
  ens160.wait();
  ens160_result = ens160.update();
  if (ens160_result == RESULT_OK) {
    if (ens160.hasNewData()) {
      aqi_uba = (uint8_t) ens160.getAirQualityIndex_UBA();
      eco2 = ens160.getEco2();
      tvoc = ens160.getTvoc();
      Serial.printf(">AQI-UBA:%d\n", aqi_uba);
      Serial.printf(">TVOC:%d\n", tvoc);
      Serial.printf(">eCO2:%d\n", eco2);
    }
  } else {
    Serial.printf("ENS160 was not RESULT_OK. Was %02x\n", (uint8_t) ens160_result);
  }

}

void reset_accumulators() {
  temp_sum = 0.0f;
  humidity_sum = 0.0f;
  eco2_sum = 0;
  tvoc_sum = 0;
  aqi_sum = 0;
  measurement_sample_cnt = 0;
}

void accumulate_measurements() {
  temp_sum += temp_float;
  humidity_sum += humidity_float;
  eco2_sum += eco2;
  tvoc_sum += tvoc;
  aqi_sum += aqi_uba;
  measurement_sample_cnt++;
}

void finalize_averaged_measurements() {
  avg_temp = temp_sum / measurement_sample_cnt;
  avg_humidity = humidity_sum / measurement_sample_cnt;
  avg_eco2 = (uint16_t) (eco2_sum / measurement_sample_cnt);
  avg_tvoc = (uint16_t) (tvoc_sum / measurement_sample_cnt);
  avg_aqi = uint8_t ((aqi_sum / measurement_sample_cnt) + 0.5f); // round to nearest whole AQI bucket

  Serial.printf(">Averaged over %d samples: temp=%2.1f hum=%2.1f eco2=%d tvoc=%d aqi=%d\n",
      measurement_sample_cnt, avg_temp, avg_humidity, avg_eco2, avg_tvoc, avg_aqi);

  //Set birb state if not forced
  if (birb_state_override == OFF) {
    birb_is_dead = (avg_aqi >= AQI_LIMIT);
  }
  Serial.printf(">Birb state:%d\n", birb_is_dead);
}

/** Publish sensor data via MQTT to the specified topic */
void publish_sensor_data() {
  char payload_buffer[100];
  const char* state_str = birb_is_dead ? "ON" : "OFF";
  snprintf(payload_buffer, 100, "{\"state\": \"%3s\",\"temp\": %2.1f,\"hum\": %2.1f,\"eco2\": %d,\"tvoc\": %d,\"aqi\": %d}", state_str, avg_temp, avg_humidity, avg_eco2, avg_tvoc, avg_aqi);
  Serial.printf("Publishing payload '%s'\n", payload_buffer);
  if (!mqtt_client.publish(MQTT_STATE_TOPIC, payload_buffer)) {
    Serial.println("Failed to publish MQTT payload!");
    // MQTT disconnection will be
    // detected by ensure_mqtt_connection() at the start of the next loop iteration.
  }
}

/** Wait for some time before performing the next sample */
void delay_for_next_sample() {
  //TODO: Deep sleep to save energy: replace delay() with ESP.deepSleep(SAMPLE_PERIOD_MS * 1000UL).
  //  Caveats:
  //  - Deep sleep causes a full reset on wake; WiFi and MQTT must reconnect every cycle.
  //  - Persistent state (temp_sum/humidity_sum/eco2_sum/tvoc_sum/aqi_sum,
  //    measurement_sample_cnt, aqi_latest, birb_position) must be
  //    stored in RTC memory: prefix globals with RTC_DATA_ATTR.
  //  - MQTT subscriptions (state_override) won't persist; re-subscribe on each wake.
  //  - Connect D0 (GPIO16) to RST pin to enable wake-from-deep-sleep on ESP8266.
  uint32_t start_ms;
  uint32_t elapsed_ms;

  Serial.printf("Delaying %d ms until next sample ...\n\n", SAMPLE_PERIOD_MS);
  // ESP.deepSleep(SAMPLE_PERIOD_MS * 1000UL);

  start_ms = millis();
  do {
    mqtt_client.loop();
    delay(MQTT_LOOP_POLL_MS);
    yield();
    elapsed_ms = millis() - start_ms;
  } while (elapsed_ms < SAMPLE_PERIOD_MS);
}

/** Sets the position of the birb servo, in interval [0;180] degrees */
void set_birb_position(int target_pos, uint8_t degs_per_second = 30) {
  float degs_per_step;
  int delta_degs;
  int num_steps;
  float float_pos;

  //Ensure variables within legal ranges
  if (degs_per_second == 0) {
    degs_per_second = 20;
  }
  if (target_pos > 180) {
    target_pos = 180;
  } else if (target_pos < 0) {
    target_pos = 0;
  }

  degs_per_step = degs_per_second / ((float) BIRB_STEPS_PER_SECOND);
  delta_degs = target_pos - birb_position;
  if (delta_degs == 0) {
    Serial.printf("Bird target position == current position (%d). Not moving\n", birb_position);
    return;
  }
  num_steps = ABS(delta_degs) / degs_per_step;
  degs_per_step = SIGN(delta_degs) * degs_per_step;

  Serial.printf("Moving bird from start=%d to target=%d. num_steps=%d, degs_per_step=%2.2f\n", birb_position, target_pos, num_steps, degs_per_step);
  float_pos = birb_position;
  for(int step=0; step<num_steps; step++) {
    float_pos += degs_per_step;
    ISR_Servo.setPosition(servo_idx, (uint16_t) float_pos);
    delay(BIRB_STEP_PERIOD_MS);
    mqtt_client.loop(); // keep MQTT alive during long servo moves
  }

  //Ensure rounding errors don't make the bird drift
  ISR_Servo.setPosition(servo_idx, target_pos);
  birb_position = target_pos;
  Serial.printf(">Position:%d\n", birb_position);
}

void set_birb_position_fast(int target_pos) {
  if (birb_position != target_pos) {
    Serial.printf("Moving bird from start=%d to target=%d\n", birb_position, target_pos);
    ISR_Servo.setPosition(servo_idx, target_pos);
    birb_position = target_pos;
    Serial.printf(">Position:%d\n", birb_position);
  }
}


void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  should_send_discovery_message = true;
  should_send_mqtt_updates = true;
  //We default to a live bird
  birb_is_dead = false;
  birb_state_override = OFF;
  avg_aqi = 0;

  //Setup AHT sensor on I2C bus
  Serial.println("Starting AHT21");
  aht.begin(&Wire);

  //Setup ENS160 sensor on I2C bus
  //TODO: Replace the blocking while() with a timeout loop (e.g. 5 attempts).
  //  On failure: set global bool ens160_available = false; guard all ENS160 reads in
  //  read_sensors() with `if (ens160_available)`. Publish an MQTT availability message
  //  (e.g. a dedicated topic or MQTT LWT) so HA marks the sensors as unavailable.
  Serial.println("Starting ENS160");
  ens160.enableDebugging(Serial);
  ens160.begin(&Wire);
  while (!ens160.init()) {
    Serial.print(".");
    delay(1000);
  }
  ens160.startStandardMeasure(); //Set to STANDARD mode for performing operations
  Serial.println("\nENS160 online");

  reset_accumulators();

  //Setup servo motor for bird
  servo_idx = ISR_Servo.setupServo(SERVO_PIN, SERVO_MIN_MICROS, SERVO_MAX_MICROS);
  birb_position = 0;
  Serial.printf("Servo attached to pin %d (servo index %d)\n", SERVO_PIN, servo_idx);
  ISR_Servo.setPosition(servo_idx, birb_position);

  mqtt_client.setServer(MQTT_BROKER_ADDRESS, MQTT_BROKER_PORT);
  mqtt_client.setCallback(mqtt_callback);
  mqtt_client.setBufferSize(2048); //Large buffer required for discovery message
}

void birb_loop() {
  //Send discovery payload if first time OR re-prompted for discovery (e.g. after HA reboot)
  //Make this code more robust: If wifi/mqtt is not connected, attempt to reconnect.
  //Set timer to retry connection if it didn't work
  ensure_mqtt_connection();
  mqtt_client.loop();

  if (should_send_discovery_message) {
    bool res = mqtt_client.publish(MQTT_DISCOVERY_TOPIC, MQTT_HA_DISCOVERY_PAYLOAD);
    if (!res) {
      Serial.println("Failed to publish discovery payload ... Trying again next time");
      should_send_discovery_message = true;
      should_send_mqtt_updates = false;
    } else {
      Serial.println("Published discovery payload");
      should_send_discovery_message = false;
      should_send_mqtt_updates = true;
    }
  }

  //Take one measurement every SAMPLE_PERIOD_MS and aggregate over AVERAGE_SAMPLE_COUNT samples.
  read_sensors();
  accumulate_measurements();

  //Publish averaged data only after the full sample window is complete.
  if (measurement_sample_cnt == AVERAGE_SAMPLE_COUNT) {
    finalize_averaged_measurements();
    if (should_send_mqtt_updates) {
      publish_sensor_data();
    }
    reset_accumulators();
  }

  //Change birb position using averaged AQI / override state.
  if (birb_is_dead) {
    set_birb_position_fast(0);
  } else {
    set_birb_position_fast(180);
  }

  //Wait until later
  delay_for_next_sample();
}

void loop() {
  birb_loop();
  //TODO-list
  /*
  - Deep sleep: See delay_for_next_sample() comment for full guidance.
  - Increasing reconnect timeout: Add a static backoff counter in ensure_mqtt_connection():
      static uint32_t retry_delay_ms = 30000;
      On each failure: retry_delay_ms = min(retry_delay_ms * 2, 300000UL);
      On success: retry_delay_ms = 30000;
      Call start_wifi_mqtt_reconnect_timer() with the current retry_delay_ms value.
  - HA discovery re-send on HA reboot: Handled via MQTT_HA_STATUS_TOPIC "online" callback.
      Test by restarting HA and confirming discovery is re-published and entities reappear.
  - ENS160 non-blocking init: See setup() comment for full guidance.
  */

}
