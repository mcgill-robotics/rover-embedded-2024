#include <Arduino.h>
#include "BluetoothSerial.h"
#include "esp_gap_bt_api.h"

// INSTRUCTIONS
//  1. Set RADIO_MODE to 0 for TX, 1 for RX and upload to ESP32.
//  2. Power on slave (RX) ESP32.
//  3. Power on master (TX) ESP32.
//  4. Wait for master to connect to slave.
//  5. Press button on TX ESP32 to send "ON" to RX ESP32.

#define DEBUG_MODE 0

#define RADIO_MODE_TX 0
#define RADIO_MODE_RX 1

// COMPILATION FLAGS
#define RADIO_MODE RADIO_MODE_RX

// PIN DEFINITIONS
const int BUTTON_PIN = 2;
const int LED_PIN = 2;
const int TRANSISTOR_PIN = 14;
const int BUZZER=5;

// BLUETOOTH ------------------------------------------------------------------------------
// BT: Bluetooth availabilty check
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run make menuconfig to and enable it
#endif
// BT: Serial Bluetooth availabilty check
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial ESP_BT; // Object for Bluetooth
const char *server_name = "ESP32_RX";
const int maxRetries = 100000; // Maximum number of connection attempts
const int retryDelay = 500;    // Delay between retries in milliseconds

int microsBetweenReadings = 1000;
int lastReadTime = 0;

void tx_setup()
{
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), stopRover, RISING);
  Serial.begin(115200);

  // Initialize Bluetooth with name ESP32_TX, true sets it as master
  ESP_BT.begin("ESP32_TX", true);

  int attempts = 0;
  while (!ESP_BT.connect(server_name) && attempts < maxRetries)
  {
    attempts++;
    Serial.println("Attempting to connect to " + String(server_name) + ", attempt " + String(attempts));
    delay(retryDelay);
  }

  if (ESP_BT.connected())
  {
    Serial.println("Connected to " + String(server_name));
  }
  else
  {
    Serial.println("Failed to connect to " + String(server_name));
  }
}

void tx_loop()
{
  // Read the button state. If it's HIGH, send "ON" to the receiver. If it's LOW, send "OFF".
  int button_state = digitalRead(BUTTON_PIN);
  Serial.println("Button state: " + String(button_state));
  if (button_state == HIGH)
  {
    stopRover();
  }
  else
  {
    ESP_BT.println("OFF"); // Send "OFF" via Bluetooth
  }
  delay(10);
}
void IRAM_ATTR stopRover(){
    ESP_BT.println("ON"); // Send "ON" via Bluetooth
    tone(BUZZER, 1000);
    delay(500);
    noTone(BUZZER);
    delay(500);
}

void rx_setup()
{
  // Transistor gate connected to GPIO 14.
  pinMode(TRANSISTOR_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(TRANSISTOR_PIN, HIGH);

  Serial.begin(115200);     // Start Serial communication for debugging
  ESP_BT.begin("ESP32_RX"); // Start Bluetooth with name ESP32_RX
  Serial.println("Bluetooth Switch is Ready to Pair");
  lastReadTime = micros();
}

void rx_loop(bool kill)
{
  // LED DEBUG
  // if (micros() - lastReadTime >= microsBetweenReadings)
  // {
  //   lastReadTime = micros();
  //   digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  //   Serial.println("LED state: " + String(digitalRead(LED_PIN)));
  // }
    if(ESP_BT.connected()){
        if (ESP_BT.available())
        {                                            // Check if data is available to read
            String msg = ESP_BT.readStringUntil('\n'); // Read the incoming data
            msg.trim();                                // Remove any whitespace

            // Flip if necessary.
            if (msg == "ON")
            {
            digitalWrite(LED_PIN, HIGH); // Turn the LED on
            digitalWrite(TRANSISTOR_PIN, LOW);
            delay(250);
            tone(BUZZER, 85);
            delay(500);
            noTone(BUZZER);
            delay(250);
            Serial.println("SWITCH HIGH");
            kill=true;
            }
            else if (!kill){
                digitalWrite(LED_PIN, LOW);
                digitalWrite(TRANSISTOR_PIN, HIGH);
            }
            else{
                digitalWrite(LED_PIN, HIGH);
                digitalWrite(TRANSISTOR_PIN, LOW);
            }
        }
    }
    else{
        digitalWrite(TRANSISTOR_PIN, LOW);
    }
}

void setup()
{

bool kill = false;
#if DEBUG_MODE == 1
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
#else
  if (RADIO_MODE == 0)
  {
    tx_setup();
  }
  else
  {
    rx_setup();
  }
#endif
}

void loop()
{
#if DEBUG_MODE == 1
  digitalWrite(LED_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_PIN, LOW);
  delay(1000);
#else
  if (RADIO_MODE == 0)
  {
    tx_loop();
  }
  else
  {
    rx_loop(kill);
  }
#endif
}