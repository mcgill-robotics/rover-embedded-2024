#define DEBUG_MODE 0

#define RADIO_MODE_TX 0
#define RADIO_MODE_RX 1

// COMPILATION FLAGS
#define RADIO_MODE RADIO_MODE_RX // toggle between rx and tx for transistor or reciever code

// PIN DEFINITIONS
const int BUTTON_PIN = 0;
const int LED_PIN = 2;
const int TRANSISTOR_PIN = 14;
const int BUZZER = 5;