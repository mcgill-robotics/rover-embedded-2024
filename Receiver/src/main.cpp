#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <iostream>
#include <array>

#define DEBUG_PRINT 1
String data;

// NRF
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";
uint32_t lastTime = 0;

//DC motors
const int pwm0 = 2;   // pwm: set to 0-255 for speed(255 is fastest)
const int dir0 = 3;   // dir: set to high/low for direction
const int pwm1 = 4; 
const int dir1 = 5;  
int dc_speed0 = 200;  //sets the speed of DC motors
int dc_speed1 = 100;

//stepper motor
#define dirPin 9                //dirPin: set high or low for CW/CCW
#define stepPin 10              //stepPin: write HIGH then LOW for 1 step
#define stepsPerRevolution 200  //number of steps per a full revolution
int revolutions = 0.25; //controls revolutions: revolutions is number of revolutions(0.25 for 90 degrees)
int delay_sec = 1000;   //controls speed: lower delay time is faster rotation speed  

//geiger
#define LOG_PERIOD 15000 // count rate
unsigned long counts; //variable for GM Tube events
unsigned long previousMillis; //variable for measuring time
unsigned long geiger_count;
void impulse() {counts++;} //counter for geiger

void setup() { 
  Serial.begin(9600);

  //geiger
  counts = 0;
  Serial.begin(9600);
  pinMode(6, INPUT);
  attachInterrupt(digitalPinToInterrupt(6), impulse, FALLING); //define external interrupts
  Serial.println("Start counter");

  //nrf
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_HIGH);
  radio.startListening();
  lastTime = millis();

  //stepper
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, HIGH); //HIGH for cw, LOW for ccw

  //dc
  pinMode(pwm0,OUTPUT); 
  pinMode(dir0,OUTPUT); 
  pinMode(pwm1,OUTPUT); 
  pinMode(dir1,OUTPUT); 
}

unsigned long geiger_loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis > LOG_PERIOD) {
    previousMillis = currentMillis;
    unsigned long result = counts;
    counts = 0;
    return result;
  }
  return 0; // Or any other suitable default value
}

//control DC 0
void dc_forward0() {  // rotate CCW: set dir0 to HIGH
  digitalWrite(dir0, HIGH);
  analogWrite(pwm0, dc_speed0); 
}
void dc_reverse0() {  // rotate CW: set dir0 to LOW
  digitalWrite(dir0, LOW);
  analogWrite(pwm0, dc_speed0); 
}
void dc_brake0() {  // brake: set pwm0 to 0
  analogWrite(pwm0, 0); 
}

//control DC 1
void dc_forward1() {  
  digitalWrite(dir1, HIGH);
  analogWrite(pwm1, dc_speed1);
}
void dc_reverse1() {  
  digitalWrite(dir1, LOW);
  analogWrite(pwm1, dc_speed1);
}
void dc_brake1() {  
  analogWrite(pwm1, 0);
}


//rotates the stepper for "revolutions" amount of revolutions
void step_rotate() {   
  for (int i = 0; i < revolutions * stepsPerRevolution; i++) {  //rotate Stepper for given speed/revolutions
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delay_sec);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delay_sec);
  }
}


void loop()
{
#if DEBUG_PRINT == 1
  if (millis() - lastTime > 1000)
  {
    Serial.printf("RX Loop %lu\r\n", millis()); // Use %lu for millis() as it returns unsigned long
    lastTime = millis();
  }
#endif
  bool rx_flag = radio.available();
  if (rx_flag)
  {
    char rx_buffer[256];
    uint8_t len = radio.getDynamicPayloadSize(); // Get dynamic payload size
    if (len > 255)
      len = 255; // Ensure len does not exceed buffer size - 1 (for null terminator)
    radio.read(rx_buffer, len);
    rx_buffer[len] = '\0'; // Null-terminate the received string

    geiger_count = geiger_loop();   
    data = geiger_count + rx_buffer;            //"data" is 1 line of CSV data: geiger(field 1), moisture(field 2-5), and pH data(field 6-9) in CSV format
    Serial.printf("time: %lu\r\n", millis());
  }
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "science"); //node name is "science"
	ros::NodeHandle n;
	ros::Publisher science_pub = n.advertise<std_msgs::String>("science_data", 10); //publish to topic "science_data"
	ros::Rate loop_rate(1); //frequency to publish at

	int count = 0;
	while (ros::ok()) {	//keep looping until CTRL+c
		std_msgs::String msg;
		std::stringstream ss;
		ss << "200,90,90,90,90,400.50,400.50,400.50,400.50\n" << count; //append 1 line of CSV data and count
		msg.data = ss.str();
		ROS_INFO("%s", msg.data.c_str()); //ros_info prints the string, timestamp, and node name
		science_pub.publish(msg); //publisher object science_pub publishes msg to chatter topic
		ros::spinOnce();
		loop_rate.sleep();
		++count; 
	}
	return 0;
}

