#include "gps.h"
#include "TinyGPSPlus.h"
#include "Quaternion.h"
#include "XimuReceiver.h"

TinyGPSPlus gps;
XimuReceiver imu;
float quaternion[6];

float coords[2];

void setup(){
  Serial.begin(9600);
  Serial2.begin(115200);
  //Serial2.print("hi");
}

QuaternionStruct quatStruct;
Quaternion quat;
EulerAnglesStruct eulerStruct;

void loop(){
  coords[0] = gps.location.lat();
  coords[1] = gps.location.lng();

//   while (Serial1.available()) gps.encode(Serial1.read());
//   while (Serial2.available()) imu.processNewChar(Serial2.read());
    Serial.print(coords[0]);
    Serial.print(" ");
    Serial.println(coords[1]);
    delay(1000);
//   if(imu.isQuaternionGetReady()) {
//     quatStruct = imu.getQuaternion();
//     quat = Quaternion(quatStruct.w, quatStruct.x, quatStruct.y, quatStruct.z);
//     eulerStruct = quat.getEulerAngles();
//     quaternion[0] = quatStruct.x;
//     quaternion[1] = quatStruct.y;
//     quaternion[2] = quatStruct.z;
//     quaternion[3] = eulerStruct.roll;
//     quaternion[4] = eulerStruct.pitch;
//     quaternion[5] = eulerStruct.yaw;
//   }
}