#include "AdvancedTurtleBot3.h"
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include "MPU9250.h"

float checkVoltage() {
  int adc_value = analogRead(BDPIN_BAT_PWR_ADC);  
  float vol_value = map(adc_value, 0, 1023, 0, 330*57/10);
  vol_value = vol_value / 100;
  return vol_value;
}

AdvancedTurtleBot3 bot;
double x = 0;
double z = 0;

long timerOdom = 0;
long timerSensors = 0;
long timerVoltage = 0;

void messageCb(const geometry_msgs::Twist &twist) {
  x = twist.linear.x;
  z = twist.angular.z;

  bot.setGoalVelocity(x, z);
}

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> subCMD("cmd_vel", messageCb);

geometry_msgs::Vector3 msgTwist;
ros::Publisher pubVelocity("velocity_data", &msgTwist);

std_msgs::Float32 voltageMsg;
ros::Publisher pubVoltage("voltage", &voltageMsg);

#ifdef LINE_SENSOR_ON
std_msgs::Bool msgLine;
ros::Publisher pubLine("line_sensor_data", &msgLine);
#endif

#ifdef IR_ARRAY_SENSOR_ON
std_msgs::Int32MultiArray msgIRArray;
ros::Publisher pubIRArray("IR_array_sensor_data", &msgIRArray);
#endif

#ifdef COLOR_SENSOR_ON
std_msgs::Int32MultiArray msgColor;
ros::Publisher pubColor("color_data", &msgColor);
#endif

#ifdef IMU_SENSOR_ON
std_msgs::Float64MultiArray msgIMU;
ros::Publisher pubIMU("IMU_data", &msgIMU);
MPU9250 IMU(Wire, IMU_SENSOR_ID);
#endif

void getDataIMU(void);
void getDataColor(void);
void getIRArrayData(void);

void setup() {
  bot.init();
  
#ifdef IMU_SENSOR_ON
  IMU.begin();
  nh.advertise(pubIMU);
  msgIMU.data = (float*)malloc(sizeof(float) * 10);
  msgIMU.data_length = 10;
#endif

  nh.initNode();
  nh.getHardware()->setBaud(BAUDRATE);

  nh.subscribe(subCMD);
  
  nh.advertise(pubVelocity);
  nh.advertise(pubVoltage);

#ifdef IR_ARRAY_SENSOR_ON
  nh.advertise(pubIRArray);
  msgIRArray.data = (int32_t*)malloc(sizeof(int32_t) * 7);
  msgIRArray.data_length = 7;
#endif

#ifdef LINE_SENSOR_ON
  nh.advertise(pubLine);
#endif

#ifdef COLOR_SENSOR_ON
  nh.advertise(pubColor);
  msgColor.data = (long*)malloc(sizeof(int) * 6);
  msgColor.data_length = 6;
#endif
}

void loop() {

  if (millis() - timerOdom > 100) {
    timerOdom = millis();
    bot.calcRealWheelVel();
    msgTwist.x = bot.realWheelVelocityDBL[0];
    msgTwist.y = bot.realWheelVelocityDBL[1];
    pubVelocity.publish(&msgTwist);  
  }

  if (millis() - timerSensors > 100) {
    timerSensors = millis();
    #ifdef COLOR_SENSOR_ON
      getDataColor();
      pubColor.publish(&msgColor);
    #endif
    
    #ifdef IMU_SENSOR_ON
      getDataIMU();
      pubIMU.publish(&msgIMU);
    #endif
    
    #ifdef LINE_SENSOR_ON
      msgLine.data = bot.getLineSensorData();
      pubLine.publish(&msgLine);
    #endif
    
    #ifdef IR_ARRAY_SENSOR_ON
      getIRArrayData();
      pubIRArray.publish(&msgIRArray);
    #endif
  }
  
  if (millis() - timerVoltage > 1000) {
    timerVoltage = millis();
    voltageMsg.data = checkVoltage();
    pubVoltage.publish(&voltageMsg);
  }

  nh.spinOnce();
}

#ifdef IMU_SENSOR_ON
void getDataIMU(void) {
  IMU.readSensor();
  msgIMU.data[0] = IMU.getAccelX_mss();
  msgIMU.data[1] = IMU.getAccelY_mss();
  msgIMU.data[2] = IMU.getAccelZ_mss();
  msgIMU.data[3] = IMU.getGyroX_rads();
  msgIMU.data[4] = IMU.getGyroY_rads();
  msgIMU.data[5] = IMU.getGyroZ_rads();
  msgIMU.data[6] = IMU.getMagX_uT();
  msgIMU.data[7] = IMU.getMagY_uT();
  msgIMU.data[8] = IMU.getMagZ_uT();
  msgIMU.data[9] = IMU.getTemperature_C();
}
#endif

#ifdef COLOR_SENSOR_ON
void getDataColor(void) {
  msgColor.data[0] = bot.getColorSensorData()[0];
  msgColor.data[1] = bot.getColorSensorData()[1];
  msgColor.data[2] = bot.getColorSensorData()[2];
  msgColor.data[3] = bot.getColorSensorData()[3];
  msgColor.data[4] = bot.getColorSensorData()[4];
  msgColor.data[5] = bot.getColorSensorData()[5];
}
#endif

#ifdef IR_ARRAY_SENSOR_ON
void getIRArrayData(void) {
  msgIRArray.data[0] = bot.getIRArrayData()[0];
  msgIRArray.data[1] = bot.getIRArrayData()[1];
  msgIRArray.data[2] = bot.getIRArrayData()[2];
  msgIRArray.data[3] = bot.getIRArrayData()[3];
  msgIRArray.data[4] = bot.getIRArrayData()[4];
  msgIRArray.data[5] = bot.getIRArrayData()[5];
  msgIRArray.data[6] = bot.getIRArrayData()[6];
}
#endif