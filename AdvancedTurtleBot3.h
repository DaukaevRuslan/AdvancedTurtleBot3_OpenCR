#ifndef ADVANCED_TURTLEBOT3
#define ADVANCED_TURTLEBOT3

#include <DynamixelWorkbench.h>
#include "Adafruit_TCS34725.h"
#include "AdvancedTurtleBot3Config.h"
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include "MPU9250.h"
#include <math.h>

class AdvancedTurtleBot3 {

  public:
    
    void init(void);
    void setGoalVelocity(float xV, float wZ);
    uint32_t getLineSensorData(void);

    int32_t *realWheelVelocityINT = new int32_t[WHEEL_NUM];
    double *realWheelVelocityDBL = new double[WHEEL_NUM];
    int32_t *goalWheelVelocity = new int32_t[WHEEL_NUM];
    void calcRealWheelVel(void);
    int *getColorSensorData(void);

    uint32_t *getIRArrayData(void);
    
  private:

    DynamixelWorkbench dxlWB;
    Adafruit_TCS34725 tcs = Adafruit_TCS34725(); 
    uint8_t dxlID[WHEEL_NUM] = {DXL_ID_1, DXL_ID_2};
    uint32_t *IRArray = new uint32_t[7];
};

#endif