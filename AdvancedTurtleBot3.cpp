#include "AdvancedTurtleBot3.h"

void AdvancedTurtleBot3::init(void) {
  
  tcs.begin();

  const char *log;
  bool result = false;
  uint16_t modelNumber = 0;

  result = dxlWB.init(DEVICE_NAME, BAUDRATE, &log);
  for (int i = 0; i < WHEEL_NUM; ++i) {
    result = dxlWB.ping(dxlID[i], &modelNumber, &log);
    Serial.print("Init for "); Serial.print(i); Serial.print(" dxl: "); Serial.println(result);

    result = dxlWB.wheelMode(dxlID[i], 0, &log);
    Serial.print("WheelMode for "); Serial.print(i); Serial.print(" dxl: "); Serial.println(result);
  }
}

void AdvancedTurtleBot3::setGoalVelocity(float vX, float wZ) {
 
  const char *log;
  bool result;

  goalWheelVelocity[0] = ((2 * vX - L * wZ) / (2 * R)) * 9.24 / 0.229;
  goalWheelVelocity[1] = ((2 * vX + L * wZ) / (2 * R)) * 9.24 / 0.229;

  for (int i = 0; i < WHEEL_NUM; ++i) {
    dxlWB.ping(dxlID[i]);
    result = dxlWB.goalVelocity(dxlID[i], goalWheelVelocity[i], &log);
    Serial.print("Init for "); Serial.print(i); Serial.print(" dxl: "); Serial.println(log);
  }
}

uint32_t AdvancedTurtleBot3::getLineSensorData(void) {
  uint32_t temp = 0;
  dxlWB.ping(LINE_SENSOR_ID);
  dxlWB.readRegister((uint8_t)10, (uint16_t)27,  (uint16_t)2, &temp);
  return temp;
}






                                        

void AdvancedTurtleBot3::calcRealWheelVel(void) {

  int32_t tempVel = 0;
  for (int i = 0; i < WHEEL_NUM; ++i) {

    uint32_t get_data = 0;
    dxlWB.readRegister(dxlID[i], (uint16_t)128, (uint16_t)4, &get_data);
    delay(1);
    realWheelVelocityDBL[i] = (double)(((int32_t)get_data) * 0.229f / 9.24f);

  }
}

int* AdvancedTurtleBot3::getColorSensorData(void) {
  uint16_t r, g, b, c, colorTemp, lux;

  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);

  int* arr = new int[6];

  arr[0] = colorTemp;
  arr[1] = lux;
  arr[2] = r;
  arr[3] = g;
  arr[4] = b;
  arr[5] = c;

  return arr;
}

uint32_t* AdvancedTurtleBot3::getIRArrayData(void) {
  uint32_t respf;
  int idx;
  bool result = true;
  for(int i = 0; i < 7;i++){
    dxlWB.ping((uint8_t)IR_ARRAY_SENSOR_ID);
    result = dxlWB.readRegister((uint8_t)IR_ARRAY_SENSOR_ID, (uint16_t)(24+2*i), (uint16_t)2, &respf);   
    if(result == false){
      Serial.println("LOL");
      IRArray[i] = 0;
    }
    else
      IRArray[i] = respf;
   delay(1);

  }
  return IRArray; 
}