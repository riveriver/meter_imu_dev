#include <Arduino.h>
#include <MeterImu.h>

MeterImu imu;
TaskHandle_t *imu_handle;
TaskHandle_t *communicate_handle;
const int imu_task_time = 5;
const int com_task_time = 200;

void ImuTask(void *pvParameter);
void CommunicateTask(void *pvParameter);

void setup() {
    Serial.begin(115200);
    ESP_LOGE("[Debug]","Serial system begin");
    // pinMode(19,OUTPUT); 
    // digitalWrite(19,LOW); 
    xTaskCreatePinnedToCore(ImuTask, "Core_1_IMU", 16384, NULL, 3, imu_handle, 1);
    xTaskCreatePinnedToCore(CommunicateTask, "Core_1_COM", 8192, NULL, 2,communicate_handle, 1);
}

void loop() {}

void ImuTask(void *pvParameter) {

  imu.Init();

  BaseType_t xWasDelayed;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while(1){

    imu.Process();
    
    xWasDelayed = xTaskDelayUntil(&xLastWakeTime, imu_task_time);
    if (!xWasDelayed && millis() > 1000) {
      // ESP_LOGE("[Debug]","[Warning] ImuTask Time Out.");
      xLastWakeTime = xTaskGetTickCount();
    }
  }
}

void CommunicateTask(void *pvParameter) {

  Serial1.begin(921600, SERIAL_8N1, 0, 1);

  BaseType_t xWasDelayed;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1){
    
    if(imu.cali_cmd == 0){
      imu.sendMeasureInfo();
    }
    int start = millis();
    while (Serial1.available() && millis() - start < 1000) {
      imu.RxFromS3(Serial1.read());
    }

    xWasDelayed = xTaskDelayUntil(&xLastWakeTime, com_task_time);
    if (!xWasDelayed && millis() > 10000) {
      // ESP_LOGE("[Debug]","[Warning] CommunicateTask Time Out.");
      xLastWakeTime = xTaskGetTickCount();
    }
  }
}