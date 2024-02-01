/*
version 500
UI_Threshold 0.05f
win_size 200
sample_freq 200.0f
ODR_200HZ ==> 根据线程实际运行时间设定。经实际测试，ImuTask任何执行间隔不得低于3ms;
setODRAndFSR(GYRO, ODR_200HZ, FSR_7);
setODRAndFSR(ACCEL, ODR_200HZ, FSR_3);
GYRO ==> FSR_7：+-15.627°
ACC  ==> FSR_3: +-2g 
*/

#include <Arduino.h>
#include <Preferences.h>
#include <ArduinoEigenDense.h>
#include <WC_ICM42688.h>
#include <riverMahony.h>
#include <SlidingWindowFilter.h>

class MeterImu : WC_ICM42688_SPI{
  public:
    const int version = 605;
    byte  cali_cmd = 0;
    int   sampling_num     = 10000;
    float cali_threshold   = 20.0f;
    MeterImu() : WC_ICM42688_SPI(6,&SPI){};
    byte Init();
    void Process();
    void sendMeasureInfo();
    void sendCaliInfo(String info_str);
    void RxFromS3(unsigned char rc); 
    void parseCaliInfo(int info);
  private:
    bool  DEBUG_PRINT = true;
    unsigned long print_timer = 0;
    const int print_interval = 1000;
    String print_str = "";
    String cali_str = "";
    float UI_Threshold = 0.05f;
    bool  warm_onoff = false;
    const byte SPI_SCK  = 7 ;
    const byte SPI_MISO = 10;
    const byte SPI_MOSI = 3 ;
    const byte SPI_CS   = 6 ;
    Eigen::Vector3f bias;
    // TODO scale理论上可以改为Eigen::Vector3f；
    Eigen::Matrix3f scale;
    Eigen::Vector3f accm_raw;
    Eigen::Vector3f gyro_raw;
    Eigen::Vector3f accm_filter;
    Eigen::Vector3f accm_cali;
    Eigen::Vector3f gyro_filter;
    Eigen::Vector3f euler_angle;
    Eigen::Vector3f euler_angle_rad;
    SlidingWindowFilter<Eigen::Vector3f> accm_raw_window_filter;
    SlidingWindowFilter<Eigen::Vector3f> gyro_raw_window_filter;
    // Madgwick algorithm;
    // Mahony algorithm;
    byte  closest_gravity  = 3;
    float temperature  = 0.0f;
    byte  cali_status = 0;
    int   cali_info = 0;
    byte  cali_flag[7] = {0};
    float cali_data[6][3] = {0};
    float UIX[2];
    float UIY[2];
    float UIZ[2];
    Preferences pref;
    void Update();
    void ReadICM42688();
    void CollectCaliData();
    void DoAccelCalibrationFull();
    void SaveBiasScale();
    void GetBiasScale();
    float get_roll_ui();
    float get_pitch_ui();
    float get_yaw_ui();
    void ComputeEuler(Eigen::Vector3f acc);
    void DebugBiasScale();
};

byte MeterImu::Init(){
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);
  byte status;
  while ((status = WC_ICM42688_SPI::begin()) != 0) {
    if (status == -1) {
      ESP_LOGE("MeterImu::Init","bus data access error");
    } else{
      ESP_LOGE("MeterImu::Init","Chip versions do not match");
    }
    delay(1000);
  }
  ESP_LOGE("MeterImu::Init","ICM42688 begin success!!!");
  /* 设置传感器的输出数据率ODR和满量程范围FSR 
    GYRO ==> FSR_7：+-15.627°
    ACC  ==> FSR_3: +-2g 
    ODR_200HZ ==> 根据线程实际运行时间设定。经实际测试，ImuTask任何执行间隔不得低于3ms;
    */
  setODRAndFSR(GYRO, ODR_200HZ, FSR_7);
  setODRAndFSR(ACCEL, ODR_200HZ, FSR_3);
  startTempMeasure();// 启动温度测量
  startGyroMeasure(LN_MODE);// 启动陀螺仪测量:使用低噪声模式进行测量
  startAccelMeasure(LN_MODE);// 启动加速度计测量:使用低噪声模式进行测量
  // algorithm.begin(200.0f);//该值越小测量速度越快，该值越大测量值越稳定
  GetBiasScale(); // 获取标定参数
  delay(1000);  // Wait for sensor to stabilize
  print_timer = millis();
  return 0;
} 

void MeterImu::Process() {
switch(cali_cmd){
case 0:
  Update();
  if(DEBUG_PRINT && millis() - print_timer > print_interval){
    print_str += "a_r(";
    print_str += String(accm_raw(0));
    print_str += ", ";
    print_str += String(accm_raw(1));
    print_str += ", ";
    print_str += String(accm_raw(2));
    print_str += ") ";
    print_str += "a_f(";
    print_str += String(accm_filter(0));
    print_str += ", ";
    print_str += String(accm_filter(1));
    print_str += ", ";
    print_str += String(accm_filter(2));
    print_str += ") ";
    print_str += "e_r(";
    print_str += String(euler_angle(0));
    print_str += ", ";
    print_str += String(euler_angle(1));
    print_str += ", ";
    print_str += String(euler_angle(2));
    print_str += ") ";
    print_str += String(cali_cmd);
    Serial.println(print_str);
    print_str = "";
    print_timer = millis();
  }
  break;
case 7:
  DoAccelCalibrationFull();
  break;
default:
  CollectCaliData();
  break;
}
}

void MeterImu::Update() {
  ReadICM42688();
  // cali
  accm_cali = scale * (accm_filter - bias);
  // accm_cali = accm_filter;
  // judge closest_gravity
  for(int i = 0;i < 2;i++){
      if (accm_cali[i] >  8194){closest_gravity = i;break;}
      if (accm_cali[i] < -8194){closest_gravity = i + 3;break;}
  }
  ComputeEuler(accm_cali);
}

void MeterImu::ReadICM42688() {
  // read
  accm_raw(0) = getAccelDataX();
  accm_raw(1) = getAccelDataY();
  accm_raw(2) = getAccelDataZ();
  accm_filter = accm_raw_window_filter.filterEigen(accm_raw);
  temperature = getTemperature();
  // accm_filter = accm_raw;
  // gyro_raw(0) = getGyroDataX();
  // gyro_raw(1) = getGyroDataY();
  // gyro_raw(2) = getGyroDataZ();
  // gyro_filter = gyro_raw_window_filter.filterEigen(gyro_raw);
  // gyro_filter = gyro_raw;
}

void MeterImu::CollectCaliData() {
  // check calibration step if right === [1-6]
  if (cali_cmd < 1 || cali_cmd > 6) {
    cali_str = "[E]cmd:";
    cali_str += String(cali_cmd);
    sendCaliInfo(cali_str);
    return;
  }

  // check if cali already
  // if (cali_flag[cali_cmd - 1] == 1) {
  //   return;
  // }

  int  counts = 0;
  byte progress_10 = 0;
  byte pre_progress_10 = 0;
  float threshold = cali_threshold;
  Eigen::Vector3f now;
  Eigen::Vector3f sum;
  Eigen::Vector3f start;

  while (counts < sampling_num) {
    ReadICM42688();
    if (counts == 0) {
      start = accm_filter;
      sum.setZero();
    }
    now = accm_filter;
    float error = (now - start).norm();
    if (error > threshold) {
      counts = 0;
      cali_str = "[W]";
      cali_str += String(error,1);
      cali_str += "/";
      cali_str += String(threshold,1);
      sendCaliInfo(cali_str);
      return;
    }
    sum += now;
    counts++;
    progress_10 = counts * 10 / sampling_num;
    if(pre_progress_10 != progress_10){
      pre_progress_10 = progress_10;
      cali_str = "[D]";
      cali_str += String(progress_10 * 10);
      cali_str += "%";
  
      sendCaliInfo(cali_str);
    }
  }

  Eigen::Vector3f avg{sum / counts};
  for(int i = 0;i < 3;i++){
    cali_data[cali_cmd - 1][i] = avg[i];
  }
  cali_str = "[D]";
  switch (cali_cmd)
  {
    case 1:
    case 2:
    cali_str += String(cali_data[cali_cmd - 1][0],2);
    break;
    case 3:
    case 4:
    cali_str += String(cali_data[cali_cmd - 1][1],2);
    break;
    case 5:
    case 6:
    cali_str += String(cali_data[cali_cmd - 1][2],2);
    break;
  default:
    break;
  }
  sendCaliInfo(cali_str);
  vTaskDelay(500);// 保证可以通讯到
  cali_flag[cali_cmd - 1] = true;
  cali_str = "";
  for (int j = 0; j < 6; ++j) {
    if(cali_flag[j] == 1){
      cali_str += "[";
      cali_str += String(j + 1);
      cali_str += "]";
    }
    else{
      cali_str += String(j + 1);
    }
  }
  sendCaliInfo(cali_str);
  vTaskDelay(500);// 保证可以通讯到
  if(DEBUG_PRINT == true){
    for (int j = 0; j < 3;j++) {
      Serial.print(String(cali_data[cali_cmd - 1][j], 2));
      Serial.print(",");
    }
    Serial.println("\n");
  }
  counts = 0;
  sum.setZero();
  cali_cmd = 0;
}

enum detect_orientation_return {
  ORIENTATION_TOP_DOWN,
  ORIENTATION_BOTTOM_DOWN,
  ORIENTATION_LEFT_DOWN,
  ORIENTATION_RIGHT_DOWN,
  ORIENTATION_BACK_DOWN,
  ORIENTATION_FRONT_DOWN,
  ORIENTATION_ERROR
};

void MeterImu::DoAccelCalibrationFull() {

  // 值检查
  for(int i = 0;i < 6;i++){
    if(abs(abs(cali_data[i][i / 2]) - 16384) > 300){
      cali_str = "[EV]";
      cali_str += String(i + 1);
      cali_str += ":";
      cali_str += String(cali_data[i][i / 2]);
      sendCaliInfo(cali_str);
      vTaskDelay(500);// 保证可以通讯到
      cali_cmd = 0;
      return;
    }
  }

  // 符号检查
  for(int i = 0;i < 3;i++){
    if((cali_data[i * 2][i]) < 0){
      cali_str  = "[ES]";
      cali_str += String(i * 2 + 1);
      cali_str += ":";
      cali_str += String(cali_data[i * 2][i]);
      sendCaliInfo(cali_str);
      vTaskDelay(500);// 保证可以通讯到
      cali_cmd = 0;
      return;
    }
    if((cali_data[i * 2 + 1][i]) > 0){
      cali_str  = "[ES]";
      cali_str += String(i * 2 + 1 + 1);
      cali_str += ":";
      cali_str += String(cali_data[i * 2 + 1][i]);
      sendCaliInfo(cali_str);
      vTaskDelay(500);// 保证可以通讯到
      cali_cmd = 0;
      return;
    }
  }
  /*=== 计算 offset ===*/
  Eigen::Vector3f accel_top_down{cali_data[ORIENTATION_TOP_DOWN]};
  Eigen::Vector3f accel_bottom_down{cali_data[ORIENTATION_BOTTOM_DOWN]};
  Eigen::Vector3f accel_left_down{cali_data[ORIENTATION_LEFT_DOWN]};
  Eigen::Vector3f accel_right_down{cali_data[ORIENTATION_RIGHT_DOWN]};
  Eigen::Vector3f accel_back_down{cali_data[ORIENTATION_BACK_DOWN]};
  Eigen::Vector3f accel_front_down{cali_data[ORIENTATION_FRONT_DOWN]};
  Eigen::Vector3f bias_v;
  bias_v(0) = (accel_top_down(0) + accel_bottom_down(0)) * 0.5f;
  bias_v(1) = (accel_left_down(1) + accel_right_down(1)) * 0.5f;
  bias_v(2) = (accel_back_down(2) + accel_front_down(2)) * 0.5f;
  /*=== 计算出 accel_T ===*/
  Eigen::Matrix3f mat_A;
  mat_A.row(0) = accel_top_down  - bias_v;
  mat_A.row(1) = accel_left_down - bias_v;
  mat_A.row(2) = accel_back_down - bias_v;
  Eigen::Matrix3f accel_T = mat_A.inverse() * 16384;

  /*=== 设置 bias 和 scale ===*/
  bias = bias_v;
  scale << accel_T(0,0), 0, 0,
           0, accel_T(1,1), 0,
           0, 0, accel_T(2,2);
  // scale = accel_T.diagonal();
  SaveBiasScale();
  DebugBiasScale();
  cali_str = "";
  cali_str += String(scale(0,0),4);
  cali_str += ",";
  cali_str += String(scale(1,1),4);
  cali_str += ",";
  cali_str += String(scale(2,2),4);
  sendCaliInfo(cali_str);
  vTaskDelay(500);// 等待通讯完成
  cali_cmd = 0;
}

void MeterImu::parseCaliInfo(int info){
  byte cmd  = info / 1000;
  byte huns = (info / 100) % 10;
  byte tens = (info / 10) % 10;
  byte ones = (info) % 10;
  byte option = huns;
  byte data = tens * 10 + ones;
  if(cmd != 7){
    ESP_LOGE("CALI","info:%d",info);
    return;
  }
  if(option == 1 && (0 <= data) && (data <= 7)){
    cali_cmd = data;
    ESP_LOGE("CALI","cmd:%d",cali_cmd);
  }
  else if(option == 2 && (0 <= data) && (data <= 6)){
    cali_threshold = data * 10.0f;
    ESP_LOGE("CALI","cali_threshold:%f",cali_threshold);
  }
  else{ESP_LOGE("CALI","info:%d",info);}
}

char rx_buffer[128];
const int rx_max_num = 128;
void MeterImu::RxFromS3(unsigned char rc) {
  static boolean rx_progress = false;  // 是否正在接收数据
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';

  // 已经开始接收数据，那么它会判断当前接收到的字符是否为结束标记
  if (rx_progress == true) {
    if (rc != endMarker) {
      rx_buffer[ndx] = rc;
      ndx++;
      if (ndx >= rx_max_num) {
        ndx = rx_max_num - 1;
      }
    } else {
      rx_buffer[ndx] = '\0';  // terminate the string
      rx_progress = false;
      ndx = 0;
      char* strtokIndx;
      strtokIndx = strtok(rx_buffer, ",");  // get the first part - the string
      if (strtokIndx != NULL) cali_info = atoi(strtokIndx);
      parseCaliInfo(cali_info);
    }
  } else if (rc == startMarker) {
    rx_progress = true;
  }
}


float MeterImu::get_roll_ui() {
  if (euler_angle(0) > UIX[0]) {
    UIX[0] =euler_angle(0);
  }
  if (euler_angle(0) < UIX[1]) {
    UIX[1] =euler_angle(0);
  }
  if ((fabs(UIX[1] - UIX[0])) < UI_Threshold) {
    return ((UIX[0] + UIX[1]) / 2);
  }
  UIX[0] = euler_angle[0];
  UIX[1] = euler_angle[0];
  return euler_angle[0];
}

float MeterImu::get_pitch_ui() {
  if (euler_angle(1) > UIY[0]) {
    UIY[0] = euler_angle(1);
  }
  if (euler_angle(1) < UIY[1]) {
    UIY[1] = euler_angle(1);
  }
  if ((fabs(UIY[1] - UIY[0])) < UI_Threshold) {
    return ((UIY[0] + UIY[1]) / 2);
  }
  UIY[0] = euler_angle(1);
  UIY[1] = euler_angle(1);
  return euler_angle(1);
}

float MeterImu::get_yaw_ui() {
  if (euler_angle(2) > UIZ[0]) {
    UIZ[0] = euler_angle(2);
  }
  if (euler_angle(2) < UIZ[1]) {
    UIZ[1] = euler_angle(2);
  }
  if ((fabs(UIZ[1] - UIZ[0])) < UI_Threshold) {
    return ((UIZ[0] + UIZ[1]) / 2);
  }
  UIZ[0] = euler_angle(2);
  UIZ[1] = euler_angle(2);
  return euler_angle(2);
}

void MeterImu::GetBiasScale() {

  while (!pref.begin("imu_cali", false)) {
    Serial.println("imu_cali get Fail");
  }

  // DEBUG
  // bias(0) = 0.0;
  // bias(1) = 0.0;
  // bias(2) = 0.0;
  // scale(0,0)  = 1.0;
  // scale(1,1)  = 1.0;
  // scale(2,2)  = 1.0;
  // SaveBiasScale();

  bias(0) = pref.getFloat("offset0", 0.0);
  bias(1) = pref.getFloat("offset1", 0.0);
  bias(2) = pref.getFloat("offset2", 0.0);
  // cali_str += "B<<";
  // cali_str += String(bias(0),4);
  // cali_str += ",";
  // cali_str += String(bias(1),4);
  // cali_str += ",";
  // cali_str += String(bias(2),4);
  scale(0,0)  = pref.getFloat("scale0", 1.0);
  scale(1,1)  = pref.getFloat("scale1", 1.0);
  scale(2,2)  = pref.getFloat("scale2", 1.0);
  cali_str = "";
  cali_str += String(scale(0,0),4);
  cali_str += ",";
  cali_str += String(scale(1,1),4);
  cali_str += ",";
  cali_str += String(scale(2,2),4);
  sendCaliInfo(cali_str);
  pref.end();
  DebugBiasScale();

}

void MeterImu::SaveBiasScale() {
  while (!pref.begin("imu_cali", false)) {
      Serial.println("imu_cali put fail");
  }
  pref.putFloat("offset0", bias(0));
  pref.putFloat("offset1", bias(1));
  pref.putFloat("offset2", bias(2));
  // cali_str += "B<<";
  // cali_str += String(bias(0),1);
  // cali_str += ",";
  // cali_str += String(bias(1),1);
  // cali_str += ",";
  // cali_str += String(bias(2),1);
  // sendCaliInfo(cali_str);
  pref.putFloat("scale0", scale(0,0));
  pref.putFloat("scale1", scale(1,1));
  pref.putFloat("scale2", scale(2,2));
  pref.end();
}

void MeterImu::ComputeEuler(Eigen::Vector3f acc) {
  /* single axis */
  // euler_angle(0) = atan2f(acc(1),acc(2)) * 57.29578f;
  // euler_angle(1) = -atan2f(acc(0),sqrt(acc(1) * acc(1) + acc(2) * acc(2))) * 57.29578f;
  // euler_angle(2) = 0;
  /* dual axis */ 
  euler_angle(0) = atan2f(-acc[0], acc[2]) * 57.29578f;
  euler_angle(1) = atan2f(acc(0),acc(1))  * 57.29578f;
  euler_angle(2) = atan2f(acc[1], acc[2])  * 57.29578f;

  /* three axis*/
  // euler_angle(0) = atan2f(acc(0),sqrtf(acc(1) * acc(1) + acc(2) * acc(2)))* 57.29578f;
  // euler_angle(1) = atan2f(acc(1),sqrtf(acc(0) * acc(0) + acc(2) * acc(2)))* 57.29578f;
  // euler_angle(2) = atan2f(sqrtf(acc(0) * acc(0) + acc(1) * acc(1)),acc(2))* 57.29578f;
  /* Madgwick */
  /* Mahony */
  // algorithm.updateIMU(gyro_filter(0),gyro_filter(1),gyro_filter(2),accm_cali(0),accm_cali(1),accm_cali(2));
  // euler_angle << algorithm.getRoll(),algorithm.getPitch(),algorithm.getYaw();
}


void MeterImu::sendCaliInfo(String info_str) {
  String str;
  str = "";
  str += "<";
  str += String('B');
  str += info_str;
  str += ">";
  Serial1.print(str);
}

void MeterImu::sendMeasureInfo() {
  String tx_str;
  tx_str = "";
  tx_str += "<";
  tx_str += String('A');
  tx_str += String(version);
  tx_str += ",";
  tx_str += String(euler_angle(0), 3);
  tx_str += ",";
  tx_str += String(euler_angle(1), 3);
  tx_str += ",";
  tx_str += String(euler_angle(2), 3);
  tx_str += ",";
  tx_str += String(get_roll_ui(), 3);
  tx_str += ",";
  tx_str += String(get_pitch_ui(), 3);
  tx_str += ",";
  tx_str += String(get_yaw_ui(), 3);
  tx_str += ",";
  tx_str += String(closest_gravity);
  tx_str += ",";
  tx_str += String(temperature);
  tx_str += ",";
  tx_str += ">";
  Serial1.print(tx_str);
}

void MeterImu::DebugBiasScale(){
  if(DEBUG_PRINT != true){return;}
  String vectorStr = "";
  vectorStr +=  "bias << \n";
  vectorStr += String(bias(0), 4);
  vectorStr += ", ";
  vectorStr += String(bias(1), 4);
  vectorStr += ", ";
  vectorStr += String(bias(2), 4);
  vectorStr += ";";
  Serial.println(vectorStr);
  String matrixStr = "scale << \n";
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      matrixStr += String(scale(i, j), 4);
      matrixStr += "\t";
    }
    matrixStr += "\n";
  }
  Serial.println(matrixStr);
}