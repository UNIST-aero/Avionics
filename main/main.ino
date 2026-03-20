#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_BMP280.h>

struct MPUData{
  float AcX;
  float AcY;
  float AcZ;
  float Tmp;
  float GyX;
  float GyY;
  float GyZ;
};

const int MPU_ADDR = 0x68;
// 1. 태스크 핸들러 변수 선언
TaskHandle_t TaskHandle;
SemaphoreHandle_t xMutex;
struct MPUData* MPUAddr = nullptr;

Adafruit_BMP280 bmp;
float seaLevelPressure = 1019.6;
float height_ini;
float altitude;

const int CS = 5;
const int SD_MOSI = 23;
const int SD_MISO = 19;
const int SD_SCK = 18;

File dataFile;

float AngleX = 0;
float AngleY = 0;
float AngleZ = 0;

unsigned long last_ms_imu = 0;

void setup() {
  Wire.begin(21, 22);
  xMutex = xSemaphoreCreateMutex();
  Serial.begin(115200);

  beginMPU();
  //beginSD();
  beginBmp();

  // 2. 태스크 생성 시 핸들러 주소(&myTaskHandle) 전달
  xTaskCreatePinnedToCore(
    sensoringTask,          // 태스크 함수
    "Task_Example",  // 이름
    10000,           // 스택 크기
    NULL,            // 파라미터
    1,               // 우선순위
    &TaskHandle,   // 핸들러 연결
    0                // 코어 번호
  );
}

void loop() {
  if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
    // --- 임계 구역 (Critical Section) 시작 ---
    
    // --- 임계 구역 끝 ---
    xSemaphoreGive(xMutex); // 뮤텍스 반납
  }
}

void sensoringTask(void *pvParameters) {
  while(1) {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      // --- 임계 구역 (Critical Section) 시작 ---
      updateMPUData();
      altitude = bmp.readAltitude(seaLevelPressure) - height_ini;
      Serial.print("Alt:");
      Serial.println(altitude);
      //saveToSD();
      // --- 임계 구역 끝 ---
      xSemaphoreGive(xMutex); // 뮤텍스 반납
      delay(50);
    }

  }
}

#pragma region MPU6050
void beginMPU(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0);  
  // 가속도 설정 레지스터
  Wire.write(0x1C); 
  Wire.write(0x18); // 0x18 = +/- 16g
  // 자이로 설정 레지스터
  Wire.write(0x1B); 
  Wire.write(0x18); // 0x18 = +/- 2000 deg/s
  Wire.endTransmission(true);

  MPUAddr = new struct MPUData();//(struct MPUData*)malloc()
  Serial.println("MPU Start");
}

void updateMPUData(){
  int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  
  // 14바이트 데이터 요청 (가속도 6 + 온도 2 + 자이로 6)
  Wire.requestFrom(MPU_ADDR, 14, true);
  
  // 데이터 병합 (상위 8비트 << 8 | 하위 8비트)
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  MPUAddr->AcX = AcX / 2048.0f;
  MPUAddr->AcY = AcY / 2048.0f;
  MPUAddr->AcZ = AcZ / 2048.0f;
  MPUAddr->Tmp = Tmp / 340.f + 36.53f;
  MPUAddr->GyX = GyX / 16.4f;
  MPUAddr->GyY = GyY / 16.4f;
  MPUAddr->GyZ = GyZ / 16.4f;

  unsigned long curr_ms = millis();
  AngleX += MPUAddr->GyX * (curr_ms - last_ms_imu) / 1000.0f;
  AngleY += MPUAddr->GyY * (curr_ms - last_ms_imu) / 1000.0f;
  AngleZ += MPUAddr->GyZ * (curr_ms - last_ms_imu) / 1000.0f;

  Serial.print(AngleX);
  Serial.print(",");
  Serial.print(AngleY);
  Serial.print(",");
  Serial.print(AngleZ);
  Serial.print("\n");

}
#pragma endregion

#pragma region SD
void beginSD(){
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, CS);
  if (!SD.begin(CS)) {
    Serial.println("SD Init Failed");
    while (1); //연결 안 될 경우
  }

  if (SD.exists("/data.csv")) {
    SD.remove("/data.csv"); // 기존 파일 삭제 (매번 새 파일을 만들고 싶을 때)
    Serial.println("Existing data.csv removed.");
  }

  dataFile = SD.open("/data.csv", FILE_APPEND);
  Serial.println("SD Start");
}

void saveToSD() {
  if (!dataFile) {
    dataFile = SD.open("/data.csv", FILE_APPEND);
  }

  if (dataFile) {
    dataFile.print(MPUAddr->AcX); dataFile.print(",");
    dataFile.print(MPUAddr->AcY); dataFile.print(",");
    dataFile.print(MPUAddr->AcZ); dataFile.print(",");
    dataFile.print(MPUAddr->GyX); dataFile.print(",");
    dataFile.print(MPUAddr->GyY); dataFile.print(",");
    dataFile.print(MPUAddr->GyZ); dataFile.print(",");
    dataFile.println(MPUAddr->Tmp); dataFile.print(",");
    dataFile.println(altitude);

    static int count = 0; // 올라 갈 때, 내려 갈 때, 버퍼 간격 바꾸기
    if (++count >= 10) {
      dataFile.flush();
      count = 0;
    }
  }
}
#pragma endregion


#pragma region Bmp
void beginBmp(){
  bool status = bmp.begin(0x76);
  if (!status) {
    status = bmp.begin(0x77);
  }

  if (!status) {
    Serial.println("Can't find BMP280.");
    while (1);
  }

  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,     // Normal mode
    Adafruit_BMP280::SAMPLING_X2,     // Temperature oversampling
    Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
    Adafruit_BMP280::FILTER_X16,      // Filtering
    Adafruit_BMP280::STANDBY_MS_500   // Standby time
  );
  height_ini = bmp.readAltitude(seaLevelPressure); // define initial altitude

  Serial.println("Bmp Start");
}
#pragma endregion


#pragma region Utility



#pragma endregion