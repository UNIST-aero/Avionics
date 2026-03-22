// 필요 라이브러리
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <ESP32Servo.h>
#include <math.h>
#include <Adafruit_BMP280.h>

// MPU6050 IMU 센서의 출력 데이터 구조체

struct MPUData{
  float AcX; // X 방향 가속도, g (9.81 m/s^2)
  float AcY; 
  float AcZ;
  float Tmp; // 온도 'C
  float GyX; // X 방향 각속도, deg/s
  float GyY;
  float GyZ;
};

// 서보 모터 정의
Servo servo;
const int SERVO_PIN = 14;

// MPU6050 기본 I2C 주소
const int MPU_ADDR = 0x68;

// 1. 태스크 핸들러 변수 선언
TaskHandle_t TaskHandle;
SemaphoreHandle_t xMutex;
struct MPUData* MPUAddr = nullptr;

// 기압고도계 및 해수면 기압 정의
Adafruit_BMP280 bmp;
float seaLevelPressure = 1019.6; // 기상청 참고해서 그때의 해면기압 넣기
float altitude; // 현재 해수면 기준 고도
float height_ini;

// SD 카드 핀 정의
const int CS = 5;
const int SD_MOSI = 23;
const int SD_MISO = 19;
const int SD_SCK = 18;
char filename[20];

// SD 카드에 저장할 파일 저장
File dataFile;

// 현재 기체의 각도 (자이로 센서 적분 기준)
float AngleX = 0;
float AngleY = 0;
float AngleZ = 0;

// 마지막으로 IMU에서 값을 불러온 시점(millis 기준)
unsigned long last_ms_imu = 0;

const float LAUNCH_THRESHOLD = 50; // 사출 기준 고도 (m)
const float FALL_MARGIN = 5; // 최고 고도 기준 추락 허용 고도 (m) -> 최고 고도 기준 --m 이상 하강 시 사출 조건 충족
const int COUNT_LIMIT = 3; // 최고 고도 기준 추락이 --번 감지되면 사출 조건 충족

float maxHeight = 0; // 최고 도달 고도 (m)
int fallCount = 0; // 추락 감지 횟수
bool isLaunched = false; // 발사 여부
bool isDeployed = false; // 사출 여부

// 시작 코드
void setup() {
  Wire.begin(21, 22);
  xMutex = xSemaphoreCreateMutex();
  Serial.begin(115200);

  beginMPU();
  beginSD();
  beginBmp();
  servo.attach(SERVO_PIN);
  servo.write(0);

  height_ini = bmp.readAltitude(seaLevelPressure);

  // 2. 태스크 생성 시 핸들러 주소(&myTaskHandle) 전달
  xTaskCreatePinnedToCore(
    loggingTask,          // 태스크 함수
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

    updateMPUData();
    altitude = bmp.readAltitude(seaLevelPressure);

    checklaunch(altitude - height_ini); // 발사 여부 판정
    float g = sqrt(
      MPUAddr->AcX * MPUAddr->AcX +
      MPUAddr->AcY * MPUAddr->AcY +
      MPUAddr->AcZ * MPUAddr->AcZ
    ); // g 단위로 현재 총 가속도
    if (checkHeight(altitude - height_ini, g) && isLaunched && !isDeployed){ // 발사 이후, --번 이상의 추락 감지 시 사출
      Deploy();
    }
    // --- 임계 구역 끝 ---
    xSemaphoreGive(xMutex); // 뮤텍스 반납
  }
}

// 발사 감지
void checklaunch(float h) {
  if(!isLaunched && (h > LAUNCH_THRESHOLD)) {
    isLaunched = true;
  }
}

// 추락 감지
bool checkHeight(float h, float g) {
  if(h > maxHeight) {
    maxHeight = h;
    fallCount = 0;
    return false;
  }
  if((g < 5) || (maxHeight - h) > FALL_MARGIN) {
    fallCount++;
  } else {
    fallCount = 0;
  }

  if(fallCount >= COUNT_LIMIT) {
    return true;
  }
  return false;
}

// 사출
void Deploy() {
  for(int i = 0; i<10; i++) {
    servo.write(90);
    delay(300);
    servo.write(10);
    delay(300);
  }
  isDeployed = true;
}

// 데이터 로깅 멀티스레딩
void loggingTask(void *pvParameters) {
  while(1) {
    //if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
      // --- 임계 구역 (Critical Section) 시작 ---
      saveToSD();
      // --- 임계 구역 끝 ---
      //xSemaphoreGive(xMutex); // 뮤텍스 반납
      vTaskDelay(50 / portTICK_PERIOD_MS);
    //}

  }
}


#pragma region MPU6050
void beginMPU(){
  // Wake up
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Accel
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x18);
  Wire.endTransmission(true);

  // Gyro
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x18);
  Wire.endTransmission(true);

  MPUAddr = new struct MPUData();//(struct MPUData*)malloc()
  Serial.println("MPU Start");

  last_ms_imu = millis();
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
  last_ms_imu = curr_ms;

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

  int fileIndex = 1;
  while (true) {
    sprintf(filename, "/data%d.csv", fileIndex);
    if (!SD.exists(filename)) {
      break;
    }
    fileIndex++;
  }

  dataFile = SD.open(filename, FILE_WRITE);
  dataFile.print("AcX,AcY,AcZ,GyX,GyY,GyZ,Tmp,altitude(abs),altitude(rel),isLaunched,isDeployed\n");
  Serial.println("SD Start");
}

void saveToSD() {
  if (!dataFile) {
    dataFile = SD.open(filename, FILE_APPEND);
  }

  if (dataFile) {
    dataFile.print(MPUAddr->AcX); dataFile.print(",");
    dataFile.print(MPUAddr->AcY); dataFile.print(",");
    dataFile.print(MPUAddr->AcZ); dataFile.print(",");
    dataFile.print(MPUAddr->GyX); dataFile.print(",");
    dataFile.print(MPUAddr->GyY); dataFile.print(",");
    dataFile.print(MPUAddr->GyZ); dataFile.print(",");
    dataFile.print(MPUAddr->Tmp); dataFile.print(",");
    dataFile.print(altitude); dataFile.print(",");
    dataFile.print(altitude - height_ini); dataFile.print(",");
    dataFile.print(isLaunched); dataFile.print(",");
    dataFile.print(isDeployed); dataFile.print("\n");

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

  Serial.println("Bmp Start");
}
#pragma endregion


#pragma region Utility



#pragma endregion