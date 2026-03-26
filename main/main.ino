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

#pragma pack(push, 1)
struct LogData {
  unsigned long timestamp; 
  float AcX, AcY, AcZ; 
  float GyX, GyY, GyZ; 
  float Tmp; 
  float altitude_abs; // 절대 고도 (sea level 기준)
  float altitude_rel; // 상대 고도 (초기 고도 기준)
  bool isLaunched; 
  bool isDeployed; 
  bool Buzzer1; 
  bool Buzzer2; 
};
#pragma pack(pop)
// MPU6050 기본 I2C 주소
const int MPU_ADDR = 0x68;

#pragma region Servo
// 서보 모터 정의
Servo servo;
const int SERVO_PIN = 25;
#pragma endregion


//부저 핀 번호
const int speakerPin = 27;

#pragma region Task
// 1. 태스크 핸들러 변수 선언
TaskHandle_t TaskHandle;
//emaphoreHandle_t xMutex;
struct MPUData* MPUAddr = nullptr;
#pragma endregion

#pragma region Bmp
// 기압고도계 및 해수면 기압 정의
Adafruit_BMP280 bmp;
float seaLevelPressure = 1019.6; // 기상청 참고해서 그때의 해면기압 넣기
float altitude; // 현재 해수면 기준 고도
float height_ini;
#pragma endregion

#pragma region SDData
// SD 카드 핀 정의
const int CS = 5;
const int SD_MOSI = 23;
const int SD_MISO = 19;
const int SD_SCK = 18;
char filename[20];

// SD 카드에 저장할 파일 저장
File dataFile;

// 데이터 저장을 위한 큐 핸들
QueueHandle_t sdQueue;

const int msgSize = 50;
#pragma endregion

// 현재 기체의 각도 (자이로 센서 적분 기준)
float AngleX = 0;
float AngleY = 0;
float AngleZ = 0;

// 마지막으로 IMU에서 값을 불러온 시점(millis 기준)
unsigned long last_ms_imu = 0;

const float LAUNCH_THRESHOLD = 35; // 사출 기준 고도 (m, 원래 값 : 35)
const float FALL_MARGIN = 7; // 최고 고도 기준 추락 허용 고도 (m, 원래 값 : 7) -> 최고 고도 기준 --m 이상 하강 시 사출 조건 충족
const int COUNT_LIMIT = 50; // 최고 고도 기준 추락이 --번 감지되면 사출 조건 충족(원래 값 : 200)

float maxHeight = 0; // 최고 도달 고도 (m)
int fallCount = 0; // 추락 감지 횟수
bool isLaunched = false; // 발사 여부
bool isDeployed = false; // 사출 여부

bool Buzzer1 = false; // 발사 부저 울림 여부
bool Buzzer2 = false; // 사출 부저 울림 여부




// 시작 코드
void setup() {
  Wire.begin(21, 22);
  //xMutex = xSemaphoreCreateMutex();
  Serial.begin(115200);
  sdQueue = xQueueCreate(50, sizeof(LogData)); //ESP32의 RAM을 믿는다
  
  servo.attach(SERVO_PIN);
  servo.write(90); 

  ledcAttach(speakerPin, 1000,8);


  /*ledcAttach(buzzerPin, 1000, 8);
  ledcWriteTone(buzzerPin, 1000);
  delay(200);
  ledcWriteTone(buzzerPin, 0);
  delay(200);*/

  beginMPU();
  beginSD();
  beginBmp();

  delay(500);

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
  //if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
    // --- 임계 구역 (Critical Section) 시작 ---
    updateMPUData();
    altitude = bmp.readAltitude(seaLevelPressure);
    checklaunch(altitude - height_ini); // 발사 여부 판정

    float g = sqrt(
      MPUAddr->AcX * MPUAddr->AcX +
      MPUAddr->AcY * MPUAddr->AcY +
      MPUAddr->AcZ * MPUAddr->AcZ
    ); // g 단위로 현재 총 가속도
    if(!isLaunched) {
      ledcWriteTone(speakerPin, 261);
      delay(100);
      ledcWriteTone(speakerPin, 0);
      delay(1000);
    }
    if(isLaunched && !Buzzer1) {
      Buzzer1 = true;
      ledcWriteTone(speakerPin, 261);
      delay(1000);
      ledcWriteTone(speakerPin,0);
      delay(500);
    }
    if (checkHeight(altitude - height_ini, g) && isLaunched && !isDeployed){ // 발사 이후, --번 이상의 추락 감지 시 사출
      Deploy();
    }
    LogData sendData;
    sendData.timestamp = millis();
    sendData.AcX = MPUAddr->AcX;
    sendData.AcY = MPUAddr->AcY;
    sendData.AcZ = MPUAddr->AcZ;
    sendData.GyX = MPUAddr->GyX;
    sendData.GyY = MPUAddr->GyY;
    sendData.GyZ = MPUAddr->GyZ;
    sendData.Tmp = MPUAddr->Tmp;
    sendData.altitude_abs = altitude;
    sendData.altitude_rel = altitude - height_ini;
    sendData.isLaunched = isLaunched;
    sendData.isDeployed = isDeployed;
    sendData.Buzzer1 = Buzzer1;
    sendData.Buzzer2 = Buzzer2;

    xQueueSend(sdQueue, &sendData, 0); // 데이터 큐에 저장
}


// 사출
void Deploy() {
  isDeployed = true;
  Buzzer2 = true;
  for(int i = 0; i<3; i++) {
    servo.write(0);
    delay(300);
    servo.write(90);
    delay(300);
  }
  ledcWriteTone(speakerPin, 261);
  delay(1000);
  ledcWriteTone(speakerPin,0);
  delay(500);
}

// 데이터 로깅 멀티스레딩
void loggingTask(void *pvParameters) {
  LogData receivedData;

  while(1) {
    if (xQueueReceive(sdQueue, &receivedData, portMAX_DELAY) == pdPASS) {
      saveToSD(receivedData);
    }
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
  if (!SD.begin(CS, SPI, 30000000)) { // ESP32는 40MHz까지 안정적
    Serial.println("SD Init Failed");
    while (1); //연결 안 될 경우
  }

  int fileIndex = 1;
  while (true) {
    sprintf(filename, "/data%d.bin", fileIndex);
    if (!SD.exists(filename)) {
      break;
    }
    fileIndex++;
  }

  dataFile = SD.open(filename, FILE_WRITE);
  Serial.println("SD Start");
}

void saveToSD(LogData data) {
  if (!dataFile) {
    dataFile = SD.open(filename, FILE_APPEND);
  }

  if (dataFile) {
    
    dataFile.write((const uint8_t*)&data, sizeof(data));

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
  if((maxHeight - h) > FALL_MARGIN) {
    fallCount++;
  } else {
    fallCount = 0;
  }

  if(fallCount >= COUNT_LIMIT) {
    return true;
  }
  return false;
}
#pragma endregion