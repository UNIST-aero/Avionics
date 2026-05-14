#include <Wire.h>
#include <SPI.h>
#include <ESP32Servo.h>
#include <math.h>
#include <SparkFun_BMP581_Arduino_Library.h> 
#include <HardwareSerial.h>

// --- 상수 및 설정 ---
const int I2C_SDA = 21;
const int I2C_SCL = 22;
const int MPU_ADDR = 0x68;
const int SERVO_PIN = 25;
const int speakerPin = 27;
const int loopTerm = 20; 
const float FALL_MARGIN = 5.0f; // 최고점 대비 5m 하강 시 사출
const float EMA_ALPHA = 0.1f;   // 고도 필터링 상수 (작을수록 부드럽지만 지연 발생)

enum SystemState : uint8_t {
  STATE_READY = 0,        // 발사 대기
  STATE_LAUNCHED = 1,     // 발사됨 (가속 상승 중)
  STATE_BOOST_DOWN = 2,   // 엔진 연소 종료 (관성 비행 진입)
  STATE_DECELERATED = 3,  // 낙하산 사출 대기 (최고점 탐색)
  STATE_DEPLOYED = 4      // 사출 완료
};

SystemState currentState = STATE_READY;

// --- 데이터 패킷 구조체 ---
// 주의: Nano 수신부 코드의 구조체도 이와 똑같이 맞춰주셔야 합니다!
#pragma pack(push, 1)
typedef struct {
  uint8_t header1 = 0xAF;
  uint8_t header2 = 0xFA;
  uint32_t timestamp;  
  float AcX, AcY, AcZ; 
  float GyX, GyY, GyZ; 
  float Tmp; 
  float raw_altitude;       // 원본 고도 (노이즈 있음)
  float filtered_altitude;  // EMA 필터링된 고도 (낙하 감지용)
  uint8_t state;        
  uint8_t checksum;    
} SendData_t;
#pragma pack(pop)

// --- 객체 및 전역 변수 ---
Servo servo;
BMP581 bmp;
QueueHandle_t sendQueue;
HardwareSerial SerialNano(2); // UART 2를 NANO 통신용으로

float groundPressure = 1013.25f; // 캘리브레이션 시 덮어씌워질 발사대 기압
float calAcX, calAcY, calAcZ, calGyX, calGyY, calGyZ;
float filtered_alt = 0.0f;
bool isAltInitialized = false;

// --- 함수 선언 ---
void beginMPU();
void calibrationSensors();
bool updateMPUData(SendData_t& data);
bool readAltitude(SendData_t& data);
void checkState(SendData_t& data);
void updateChecksum(SendData_t& data);
void recoverI2C();
void Deploy();
void controlTask(void *pvParameters);
void serialTask(void *pvParameters);

void setup() {
  Serial.begin(115200);
  SerialNano.begin(115200, SERIAL_8N1, 16, 17);
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  
  sendQueue = xQueueCreate(20, sizeof(SendData_t));
  servo.attach(SERVO_PIN);
  servo.write(90); // 서보 초기 위치 (닫힘)

  if (bmp.beginI2C() != BMP5_OK) {
    Serial.println("BMP581 Fail - Check Wiring!");
    while(1);
  }

  beginMPU();
  Serial.println("Calibrating Sensors... DO NOT MOVE!");
  calibrationSensors(); // MPU 영점 및 발사대 초기 기압 측정
  
  currentState = STATE_READY;
  Serial.println("System Ready for Launch");

  xTaskCreatePinnedToCore(controlTask, "Ctrl", 10000, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(serialTask, "Serial", 10000, NULL, 1, NULL, 1);
}

void loop() { vTaskDelay(1000 / portTICK_PERIOD_MS); }

// --- 메인 로직 태스크 ---
void controlTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(loopTerm);

  while(1) {
    SendData_t currentData;
    updateMPUData(currentData);
    readAltitude(currentData);
    
    currentData.timestamp = millis();
    currentData.state = (uint8_t)currentState;
    
    checkState(currentData);
    updateChecksum(currentData);

    if (xQueueSend(sendQueue, &currentData, 0) == errQUEUE_FULL) {
      SendData_t dummy;
      xQueueReceive(sendQueue, &dummy, 0); // 큐가 꽉 차면 가장 오래된 데이터 버림
      xQueueSend(sendQueue, &currentData, 0);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void serialTask(void *pvParameters) {
  SendData_t data;
  while(1) {
    if (xQueueReceive(sendQueue, &data, portMAX_DELAY) == pdPASS) {
      // 1. 외부 기기(Nano)로 바이너리 데이터 전송
      SerialNano.write((const uint8_t*)&data, sizeof(SendData_t));

      // 2. PC 시리얼 모니터 출력
      Serial.print("T:"); Serial.print(data.timestamp);
      Serial.print("\tSt:"); Serial.print(data.state);
      Serial.print("\tRawAlt:"); Serial.print(data.raw_altitude, 2);
      Serial.print("\tF_Alt:"); Serial.print(data.filtered_altitude, 2);
      Serial.print("\tAccZ:"); Serial.print(data.AcZ, 2);
      Serial.println(); 
    }
  }
}

// --- 센서 초기화 및 측정 ---
void beginMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x18); // Accel 16g
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x18); // Gyro 2000deg/s
  Wire.endTransmission(true);
}

void calibrationSensors() {
  float sx=0, sy=0, sz=0, gx=0, gy=0, gz=0;
  float sumPressure = 0;
  int validBmpReadings = 0;

  for(int i=0; i<500; i++) {
    // 1. MPU 데이터 수집
    SendData_t d;
    updateMPUData(d);
    sx+=d.AcX; sy+=d.AcY; sz+=d.AcZ;
    gx+=d.GyX; gy+=d.GyY; gz+=d.GyZ;

    // 2. BMP 초기 기압 수집 (발사대 고도를 0m로 맞추기 위함)
    bmp5_sensor_data s = {0};
    if (bmp.getSensorData(&s) == BMP5_OK) {
      sumPressure += (s.pressure / 100.0f); // Pa to hPa
      validBmpReadings++;
    }
    delay(2);
  }
  
  calAcX=sx/500.0; calAcY=sy/500.0; calAcZ=(sz/500.0)-1.0; 
  calGyX=gx/500.0; calGyY=gy/500.0; calGyZ=gz/500.0;

  // 평균 기압을 해수면 기압 대신 사용하여 현재 위치를 0m(상대고도)로 설정
  if(validBmpReadings > 0) {
    groundPressure = sumPressure / validBmpReadings;
  }
}

bool updateMPUData(SendData_t& data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) { recoverI2C(); return false; }
  
  if (Wire.requestFrom(MPU_ADDR, 14, true) != 14) return false;

  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  int16_t tp = Wire.read() << 8 | Wire.read();
  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();

  data.AcX = ax / 2048.0f - calAcX;
  data.AcY = ay / 2048.0f - calAcY;
  data.AcZ = az / 2048.0f - calAcZ;
  data.Tmp = tp / 340.0f + 36.53f;
  data.GyX = gx / 16.4f - calGyX;
  data.GyY = gy / 16.4f - calGyY;
  data.GyZ = gz / 16.4f - calGyZ;
  return true;
}

bool readAltitude(SendData_t& data) {
  bmp5_sensor_data s = {0};
  if (bmp.getSensorData(&s) == BMP5_OK) {
    // 캘리브레이션된 발사대 기압(groundPressure)을 기준으로 상대 고도 계산
    float raw_alt = 44330.0 * (1.0f - pow((s.pressure / 100.0f) / groundPressure, 0.1903f));
    
    // EMA 필터 적용
    if (!isAltInitialized) {
      filtered_alt = raw_alt;
      isAltInitialized = true;
    } else {
      filtered_alt = (EMA_ALPHA * raw_alt) + ((1.0f - EMA_ALPHA) * filtered_alt);
    }

    data.raw_altitude = raw_alt;
    data.filtered_altitude = filtered_alt;
    return true;
  }
  return false;
}

// --- 상태 관리 로직 ---
void checkState(SendData_t& data) {
  static int count = 0;
  static float maxHeight = -1000.0f; // 매우 낮은 값으로 초기화

  switch(currentState) {
    case STATE_READY:
      // 발사 감지: Z축 가속도가 2.5g 이상으로 100ms(5루프) 유지될 때
      if(data.AcZ > 2.5f) { 
        if(++count > 5) { currentState = STATE_LAUNCHED; count = 0; } 
      } else { count = 0; }
      break;
      
    case STATE_LAUNCHED:
      // MECO(연소 종료) 감지: 관성 비행 시작으로 가속도가 0.5g 이하로 떨어질 때
      if(data.AcZ < 0.5f) { 
        if(++count > 5) { currentState = STATE_BOOST_DOWN; count = 0; } 
      } else { count = 0; }
      break;
      
    case STATE_BOOST_DOWN:
      // 관성 비행 진입 직후의 불안정한 센서 데이터를 무시하기 위한 대기 시간
      // 약 1초(50루프 * 20ms) 대기 후 본격적인 최고점 탐색 상태로 전환
      if(++count > 50) { 
        currentState = STATE_DECELERATED; 
        count = 0; 
        maxHeight = data.filtered_altitude; // 현재 고도를 초기 최고점으로 세팅
      }
      break;
      
    case STATE_DECELERATED:
      // 최고 고도 갱신
      if(data.filtered_altitude > maxHeight) {
        maxHeight = data.filtered_altitude;
      }
      // 현재 필터링된 고도가 최고 고도 대비 FALL_MARGIN(5m) 이상 떨어졌을 때 사출
      if((maxHeight - data.filtered_altitude) > FALL_MARGIN) { 
        if(++count > 3) { Deploy(); count = 0; } 
      } else { count = 0; }
      break;
      
    case STATE_DEPLOYED:
      // 사출 완료 후 추가 동작 없음
      break;
  }
}

void Deploy() {
  currentState = STATE_DEPLOYED;
  servo.write(0); // 낙하산 사출 (서보 모터 개방)
  Serial.println("!!! PARACHUTE DEPLOYED !!!");
}

void updateChecksum(SendData_t& data) {
  uint8_t sum = 0;
  uint8_t* p = (uint8_t*)&data;
  for (size_t i = 0; i < sizeof(SendData_t) - 1; i++) sum += p[i];
  data.checksum = sum;
}

void recoverI2C() {
  Wire.end();
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(I2C_SCL, LOW); delayMicroseconds(5);
    digitalWrite(I2C_SCL, HIGH); delayMicroseconds(5);
  }
  Wire.begin(I2C_SDA, I2C_SCL, 400000);
  beginMPU();
  Serial.println("I2C Recovered");
}