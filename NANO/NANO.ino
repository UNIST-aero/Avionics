#include <SPI.h>
#include <SD.h>
#include <Servo.h>

#define SD_CS 10
#define LED_PIN 7
#define SERVO_PIN 9

// ESP32와 완벽히 동일한 구조체
#pragma pack(push, 1)
typedef struct {
  uint8_t header1 = 0xAF;
  uint8_t header2 = 0xFA;
  uint32_t timestamp;  
  float AcX, AcY, AcZ; 
  float GyX, GyY, GyZ; 
  float Tmp; 
  float raw_altitude;       // 칼만 변수 대신 추가됨
  float filtered_altitude;  // 칼만 변수 대신 추가됨
  uint8_t state;        
  uint8_t checksum;    
} SendData_t;
#pragma pack(pop)

Servo deployServo;
File logFile;
char filename[20];
uint8_t packetBuffer[sizeof(SendData_t)];
int packetIndex = 0;
bool sdFailed = false;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200); 
  deployServo.attach(SERVO_PIN);
  deployServo.write(90); // 대기 각도

  if (!SD.begin(SD_CS)) {
    sdFailed = true;
    digitalWrite(LED_PIN, LOW);
  } else {
    for (int i = 1; i < 1000; i++) {
      sprintf(filename, "LOG%03d.BIN", i);
      if (!SD.exists(filename)) {
        logFile = SD.open(filename, FILE_WRITE);
        break;
      }
    }
    if (logFile) digitalWrite(LED_PIN, HIGH);
  }
}

void loop() {
  // 시리얼 버퍼에 데이터가 있을 때 빠르게 모두 읽어들임
  while (Serial.available()) {
    uint8_t b = Serial.read();
    packetBuffer[packetIndex++] = b;

    // 1. 헤더 검사 (첫 2바이트)
    if (packetIndex == 1 && packetBuffer[0] != 0xAF) {
      packetIndex = 0; continue;
    }
    if (packetIndex == 2 && packetBuffer[1] != 0xFA) {
      packetIndex = 0; continue;
    }

    // 2. 패킷 완성 시 처리
    if (packetIndex == sizeof(SendData_t)) {
      if (verifyChecksum(packetBuffer)) {
        SendData_t* pkt = (SendData_t*)packetBuffer;
        
        handleServo(pkt->state);
        saveToSD(packetBuffer, sizeof(SendData_t));
        
        digitalWrite(LED_PIN, HIGH);
      } else {
        digitalWrite(LED_PIN, LOW); // 체크섬 오류 표시
      }
      packetIndex = 0; // 다음 패킷 준비를 위해 인덱스 초기화
    }
  }
}

bool verifyChecksum(uint8_t* data) {
  uint8_t sum = 0;
  for (size_t i = 0; i < sizeof(SendData_t) - 1; i++) sum += data[i];
  return (sum == data[sizeof(SendData_t) - 1]);
}

void saveToSD(uint8_t* data, size_t len) {
  if (sdFailed || !logFile) return;
  
  if (logFile.write(data, len) != len) {
    sdFailed = true;
    logFile.close();
    digitalWrite(LED_PIN, LOW); // SD 기록 실패 시 LED 끄기
    return;
  }

  static unsigned long lastFlush = 0;
  //flush 주기를 1초에서 3초로 늘려 시리얼 버퍼 병목 완화
  if (millis() - lastFlush > 3000) { 
    logFile.flush();
    lastFlush = millis();
  }
}

void handleServo(uint8_t state) {
  static bool isDeployed = false;
  // ESP32의 새로운 STATE_DEPLOYED 번호인 4로 변경
  if (state == 4 && !isDeployed) { 
    deployServo.write(0); // 사출 각도
    isDeployed = true;
  }
}