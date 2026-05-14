import struct
import csv
import os

# --- 구조체 포맷 매핑 ---
# <  : 리틀 엔디안 (ESP32, 아두이노 기본)
# B  : uint8_t (1 byte)
# I  : uint32_t (4 bytes)
# 9f : float 9개 연속 (AcX~Z, GyX~Z, Tmp, raw_alt, filtered_alt) - 4 bytes * 9
# B  : uint8_t state (1 byte)
# B  : uint8_t checksum (1 byte)
FORMAT = "<BB I 9f B B"
PACKET_SIZE = struct.calcsize(FORMAT) # 정확히 44 bytes 나옵니다.

COLUMNS = [
    "header1", "header2", "timestamp", 
    "AcX", "AcY", "AcZ", "GyX", "GyY", "GyZ", "Tmp", 
    "raw_altitude", "filtered_altitude", "state", "checksum"
]

def convert_flight_log(bin_file_path, csv_file_path):
    if not os.path.exists(bin_file_path):
        print(f"오류: {bin_file_path} 파일을 찾을 수 없습니다.")
        return

    # SD 카드 로그 파일을 메모리에 한 번에 읽어옵니다.
    with open(bin_file_path, "rb") as f:
        raw_data = f.read()
        
    valid_packets = 0
    corrupted_packets = 0
    
    with open(csv_file_path, "w", newline="") as csv_f:
        writer = csv.writer(csv_f)
        writer.writerow(COLUMNS)
        
        i = 0
        # 파일 끝까지 바이트 단위로 스캔 (데이터 카빙 방식)
        while i <= len(raw_data) - PACKET_SIZE:
            
            # 1. 패킷 헤더(0xAF 0xFA) 동기화 확인
            if raw_data[i] == 0xAF and raw_data[i+1] == 0xFA:
                packet = raw_data[i : i+PACKET_SIZE]
                
                # 2. 파이썬에서 체크섬 검증 (아두이노 코드와 동일한 로직)
                # 마지막 바이트(checksum)를 제외한 나머지 바이트의 합을 구함
                calculated_checksum = sum(packet[:-1]) & 0xFF 
                received_checksum = packet[-1]
                
                # 3. 체크섬이 일치하면 정상 데이터로 판단하고 CSV에 작성
                if calculated_checksum == received_checksum:
                    unpacked_data = struct.unpack(FORMAT, packet)
                    writer.writerow(unpacked_data)
                    
                    valid_packets += 1
                    i += PACKET_SIZE # 유효한 패킷 크기만큼 인덱스 점프
                    continue
                else:
                    corrupted_packets += 1
            
            # 헤더가 아니거나, 체크섬이 틀렸다면 (진동으로 인한 데이터 훼손)
            # 1바이트만 전진해서 다음 유효한 헤더를 다시 찾음
            i += 1

    print("\n" + "="*40)
    print(f"🚀 파일 변환 완료: {bin_file_path} -> {csv_file_path}")
    print(f"✅ 정상 추출된 데이터 : {valid_packets} 줄")
    if corrupted_packets > 0:
        print(f"❌ 손상되어 버려진 데이터 : {corrupted_packets} 개 (진동/전원 불안정)")
    print("="*40 + "\n")

# --- 실행 부분 ---
# 실제 SD 카드에 저장된 파일명과 변환할 CSV 파일명을 적어주세요.
# 같은 폴더에 LOG001.BIN 파일이 있다고 가정합니다.
convert_flight_log("LOG001.BIN", "flight_data.csv")
