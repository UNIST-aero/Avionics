import struct
import csv
import os

# 1. 아두이노에서 정의한 구조체와 동일한 포맷 설정 (중요!)
# I: unsigned int (timestamp), f: float, ?: bool
# 구조: timestamp(I) + float 9개(9f) + bool 4개(4?)
struct_format = 'I9f4?'
struct_size = struct.calcsize(struct_format)

def recover_binary_data(bin_filename, csv_filename):
    if not os.path.exists(bin_filename):
        print(f"❌ 에러: {bin_filename} 파일을 찾을 수 없습니다.")
        return

    print(f"📂 {bin_filename} 복구 시작...")

    with open(bin_filename, 'rb') as bin_file, open(csv_filename, 'w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        
        # CSV 헤더 작성
        header = [
            "Timestamp(ms)", "AcX", "AcY", "AcZ", 
            "GyX", "GyY", "GyZ", "Tmp", 
            "altitude_abs", "altitude_rel", 
            "isLaunched", "isDeployed", "Buzzer1", "Buzzer2"
        ]
        writer.writerow(header)

        count = 0
        while True:
            # 구조체 크기만큼 바이트 읽기
            chunk = bin_file.read(struct_size)
            if len(chunk) < struct_size:
                break
            
            # 이진 데이터를 파이썬 변수로 변환
            data = struct.unpack(struct_format, chunk)
            writer.writerow(data)
            count += 1

    print(f"✅ 복구 완료! 총 {count}행의 데이터가 {csv_filename}에 저장되었습니다.")

if __name__ == "__main__":
    # 파일 경로 설정 (SD 카드에서 가져온 파일명으로 수정하세요)
    input_file = "data1.bin" 
    output_file = "recovered_data1.csv"
    recover_binary_data(input_file, output_file)