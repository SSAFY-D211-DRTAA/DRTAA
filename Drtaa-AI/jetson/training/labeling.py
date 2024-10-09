import os
import csv

# 데이터 폴더 경로 설정
wakeword_dir = "dataset/wakeword"
not_wakeword_dir = "dataset/not_wakeword"
csv_filename = "dataset/labels.csv"

# CSV 파일을 생성하고 라벨 정보를 기록하는 함수
def create_labels_csv():
    with open(csv_filename, mode='w', newline='') as csv_file:
        fieldnames = ['filename', 'label']  # 파일 이름과 라벨을 기록
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

        # CSV 파일에 헤더 쓰기
        writer.writeheader()

        # 웨이크 워드 데이터 라벨링 (label=1)
        for filename in os.listdir(wakeword_dir):
            if filename.endswith(".wav"):
                writer.writerow({'filename': os.path.join(wakeword_dir, filename), 'label': 1})

        # 일반 음성 및 소음 데이터 라벨링 (label=0)
        for filename in os.listdir(not_wakeword_dir):
            if filename.endswith(".wav"):
                writer.writerow({'filename': os.path.join(not_wakeword_dir, filename), 'label': 0})

    print(f"CSV file '{csv_filename}' created with labels for {len(os.listdir(wakeword_dir)) + len(os.listdir(not_wakeword_dir))} files.")

if __name__ == "__main__":
    # 라벨 CSV 파일 생성
    create_labels_csv()
