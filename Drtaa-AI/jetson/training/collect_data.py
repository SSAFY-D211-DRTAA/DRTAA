import os
import wave
import pyaudio
import time

# 저장할 폴더 경로
wakeword_dir = "dataset/wakeword"
not_wakeword_dir = "dataset/not_wakeword"

# 폴더가 존재하지 않으면 생성
os.makedirs(wakeword_dir, exist_ok=True)
os.makedirs(not_wakeword_dir, exist_ok=True)

# 녹음 설정
FORMAT = pyaudio.paInt16  # 16-bit resolution
CHANNELS = 1  # 모노 오디오
RATE = 16000  # 샘플링 레이트 (16kHz)
CHUNK = 1024  # 버퍼 크기
RECORD_SECONDS = 3  # 녹음 시간 (2초)
p = pyaudio.PyAudio()

# 녹음 함수
def record_audio(filename):
    stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)
    print(f"Recording... ({filename})")
    frames = []
    
    for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
        data = stream.read(CHUNK)
        frames.append(data)
    print("Recording finished.")
    
    stream.stop_stream()
    stream.close()
    
    # 녹음된 데이터를 파일로 저장
    wf = wave.open(filename, 'wb')
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()

# 반복적으로 음성 데이터를 수집
def collect_data(num_samples, data_type="wakeword"):
    # 저장할 폴더 결정
    if data_type == "wakeword":
        save_dir = wakeword_dir
    else:
        save_dir = not_wakeword_dir
    
    for i in range(num_samples):
        filename = os.path.join(save_dir, f"{data_type}_{i + 1}.wav")
        print(f"Saving to: {filename}")
        record_audio(filename)
        
        # 사용자에게 대기 시간을 주거나, 파일명 확인을 위해 잠시 대기
        time.sleep(2)

if __name__ == "__main__":
    print("Select data type: (1) Wakeword or (2) Not Wakeword")
    choice = input("Enter 1 or 2: ")

    if choice == "1":
        data_type = "wakeword"
    elif choice == "2":
        data_type = "not_wakeword"
    else:
        print("Invalid choice, exiting.")
        exit()

    num_samples = int(input("Enter the number of samples to collect: "))
    
    # 데이터 수집 시작
    collect_data(num_samples, data_type)

# 녹음 세션 종료
p.terminate()