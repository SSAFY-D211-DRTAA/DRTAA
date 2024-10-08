import pyaudio
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import load_model
import librosa

# --------------------------------------------
# 1. 모델 불러오기
# --------------------------------------------

model_path = 'wakeword_model.h5'

def load_wakeword_model():
    model = load_model(model_path)
    print(f"모델이 {model_path}에서 불러와졌습니다.")
    return model

# --------------------------------------------
# 2. 실시간 음성 녹음 및 처리 함수
# --------------------------------------------

# 음성 설정
FORMAT = pyaudio.paInt16  # 16-bit resolution
CHANNELS = 1              # 모노 채널
RATE = 16000              # 16kHz 샘플링 레이트
CHUNK = 1024              # 버퍼 크기
RECORD_SECONDS = 3        # 1초 동안 녹음
SILENCE_THRESHOLD = 0.01  # 무음 임계값

# PyAudio 객체 생성
p = pyaudio.PyAudio()

def process_audio_data(audio_data, model):
    # 음성 데이터를 NumPy 배열로 변환
    audio_data = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32)

    # 데이터 정규화
    audio_data = audio_data / np.max(np.abs(audio_data))
    # 입력 데이터를 16000 샘플로 맞추기
    if len(audio_data) < 16000:
        # 데이터가 짧으면 0으로 패딩
        padding = np.zeros(16000 - len(audio_data))
        audio_data = np.concatenate((audio_data, padding))
    elif len(audio_data) > 16000:
        # 데이터가 길면 앞에서부터 16000개만 사용
        audio_data = audio_data[:16000]

    # 모델 입력 크기에 맞게 reshape (1, 16000, 1)
    audio_data = audio_data.reshape(1, 16000, 1)

    # 모델로 예측
    prediction = model.predict(audio_data)

    # 웨이크 워드 감지 여부 판단
    if prediction[0] > 0.7:
        print(f"웨이크 워드 감지됨! {prediction[0]}")
    else:
        print(f"웨이크 워드 감지되지 않음. {prediction[0]}")

# --------------------------------------------
# 3. 실시간 음성 처리 루프
# --------------------------------------------

def listen_for_wakeword(model):
    # 오디오 스트림 열기
    stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)

    print("실시간 웨이크 워드 감지를 시작합니다...")

    try:
        while True:
            # 1초 동안 데이터를 수집 (RATE * RECORD_SECONDS만큼 수집)
            frames = []

            for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
                data = stream.read(CHUNK)
                frames.append(data)

            # 수집된 데이터를 하나의 큰 데이터로 합침
            audio_data = b''.join(frames)

            # 음성 데이터가 무음인지 체크
            audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32)
            if np.max(np.abs(audio_np)) < SILENCE_THRESHOLD:
                print("무음 상태입니다. 계속 대기 중...")
                continue

            # 수집된 데이터 처리
            process_audio_data(audio_data, model)

    except KeyboardInterrupt:
        # 스트림 종료
        print("종료합니다.")
        stream.stop_stream()
        stream.close()

# --------------------------------------------
# 4. 메인 실행 부분
# --------------------------------------------

if __name__ == "__main__":
    # 모델 불러오기
    model = load_wakeword_model()

    # 실시간 웨이크 워드 감지 시작
    listen_for_wakeword(model)

    # PyAudio 종료
    p.terminate()
