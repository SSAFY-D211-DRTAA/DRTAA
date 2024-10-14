import sys
import pyaudio
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import load_model
import speech_recognition as sr
from gtts import gTTS
import soundfile as sf
import os
import requests
from pydub import AudioSegment
import re
import platform
from PyQt5.QtWidgets import QApplication, QLabel, QPushButton, QVBoxLayout, QWidget
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import QThread, pyqtSignal

# --------------------------------------------
# 1. 모델 불러오기
# --------------------------------------------

model_path = 'one_depth_wakeword_model.h5'

def load_wakeword_model():
    model = load_model(model_path)
    print(f"모델이 {model_path}에서 불러와졌습니다.")
    return model

# --------------------------------------------
# 2. 음성 인식 및 추천 장소 로직
# --------------------------------------------

def open_audio_file(file_path):
    system_name = platform.system()

    if system_name == "Windows":
        os.system(f"start {file_path}")
    elif system_name == "Darwin":  # macOS
        os.system(f"open {file_path}")
    elif system_name == "Linux":
        os.system(f"xdg-open {file_path}")
    else:
        print(f"지원하지 않는 운영체제입니다: {system_name}")

def convert_mp3_to_wav(mp3_file, wav_file):
    audio = AudioSegment.from_mp3(mp3_file)
    audio.export(wav_file, format="wav")

def convert_text_to_speech_and_modify(text):
    tts = gTTS(text=text, lang='ko')
    tts.save("output/output.mp3")

    # MP3를 WAV로 변환
    convert_mp3_to_wav("output/output.mp3", "output/output.wav")

    # 변환된 음성 파일 실행
    open_audio_file("output/output.wav")

def get_recommendations(user_request):
    url = "https://j11d211.p.ssafy.io/ai-api-server/recommend_place"
    data = {"user_request": user_request}

    try:
        response = requests.post(url, json=data)
        if response.status_code == 200:
            result = response.json()
            recommendation = result.get("recommendation", "추천 정보 없음")
            print(f"추천 장소: {recommendation}")
            return recommendation
        else:
            print(f"오류 발생: {response.status_code}")
            return None
    except Exception as e:
        print(f"API 호출 중 오류 발생: {e}")
        return None

def record_and_recognize_speech(update_status_signal):
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        print("말씀하세요...")
        recognizer.adjust_for_ambient_noise(source, duration=1)
        audio = recognizer.listen(source, timeout=3, phrase_time_limit=3)

    try:
        text = recognizer.recognize_google(audio, language="ko-KR")
        print(f"인식된 텍스트: {text}")
        
        # 텍스트가 성공적으로 인식되면 상태를 'processing'으로 변경
        update_status_signal.emit(f"인식된 텍스트: {text}", "processing")

        # 인식된 텍스트를 파일에 저장
        with open("output/stt.txt", 'w') as file:
            file.write(text)
        print("음성 데이터가 파일에 저장되었습니다.")
        return text
    except sr.UnknownValueError:
        print("음성을 이해할 수 없습니다.")
        return None
    except sr.RequestError:
        print("API 요청 중 문제가 발생했습니다.")
        return None

# --------------------------------------------
# 3. 웨이크 워드 감지 스레드
# --------------------------------------------

class WakewordDetectionThread(QThread):
    update_status_signal = pyqtSignal(str, str)  # 상태 업데이트를 위한 신호 정의

    def __init__(self, model, parent=None):
        super(WakewordDetectionThread, self).__init__(parent)
        self.model = model
        self.p = pyaudio.PyAudio()
        self.is_running = True

    def run(self):
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 16000
        CHUNK = 1024
        RECORD_SECONDS = 3
        SILENCE_THRESHOLD = 0.01

        stream = self.p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)

        while self.is_running:
            frames = []
            for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
                data = stream.read(CHUNK)
                frames.append(data)

            audio_data = b''.join(frames)
            audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32)

            if np.max(np.abs(audio_np)) < SILENCE_THRESHOLD:
                self.update_status_signal.emit("무음 상태입니다. 계속 대기 중...", "silent")
                continue

            self.process_audio_data(audio_data)

        stream.stop_stream()
        stream.close()
        self.p.terminate()

    def process_audio_data(self, audio_data):
        audio_data = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32)
        audio_data = audio_data / np.max(np.abs(audio_data))

        if len(audio_data) < 16000:
            padding = np.zeros(16000 - len(audio_data))
            audio_data = np.concatenate((audio_data, padding))
        elif len(audio_data) > 16000:
            audio_data = audio_data[:16000]

        audio_data = audio_data.reshape(1, 16000, 1)
        prediction = self.model.predict(audio_data)

        if prediction[0] > 0.5:
            self.update_status_signal.emit(f"웨이크 워드 감지됨! {prediction[0]}", "wakeword_detected")
            self.run_speech_recognition()
        else:
            self.update_status_signal.emit(f"웨이크 워드 감지되지 않음. {prediction[0]}", "listening")

    def run_speech_recognition(self):
        print("웨이크 워드 감지 후 음성 인식을 시작합니다...")
        
        # 음성 인식
        speech_text = record_and_recognize_speech(self.update_status_signal)
        if not speech_text:
            return
        
        # 추천 장소 받기
        recommendations = get_recommendations(speech_text)
        if recommendations:
            convert_text_to_speech_and_modify(f"추천된 장소는 {recommendations}입니다.")

    def stop(self):
        self.is_running = False

# --------------------------------------------
# 4. PyQt5 GUI 정의
# --------------------------------------------

class WakewordApp(QWidget):
    def __init__(self):
        super().__init__()

        self.image_paths = {
            "waiting": 'images/default.png',
            "listening": 'images/default.png',
            "wakeword_detected": 'images/wakeword_detected.png',
            "processing": 'images/processing.png',
            "silent": 'images/error.png',
            "result": 'images/result.png'
        }

        # 모델 로드
        self.model = load_wakeword_model()

        # WakewordDetectionThread 설정
        self.wakeword_thread = WakewordDetectionThread(self.model)
        self.wakeword_thread.update_status_signal.connect(self.update_status)

        self.initUI()

    def initUI(self):
        self.setWindowTitle('Wakeword Detection System')

        # 이미지를 표시할 QLabel
        self.image_label = QLabel(self)
        self.image_label.setPixmap(QPixmap(self.image_paths["waiting"]).scaled(1080, 720))

        # 감지 시작 버튼
        self.start_button = QPushButton(' ', self)
        self.start_button.clicked.connect(self.start_wakeword_detection)

        # # 감지 종료 버튼
        # self.stop_button = QPushButton('Stop Wakeword Detection', self)
        # self.stop_button.clicked.connect(self.stop_wakeword_detection)

        # 레이아웃 설정
        layout = QVBoxLayout()
        layout.addWidget(self.image_label)
        layout.addWidget(self.start_button)
        # layout.addWidget(self.stop_button)

        self.setLayout(layout)
        self.show()

    def update_status(self, message, image_key):
        print(message)
        self.image_label.setPixmap(QPixmap(self.image_paths[image_key]).scaled(1080, 720))

    def start_wakeword_detection(self):
        print("실시간 웨이크 워드 감지를 시작합니다...")
        self.image_label.setPixmap(QPixmap(self.image_paths["listening"]).scaled(1080, 720))
        self.wakeword_thread.start()

    def stop_wakeword_detection(self):
        self.wakeword_thread.stop()
        self.wakeword_thread.wait()
        print("웨이크 워드 감지가 중지되었습니다.")
        self.image_label.setPixmap(QPixmap(self.image_paths["waiting"]).scaled(1080, 720))

# --------------------------------------------
# 5. 메인 실행 부분
# --------------------------------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = WakewordApp()
    sys.exit(app.exec_())
