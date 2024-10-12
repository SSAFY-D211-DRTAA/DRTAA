import sys
import pyaudio
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import load_model
import librosa
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

# --------------------------------------------
# 1. 모델 불러오기
# --------------------------------------------

model_path = 'one_depth_wakeword_model.h5'

def load_wakeword_model():
    model = load_model(model_path)
    return model

# --------------------------------------------
# 2. PyQt5 GUI 정의
# --------------------------------------------

class WakewordApp(QWidget):
    def __init__(self):
        super().__init__()

        # 이미지 파일 경로 설정 (상태에 따른 이미지)
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

        # PyAudio 설정
        self.p = pyaudio.PyAudio()

        self.initUI()

    def initUI(self):
        self.setWindowTitle('Wakeword Detection System')

        # 이미지를 표시할 QLabel
        self.image_label = QLabel(self)
        self.image_label.setPixmap(QPixmap(self.image_paths["waiting"]).scaled(1080, 720))

        # 상태 메시지를 표시할 QLabel
        self.status_label = QLabel('Ready to detect wakewords...', self)
        self.status_label.setStyleSheet("font-size: 16px;")

        # 감지 시작 버튼
        self.start_button = QPushButton('Start Wakeword Detection', self)
        self.start_button.clicked.connect(self.start_wakeword_detection)

        # 레이아웃 설정
        layout = QVBoxLayout()
        layout.addWidget(self.image_label)
        layout.addWidget(self.status_label)
        layout.addWidget(self.start_button)

        self.setLayout(layout)
        self.show()

    def update_status(self, message, image_key):
        """Update the status label with a new message and change the image."""
        self.status_label.setText(message)
        self.image_label.setPixmap(QPixmap(self.image_paths[image_key]).scaled(200, 200))

    def start_wakeword_detection(self):
        self.update_status("실시간 웨이크 워드 감지를 시작합니다...", "listening")
        self.listen_for_wakeword()

    def listen_for_wakeword(self):
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 16000
        CHUNK = 1024
        RECORD_SECONDS = 3
        SILENCE_THRESHOLD = 0.01

        stream = self.p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)

        try:
            while True:
                frames = []
                for _ in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
                    data = stream.read(CHUNK)
                    frames.append(data)

                audio_data = b''.join(frames)
                audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32)

                if np.max(np.abs(audio_np)) < SILENCE_THRESHOLD:
                    self.update_status("무음 상태입니다. 계속 대기 중...", "silent")
                    continue

                self.process_audio_data(audio_data)
        except KeyboardInterrupt:
            self.update_status("종료합니다.", "waiting")
            stream.stop_stream()
            stream.close()

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
            self.update_status(f"웨이크 워드 감지됨! {prediction[0]}", "wakeword_detected")
            self.run_speech_recognition()
        else:
            self.update_status(f"웨이크 워드 감지되지 않음. {prediction[0]}", "listening")

    def run_speech_recognition(self):
        recognizer = sr.Recognizer()
        self.update_status("음성 인식 중...", "wakeword_detected")

        with sr.Microphone() as source:
            print("말씀하세요...")
            recognizer.adjust_for_ambient_noise(source, duration=1)
            audio = recognizer.listen(source, timeout=3, phrase_time_limit=3)

        try:
            text = recognizer.recognize_google(audio, language="ko-KR")
            self.update_status(f"인식된 텍스트: {text}", "processing")
            with open("output/stt.txt", 'w') as file:
                file.write(text)
            print("음성 데이터가 파일에 저장되었습니다.")
            self.get_recommendations(text)
        except sr.UnknownValueError:
            self.update_status("음성을 이해할 수 없습니다.", "silent")
        except sr.RequestError:
            self.update_status("API 요청 중 문제가 발생했습니다.", "silent")

    def get_recommendations(self, user_request):
        # 요청할 URL
        url = "https://j11d211.p.ssafy.io/ai-api-server/recommend_place"
        data = {"user_request": user_request}

        try:
            response = requests.post(url, json=data)
            if response.status_code == 200:
                result = response.json()
                recommendation = result.get("recommendation", "추천 정보 없음")
                self.update_status(f"추천 장소: {recommendation}", "result")
                self.convert_text_to_speech_and_modify(recommendation)
            else:
                self.update_status(f"오류 발생: {response.status_code}", "silent")
        except Exception as e:
            self.update_status(f"API 호출 중 오류 발생: {e}", "silent")

    def convert_text_to_speech_and_modify(self, text):
        """Convert text to speech and play it back."""
        tts = gTTS(text=text, lang='ko')
        tts.save("output/output.mp3")
        self.convert_mp3_to_wav("output/output.mp3", "output/output.wav")

        # 변환된 음성 파일 실행
        self.open_audio_file("output/output.wav")

    def convert_mp3_to_wav(self, mp3_file, wav_file):
        audio = AudioSegment.from_mp3(mp3_file)
        audio.export(wav_file, format="wav")

    def open_audio_file(self, file_path):
        system_name = platform.system()

        if system_name == "Windows":
            os.system(f"start {file_path}")
        elif system_name == "Darwin":  # macOS
            os.system(f"open {file_path}")
        elif system_name == "Linux":
            os.system(f"xdg-open {file_path}")
        else:
            print(f"지원하지 않는 운영체제입니다: {system_name}")

# --------------------------------------------
# 3. 메인 실행 부분
# --------------------------------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = WakewordApp()
    sys.exit(app.exec_())
