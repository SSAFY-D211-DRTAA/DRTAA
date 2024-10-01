from gtts import gTTS
import librosa
import soundfile as sf
import os

# 변환할 텍스트 입력
text = "안녕하세요, 여기는 Jetson Orin Nano 환경에서 음성을 처리하는 예제입니다."

# 텍스트를 음성으로 변환
tts = gTTS(text=text, lang='ko')

# 파일로 저장
tts.save("output/output.mp3")

# 음성 파일 로드
y, sr = librosa.load("output/output.mp3")

# 피치 변형 (5반음)
y_shifted = librosa.effects.pitch_shift(y, sr=sr, n_steps=1)

# 속도 변환 (1.2배 빠르게)
y_speed = librosa.effects.time_stretch(y_shifted, rate=1.0)

# 변형된 파일 저장
sf.write("output/output.mp3", y_speed, sr)

# 변환된 음성 파일 실행
os.system("start output/output.mp3")