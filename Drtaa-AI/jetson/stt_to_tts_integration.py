import speech_recognition as sr
from gtts import gTTS
import librosa
import soundfile as sf
import os
import requests
from pydub import AudioSegment
import re
import platform

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

# MP3 파일을 로드하여 WAV로 변환
def convert_mp3_to_wav(mp3_file, wav_file):
    audio = AudioSegment.from_mp3(mp3_file)
    audio.export(wav_file, format="wav")

def convert_text_to_speech_and_modify(text):
    # 텍스트를 음성으로 변환
    tts = gTTS(text=text, lang='ko')
    tts.save("output/output.mp3")

    # MP3를 WAV로 변환
    convert_mp3_to_wav("output/output.mp3", "output/output.wav")

    # 변환된 WAV 파일 로드
    y, sr = librosa.load("output/output.wav")

    # # 피치 및 속도 변형
    # y_shifted = librosa.effects.pitch_shift(y, sr=sr, n_steps=1)
    # y_speed = librosa.effects.time_stretch(y_shifted, rate=1.0)

    # 변형된 음성 파일 저장
    sf.write("output/output.wav", y, sr)

    # 변환된 음성 파일 실행
    open_audio_file("output/output.wav")

def convert_mp3_to_wav(mp3_file, wav_file):
    audio = AudioSegment.from_mp3(mp3_file)
    audio.export(wav_file, format="wav")

def get_recommendations(user_request):
    # 요청할 URL
    url = "https://j11d211.p.ssafy.io/ai-api-server/recommend_place"
    
    # 요청에 포함할 데이터 (JSON 형식)
    data = {
        "user_request": user_request
    }

    try:
        # POST 요청 보내기
        response = requests.post(url, json=data)
        
        # 응답 상태 확인
        if response.status_code == 200:
            # 응답에서 JSON 데이터 추출
            result = response.json()

            # 추천 장소와 기타 정보 추출
            recommendation = result.get("recommendation", "추천 정보 없음")
            write_response = result.get("write_place_response", "응답 없음")

            # 출력
            print(f"추천 장소: {recommendation}")
            print(f"응답 메시지: {write_response}")

            match = re.search(r"추천 이유:\s*(.+)", recommendation)
            if match:
                return match.group(1).strip()  # 추천 이유만 반환
            else:
                return "추천 이유를 찾을 수 없습니다."

            return recommendation
        else:
            print(f"오류 발생: {response.status_code}")
            return None, None  # None 반환 시 두 값 모두 None으로 처리
    except Exception as e:
        print(f"API 호출 중 오류 발생: {e}")
        return None, None  # 예외 발생 시 None 반환


# 음성을 인식하여 텍스트로 변환
def record_and_recognize_speech():
    recognizer = sr.Recognizer()
    # Microphone 클래스에서 device_index를 설정
    with sr.Microphone(device_index=9) as source:  # Microphone with index 9: default
        print("말씀하세요...")
        recognizer.adjust_for_ambient_noise(source, duration=1)  # 배경 소음 조정
        audio = recognizer.listen(source, timeout=5, phrase_time_limit=5)
    try:
        text = recognizer.recognize_google(audio, language="ko-KR")
        print(f"인식된 텍스트: {text}")

        with open("output/stt.txt", 'w') as file:
            file.write(text)
        print("음성 데이터가 파일에 저장되었습니다.")
    except sr.UnknownValueError:
        print("음성을 이해할 수 없습니다.")
        return None
    except sr.RequestError:
        print("API 요청 중 문제가 발생했습니다.")
        return None
    return text

# 파일에서 텍스트를 읽어오는 함수
def read_command_from_file(file_path):
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            command = file.read().strip()
            return command
    except FileNotFoundError:
        print("파일을 찾을 수 없습니다.")
        return None

# 전체 플로우 실행 함수
def main():
    # 1. 음성 인식 및 텍스트 저장
    speech_text = record_and_recognize_speech()
    if not speech_text:
        return

    # 2. 파일에서 지역명 읽기
    command = read_command_from_file("output/stt.txt")
    if not command:
        return
    recommendations = get_recommendations(command)
    
    # 3. 추천된 장소를 음성으로 변환 및 수정
    convert_text_to_speech_and_modify(f"{recommendations}")

# 플로우 시작
if __name__ == "__main__":
    main()