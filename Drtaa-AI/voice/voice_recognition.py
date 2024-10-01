import speech_recognition as sr

recognizer = sr.Recognizer()

# 음성 녹음
with sr.Microphone() as source:
    print("말씀하세요...")
    audio = recognizer.listen(source)

# 녹음한 음성을 텍스트로 변환
try:
    text = recognizer.recognize_google(audio, language="ko-KR")
    print(f"인식된 텍스트: {text}")

    # 파일에 텍스트 저장
    with open("stt.txt", 'w') as file:
        file.write(text)
    print("음성 데이터가 파일에 저장되었습니다.")
except sr.UnknownValueError:
    print("음성을 이해할 수 없습니다.")
except sr.RequestError:
    print("API 요청 중 문제가 발생했습니다.")