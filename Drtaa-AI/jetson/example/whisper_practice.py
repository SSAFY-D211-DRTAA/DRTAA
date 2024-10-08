import pyaudio
import wave
import torchaudio
import whisper
import os

# 음성을 녹음하여 wav 파일로 저장하는 함수
def record_audio(output_file="input.wav", duration=5, rate=16000, chunk=1024):
    p = pyaudio.PyAudio()
    
    # 마이크에서 오디오 스트림 열기
    stream = p.open(format=pyaudio.paInt16,
                    channels=1,
                    rate=rate,
                    input=True,
                    frames_per_buffer=chunk)

    print("Recording...")
    frames = []

    for _ in range(0, int(rate / chunk * duration)):
        data = stream.read(chunk)
        frames.append(data)

    print("Finished recording.")

    # 스트림 종료
    stream.stop_stream()
    stream.close()
    p.terminate()

    # 오디오 데이터를 파일로 저장
    with wave.open(output_file, 'wb') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
        wf.setframerate(rate)
        wf.writeframes(b''.join(frames))

# Whisper 모델을 사용하여 음성을 텍스트로 변환하는 함수
def transcribe_audio(file_path, model_size="medium"):
    model = whisper.load_model(model_size)

    # 오디오 로드 및 패딩/트리밍
    audio = whisper.load_audio(file_path)
    audio = whisper.pad_or_trim(audio)

    # log-Mel 스펙트로그램 생성 및 모델 장치로 이동
    mel = whisper.log_mel_spectrogram(audio).to(model.device)

    # 음성을 텍스트로 변환 (언어 지정)
    options = whisper.DecodingOptions(language="ko")
    result = whisper.decode(model, mel, options)

    return result.text

# 음성 녹음 및 변환 실행
def main():
    # 녹음할 파일 이름과 녹음 시간
    output_file = "input.wav"
    duration = 5  # 녹음 시간(초)

    # 1. 음성을 녹음
    record_audio(output_file, duration=duration)

    # 2. 녹음된 음성을 Whisper 모델을 사용해 텍스트로 변환
    if os.path.exists(output_file):
        transcription = transcribe_audio(output_file)
        print(f"Transcribed text: {transcription}")
    else:
        print(f"{output_file} 파일이 존재하지 않습니다.")

if __name__ == "__main__":
    main()
