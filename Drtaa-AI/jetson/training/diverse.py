import numpy as np
import librosa
import soundfile as sf

# 노이즈 추가 함수
def add_noise(audio, noise_factor=0.005):
    noise = np.random.randn(len(audio))
    augmented_audio = audio + noise_factor * noise
    return augmented_audio

# 시간 확장/축소 함수 (속도를 변경)
def time_stretch(audio, rate=1.0):
    if len(audio) < 2:  # 시간 확장을 위해 충분한 데이터가 필요합니다.
        raise ValueError("Audio data too short to apply time stretch.")
    return librosa.effects.time_stretch(audio, rate)

# 피치 변환 함수 (피치를 높이거나 낮춤)
def pitch_shift(audio, sr, n_steps=0):
    return librosa.effects.pitch_shift(audio, sr=sr, n_steps=n_steps)

# 음성 자르기 및 패딩
def crop_or_pad(audio, target_length=16000):
    if len(audio) > target_length:
        audio = audio[:target_length]
    elif len(audio) < target_length:
        audio = np.pad(audio, (0, target_length - len(audio)))
    return audio

# 여러 증강 기법을 적용하는 함수
def augment_audio(audio, sr=16000):
    augmented_audios = []

    # 1. 원본 오디오
    augmented_audios.append(audio)

    # 2. 노이즈 추가
    audio_with_noise = add_noise(audio)
    augmented_audios.append(audio_with_noise)

    # 3. 시간 확장 및 축소 (속도 변경)
    if len(audio) > 1:  # 충분한 데이터 길이 체크
        try:
            audio_speed_up = time_stretch(audio, rate=1.1)  # 10% 빠르게
            audio_slow_down = time_stretch(audio, rate=0.9)  # 10% 느리게
            augmented_audios.append(crop_or_pad(audio_speed_up, target_length=16000))
            augmented_audios.append(crop_or_pad(audio_slow_down, target_length=16000))
        except Exception as e:
            print(f"Time stretching failed: {e}")

    # 4. 피치 변환
    audio_pitch_up = pitch_shift(audio, sr=sr, n_steps=2)  # 피치 높이기
    audio_pitch_down = pitch_shift(audio, sr=sr, n_steps=-2)  # 피치 낮추기
    augmented_audios.append(audio_pitch_up)
    augmented_audios.append(audio_pitch_down)

    return augmented_audios

# 오디오 저장 함수
def save_augmented_audios(augmented_audios, sr=16000, base_filename='dataset/not_wakeword/not_wakeword'):
    for idx, audio in enumerate(augmented_audios):
        filename = f"{base_filename}_{idx+47}.wav"  # 파일명에 번호 추가
        sf.write(filename, audio, sr)
        print(f"Saved {filename}")

# 예시 사용
file_path = 'dataset/not_wakeword/not_wakeword_10.wav'
audio, sr = librosa.load(file_path, sr=16000)

# 증강된 오디오 생성
augmented_audios = augment_audio(audio, sr=sr)

# 오디오 저장
save_augmented_audios(augmented_audios, sr=sr)
