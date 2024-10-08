import os
import librosa
import numpy as np
import pandas as pd
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv1D, MaxPooling1D, Flatten, Dense
from sklearn.model_selection import train_test_split

# --------------------------------------------
# 1. 데이터 전처리 함수
# --------------------------------------------

def load_audio_file(file_path, target_sr=16000):
    audio, sr = librosa.load(file_path, sr=target_sr)
    # 1초(16,000 samples) 길이로 맞춤
    if len(audio) > target_sr:
        audio = audio[:target_sr]
    elif len(audio) < target_sr:
        audio = np.pad(audio, (0, target_sr - len(audio)))  # 부족한 부분 0으로 채움
    return audio

def load_dataset(csv_path, target_sr=16000):
    labels_df = pd.read_csv(csv_path)

    X = []
    y = []

    # 파일을 로드하고 라벨을 배열에 저장
    for index, row in labels_df.iterrows():
        file_path = row['filename']
        label = row['label']
        
        # 오디오 파일 로드 및 전처리
        audio_data = load_audio_file(file_path, target_sr)
        
        # 데이터를 배열에 추가
        X.append(audio_data)
        y.append(label)

    # NumPy 배열로 변환
    X = np.array(X)
    y = np.array(y)

    # CNN 모델에 맞는 형식으로 reshape (채널을 추가)
    X = X.reshape(X.shape[0], X.shape[1], 1)
    return X, y

# --------------------------------------------
# 2. 모델 설계 함수
# --------------------------------------------

def create_model(input_shape):
    model = Sequential()

    # # 1D Conv Layer
    # model.add(Conv1D(32, kernel_size=3, activation='relu', input_shape=input_shape))
    # model.add(MaxPooling1D(pool_size=2))

    # model.add(Conv1D(64, kernel_size=3, activation='relu'))
    # model.add(MaxPooling1D(pool_size=2))

    # model.add(Flatten())
    # model.add(Dense(64, activation='relu'))  # 밀집 레이어
    # model.add(Dense(1, activation='sigmoid'))  # 이진 분류 (웨이크 워드 여부)

    # 첫 번째 블록
    model.add(Conv1D(64, kernel_size=3, activation='relu', input_shape=input_shape))
    model.add(MaxPooling1D(pool_size=2))
    
    # 두 번째 블록
    model.add(Conv1D(128, kernel_size=3, activation='relu'))
    model.add(MaxPooling1D(pool_size=2))
    
    # 세 번째 블록
    model.add(Conv1D(256, kernel_size=3, activation='relu'))
    model.add(MaxPooling1D(pool_size=2))
    
    # Flatten and Dense
    model.add(Flatten())
    model.add(Dense(128, activation='relu'))
    model.add(Dense(1, activation='sigmoid'))  # 이진 분류

    # 모델 컴파일
    model.compile(optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])
    return model

# --------------------------------------------
# 3. 모델 훈련 및 평가
# --------------------------------------------

def train_model(X, y):
    # 훈련 데이터와 검증 데이터 분리 (80% 훈련, 20% 검증)
    X_train, X_val, y_train, y_val = train_test_split(X, y, test_size=0.2, random_state=42)

    # 모델 생성
    input_shape = (16000, 1)  # 1초 길이의 음성 데이터
    model = create_model(input_shape)

    # 모델 훈련
    model.fit(X_train, y_train, epochs=10, batch_size=32, validation_data=(X_val, y_val))

    # 모델 평가
    loss, accuracy = model.evaluate(X_val, y_val)
    print(f"검증 데이터셋에서의 정확도: {accuracy * 100:.2f}%")

    return model


def save_model(model, model_path='wakeword_model.h5'):
    # 모델을 파일로 저장
    model.save(model_path)
    print(f"모델이 {model_path}에 저장되었습니다.")


# --------------------------------------------
# 4. 실행 코드
# --------------------------------------------

if __name__ == "__main__":
    # CSV 파일 경로
    csv_path = 'dataset/labels.csv'

    # 데이터 로드 및 전처리
    print("데이터를 로드하고 전처리 중입니다...")
    X, y = load_dataset(csv_path)

    # 모델 훈련 및 평가
    print("모델을 훈련하고 있습니다...")
    model = train_model(X, y)
    print("모델 훈련 완료.")
    save_model(model)
