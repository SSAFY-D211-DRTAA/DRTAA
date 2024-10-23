# Drtaa, 다타
![main](https://github.com/user-attachments/assets/5d9e3fad-8d61-49f2-b6ff-4ebc92bb81d2)

<br>

# 프로젝트 개요
### 🚗 개발 기간
2024.08.19 ~ 2024.10.11 (7주)
### 🚗 팀원 소개
| <div align="center">**이름**</div> | <div align="center">**역할**</div> |
| :---: | :---: |
| 이정재 | 팀장, AI, Auto Control |
| 김민규 | Android, AI |
| 윤가희 | BackEnd, Infra |
| 이재영 | Auto Control |
| 이현진 | Android |
| 최승준 | Android, BackEnd, Infra |
### 🚗 기획 배경
기존의 렌트카 및 이동 수단 서비스는 여러 가지 불편함을 수반합니다.<br>
특히, **운전 면허**가 필요하거나 **주차 문제**로 인해 목적지에서 주차 공간을 찾는 데 스트레스를 받는 경우가 많습니다.<br>
이러한 불편함을 해결하고, 더 나은 여행 경험을 제공하기 위해 **자율주행 기반 렌트 및 여행 서비스**를 기획하게 되었습니다.
### 🚗 목표
자율주행 기술을 활용해 운전 스트레스 없이 **어디서든 쉽게 이용할 수 있는 렌트카 서비스**를 제공하는 것입니다.<br>
고객이 **여행에 온전히 집중**할 수 있는 편리한 환경을 조성하고자 합니다.

<br>

# 서비스 기능 소개
### 🚗 로그인 및 메인 화면
| <div align="center">**로그인 화면**</div> | <div align="center">**로그인 동작**</div> |
| :---: | :---: |
| ![login](https://github.com/user-attachments/assets/1f3fa2c4-022c-49eb-b21b-d4903348a314) |  |


<br>

| <div align="center">**메인 화면**</div> | <div align="center">**메인 화면 동작**</div> |
| :---: | :---: |
| ![home](https://github.com/user-attachments/assets/46b0e684-a629-42d5-a5d6-01395c1d5501) |  |


<br>

### 🚗 렌트 요청 및 결제
| <div align="center">**렌트 요청 화면**</div> | <div align="center">**렌트 요청 동작**</div> |
| :---: | :---: |
| ![rent](https://github.com/user-attachments/assets/e71bb58e-5599-4b78-bbbc-0402884d0e10) |  |


<br>

### 🚗 여행 일정 관리
| <div align="center">**여행 일정 화면**</div> | <div align="center">**여행 일정 동작**</div> |
| :---: | :---: |
| ![여행 일정 화면](/Readme-Img/최종 녹화 - frame at 2m12s.jpg) |  |

<br>

### 🚗 렌트 호출
| <div align="center">**렌트 호출 화면**</div> | <div align="center">**렌트 호출 동작**</div> |
| :---: | :---: |
| ![렌트 호출 화면](/Readme-Img/최종 녹화 - frame at 2m34s.jpg) |  |

| <div align="center">**차량 주차 화면**</div> |
| :---: |
|  |

<br>

### 🚗 렌트 탑승 및 하차
| <div align="center">**렌트 탑승 화면**</div> | <div align="center">**렌트 탑승 동작**</div> |
| :---: | :---: |
| ![렌트 탑승 화면.jpg](/Readme-Img/렌트 탑승 화면.jpg) |  |

<br>

| <div align="center">**렌트 하차 화면**</div> | <div align="center">**렌트 하차 동작**</div> |
| :---: | :---: |
| ![렌트 하차 화면.jpg](/Readme-Img/렌트 하차 화면.jpg) |  |

| <div align="center">**차량 주차 화면**</div> | <div align="center">**차량 배회 화면**</div> |
| :---: | :---: |
|  |  |

<br>

### 🚗 관광지 추천
| <div align="center">**관광지 추천 화면**</div> | <div align="center">**관광지 추천 동작**</div> |
| :---: | :---: |
| ![image.png](/Readme-Img/image.png) |  |

<br>

### 🚗 렌트 알림
| <div align="center">**렌트 3일 전 알림 동작**</div> | <div align="center">**렌트 하루 전 알림 동작**</div> |
| :---: | :---: |
|  |  |

<br>

| <div align="center">**렌트 시작 알림 동작**</div> | <div align="center">**렌트 종료 알림 동작**</div> |
| :---: | :---: |
|  |  |

<br>

# 기술 스택
<details>
  <summary><h3>🚗 Auto Control</h3></summary>
  
  - Autonomous Driving
    - Ubuntu `24.02`
    - ROS `noetic`
    - Docker
  - Simulator
    - MORAI-Sim `22.R2.1`
</details>
<details>
  <summary><h3>🚗 Android</h3></summary>
  
  - Android Studio `2024.1.1`
- AndroidX
    - android-gradlePlugin: `com.android.tools.build:gradle:8.5.0`
    - androidx-core-ktx: `androidx.core:core-ktx:1.13.1`
    - androidx-appcompat: `androidx.appcompat:appcompat:1.7.0`
    - material: `com.google.android.material:material:1.12.0`
    - androidx-activity: `androidx.activity:activity:1.9.1`
    - androidx-constraintlayout: `androidx.constraintlayout:constraintlayout:2.1.4`
    - lifecycle-runtime-ktx: `androidx.lifecycle:lifecycle-runtime-ktx:2.8.3`
    - lifecycle-extensions: `androidx.lifecycle:lifecycle-extensions:2.2.0`
- 네이버 지도
    - map-sdk: `com.naver.maps:map-sdk:3.19.1`
- 정적 코드 분석
    - detekt-gradle: `io.gitlab.arturbosch.detekt:detekt-gradle-plugin:1.23.5`
    - detekt-formatting: `io.gitlab.arturbosch.detekt:detekt-formatting:1.23.5`
- 결제
    - bootpay: `io.github.bootpay:android:4.4.3`
- Kotlin, Coroutine
    - kotlin-gradlePlugin: `org.jetbrains.kotlin:kotlin-gradle-plugin:1.9.22`
    - coroutines-core: `org.jetbrains.kotlinx:kotlinx-coroutines-core:1.7.3`
    - coroutines-android: `org.jetbrains.kotlinx:kotlinx-coroutines-android:1.7.3`
    - kotlinx-coroutines-test: `org.jetbrains.kotlinx:kotlinx-coroutines-test:1.7.1`
- Android Jetpack
    - navigation-fragment-ktx: `androidx.navigation:navigation-fragment-ktx:2.7.7`
    - navigation-ui-ktx: `androidx.navigation:navigation-ui-ktx:2.7.7`
    - androidx-datastore-preferences-core: `androidx.datastore:datastore-preferences-core:1.1.1`
    - datastore-preferences: `androidx.datastore:datastore-preferences:1.1.1`
    - androidx-paging-runtime-ktx: `androidx.paging:paging-runtime-ktx:3.3.2`
- Logging
    - timber: `com.jakewharton.timber:timber:5.0.1`
- Sign
    - androidx-credentials: `androidx.credentials:credentials:1.2.2`
    - androidx-credentials-play-services-auth: `androidx.credentials:credentials-play-services-auth:1.2.2`
    - googleid: `com.google.android.libraries.identity.googleid:googleid:1.1.1`
    - naver-oauth: `com.navercorp.nid:oauth:5.10.0`
- Serialization
    - kotlinx-serialization-core: `org.jetbrains.kotlinx:kotlinx-serialization-core:1.6.2`
    - kotlinx-serialization-json: `org.jetbrains.kotlinx:kotlinx-serialization-json:1.6.2`
    - gson: `com.google.code.gson:gson:2.10.1`
- Testing
    - robolectric: `org.robolectric:robolectric:4.10.3`
    - mockk: `io.mockk:mockk:1.13.3`
    - junit: `junit:junit:4.13.2`
    - androidx-junit: `androidx.test.ext:junit:1.2.1`
    - androidx-espresso-core: `androidx.test.espresso:espresso-core:3.6.1`
- Hilt
    - hilt-android-compiler: `com.google.dagger:hilt-android-compiler:2.51.1`
    - hilt-android-testing: `com.google.dagger:hilt-android-testing:2.51.1`
    - hilt-android: `com.google.dagger:hilt-android:2.51.1`
    - hilt-compiler: `com.google.dagger:hilt-compiler:2.51.1`
    - androidx-hilt-navigation-fragment: `androidx.hilt:hilt-navigation-fragment:1.2.0`
    - androidx-hilt-compiler: `androidx.hilt:hilt-compiler:1.2.0`
- Network
    - okhttp: `com.squareup.okhttp3:okhttp:4.12.0`
    - okhttp-bom: `com.squareup.okhttp3:okhttp-bom:4.12.0`
    - okhttp-loggingInterceptor: `com.squareup.okhttp3:logging-interceptor:4.12.0`
    - retrofit: `com.squareup.retrofit2:retrofit:2.11.0`
    - retrofit-converter-gson: `com.squareup.retrofit2:converter-gson:2.11.0`
    - retrofit-converter-sclars: `com.squareup.retrofit2:converter-scalars:2.11.0`
    - retrofit-converter-kotlinxSerialization: `com.squareup.retrofit2:converter-kotlinx-serialization:2.11.0`
    - hivemq-mqtt-client: `com.hivemq:hivemq-mqtt-client:1.3.3`
- Media
    - glide: `com.github.bumptech.glide:glide:4.15.0`
    - lottie: `com.airbnb.android:lottie:6.5.0`
    - circleimageview: `de.hdodenhof:circleimageview:3.1.0`
    - material-calendarview: `com.github.prolificinteractive:material-calendarview:2.0.0`
    - threetenabp: `com.jakewharton.threetenabp:threetenabp:1.2.1`
- PlayService
    - play-services-basement: `com.google.android.gms:play-services-basement:18.4.0`
    - play-services-location: `com.google.android.gms:play-services-location:21.3.0`
- Permission
    - ted-permission-normal: `io.github.ParkSangGwon:tedpermission-normal:3.3.0`
    - ted-permission-coroutine: `io.github.ParkSangGwon:tedpermission-coroutine:3.3.0`
- QR
    - zxing-android-embedded: `com.journeyapps:zxing-android-embedded:4.3.0`
- Firebase
    - firebase-bom: `com.google.firebase:firebase-bom:33.3.0`
    - firebase-messaging-ktx: `com.google.firebase:firebase-messaging-ktx:24.0.1`
</details>
<details>
  <summary><h3>🚗 BackEnd</h3></summary>
  
  - IntelliJ `2024.2.2`
- Java OpenJDK `17.0.12`
- Spring Boot `3.3.3`
- Spring Security `6.3.3`
- Spring Batch `5.1.2`
- Spring Data JPA `3.3.3`
- JWT `0.11.5`
- WebSocket `6.1.13`
- MQTT `5.5.0`
- Firebase-Admin `9.3.0`
- Lombok `1.18.34`
- Gradle `8.8.0`
- Swagger `3.0.0`
</details>
<details>
  <summary><h3>🚗 AI</h3></summary>
  
  - Tensorflow `2.17.0`
- SciKit-Learn `1.5.2`
- SciPy `1.58.2`
- Docker
- Pyaudio - SpeechRecognizer `0.2.11`
- Pydub `0.25.1`
- Langchain `0.3.3`
- Langchain-OpenAI `0.2.2`
- Librosa `0.10.2.post1`
- boto3 `1.35.36`
- Flask `3.0.3`
- Firebase-Admin `6.5.0`
</details>
<details>
  <summary><h3>🚗 Database</h3></summary>
  
  - MySQL `8.0.38`
- Redis `7.4.0`
- AWS S3 Bucket Cloud
- Firebase
</details>
<details>
  <summary><h3>🚗 Infra(CI/CD)</h3></summary>
  
  - AWS EC2 `20.04.6`
- Docker `27.1.1`
- Docker Compose `2.20.2`
- Nginx `1.18.0`
- Jenkins `2.475`
- GitLab Runner `16.0.2`
</details>
<details>
  <summary><h3>🚗 Tools</h3></summary>
  
  - Jira
- GitLab
- Notion
- Matter Most
</details>

<br>

# 프로젝트 산출물
### 🚗 와이어 프레임
![image](/uploads/e249db762e9f49ba0823c1619f611098/image.png)
### 🚗 화면 디자인
![image](/uploads/b79f641cd849fbfafbba0b7316f0ca51/image.png)
### 🚗 API 명세서
- Notion
![image](/uploads/476db17970bb18aafd446928d4f957a3/image.png)
- Swagger
![image](/uploads/03cc993abdd3a1ec0393f9d1dd6290c4/image.png)
### 🚗 시스템 아키텍처
![image](/uploads/059b91229ffa5fffb92e925ab6526f70/image.png)
### 🚗 통신 아키텍처
![image](/uploads/e1600a3670a64d4a0b41a37f235e48f0/image.png)
### 🚗 ERD
![image](/uploads/2d922618efa945d64d0b5ad8764f44a5/image.png)
### 🚗 Android Module
![image](/uploads/e18946e9a853ea90cf17c44d8fa5aeea/image.png)
