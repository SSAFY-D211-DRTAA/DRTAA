# Drtaa-Android

### 기술 스택

- Coroutine, Flow
- Hilt(apply MultiModule), build-logic
- Android Jetpack - Navigation, Paging, ViewModel, DataBinding, DataStore, Safe Args
- OkHttp3, Retrofit2
- NaverMap API, Naver Geocoding API, Tmap API
- GoogleSignIn, NaverIdLogin
- Glide
- Timber
- Detekt

### 모듈 구조

- `:app` -> 다른 모듈들을 모두 받아서 빌드하는 메인 모듈
- `:core-data` -> 데이터 관련 로직 + 비즈니스 로직까지 모두 포함 (DB 작업, 네트워크 작업)
- `:core-network` -> 네트워크 관련 로직(api 호출)
- `:core-model` -> 전체에서 사용되는 데이터클래스, 인터페이스, 상수 같은 것들
- `:core-ui` -> Base코드들
- `:feature-*` -> 기능 별 모듈

![dartta_module_graph](Drtaa-Android/project.dot.png)