# Drtaa ROS
## 인지
### 1. velodyne LiDAR
- Velodyne LiDAR 센서를 사용하여 주변 환경의 3D 포인트 클라우드를 수집하고, 이를 통해 객체를 감지하고 클러스터링  
- DBSCAN 알고리즘을 사용하여 포인트 클라우드 데이터를 클러스터링
- 각 클러스터의 중심 좌표를 계산하여 PoseArray로 출력
### 2. GPS, IMU Sensor
- GPS와 IMU 센서를 통해 차량의 위치 및 자세 정보를 수집
- 겅로 계획 및 차량 제어에 활용

## 판단
### 1. Global Path Planning  : A* algorithm
- A* 알고리즘을 사용하여 시작 지점에서 목표 지점까지의 최적 경로를 계산
- MGeo Data (node, link)를 기반으로 생성
- 경로 평활화 기법을 적용하여 부드러운 경로 제공
- 목적지 도착 후 차량 주행 정지
- 새로운 목적지 갱신에 따른 새로운 전역 경로 생성
### 2. Local Path Planning
- 현재 위치와 전역 경로를 기반으로 생성
- 평활화 기법 적용으로 급격한 변화 없이 부드럽게 이동

## 제어
### 1. Pure Pursuit
- 차량의 조향각 계산
- 전방 경로 상의 목표지점을 추적하여 조향각 결정

### 2. Adaptive Cruise Control (ACC)
- 전방 차량과의 거리 및 속도 차이를 기반으로 차량 속도 조절
- 주변 객체를 감지하여 안전한 주행

### 3. Traffic Light Management
- 교통 신호등을 감지하여 좌회전 여부 및 직진 가능 여부를 판단
- Node의 정지선 데이터를 활용한 속도 조절
