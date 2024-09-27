package com.d211.drtaa.domain.rent.entity.car;

public enum RentDrivingStatus {
    idling,     // 상태 없음
    calling,    // 호출
    driving,    // 주행
    parking,    // 주차
    waiting,    // 배회
    charging    // 충전
}
