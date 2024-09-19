package com.d211.drtaa.global.service.redis;

import java.time.Duration;

public interface RedisService {
    // Redis에 값을 추가하거나 수정
    int setValues(String key, String value);
    int setValues(String key, String value, Duration duration); // 만료 시간 설정 가능

    // Redis에서 값을 조회
    String getValue(String key);

    // Redis에서 값을 삭제
    int deleteValue(String key);
}

