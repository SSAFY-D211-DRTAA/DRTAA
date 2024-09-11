package com.d211.drtaa.global.util.redis;

import lombok.RequiredArgsConstructor;
import org.springframework.data.redis.core.RedisTemplate;
import org.springframework.stereotype.Service;

import java.util.concurrent.TimeUnit;

@Service
@RequiredArgsConstructor
public class RedisUtils {

    // RedisTemplate을 사용해 Redis에 데이터를 저장하고 가져오는데 사용
    private final RedisTemplate<String, Object> redisTemplate;

    // 데이터를 Redis에 저장
    public void setData(String key, String value,Long expiredTime){
        // opsForValue() 메서드를 사용해 문자열(String) 형식의 값을 저장하고,
        // 만료 시간(expiredTime)은 밀리초 단위로 설정
        redisTemplate.opsForValue().set(key, value, expiredTime, TimeUnit.MILLISECONDS);
    }

    // Redis에서 데이터를 가져옴
    public String getData(String key){
        // opsForValue() 메서드를 사용해 key에 해당하는 값을 가져옴
        // 가져온 값은 String으로 캐스팅해 반환
        return (String) redisTemplate.opsForValue().get(key);
    }

    // Redis에서 데이터를 삭제
    public void deleteData(String key){
        // 해당 key의 데이터를 삭제
        redisTemplate.delete(key);
    }
}
