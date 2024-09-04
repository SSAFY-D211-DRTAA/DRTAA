package com.d211.drtaa.global.service.redis;

import lombok.RequiredArgsConstructor;
import org.springframework.data.redis.core.StringRedisTemplate;
import org.springframework.stereotype.Service;

import java.time.Duration;

@Service
@RequiredArgsConstructor
public class RedisServiceImpl implements RedisService {

    private final StringRedisTemplate stringRedisTemplate;

    public String getValue(String key) {
        return stringRedisTemplate.opsForValue().get(key);
    }

    public int setValues(String key, String value) {
        return setValues(key, value, null);
    }

    public int setValues(String key, String value, Duration duration) {
        try {
            if (duration == null) {
                stringRedisTemplate.opsForValue().set(key, value);
            } else {
                stringRedisTemplate.opsForValue().set(key, value, duration);
            }
            return 1; // 성공
        } catch (Exception e) {
            // 예외 처리
            System.out.println(e.getMessage());
            return 0; // 실패
        }
    }

    public int deleteValue(String key) {
        try {
            Boolean deleted = stringRedisTemplate.delete(key);
            return deleted != null && deleted ? 1 : 0; // 삭제 성공 여부
        } catch (Exception e) {
            // 예외 처리
            return 0; // 실패
        }
    }
}
