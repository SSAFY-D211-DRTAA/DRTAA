package com.d211.drtaa.global.service.redis;

import lombok.RequiredArgsConstructor;
import org.springframework.data.redis.core.RedisTemplate;
import org.springframework.data.redis.core.StringRedisTemplate;
import org.springframework.stereotype.Service;

import java.time.Duration;
import java.util.concurrent.TimeUnit;

@Service
@RequiredArgsConstructor
public class RedisServiceImpl implements RedisService {

    private final RedisTemplate<String, String> redisTemplate;
    private final long REFRESH_TOKEN_EXPIRE_TIME = 30 * 24 * 60 * 60 * 1000L; // 30일

    @Override
    public void saveRefreshToken(String userProviderId, String refreshToken) {
        redisTemplate.opsForValue().set(
                "RefreshToken:" + userProviderId,
                refreshToken,
                REFRESH_TOKEN_EXPIRE_TIME,
                TimeUnit.MILLISECONDS
        );
    }

    @Override
    public String getRefreshToken(String userProviderId) {
        return redisTemplate.opsForValue().get("RefreshToken:" + userProviderId);
    }

    @Override
    public void deleteRefreshToken(String userProviderId) {
        redisTemplate.delete("RefreshToken:" + userProviderId);
    }
}
