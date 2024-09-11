package com.d211.drtaa.global.util.jwt;

import lombok.AllArgsConstructor;
import lombok.Getter;
import org.springframework.data.annotation.Id;
import org.springframework.data.redis.core.RedisHash;

@RedisHash(value = "refreshToken", timeToLive = 10)
@AllArgsConstructor
@Getter
public class RefreshToken {
    @Id
    private Long userId;
    private String refreshToken;
}
