package com.d211.drtaa.global.config.redis;

import org.springframework.cache.CacheManager;
import org.springframework.cache.annotation.EnableCaching;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.data.redis.cache.RedisCacheConfiguration;
import org.springframework.data.redis.cache.RedisCacheManager;
import org.springframework.data.redis.connection.RedisConnectionFactory;
import org.springframework.data.redis.serializer.GenericJackson2JsonRedisSerializer;
import org.springframework.data.redis.serializer.RedisSerializationContext;
import org.springframework.data.redis.serializer.StringRedisSerializer;

import java.time.Duration;

@Configuration
@EnableCaching // Spring 캐싱 기능 활성화
public class RedisCacheConfig {

    @Bean
    public CacheManager rcm(RedisConnectionFactory cf) {
        // RedisCacheConfiguration 객체를 생성하여 기본 캐시 설정을 정의
        // 키와 값을 직렬화하는 방법, 캐시의 만료 시간을 설정
        RedisCacheConfiguration redisCacheConfiguration = RedisCacheConfiguration.defaultCacheConfig()
                // 키는 문자열로 직렬화
                .serializeKeysWith(RedisSerializationContext.SerializationPair.fromSerializer(new StringRedisSerializer()))
                // 값은 JSON 형식으로 직렬화
                .serializeValuesWith(RedisSerializationContext.SerializationPair.fromSerializer(new GenericJackson2JsonRedisSerializer()))
                // 캐시 데이터의 만료 시간을 3분으로 설정
                .entryTtl(Duration.ofMinutes(3L));

        // RedisConnectionFactory를 사용하여 Redis와의 연결을 설정하고,
        // redisCacheConfiguration을 기본 설정으로 사용
        return RedisCacheManager
                .RedisCacheManagerBuilder
                .fromConnectionFactory(cf)
                .cacheDefaults(redisCacheConfiguration)
                .build();
    }
}