package com.d211.drtaa.global.config.security;

import com.d211.drtaa.global.config.jwt.JwtAuthenticationFilter;
import com.d211.drtaa.global.util.jwt.JwtTokenProvider;
import lombok.RequiredArgsConstructor;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.security.config.annotation.web.builders.HttpSecurity;
import org.springframework.security.config.annotation.web.configuration.EnableWebSecurity;
import org.springframework.security.config.http.SessionCreationPolicy;
import org.springframework.security.web.SecurityFilterChain;
import org.springframework.security.web.authentication.UsernamePasswordAuthenticationFilter;

@Configuration // 이 클래스가 Spring의 Configuration 클래스를 정의함을 나타냄
@EnableWebSecurity // Spring Security를 활성화하는 어노테이션
@RequiredArgsConstructor // final 필드에 대해 생성자를 자동으로 생성해주는 Lombok 어노테이션
public class SecurityConfig {

    private final JwtTokenProvider jwtTokenProvider; // JWT 토큰 제공자

    @Bean // 이 메서드가 Spring Bean으로 관리되도록 설정
    public SecurityFilterChain filterChain(HttpSecurity http) throws Exception {
        http
                .httpBasic(httpConfig -> httpConfig.disable()) // 기본 인증을 비활성화 (HTTP Basic Authentication)
                .csrf(csrfConfig -> csrfConfig.disable()) // CSRF 보호를 비활성화 (Cross-Site Request Forgery)
                .sessionManagement(session -> session.sessionCreationPolicy(SessionCreationPolicy.STATELESS)) // 세션 관리를 Stateless로 설정 (서버에 세션을 저장하지 않음)
                .authorizeHttpRequests(auth -> auth
                        .requestMatchers("/swagger-ui/**").permitAll() // Swagger UI 접근 허용
                        .requestMatchers("/v3/api-docs/**").permitAll() // API 문서 접근 허용
                        .requestMatchers("/user/signup/**", "/user/login/**","**/**").permitAll() // 회원가입, 로그인 접근 허용
                        .anyRequest().authenticated() // 나머지 요청은 인증 필요
                )
                .addFilterBefore(new JwtAuthenticationFilter(jwtTokenProvider), UsernamePasswordAuthenticationFilter.class); // JWT 필터를 UsernamePasswordAuthenticationFilter 앞에 추가

        return http.build(); // 설정된 HttpSecurity 객체를 빌드하여 반환
    }
}
