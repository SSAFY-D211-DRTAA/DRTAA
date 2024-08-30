package com.d211.drtaa.config.security;

import lombok.RequiredArgsConstructor;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.security.config.annotation.web.builders.HttpSecurity;
import org.springframework.security.config.annotation.web.configuration.EnableWebSecurity;
import org.springframework.security.config.http.SessionCreationPolicy;
import org.springframework.security.web.SecurityFilterChain;
import org.springframework.security.web.authentication.UsernamePasswordAuthenticationFilter;

@Configuration
@EnableWebSecurity
@RequiredArgsConstructor
public class SecurityConfig {

//    private final CustomOAuth2UserService customOAuth2UserService;
//    private final OAuth2AuthenticationSuccessHandler oAuth2AuthenticationSuccessHandler;
//    private final OAuth2AuthenticationFailureHandler oAuth2AuthenticationFailureHandler;
//    private final HttpCookieOAuth2AuthorizationRequestRepository httpCookieOAuth2AuthorizationRequestRepository;
//    private final JwtTokenProvider jwtTokenProvider;

    @Bean
    public SecurityFilterChain filterChain(HttpSecurity http) throws Exception {
        http
                .httpBasic(httpConfig -> httpConfig.disable())
                .csrf(csrfConfig -> csrfConfig.disable())
                .sessionManagement(session -> session.sessionCreationPolicy(SessionCreationPolicy.STATELESS))
                .authorizeHttpRequests(auth -> auth
                        .requestMatchers("/user/signup", "/user/login").permitAll() // 회원가입, 로그인 허용
                        .requestMatchers("/api/public/**").permitAll() // 공용 API 접근 허용
                        .requestMatchers("/swagger-ui/**").permitAll() // Swagger UI 접근 허용
                        .requestMatchers("/v3/api-docs/**").permitAll() // API 문서 접근 허용
                        .anyRequest().authenticated()
                );
//                .sessionManagement(sessions -> sessions.sessionCreationPolicy(SessionCreationPolicy.STATELESS))
//                .oauth2Login(configure ->
//                        configure.authorizationEndpoint(config -> config.authorizationRequestRepository(httpCookieOAuth2AuthorizationRequestRepository))
//                                .userInfoEndpoint(config -> config.userService(customOAuth2UserService))
//                                .successHandler(oAuth2AuthenticationSuccessHandler)
//                                .failureHandler(oAuth2AuthenticationFailureHandler)
//                )
//                .addFilterBefore(new JwtAuthenticationFilter(jwtTokenProvider), UsernamePasswordAuthenticationFilter.class);

        return http.build();
    }

}
