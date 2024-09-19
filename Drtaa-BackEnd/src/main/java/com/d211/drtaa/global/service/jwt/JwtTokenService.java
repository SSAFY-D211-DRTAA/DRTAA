package com.d211.drtaa.global.service.jwt;

import com.d211.drtaa.global.util.jwt.JwtToken;
import org.springframework.security.core.Authentication;

public interface JwtTokenService {
    // AccessToken, RefreshToken 발급
    JwtToken generateToken(String userName, String password);

    // 사용자 정보 기반 인증
    Authentication authenticate(String userName, String password);

    // Refreshtoken 저장
    void saveRefreshToken(String userName, String newRefreshToken);

    // refreshToken 조회
    String getRefreshToken(String userName);
}
