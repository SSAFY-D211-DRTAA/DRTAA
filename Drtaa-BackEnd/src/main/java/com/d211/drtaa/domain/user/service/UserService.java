package com.d211.drtaa.domain.user.service;

import com.d211.drtaa.domain.user.dto.request.SocialLoginRequestDTO;
import com.d211.drtaa.global.config.jwt.JwtToken;
import com.d211.drtaa.domain.user.dto.request.FormLoginRequestDTO;
import com.d211.drtaa.domain.user.dto.response.UserInfoResponseDTO;

public interface UserService {
    // LoginRequestDTO를 사용해 폼 로그인 실행
    JwtToken FormLogin(FormLoginRequestDTO request);

    // LoginRequestDTO를 사용해 소셜 로그인 실행
    JwtToken SocialLogin(SocialLoginRequestDTO request);

    // userEmail을 사용해 회원 정보 조회
    UserInfoResponseDTO info(String userEmail);

    // userEmail을 사용해 회원 삭제
    void delete(String userEmail);
}
