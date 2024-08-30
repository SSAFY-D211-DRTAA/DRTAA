package com.d211.drtaa.user.service;

import com.d211.drtaa.config.jwt.JwtToken;
import com.d211.drtaa.user.dto.request.FormLoginRequestDTO;
import com.d211.drtaa.user.dto.response.UserInfoResponseDTO;

public interface UserService {
    // LoginRequestDTO를 사용한 폼 로그인
    JwtToken login(FormLoginRequestDTO request);

    // userEmail을 사용한 회원 정보 조회
    UserInfoResponseDTO info(String userEmail);

}
