package com.d211.drtaa.user.service;

import com.d211.drtaa.config.jwt.JwtToken;
import com.d211.drtaa.user.dto.request.FormLoginRequestDTO;
import com.d211.drtaa.user.dto.response.UserInfoResponseDTO;

public interface UserService {
    // LoginRequestDTO를 사용해 폼 로그인 실행
    JwtToken login(FormLoginRequestDTO request);

    // userEmail을 사용해 회원 정보 조회
    UserInfoResponseDTO info(String userEmail);

    // userEmail을 사용해 회원 삭제
    void delete(String userEmail);

}
