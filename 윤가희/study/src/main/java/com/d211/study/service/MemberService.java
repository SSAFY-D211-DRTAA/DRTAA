package com.d211.study.service;

import com.d211.study.config.jwt.JwtToken;
import com.d211.study.dto.request.LoginRequest;
import lombok.RequiredArgsConstructor;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.security.authentication.BadCredentialsException;
import org.springframework.security.core.userdetails.UserDetails;
import org.springframework.security.crypto.password.PasswordEncoder;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
public class MemberService {

    private final JwtTokenService jwtTokenService;
    private final CustomUserDetailsService userDetailsService;

    @Autowired
    private PasswordEncoder passwordEncoder;

    public JwtToken login(LoginRequest request) {
        // 사용자 검증
        UserDetails userDetails = userDetailsService.loadUserByUsername(request.getMemberUsername());
        if (!passwordEncoder.matches(request.getMemberPassword(), userDetails.getPassword())) {
            throw new BadCredentialsException("유효하지 않은 비밀번호입니다.");
        }

        // 토큰 생성
        return jwtTokenService.generateToken(request.getMemberUsername(), request.getMemberPassword());
    }
}
