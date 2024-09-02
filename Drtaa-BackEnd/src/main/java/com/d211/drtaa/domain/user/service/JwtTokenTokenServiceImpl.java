package com.d211.drtaa.domain.user.service;

import com.d211.drtaa.global.config.jwt.JwtToken;
import com.d211.drtaa.global.config.jwt.JwtTokenProvider;
import com.d211.drtaa.global.exception.auth.AuthenticationFailedException;
import com.d211.drtaa.domain.user.entity.User;
import com.d211.drtaa.domain.user.repository.UserRepository;
import jakarta.persistence.EntityNotFoundException;
import jakarta.transaction.Transactional;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.security.authentication.AuthenticationManager;
import org.springframework.security.authentication.UsernamePasswordAuthenticationToken;
import org.springframework.security.core.Authentication;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
@Slf4j
public class JwtTokenTokenServiceImpl implements JwtTokenService {

    private final AuthenticationManager authenticationManager;
    private final JwtTokenProvider jwtTokenProvider;
    private final UserRepository userRepository;

    @Override
    @Transactional
    public JwtToken generateToken(String userName, String password) {
        // userName를 기반으로 Authentication 객체 생성
        Authentication authentication = authenticate(userName, password);
        // 인증 정보를 기반으로 JWT 토큰 생성
        JwtToken jwtToken = jwtTokenProvider.generateTokenForFormLogin(authentication);
        // 리프레시 토큰 저장
        saveRefreshToken(userName, jwtToken.getRefreshToken());

        return jwtToken;
    }

    @Override
    public Authentication authenticate(String userName, String password) {
        try {
            UsernamePasswordAuthenticationToken authenticationToken = new UsernamePasswordAuthenticationToken(userName, password);
            return authenticationManager.authenticate(authenticationToken);
        } catch (Exception e) {
            log.error("사용자 인증 실패: " + e.getMessage());
            throw new AuthenticationFailedException("사용자 인증에 실패했습니다.", e);
        }
    }

    @Override
    @Transactional
    public void saveRefreshToken(String userName, String newRefreshToken) {
        User user = userRepository.findByUserEmail(userName)
                .orElseThrow(() -> new EntityNotFoundException("해당 userName으로 사용자를 찾을 수 없습니다."));

        // 기존 객체의 리프레시 토큰만 업데이트
        user.setUserRefreshToken(newRefreshToken);

        // 업데이트한 회원 저장
        userRepository.save(user);
    }

    @Override
    public String getRefreshToken(String userName) {
        return userRepository.findUserRefreshTokenByUserEmail(userName)
                .orElseThrow(() -> new EntityNotFoundException("해당 refreshToken으로 사용자를 찾을 수 없습니다."));
    }
}
