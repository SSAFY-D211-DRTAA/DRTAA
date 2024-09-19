package com.d211.drtaa.global.service.jwt;

import com.d211.drtaa.domain.user.service.CustomUserDetailsService;
import com.d211.drtaa.global.service.redis.RedisService;
import com.d211.drtaa.global.util.jwt.JwtToken;
import com.d211.drtaa.global.util.jwt.JwtTokenProvider;
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
import org.springframework.security.core.userdetails.UserDetails;
import org.springframework.security.core.userdetails.UsernameNotFoundException;
import org.springframework.security.crypto.password.PasswordEncoder;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
@Slf4j
public class JwtTokenTokenServiceImpl implements JwtTokenService {

    private final PasswordEncoder passwordEncoder;
    private final AuthenticationManager authenticationManager;
    private final JwtTokenProvider jwtTokenProvider;
    private final CustomUserDetailsService userDetailsService;
    private final RedisService redisService;
    private final UserRepository userRepository;

    @Override
    @Transactional
    public JwtToken generateToken(String userName, String password, String userLogin) {
        // 사용자 정보 조회
        UserDetails userDetails = userDetailsService.loadUserByUsername(userName);

        // 인증 객체 생성
        Authentication authentication;
        
        if(userLogin == null) { // 만료되어 재발급인 경우
            // 비밀번호 검증
            if (!password.equals(userDetails.getPassword())) 
                throw new AuthenticationFailedException("비밀번호가 일치하지 않습니다.");
            
            authentication =  new UsernamePasswordAuthenticationToken(userName, password, userDetails.getAuthorities());
        } else { // 로그인인 경우
            authentication = authenticate(userName, password);
        }

        // 인증 정보를 기반으로 JWT 토큰 생성
        JwtToken jwtToken = jwtTokenProvider.generateToken(authentication);

        // 리프레시 토큰 저장 - DB
        saveRefreshToken(userName, jwtToken.getRefreshToken());

        // 리프레시 토큰 저장 - Redis
        redisService.saveRefreshToken(userName, jwtToken.getRefreshToken());

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
        User user = userRepository.findByUserProviderId(userName)
                .orElseThrow(() -> new EntityNotFoundException("해당 userName으로 사용자를 찾을 수 없습니다."));

        // 기존 객체의 리프레시 토큰만 업데이트
        user.setUserRefreshToken(newRefreshToken);

        // 업데이트한 회원 저장
        userRepository.save(user);
    }

    @Override
    public String getRefreshToken(String userName) {
        User user = userRepository.findByUserProviderId(userName)
                .orElseThrow(() -> new UsernameNotFoundException("해당 userProviderId으로 찾을 수 있는 회원이 없습니다."));

        // DB 사용
//        return userRepository.findUserRefreshTokenByUserProviderId(userName)
//                .orElseThrow(() -> new EntityNotFoundException("해당 refreshToken으로 사용자를 찾을 수 없습니다."));
        // Redis 사용
        return redisService.getRefreshToken(userName);
    }
}
