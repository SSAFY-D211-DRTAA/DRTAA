package com.d211.drtaa.global.service.fcm;

import com.d211.drtaa.domain.user.entity.User;
import com.d211.drtaa.domain.user.repository.UserRepository;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.security.core.userdetails.UsernameNotFoundException;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
@Log4j2
public class FcmServiceImpl implements FcmService {

    private final UserRepository userRepository;

    @Override
    public String getToken(String userProviderId, String token) {
        // 사용자 조회
        User user = userRepository.findByUserProviderId(userProviderId)
                .orElseThrow(() -> new UsernameNotFoundException("해당 userProviderId의 맞는 회원을 찾을 수 없습니다."));
        log.info("userId: {}", user.getUserId());
        log.info("token: {}", token);

        // fcm 토큰 저장
        user.setFcmToken(token);
        userRepository.save(user);

        return "토큰이 성공적으로 저장되었습니다.";
    }

}
