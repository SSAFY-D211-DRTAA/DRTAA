package com.d211.study.service;

import com.d211.study.config.jwt.JwtToken;
import com.d211.study.domain.Member;
import com.d211.study.dto.request.LoginRequest;
import com.d211.study.dto.response.UserInfoResponse;
import com.d211.study.repository.MemberRepository;
import lombok.RequiredArgsConstructor;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.security.authentication.BadCredentialsException;
import org.springframework.security.core.userdetails.UserDetails;
import org.springframework.security.core.userdetails.UsernameNotFoundException;
import org.springframework.security.crypto.password.PasswordEncoder;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
public class MemberService {

    @Autowired
    private PasswordEncoder passwordEncoder;
    private final JwtTokenService jwtTokenService;
    private final CustomUserDetailsService userDetailsService;
    private final MemberRepository memberRepository;

    public JwtToken login(LoginRequest request) {
        // 사용자 검증
        UserDetails userDetails = userDetailsService.loadUserByUsername(request.getMemberUsername());
        if (!passwordEncoder.matches(request.getMemberPassword(), userDetails.getPassword())) {
            throw new BadCredentialsException("유효하지 않은 비밀번호입니다.");
        }

        // 토큰 생성
        return jwtTokenService.generateToken(request.getMemberUsername(), request.getMemberPassword());
    }

    public UserInfoResponse info(String name) {
        // 사용자 정보 가져오기
        Member member = memberRepository.findByMemberUsername(name)
                .orElseThrow(() -> new UsernameNotFoundException("해당 회원을 찾을 수 없습니다."));

        UserInfoResponse userInfo = new UserInfoResponse(
                member.getMemberId(),
                member.getMemberUsername(),
                member.getMemberEmail(),
                member.getMemberPassword(),
                member.isMemberIsAdmin()
        );

        return userInfo;
    }
}
