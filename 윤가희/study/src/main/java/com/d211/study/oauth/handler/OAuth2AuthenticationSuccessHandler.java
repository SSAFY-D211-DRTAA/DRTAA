package com.d211.study.oauth.handler;

import com.d211.study.config.jwt.JwtToken;
import com.d211.study.config.jwt.JwtTokenProvider;
import com.d211.study.domain.Member;
import com.d211.study.oauth.HttpCookieOAuth2AuthorizationRequestRepository;
import com.d211.study.oauth.service.OAuth2UserPrincipal;
import com.d211.study.oauth.user.OAuth2Provider;
import com.d211.study.oauth.user.OAuth2UserUnlinkManager;
import com.d211.study.oauth.util.CookieUtils;
import com.d211.study.repository.MemberRepository;
import com.d211.study.service.MemberService;
import jakarta.servlet.http.Cookie;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.security.core.Authentication;
import org.springframework.security.web.authentication.SimpleUrlAuthenticationSuccessHandler;
import org.springframework.stereotype.Component;
import org.springframework.web.util.UriComponentsBuilder;

import static com.d211.study.oauth.HttpCookieOAuth2AuthorizationRequestRepository.REDIRECT_URI_PARAM_COOKIE_NAME;
import static com.d211.study.oauth.HttpCookieOAuth2AuthorizationRequestRepository.MODE_PARAM_COOKIE_NAME;


import java.io.IOException;
import java.util.Optional;

@Slf4j
@RequiredArgsConstructor
@Component
public class OAuth2AuthenticationSuccessHandler extends SimpleUrlAuthenticationSuccessHandler {

    private final HttpCookieOAuth2AuthorizationRequestRepository httpCookieOAuth2AuthorizationRequestRepository;
    private final OAuth2UserUnlinkManager oAuth2UserUnlinkManager;
    private final JwtTokenProvider tokenProvider;
    private final MemberRepository memberRepository;


    @Override
    public void onAuthenticationSuccess(HttpServletRequest request, HttpServletResponse response,
                                        Authentication authentication) throws IOException {

        String targetUrl;

        targetUrl = determineTargetUrl(request, response, authentication);

        if (response.isCommitted()) {
            logger.debug("Response has already been committed. Unable to redirect to " + targetUrl);
            return;
        }

        clearAuthenticationAttributes(request, response);
        getRedirectStrategy().sendRedirect(request, response, targetUrl);
    }

    protected String determineTargetUrl(HttpServletRequest request, HttpServletResponse response,
                                        Authentication authentication) {

        Optional<String> redirectUri = CookieUtils.getCookie(request, REDIRECT_URI_PARAM_COOKIE_NAME)
                .map(Cookie::getValue);

        String targetUrl = redirectUri.orElse(getDefaultTargetUrl());

        String mode = CookieUtils.getCookie(request, MODE_PARAM_COOKIE_NAME)
                .map(Cookie::getValue)
                .orElse("");

        OAuth2UserPrincipal principal = getOAuth2UserPrincipal(authentication);

        if (principal == null) {
            return UriComponentsBuilder.fromUriString(targetUrl)
                    .queryParam("error", "Login failed")
                    .build().toUriString();
        }

        if ("login".equalsIgnoreCase(mode)) {
            // 사용자 정보 저장 로직
            String email = principal.getUserInfo().getEmail();
            String username = principal.getUserInfo().getName();
            String nickname = principal.getUserInfo().getNickname();
            String accessToken = principal.getUserInfo().getAccessToken();
            
            // 사용자 정보 로그 찍기
            log.info("email={}, name={}, nickname={}, accessToken={}",
                    email,
                    username,
                    nickname,
                    accessToken
            );

            // 토큰 발급
            JwtToken tokens = tokenProvider.generateToken(authentication);

            // 사용자 정보 및 리프레시 토큰 저장
            saveOrUpdateMember(email, username, "", tokens.getRefreshToken()); // 멤버 저장 또는 업데이트

            return UriComponentsBuilder.fromUriString(targetUrl)
                    .queryParam("access_token", tokens.getAccessToken())
                    .queryParam("refresh_token", tokens.getRefreshToken())
                    .build().toUriString();

        } else if ("unlink".equalsIgnoreCase(mode)) {

            String accessToken = principal.getUserInfo().getAccessToken();
            OAuth2Provider provider = principal.getUserInfo().getProvider();

            // DB 삭제 및 리프레시 토큰 삭제
            deleteMemberByEmail(principal.getUserInfo().getEmail()); // 멤버 삭제
            oAuth2UserUnlinkManager.unlink(provider, accessToken);

            return UriComponentsBuilder.fromUriString(targetUrl)
                    .build().toUriString();
        }

        return UriComponentsBuilder.fromUriString(targetUrl)
                .queryParam("error", "Login failed")
                .build().toUriString();
    }

    private OAuth2UserPrincipal getOAuth2UserPrincipal(Authentication authentication) {
        Object principal = authentication.getPrincipal();

        if (principal instanceof OAuth2UserPrincipal) {
            return (OAuth2UserPrincipal) principal;
        }
        return null;
    }

    protected void clearAuthenticationAttributes(HttpServletRequest request, HttpServletResponse response) {
        super.clearAuthenticationAttributes(request);
        httpCookieOAuth2AuthorizationRequestRepository.removeAuthorizationRequestCookies(request, response);
    }

    public Member saveOrUpdateMember(String email, String name, String password, String refreshToken) {
        // 이메일로 멤버 찾기
        Member member = memberRepository.findByMemberEmail(email)
                .orElse(Member.builder()
                        .memberEmail(email)
                        .memberNickname(name)
                        .memberPassword(password)  // 소셜 로그인일 경우 비밀번호는 빈 문자열로 설정
                        .memberRefreshToken(refreshToken)
                        .memberIsAdmin(false) // 기본값은 사용자로 설정
                        .build());

        // 리프레시 토큰 업데이트
        member.setMemberRefreshToken(refreshToken);

        return memberRepository.save(member); // 멤버 저장 또는 업데이트
    }

    public void deleteMemberByEmail(String email) {
        memberRepository.deleteByMemberEmail(email);
    }
}
