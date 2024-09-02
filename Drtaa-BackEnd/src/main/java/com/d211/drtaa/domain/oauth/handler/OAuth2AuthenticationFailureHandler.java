package com.d211.drtaa.domain.oauth.handler;

import com.d211.drtaa.domain.oauth.repository.HttpCookieOAuth2AuthorizationRequestRepository;
import com.d211.drtaa.domain.oauth.util.CookieUtils;
import jakarta.servlet.http.Cookie;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import lombok.RequiredArgsConstructor;
import org.springframework.security.core.AuthenticationException;
import org.springframework.security.web.authentication.SimpleUrlAuthenticationFailureHandler;
import org.springframework.stereotype.Component;
import org.springframework.web.util.UriComponentsBuilder;

import java.io.IOException;

// 클래스가 자동으로 의존성을 주입받도록 설정 (Lombok의 @RequiredArgsConstructor 사용)
@RequiredArgsConstructor
// Spring의 Component로 등록하여 Bean으로 관리되게 설정
@Component
public class OAuth2AuthenticationFailureHandler extends SimpleUrlAuthenticationFailureHandler {

    // HttpCookieOAuth2AuthorizationRequestRepository 의존성 주입
    private final HttpCookieOAuth2AuthorizationRequestRepository httpCookieOAuth2AuthorizationRequestRepository;

    @Override
    public void onAuthenticationFailure(HttpServletRequest request, HttpServletResponse response,
                                        AuthenticationException exception) throws IOException {

        // 쿠키에서 리디렉션 URI를 가져오거나 기본값("/")을 사용
        String targetUrl = CookieUtils.getCookie(request, HttpCookieOAuth2AuthorizationRequestRepository.REDIRECT_URI_PARAM_COOKIE_NAME)
                .map(Cookie::getValue)
                .orElse("/");

        // 타겟 URL에 에러 메시지를 쿼리 파라미터로 추가
        targetUrl = UriComponentsBuilder.fromUriString(targetUrl)
                .queryParam("error", exception.getLocalizedMessage())
                .build().toUriString();

        // 인증 요청 쿠키를 삭제
        httpCookieOAuth2AuthorizationRequestRepository.removeAuthorizationRequestCookies(request, response);

        // 클라이언트를 에러가 포함된 타겟 URL로 리디렉션
        getRedirectStrategy().sendRedirect(request, response, targetUrl);
    }
}
