package com.d211.drtaa.oauth.handler;

import com.d211.drtaa.config.jwt.JwtToken;
import com.d211.drtaa.config.jwt.JwtTokenProvider;
import com.d211.drtaa.oauth.repository.HttpCookieOAuth2AuthorizationRequestRepository;
import com.d211.drtaa.oauth.repository.OAuth2Provider;
import com.d211.drtaa.oauth.repository.OAuth2UserUnlinkManager;
import com.d211.drtaa.oauth.service.OAuth2UserPrincipal;
import com.d211.drtaa.oauth.util.CookieUtils;
import com.d211.drtaa.user.entity.User;
import com.d211.drtaa.user.repository.UserRepository;
import com.d211.drtaa.user.service.CustomUserDetailsServiceImpl;
import jakarta.servlet.http.Cookie;
import jakarta.servlet.http.HttpServletRequest;
import jakarta.servlet.http.HttpServletResponse;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.context.annotation.Lazy;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.security.core.Authentication;
import org.springframework.security.core.GrantedAuthority;
import org.springframework.security.core.authority.SimpleGrantedAuthority;
import org.springframework.security.core.userdetails.UsernameNotFoundException;
import org.springframework.security.crypto.password.PasswordEncoder;
import org.springframework.security.web.authentication.SimpleUrlAuthenticationSuccessHandler;
import org.springframework.stereotype.Component;
import org.springframework.web.util.UriComponentsBuilder;

import java.io.IOException;
import java.util.Collection;
import java.util.Collections;
import java.util.Optional;

import static com.d211.drtaa.oauth.repository.HttpCookieOAuth2AuthorizationRequestRepository.MODE_PARAM_COOKIE_NAME;
import static com.d211.drtaa.oauth.repository.HttpCookieOAuth2AuthorizationRequestRepository.REDIRECT_URI_PARAM_COOKIE_NAME;

@Slf4j
@RequiredArgsConstructor
@Component
public class OAuth2AuthenticationSuccessHandler extends SimpleUrlAuthenticationSuccessHandler {

    private final OAuth2UserUnlinkManager oAuth2UserUnlinkManager;
    private final JwtTokenProvider tokenProvider;
    private final CustomUserDetailsServiceImpl customUserDetailsService;

    private final HttpCookieOAuth2AuthorizationRequestRepository httpCookieOAuth2AuthorizationRequestRepository;
    private final UserRepository userRepository;

    @Lazy
    @Autowired
    private PasswordEncoder passwordEncoder;


    @Override
    public void onAuthenticationSuccess(HttpServletRequest request, HttpServletResponse response,
                                        Authentication authentication) throws IOException {

        // 인증 성공 후 리디렉션할 URL을 결정하는 메서드를 호출하여 타겟 URL을 가져옴
        String targetUrl = determineTargetUrl(request, response, authentication);

        // 요청에 따라 적절한 ResponseEntity를 반환하는 메서드를 호출하여 결과 응답을 가져옴
        ResponseEntity<?> result = determineResponseEntity(request, authentication);

        // 응답이 이미 커밋되었는지 확인, 커밋된 경우 로그를 출력하고 메서드를 종료
        if (response.isCommitted()) {
            logger.debug("응답이 이미 커밋되었습니다." + targetUrl + "으로 리디렉션할 수 없습니다.");
            return;
        }

        // 인증 성공 시 생성된 인증 속성들을 제거하여 보안을 유지
        clearAuthenticationAttributes(request, response);

        // 클라이언트를 타겟 URL로 리디렉션 (응답의 Location 헤더를 설정하여 리디렉션 수행)
        getRedirectStrategy().sendRedirect(request, response, targetUrl);

        // 리디렉션 이후 응답의 상태 코드를 설정 (예: 200, 400 등)
        response.setStatus(result.getStatusCodeValue());

        // 응답 본문에 result의 내용을 JSON 형식으로 작성하여 클라이언트로 전송
        response.getWriter().write(result.getBody().toString());
    }

    protected ResponseEntity<?> determineResponseEntity(HttpServletRequest request, Authentication authentication) {
        // 쿠키에서 REDIRECT_URI_PARAM_COOKIE_NAME에 해당하는 값을 찾아 Optional로 감싸서 반환
        Optional<String> redirectUri = CookieUtils.getCookie(request, REDIRECT_URI_PARAM_COOKIE_NAME)
                .map(Cookie::getValue);

        // redirectUri가 있으면 사용하고, 없으면 기본 URL(getDefaultTargetUrl) 사용
        String targetUrl = redirectUri.orElse(getDefaultTargetUrl());

        // 쿠키에서 MODE_PARAM_COOKIE_NAME에 해당하는 값을 찾아 Optional로 감싸서 반환, 없으면 빈 문자열로 대체
        String mode = CookieUtils.getCookie(request, MODE_PARAM_COOKIE_NAME)
                .map(Cookie::getValue)
                .orElse("");

        // Authentication 객체에서 OAuth2UserPrincipal을 추출
        OAuth2UserPrincipal principal = getOAuth2UserPrincipal(authentication);

        // principal이 null일 경우, 인증 실패로 UNAUTHORIZED 응답을 반환
        if (principal == null) {
            HttpStatus status = HttpStatus.UNAUTHORIZED; // 401
            String response = "로그인 실패";

            return new ResponseEntity<>(response, status); // JSON 형식으로 실패 메시지 반환
        }

        // 모드가 "login"일 경우
        if ("login".equalsIgnoreCase(mode)) {
            // 사용자 정보를 principal에서 추출
            String email = principal.getUserInfo().getEmail();
            String username = principal.getUserInfo().getName();
            String nickname = principal.getUserInfo().getNickname();
            String accessToken = principal.getUserInfo().getAccessToken();

            // 사용자 정보 로그 출력
            log.info("email={}, name={}, nickname={}, accessToken={}",
                    email, username, nickname, accessToken);

            JwtToken tokens; // JWT 토큰 객체 선언

            // 사용자가 이미 존재하면 업데이트, 존재하지 않으면 새로 저장 후 업데이트
            if (customUserDetailsService.userExists(email)) {
                tokens = updateMember(email); // 기존 사용자 정보를 업데이트하고 JWT 토큰 발급
            } else {
                saveMember(email, username, ""); // 신규 사용자 저장
                tokens = updateMember(email); // 저장 후 사용자 정보 업데이트하고 JWT 토큰 발급
            }
            log.info("accessToken={}", tokens.getAccessToken());
            log.info("refreshToken={}", tokens.getRefreshToken());

            // JWT 토큰을 JSON 형식으로 포함하여 OK 응답 반환
            return ResponseEntity.ok(tokens);

        } else if ("unlink".equalsIgnoreCase(mode)) { // 모드가 "unlink"일 경우
            String accessToken = principal.getUserInfo().getAccessToken(); // 사용자 액세스 토큰 추출
            OAuth2Provider provider = principal.getUserInfo().getProvider(); // OAuth2 제공자 정보 추출

            // DB에서 해당 이메일의 사용자 삭제 및 제공자와의 연결 해제
            userRepository.deleteByUserEmail(principal.getUserInfo().getEmail()); // 사용자 삭제
            oAuth2UserUnlinkManager.unlink(provider, accessToken); // 제공자와의 연결 해제

            // 성공 메시지를 포함하여 OK 응답 반환
            String response  = "Unlink 성공";
            return ResponseEntity.ok(response);
        }

        // 모드가 "login"이나 "unlink"가 아닐 경우, 잘못된 요청으로 BAD_REQUEST 응답 반환
        String response = "잘못된 요청";
        return ResponseEntity.badRequest().body(response);
    }

    private OAuth2UserPrincipal getOAuth2UserPrincipal(Authentication authentication) {
        // Authentication 객체에서 Principal(사용자 정보) 객체를 가져옴
        Object principal = authentication.getPrincipal();

        // 가져온 Principal 객체가 OAuth2UserPrincipal 타입인지 확인
        if (principal instanceof OAuth2UserPrincipal)
            // OAuth2UserPrincipal 타입이면 캐스팅하여 반환
            return (OAuth2UserPrincipal) principal;

        // OAuth2UserPrincipal 타입이 아니면 null 반환
        return null;
    }

    protected void clearAuthenticationAttributes(HttpServletRequest request, HttpServletResponse response) {
        // 부모 클래스의 clearAuthenticationAttributes 메서드를 호출해 세션에 저장된 인증 관련 속성들을 제거
        super.clearAuthenticationAttributes(request);

        // 커스텀 저장소에서 인증 요청 쿠키를 제거하여 보안 유지
        httpCookieOAuth2AuthorizationRequestRepository.removeAuthorizationRequestCookies(request, response);
    }

    public void saveMember(String email, String name, String password) {
        User newMember = User.builder()
                .userEmail(email)
                .userPassword(passwordEncoder.encode(password))
                .userNickname(name)
                .userRefreshToken("")
                .userIsSocial(true)
                .userIsAdmin(false)
                .build();

        // 회원 정보 저장
        customUserDetailsService.createUser(newMember);

        // 기본 권한 설정
        Collection<GrantedAuthority> authorities = Collections.singletonList(new SimpleGrantedAuthority("ROLE_USER"));
    }

    public JwtToken updateMember(String email) {
        // 이메일로 회원 찾기
        User user = userRepository.findByUserEmail(email)
                .orElseThrow(() -> new UsernameNotFoundException("해당 회원을 찾을 수 없습니다."));

        // 회원의 권한 정보 가져오기
        Collection<GrantedAuthority> authorities = Collections.singletonList(new SimpleGrantedAuthority("ROLE_USER"));

        // 리프레시 토큰 업데이트
        JwtToken tokens = tokenProvider.generateTokenForSocialLogin(email, authorities);
        user.setUserRefreshToken(tokens.getRefreshToken());
        userRepository.save(user);

        return tokens;
    }
}


