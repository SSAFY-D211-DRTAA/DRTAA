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
import com.fasterxml.jackson.databind.ObjectMapper;
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

@Component
@RequiredArgsConstructor
@Slf4j
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
        // 요청에 따라 적절한 ResponseEntity를 반환하는 메서드를 호출하여 결과 응답을 가져옴
        ResponseEntity<?> result = determineResponseEntity(request, authentication);
        log.info("result: {}", result);

        // 응답이 이미 커밋되었는지 확인, 커밋된 경우 로그를 출력하고 메서드를 종료
        if (response.isCommitted()) {
            logger.debug("응답이 이미 커밋되었습니다. 응답을 보낼 수 없습니다.");
            return;
        }

        // 인증 성공 시 생성된 인증 속성들을 제거하여 보안을 유지
        clearAuthenticationAttributes(request, response);

        // 응답의 상태 코드를 설정 (예: 200, 400 등)
        response.setStatus(result.getStatusCodeValue());

        // 응답을 JSON 형식으로 작성
        response.setContentType("application/json;charset=UTF-8");

        // ObjectMapper를 사용하여 JSON 형식으로 변환
        ObjectMapper objectMapper = new ObjectMapper();
        String jsonResponse = objectMapper.writeValueAsString(result.getBody());

        // JSON 응답으로 JWT 토큰 등을 클라이언트로 전달
        response.getWriter().write(jsonResponse);
    }

    protected ResponseEntity<?> determineResponseEntity(HttpServletRequest request, Authentication authentication) {
        // 요청 URL을 통해 어떤 소셜 로그인을 진행하는지 확인
        String requestURI = request.getRequestURI();
        log.info("requestURI: {}", requestURI);

        // Authentication 객체에서 OAuth2UserPrincipal을 추출
        OAuth2UserPrincipal principal = getOAuth2UserPrincipal(authentication);
        log.info("principal: {}", principal);

        // principal이 null일 경우, 인증 실패로 UNAUTHORIZED 응답을 반환
        if (principal == null)
            return ResponseEntity.status(HttpStatus.UNAUTHORIZED).body("로그인 실패");

        // 구글 로그인 요청 처리
        if ("/login/oauth2/code/google".equals(requestURI))
            return handleLogin(principal, "Google");
        // 카카오 로그인 요청 처리
        else if ("/login/oauth2/code/kakao".equals(requestURI))
                return handleLogin(principal, "Kakao");
        // 네이버 로그인 요청 처리
        else if ("/login/oauth2/code/naver".equals(requestURI))
            return handleLogin(principal, "Naver");

        /* 언링크 미완성 */
        // 구글 언링크 요청 처리
        else if ("/auth/google/unlink".equals(requestURI))
            return handleUnlink(principal);
        // 카카오 언링크 요청 처리
        else if ("/auth/kakao/unlink".equals(requestURI))
            return handleUnlink(principal);
        // 네이버 언링크 요청 처리
        else if ("/auth/naver/unlink".equals(requestURI))
            return handleUnlink(principal);

        // 지원하지 않는 요청 URI인 경우, 잘못된 요청으로 BAD_REQUEST 응답 반환
        return ResponseEntity.badRequest().body("잘못된 요청");
    }

    // 로그인 처리 로직을 별도의 메서드로 분리
    private ResponseEntity<?> handleLogin(OAuth2UserPrincipal principal, String providerName) {
        // 사용자 정보를 principal에서 추출
        String email = principal.getUserInfo().getEmail();
        String username = principal.getUserInfo().getName();
        String nickname = principal.getUserInfo().getNickname();
        String accessToken = principal.getUserInfo().getAccessToken();

        // 사용자 정보 로그 출력
        log.info("Provider={}, email={}, name={}, nickname={}, accessToken={}",
                providerName, email, username, nickname, accessToken);

        JwtToken tokens; // JWT 토큰 객체 선언

        // 사용자가 이미 존재하면 업데이트, 존재하지 않으면 새로 저장 후 업데이트
        if (customUserDetailsService.userExists(email)) {
            tokens = updateMember(email); // 기존 사용자 정보를 업데이트하고 JWT 토큰 발급
        } else {
            saveMember(email, username, ""); // 신규 사용자 저장
            tokens = updateMember(email); // 저장 후 사용자 정보 업데이트하고 JWT 토큰 발급
        }

        log.info("Provider={}, accessToken={}", providerName, tokens.getAccessToken());
        log.info("Provider={}, refreshToken={}", providerName, tokens.getRefreshToken());

        // JWT 토큰을 JSON 형식으로 포함하여 OK 응답 반환
        return ResponseEntity.ok(tokens);
    }

    // 언링크 처리 로직을 별도의 메서드로 분리
    private ResponseEntity<?> handleUnlink(OAuth2UserPrincipal principal) {
        String accessToken = principal.getUserInfo().getAccessToken(); // 사용자 액세스 토큰 추출
        OAuth2Provider provider = principal.getUserInfo().getProvider(); // OAuth2 제공자 정보 추출

        // DB에서 해당 이메일의 사용자 삭제 및 제공자와의 연결 해제
        userRepository.deleteByUserEmail(principal.getUserInfo().getEmail()); // 사용자 삭제
        oAuth2UserUnlinkManager.unlink(provider, accessToken); // 제공자와의 연결 해제

        // 성공 메시지를 포함하여 OK 응답 반환
        return ResponseEntity.ok("Unlink 성공");
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
    }

    public JwtToken updateMember(String email) {
        // 이메일로 회원 찾기
        User user = userRepository.findByUserEmail(email)
                .orElseThrow(() -> new UsernameNotFoundException("해당 회원을 찾을 수 없습니다."));

        // 회원의 권한 정보 가져오기
        Collection<GrantedAuthority> authorities = Collections.singletonList(new SimpleGrantedAuthority("ROLE_USER"));

        // 리프레시 토큰 업데이트
        JwtToken tokens = tokenProvider.generateTokenForSocialLogin(email, authorities);
        
        // 리프레시 토큰 저장
        user.setUserRefreshToken(tokens.getRefreshToken());
        userRepository.save(user);

        return tokens;
    }
}


