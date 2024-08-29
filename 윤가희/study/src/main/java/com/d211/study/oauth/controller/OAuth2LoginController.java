package com.d211.study.oauth.controller;

import com.d211.study.config.jwt.JwtToken;
import com.d211.study.config.jwt.JwtTokenProvider;
import com.d211.study.oauth.service.CustomOAuth2UserService;
import com.d211.study.oauth.user.OAuth2Provider;
import com.d211.study.oauth.user.OAuth2UserInfo;
import com.d211.study.oauth.user.OAuth2UserInfoFactory;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.http.ResponseEntity;
import org.springframework.security.authentication.UsernamePasswordAuthenticationToken;
import org.springframework.security.core.Authentication;
import org.springframework.security.core.context.SecurityContextHolder;
import org.springframework.security.oauth2.client.registration.ClientRegistration;
import org.springframework.security.oauth2.client.userinfo.OAuth2UserRequest;
import org.springframework.security.oauth2.client.userinfo.OAuth2UserService;
import org.springframework.security.oauth2.core.OAuth2AccessToken;
import org.springframework.security.oauth2.core.user.OAuth2User;
import org.springframework.web.bind.annotation.*;

import java.util.Collections;
import java.util.Map;

@RestController
@RequestMapping("/api/oauth2")
@RequiredArgsConstructor
@Slf4j
public class OAuth2LoginController {

    private final CustomOAuth2UserService customOAuth2UserService;
    private final JwtTokenProvider jwtTokenProvider;

    @PostMapping("/login/{provider}")
    public ResponseEntity<?> oAuth2Login(
            @PathVariable String provider,
            @RequestBody Map<String, Object> oAuth2UserInfo) {

        try {
            OAuth2Provider oAuth2Provider = OAuth2Provider.valueOf(provider.toUpperCase());
            String accessToken = (String) oAuth2UserInfo.get("accessToken");

            if (accessToken == null) {
                return ResponseEntity.badRequest().body("Access token is missing");
            }

            // OAuth2UserInfo 객체 생성
//            OAuth2UserInfo userInfo = OAuth2UserInfoFactory.getOAuth2UserInfo(
//                    oAuth2Provider.getRegistrationId(),
//                    accessToken,
//                    oAuth2UserInfo
//            );

            // ClientRegistration 객체 생성
            ClientRegistration clientRegistration = oAuth2Provider.getClientRegistration(
                    "your-client-id",   // 적절한 Client ID를 입력
                    "your-client-secret",  // 적절한 Client Secret을 입력
                    "your-redirect-uri"    // 적절한 Redirect URI를 입력
            );

            // OAuth2UserRequest 객체 생성
            OAuth2UserRequest userRequest = new OAuth2UserRequest(
                    clientRegistration,
                    new OAuth2AccessToken(OAuth2AccessToken.TokenType.BEARER, accessToken, null, null),
                    Collections.emptyMap()  // 빈 맵을 추가 파라미터로 전달
            );

            // 사용자 정보 로드
            OAuth2User oauth2User = customOAuth2UserService.loadUser(userRequest);

            Authentication authentication = new UsernamePasswordAuthenticationToken(
                    oauth2User,
                    null,
                    oauth2User.getAuthorities()
            );

            SecurityContextHolder.getContext().setAuthentication(authentication);

            JwtToken jwtToken = jwtTokenProvider.generateToken(authentication);

            return ResponseEntity.ok(jwtToken);
        } catch (IllegalArgumentException e) {
            log.error("Invalid OAuth2 provider: {}", provider, e);
            return ResponseEntity.badRequest().body("Invalid OAuth2 provider");
        } catch (Exception e) {
            log.error("Error during OAuth2 login", e);
            return ResponseEntity.internalServerError().body("An error occurred during login");
        }
    }



    @PostMapping("/unlink/{provider}")
    public ResponseEntity<String> oAuth2Unlink(
            @PathVariable String provider,
            @RequestHeader("Authorization") String bearerToken) {

        String token = bearerToken.substring(7);
        Authentication authentication = jwtTokenProvider.getAuthentication(token);

        OAuth2Provider oAuth2Provider = OAuth2Provider.valueOf(provider.toUpperCase());

        // TODO: Implement unlink logic using oAuth2UserService or create a new service for unlinking

        return ResponseEntity.ok("User unlinked successfully");
    }
}
