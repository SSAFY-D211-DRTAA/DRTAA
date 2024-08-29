//package com.d211.study.oauth.controller;
//
//import com.d211.study.config.jwt.JwtToken;
//import com.d211.study.config.jwt.JwtTokenProvider;
//import com.d211.study.oauth.user.OAuth2Provider;
//import com.d211.study.oauth.user.OAuth2UserInfo;
//import com.d211.study.oauth.user.OAuth2UserInfoFactory;
//import lombok.RequiredArgsConstructor;
//import org.springframework.http.ResponseEntity;
//import org.springframework.security.core.Authentication;
//import org.springframework.security.core.context.SecurityContextHolder;
//import org.springframework.security.oauth2.client.userinfo.OAuth2UserService;
//import org.springframework.web.bind.annotation.*;
//
//import java.util.Map;
//
//@RestController
//@RequestMapping("/api/oauth2")
//@RequiredArgsConstructor
//public class OAuth2LoginController {
//
//    private final OAuth2UserService oAuth2UserService;
//    private final JwtTokenProvider jwtTokenProvider;
//
//    @PostMapping("/login/{provider}")
//    public ResponseEntity<JwtToken> oAuth2Login(
//            @PathVariable String provider,
//            @RequestBody Map<String, Object> oAuth2UserInfo) {
//
//        OAuth2Provider oAuth2Provider = OAuth2Provider.valueOf(provider.toUpperCase());
//        String accessToken = (String) oAuth2UserInfo.get("accessToken");
//
//        OAuth2UserInfo userInfo = OAuth2UserInfoFactory.getOAuth2UserInfo(
//                oAuth2Provider.getRegistrationId(),
//                accessToken,
//                oAuth2UserInfo
//        );
//
//        Authentication authentication = oAuth2UserService.loadUser(userInfo);
//        SecurityContextHolder.getContext().setAuthentication(authentication);
//
//        JwtToken jwtToken = jwtTokenProvider.generateToken(authentication);
//
//        return ResponseEntity.ok(jwtToken);
//    }
//
//    @PostMapping("/unlink/{provider}")
//    public ResponseEntity<String> oAuth2Unlink(
//            @PathVariable String provider,
//            @RequestHeader("Authorization") String bearerToken) {
//
//        String token = bearerToken.substring(7);
//        Authentication authentication = jwtTokenProvider.getAuthentication(token);
//
//        OAuth2Provider oAuth2Provider = OAuth2Provider.valueOf(provider.toUpperCase());
//
//        // TODO: Implement unlink logic using oAuth2UserService or create a new service for unlinking
//
//        return ResponseEntity.ok("User unlinked successfully");
//    }
//}
