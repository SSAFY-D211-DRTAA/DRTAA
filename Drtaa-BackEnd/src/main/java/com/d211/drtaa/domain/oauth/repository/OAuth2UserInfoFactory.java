package com.d211.drtaa.domain.oauth.repository;

import com.d211.drtaa.global.exception.oauth2.OAuth2AuthenticationProcessingException;

import java.util.Map;

public class OAuth2UserInfoFactory {

    // 주어진 registrationId(제공자 ID), accessToken(액세스 토큰), attributes속성)에 따라 적절한 OAuth2UserInfo 구현체를 반환
    public static OAuth2UserInfo getOAuth2UserInfo
        (String registrationId, String accessToken, Map<String, Object> attributes) {

        // registrationId가 "GOOGLE"일 경우, GoogleOAuth2UserInfo 인스턴스를 생성하여 반환
        if (OAuth2Provider.GOOGLE.getRegistrationId().equals(registrationId))
            return new GoogleOAuth2UserInfo(accessToken, attributes);

        // registrationId가 "NAVER"일 경우, NaverOAuth2UserInfo 인스턴스를 생성하여 반환
        else if (OAuth2Provider.NAVER.getRegistrationId().equals(registrationId))
            return new NaverOAuth2UserInfo(accessToken, attributes);

        // registrationId가 "KAKAO"일 경우, KakaoOAuth2UserInfo 인스턴스를 생성하여 반환
        else if (OAuth2Provider.KAKAO.getRegistrationId().equals(registrationId))
            return new KakaoOAuth2UserInfo(accessToken, attributes);

        // 위의 경우들에 해당하지 않는 registrationId일 경우, 지원되지 않는 로그인 제공자에 대한 예외를 발생
        else
            throw new OAuth2AuthenticationProcessingException(registrationId + "로그인은 지원하지 않습니다.");
    }
}
