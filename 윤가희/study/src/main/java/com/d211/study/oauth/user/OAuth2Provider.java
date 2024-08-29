package com.d211.study.oauth.user;

import lombok.Getter;
import lombok.RequiredArgsConstructor;
import org.springframework.security.oauth2.client.registration.ClientRegistration;

@Getter
@RequiredArgsConstructor
public enum OAuth2Provider {
    GOOGLE("google"),
    NAVER("naver"),
    KAKAO("kakao");

    private final String registrationId;

    public ClientRegistration getClientRegistration(String clientId, String clientSecret, String redirectUri) {
        switch (this) {
            case GOOGLE:
                return ClientRegistration.withRegistrationId("google")
                        .clientId(clientId)
                        .clientSecret(clientSecret)
                        .authorizationUri("https://accounts.google.com/o/oauth2/auth")
                        .tokenUri("https://oauth2.googleapis.com/token")
                        .userInfoUri("https://www.googleapis.com/oauth2/v3/userinfo")
                        .redirectUri(redirectUri)
                        .authorizationGrantType(OAuth2AuthorizationGrantType.AUTHORIZATION_CODE)
                        .build();
            case NAVER:
                return ClientRegistration.withRegistrationId("naver")
                        .clientId(clientId)
                        .clientSecret(clientSecret)
                        .authorizationUri("https://nid.naver.com/oauth2.0/authorize")
                        .tokenUri("https://nid.naver.com/oauth2.0/token")
                        .userInfoUri("https://openapi.naver.com/v1/nid/me")
                        .redirectUri(redirectUri)
                        .authorizationGrantType(OAuth2AuthorizationGrantType.AUTHORIZATION_CODE)
                        .build();
            case KAKAO:
                return ClientRegistration.withRegistrationId("kakao")
                        .clientId(clientId)
                        .clientSecret(clientSecret)
                        .authorizationUri("https://kauth.kakao.com/oauth2/authorize")
                        .tokenUri("https://kauth.kakao.com/oauth2/token")
                        .userInfoUri("https://kapi.kakao.com/v2/user/me")
                        .redirectUri(redirectUri)
                        .authorizationGrantType(OAuth2AuthorizationGrantType.AUTHORIZATION_CODE)
                        .build();
            default:
                throw new IllegalArgumentException("Unsupported OAuth2 provider: " + this);
        }
    }
}
