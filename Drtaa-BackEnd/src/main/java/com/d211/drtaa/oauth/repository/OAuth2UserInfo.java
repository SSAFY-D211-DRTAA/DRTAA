package com.d211.drtaa.oauth.repository;

import java.util.Map;

public interface OAuth2UserInfo {

    // OAuth2 제공자를 반환(예를 들어, Google, Naver 등의 제공자 정보를 반환)
    OAuth2Provider getProvider();

    // OAuth2 제공자로부터 받은 액세스 토큰을 반환, 이 토큰은 API 호출 시 인증에 사용
    String getAccessToken();

    // OAuth2 제공자로부터 받은 전체 사용자 정보를 Map 형태로 반환(JSON 객체 기반)
    Map<String, Object> getAttributes();

    // 사용자의 고유 ID를 반환
    String getId();

    // 사용자의 이메일 주소를 반환
    String getEmail();

    // 사용자의 전체 이름을 반환
    String getName();

    // 사용자의 이름(성)을 반환
    String getFirstName();

    // 사용자의 성(이름)을 반환
    String getLastName();

    // 사용자의 별명(닉네임)을 반환
    String getNickname();

    // 사용자의 프로필 이미지 URL을 반환
    String getProfileImageUrl();
}
