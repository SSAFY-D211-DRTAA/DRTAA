package com.d211.drtaa.oauth.service;

import com.d211.drtaa.exception.oauth.OAuth2AuthenticationProcessingException;
import com.d211.drtaa.oauth.repository.OAuth2UserInfo;
import com.d211.drtaa.oauth.repository.OAuth2UserInfoFactory;
import lombok.RequiredArgsConstructor;
import org.springframework.security.authentication.InternalAuthenticationServiceException;
import org.springframework.security.core.AuthenticationException;
import org.springframework.security.oauth2.client.userinfo.DefaultOAuth2UserService;
import org.springframework.security.oauth2.client.userinfo.OAuth2UserRequest;
import org.springframework.security.oauth2.core.OAuth2AuthenticationException;
import org.springframework.security.oauth2.core.user.OAuth2User;
import org.springframework.stereotype.Service;
import org.springframework.util.StringUtils;

@Service
@RequiredArgsConstructor
public class CustomOAuth2UserService extends DefaultOAuth2UserService {

    @Override
    public OAuth2User loadUser(OAuth2UserRequest oAuth2UserRequest) throws OAuth2AuthenticationException {
        // OAuth2UserRequest를 이용하여 OAuth2User를 로드
        OAuth2User oAuth2User = super.loadUser(oAuth2UserRequest);

        try {
            // 로드한 OAuth2User를 처리하고 반환
            return processOAuth2User(oAuth2UserRequest, oAuth2User);
        } catch (AuthenticationException ex) {
            // 인증 예외가 발생하면 다시 던짐
            throw ex;
        } catch (Exception ex) {
            // 기타 예외 발생 시, InternalAuthenticationServiceException으로 래핑하여 던짐
            throw new InternalAuthenticationServiceException(ex.getMessage(), ex.getCause());
        }
    }

    private OAuth2User processOAuth2User(OAuth2UserRequest userRequest, OAuth2User oAuth2User) {
        // OAuth2 클라이언트 등록 ID를 가져옴
        String registrationId = userRequest.getClientRegistration().getRegistrationId();

        // OAuth2 액세스 토큰 값을 가져옴
        String accessToken = userRequest.getAccessToken().getTokenValue();

        // OAuth2UserInfo를 생성하기 위해 팩토리 메서드 호출
        OAuth2UserInfo oAuth2UserInfo = OAuth2UserInfoFactory.getOAuth2UserInfo(registrationId,
                accessToken,
                oAuth2User.getAttributes());

        // OAuth2UserInfo의 필드 값 검증
        if (!StringUtils.hasText(oAuth2UserInfo.getEmail())) {
            // 이메일 정보가 없으면 예외 발생
            throw new OAuth2AuthenticationProcessingException("Email not found from OAuth2 provider");
        }

        // OAuth2UserInfo를 바탕으로 OAuth2UserPrincipal 객체 생성 및 반환
        return new OAuth2UserPrincipal(oAuth2UserInfo);
    }
}
