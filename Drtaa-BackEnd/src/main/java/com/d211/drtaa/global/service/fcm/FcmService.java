package com.d211.drtaa.global.service.fcm;

import jakarta.validation.constraints.NotBlank;

public interface FcmService {
    String getToken(String userProviderId, @NotBlank(message="토큰을 입력해야 합니다.") String token);
}
