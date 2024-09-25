package com.d211.drtaa.global.util.fcm;

import io.swagger.v3.oas.annotations.media.Schema;
import jakarta.validation.constraints.NotBlank;
import lombok.Getter;

@Getter
public class PostTokenReq {
    @NotBlank(message="토큰을 입력해야 합니다.")
    @Schema(description = "FCM Token", example = "")
    String Token;
}