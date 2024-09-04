package com.d211.drtaa.domain.user.dto.request;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Getter
public class SocialLoginRequestDTO {
    @Schema(description = "회원 고유 번호", example = "2")
    private String userProviderId;
    @Schema(description = "회원 닉네임", example = "TEST")
    private String userNickname;
    @Schema(description = "회원 프로필 사진", example = "String 이미지")
    private String userProfileImg;
    @Schema(description = "회원 로그인처", example = "Form")
    private String userLogin;
}
