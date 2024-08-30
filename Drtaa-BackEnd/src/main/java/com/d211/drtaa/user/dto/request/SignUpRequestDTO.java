package com.d211.drtaa.user.dto.request;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Getter;

@Data
@AllArgsConstructor
@Getter
public class SignUpRequestDTO {
    @Schema(description = "회원 ID", example = "test@naver.com")
    private String userEmail;
    @Schema(description = "회원 PW", example = "1234")
    private String userPassword;
    @Schema(description = "회원 닉네임", example = "TEST")
    private String userNickname;
    @Schema(description = "회원 소셜 로그인 유무", example = "false")
    private boolean userIsSocial = false;
    @Schema(description = "회원 관리자 유무", example = "false")
    private boolean userIsAdmin;
}
