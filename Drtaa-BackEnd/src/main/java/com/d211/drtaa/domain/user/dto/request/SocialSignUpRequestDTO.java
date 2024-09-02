package com.d211.drtaa.domain.user.dto.request;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Data
@AllArgsConstructor
@Getter
public class SocialSignUpRequestDTO {
    @Schema(description = "회원 고유 번호", example = "1")
    private String userProviderId;
    @Schema(description = "회원 ID", example = "test@naver.com")
    private String userEmail;
    @Schema(description = "회원 닉네임", example = "TEST")
    private String userNickname;
    @Schema(description = "회원 로그인처", example = "Form")
    private String userLogin;
    @Schema(description = "회원 관리자 유무", example = "false")
    private boolean userIsAdmin;
}
