package com.d211.drtaa.domain.user.dto.response;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;

@Data
@AllArgsConstructor
@Builder
public class UserInfoResponseDTO {
    @Schema(description = "회원 고유 번호", example = "1")
    private long userId;
    @Schema(description = "회원 ID", example = "test@naver.com")
    private String userEmail;
    @Schema(description = "회원 닉네임", example = "TEST")
    private String userNickname;
    @Schema(description = "회원 로그인처", example = "Form")
    private String userLogin;
    @Schema(description = "회원 관리자 유무", example = "false")
    private boolean userIsAdmin;
}
