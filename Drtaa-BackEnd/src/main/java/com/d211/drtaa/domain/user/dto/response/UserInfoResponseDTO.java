package com.d211.drtaa.domain.user.dto.response;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;

import java.time.LocalDateTime;

@Data
@AllArgsConstructor
@Builder
public class UserInfoResponseDTO {
    @Schema(description = "회원 고유 번호", example = "1")
    private long userId;
    @Schema(description = "회원 닉네임", example = "TEST")
    private String userNickname;
    @Schema(description = "회원 프로필 사진", example = "multipart 이미지")
    private String userProfileImg;
    @Schema(description = "회원 로그인처", example = "Form")
    private String userLogin;
    @Schema(description = "회원 관리자 유무", example = "false")
    private boolean userIsAdmin;
    @Schema(description = "회원 가입 일자", example = "2024.09.12 14:00:00")
    private LocalDateTime userSiginupDate;
}
