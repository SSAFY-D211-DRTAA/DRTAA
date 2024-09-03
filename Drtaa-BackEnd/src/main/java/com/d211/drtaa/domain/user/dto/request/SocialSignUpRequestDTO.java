package com.d211.drtaa.domain.user.dto.request;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.*;

@Data
@AllArgsConstructor
@Getter
@Builder
public class SocialSignUpRequestDTO {
    @Schema(description = "회원 고유 번호", example = "1")
    private String userProviderId;
    @Schema(description = "회원 닉네임", example = "TEST")
    private String userNickname;
    @Schema(description = "회원 로그인처", example = "Form")
    private String userLogin;
}
