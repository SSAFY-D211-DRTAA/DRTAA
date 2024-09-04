package com.d211.drtaa.domain.user.dto.request;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Getter;

@Data
@AllArgsConstructor
@Getter
public class FormLoginRequestDTO {
    @Schema(description = "회원 고유 번호", example = "1")
    private String userProviderId;
    @Schema(description = "회원 PW", example = "1234")
    private String userPassword;
    @Schema(description = "회원 로그인처", example = "Form")
    private String userLogin;
}
