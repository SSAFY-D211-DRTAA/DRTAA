package com.d211.drtaa.domain.user.dto.request;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Getter;

@Data
@AllArgsConstructor
@Getter
public class FormLoginRequestDTO {
    @Schema(description = "회원 ID", example = "test@naver.com")
    private String userEmail;
    @Schema(description = "회원 PW", example = "1234")
    private String userPassword;
}
