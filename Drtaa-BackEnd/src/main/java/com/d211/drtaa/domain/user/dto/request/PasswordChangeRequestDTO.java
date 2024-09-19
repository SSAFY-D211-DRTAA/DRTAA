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
public class PasswordChangeRequestDTO {
    @Schema(description = "회원 변경 전 비밀번호", example = "1234")
    private String oldPassword;
    @Schema(description = "회원 변경 전 비밀번호", example = "5678")
    private String newPassword;
}
