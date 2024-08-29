package com.d211.study.dto.request;

import lombok.AllArgsConstructor;
import lombok.Data;

@Data
@AllArgsConstructor
public class PasswordRequestDTO {
    private String oldPassword;
    private String newPassword;
}
