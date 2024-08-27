package com.d211.study.dto.request;

import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Getter;

@Data
@AllArgsConstructor
@Getter
public class LoginRequest {
    private String memberUsername;
    private String memberPassword;
}
