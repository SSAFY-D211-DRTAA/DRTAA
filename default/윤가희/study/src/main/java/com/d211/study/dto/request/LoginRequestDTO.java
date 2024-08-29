package com.d211.study.dto.request;

import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Getter;

@Data
@AllArgsConstructor
@Getter
public class LoginRequestDTO {
    private String memberUsername;
    private String memberPassword;
}
