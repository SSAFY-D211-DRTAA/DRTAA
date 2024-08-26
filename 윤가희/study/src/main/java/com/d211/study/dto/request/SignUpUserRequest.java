package com.d211.study.dto.request;

import lombok.AllArgsConstructor;
import lombok.Data;

@Data
@AllArgsConstructor
public class SignUpUserRequest {
    private String memberUsername;
    private String memberEmail;
    private String memberPassword;
    private boolean memberIsAdmin;
}
