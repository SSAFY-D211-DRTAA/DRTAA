package com.d211.study.dto.request;

import lombok.AllArgsConstructor;
import lombok.Data;

@Data
@AllArgsConstructor
public class SignUpRequestDTO {
    private String memberEmail;
    private String memberPassword;
    private String memberNickname;
    private boolean memberIsAdmin;
}
