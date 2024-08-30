package com.d211.drtaa.user.dto.request;

import lombok.AllArgsConstructor;
import lombok.Data;

@Data
@AllArgsConstructor
public class SignUpRequestDTO {
    private String userEmail;
    private String userPassword;
    private String userNickname;
    private boolean userIsAdmin;
}
