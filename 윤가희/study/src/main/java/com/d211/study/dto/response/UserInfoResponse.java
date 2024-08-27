package com.d211.study.dto.response;

import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Getter;

@Data
@AllArgsConstructor
@Getter
public class UserInfoResponse {
    private long memberId;
    private String memberUsername;
    private String memberEmail;
    private String memberPassword;
    private boolean memberIsAdmin;
}
