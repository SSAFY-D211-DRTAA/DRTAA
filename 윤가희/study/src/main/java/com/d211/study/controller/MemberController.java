package com.d211.study.controller;

import com.d211.study.config.security.CustomUserDetailsService;
import com.d211.study.dto.request.SignUpUserRequest;
import com.d211.study.exception.UserCreationException;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.dao.DataIntegrityViolationException;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

@Slf4j
@RestController
@RequiredArgsConstructor
@RequestMapping("/user")
public class MemberController {

    private final CustomUserDetailsService userDetailsService;

    @PostMapping("/signup")
    public ResponseEntity<?> signUp(@RequestBody SignUpUserRequest request) {
        HttpStatus status = HttpStatus.ACCEPTED; // 200

        try {
            userDetailsService.createUser(request);
            status = HttpStatus.CREATED; // 201, 요청이 성공적으로 처리 & 새로운 리소스가 생성
        }catch (DataIntegrityViolationException e) {
            status = HttpStatus.CONFLICT; // 409, 리소스 간의 충돌이 발생
        } catch (UserCreationException e) {
            status = HttpStatus.BAD_REQUEST; // 400, 클라이언트의 요청이 잘못됨
        }

        return new ResponseEntity<>(status);

    }
}
