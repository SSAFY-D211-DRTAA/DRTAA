package com.d211.study.controller;

import com.d211.study.config.jwt.JwtToken;
import com.d211.study.dto.request.PasswordRequestDTO;
import com.d211.study.dto.response.UserInfoResponse;
import com.d211.study.service.CustomUserDetailsService;
import com.d211.study.dto.request.LoginRequestDTO;
import com.d211.study.dto.request.SignUpRequestDTO;
import com.d211.study.exception.user.UserCreationException;
import com.d211.study.service.MemberService;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.dao.DataIntegrityViolationException;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.security.core.Authentication;
import org.springframework.security.core.userdetails.UsernameNotFoundException;
import org.springframework.web.bind.annotation.*;

@Slf4j
@RestController
@RequiredArgsConstructor
@RequestMapping("/user")
public class MemberController {

    private final CustomUserDetailsService userDetailsService;
    private final MemberService memberService;

    @PostMapping("/signup")
    public ResponseEntity<?> signUp(@RequestBody SignUpRequestDTO request) {
        HttpStatus status = HttpStatus.ACCEPTED; // 202

        try {
            userDetailsService.createUser(request);
            status = HttpStatus.CREATED; // 201, 요청이 성공적으로 처리 & 새로운 리소스가 생성

            return new ResponseEntity<>(status);
        }catch (DataIntegrityViolationException e) {
            status = HttpStatus.CONFLICT; // 409, 리소스 간의 충돌이 발생

            return new ResponseEntity<>(e.getMessage(), status);
        } catch (UserCreationException e) {
            status = HttpStatus.BAD_REQUEST; // 400, 클라이언트의 요청이 잘못됨

            return new ResponseEntity<>(e.getMessage(), status);

        } catch (Exception e) {
            status = HttpStatus.INTERNAL_SERVER_ERROR; // 500

            return new ResponseEntity<>(e.getMessage(), status);
        }
    }

    @GetMapping("/login")
    public ResponseEntity<?> login(@RequestBody LoginRequestDTO request) {
        HttpStatus status = HttpStatus.ACCEPTED; // 202

        try {
            JwtToken tokens = memberService.login(request);
            status = HttpStatus.OK; // 200

            return new ResponseEntity<>(tokens, status);

        } catch(UsernameNotFoundException e) {
            status = HttpStatus.NOT_FOUND; // 404

            return new ResponseEntity<>(e.getMessage(), status);
        } catch (Exception e) {
            status = HttpStatus.INTERNAL_SERVER_ERROR; // 500

            return new ResponseEntity<>(e.getMessage(), status);
        }
    }

    @GetMapping("/info")
    public ResponseEntity<?> info(Authentication authentication) {
        HttpStatus status = HttpStatus.ACCEPTED;

        try {
            UserInfoResponse response = memberService.info(authentication.getName());

            status = HttpStatus.OK;

            return new ResponseEntity<>(response, status);
        } catch (UsernameNotFoundException e) {
            status = HttpStatus.NOT_FOUND; // 404

            return new ResponseEntity<>(e.getMessage(), status);
        } catch (Exception e) {
            status = HttpStatus.INTERNAL_SERVER_ERROR; // 500

            return new ResponseEntity<>(e.getMessage(), status);
        }
    }

    @PatchMapping("/password")
    public ResponseEntity<?> passwordChange
            (Authentication authentication, @RequestBody PasswordRequestDTO request) {
        HttpStatus status = HttpStatus.ACCEPTED;

        try {
            userDetailsService.changePassword(request.getOldPassword(), request.getNewPassword());

            status = HttpStatus.OK; // 200

            return new ResponseEntity<>(status);
        } catch (Exception e) {
            status = HttpStatus.INTERNAL_SERVER_ERROR; // 500

            return new ResponseEntity<>(e.getMessage(), status);
        }
    }
}
