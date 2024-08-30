package com.d211.drtaa.user.controller;


import com.d211.drtaa.config.jwt.JwtToken;
import com.d211.drtaa.exception.user.UserCreationException;
import com.d211.drtaa.user.dto.request.FormLoginRequestDTO;
import com.d211.drtaa.user.dto.request.SignUpRequestDTO;
import com.d211.drtaa.user.dto.response.UserInfoResponseDTO;
import com.d211.drtaa.user.service.CustomUserDetailsServiceImpl;
import com.d211.drtaa.user.service.UserService;
import com.d211.drtaa.user.service.UserServiceImpl;
import io.swagger.v3.oas.annotations.Operation;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.dao.DataIntegrityViolationException;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.security.core.Authentication;
import org.springframework.security.core.userdetails.UsernameNotFoundException;
import org.springframework.web.bind.annotation.*;
import io.swagger.v3.oas.annotations.tags.Tag;

@RestController
@RequestMapping("/user")
@RequiredArgsConstructor
@Slf4j
@Tag(name = "회원 컨트롤러", description = "회원관련 기능 수행")
public class UserController {

    private final CustomUserDetailsServiceImpl userDetailsService;
    @Autowired
    private UserService userService;

    @PostMapping("/signup")
    @Operation(summary = "회원가입", description = "Form 회원가입")
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
            status = HttpStatus.INTERNAL_SERVER_ERROR; // 500, 서버 오류

            return new ResponseEntity<>(e.getMessage(), status);
        }
    }

    @PostMapping("/login")
    @Operation(summary = "로그인", description = "Form 로그인")
    public ResponseEntity<?> login(@RequestBody FormLoginRequestDTO request) {
        HttpStatus status = HttpStatus.ACCEPTED; // 202

        try {
            JwtToken tokens = userService.login(request);
            status = HttpStatus.OK; // 200, 클라이언트 요청 성공

            return new ResponseEntity<>(tokens, status);

        } catch(UsernameNotFoundException e) {
            status = HttpStatus.NOT_FOUND; // 404, 클라이언트 요청 탐색 실패

            return new ResponseEntity<>(e.getMessage(), status);
        } catch (Exception e) {
            status = HttpStatus.INTERNAL_SERVER_ERROR; // 500, 서버 오류

            return new ResponseEntity<>(e.getMessage(), status);
        }
    }

    @GetMapping("/info")
    @Operation(summary = "회원 정보 조회", description = "액세스 토큰을 사용해 회원 정보 조회")
    public ResponseEntity<?> info(Authentication authentication) {
        HttpStatus status = HttpStatus.ACCEPTED;

        try {
            UserInfoResponseDTO response = userService.info(authentication.getName());

            status = HttpStatus.OK; // 200, 클라이언트 요청 성공

            return new ResponseEntity<>(response, status);
        } catch (UsernameNotFoundException e) {
            status = HttpStatus.NOT_FOUND; // 404, 클라이언트 요청 탐색 실패

            return new ResponseEntity<>(e.getMessage(), status);
        } catch (Exception e) {
            status = HttpStatus.INTERNAL_SERVER_ERROR; // 500, 서버 오류

            return new ResponseEntity<>(e.getMessage(), status);
        }
    }



}
