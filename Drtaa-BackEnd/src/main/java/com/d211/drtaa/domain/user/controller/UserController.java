package com.d211.drtaa.domain.user.controller;


import com.d211.drtaa.domain.user.dto.request.*;
import com.d211.drtaa.domain.user.dto.response.UserInfoResponseDTO;
import com.d211.drtaa.domain.user.service.CustomUserDetailsService;
import com.d211.drtaa.domain.user.service.UserService;
import com.d211.drtaa.global.config.jwt.JwtToken;
import com.d211.drtaa.global.exception.user.UserCreationException;
import com.d211.drtaa.global.exception.user.UserNicknameDuplicateException;
import io.swagger.v3.oas.annotations.Operation;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.dao.DataIntegrityViolationException;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.security.core.Authentication;
import org.springframework.security.core.userdetails.UsernameNotFoundException;
import org.springframework.web.bind.annotation.*;
import io.swagger.v3.oas.annotations.tags.Tag;
import org.springframework.web.multipart.MultipartFile;

@RestController
@RequestMapping("/user")
@RequiredArgsConstructor
@Slf4j
@Tag(name = "회원 컨트롤러", description = "회원관련 기능 수행")
public class UserController {

    private final CustomUserDetailsService userDetailsService;
    private final UserService userService;

    @PostMapping(value = "/signup", consumes = MediaType.MULTIPART_FORM_DATA_VALUE)
    @Operation(summary = "회원가입", description = "Form 회원가입")
    public ResponseEntity signUp(
            @RequestPart("formSignUpRequestDTO") FormSignUpRequestDTO formSignUpRequestDTO,
            @RequestPart("image") MultipartFile image) {
        try {
            userDetailsService.createUser(formSignUpRequestDTO, image);

            // 200, 클라이언트 요청 성공
            return ResponseEntity.ok("회원가입 성공");
        } catch (DataIntegrityViolationException e) {
            // 409, 리소스 간의 충돌이 발생했을 때
            return ResponseEntity.status(HttpStatus.CONFLICT).body(e.getMessage());
        } catch (UserCreationException e) {
            // 400, 잘못된 요청에 대한 예외 처리
            return ResponseEntity.badRequest().body(e.getMessage());
        } catch (Exception e) {
            // 500, 서버 오류가 발생했을 때
            return ResponseEntity.status(HttpStatus.INTERNAL_SERVER_ERROR).body(e.getMessage());
        }
    }

    @GetMapping("/signup/{userPorviderId}")
    @Operation(summary = "아이디 중복 체크", description = "회원가입 시 아이디 중복 체크(소셜 로그인은 중복 위험 미존재)")
    public ResponseEntity checkNickname(@RequestParam("userPorviderId") String userPorviderId) {
        try {
            boolean chk = userService.chkUSerPorviderId(userPorviderId);

            // 닉네임 중복이 발생하지 않음
            return ResponseEntity.ok(chk);
        } catch (UserNicknameDuplicateException e) {
            // 409, 리소스 간의 충돌이 발생했을 때 -> 닉네임 중복 발생
            return ResponseEntity.status(HttpStatus.CONFLICT).body(true);
        } catch (Exception e) {
            // 500, 서버 오류가 발생했을 때
            return ResponseEntity.status(HttpStatus.INTERNAL_SERVER_ERROR).body(e.getMessage());
        }
    }

    @PostMapping("/login/form")
    @Operation(summary = "로그인", description = "Form 로그인")
    public ResponseEntity login(@RequestBody FormLoginRequestDTO request) {
        try {
            JwtToken tokens = userService.FormLogin(request);

            // 200, 클라이언트 요청 성공
            return ResponseEntity.ok(tokens);
        } catch(UsernameNotFoundException e) {
            // 404, 클라이언트 요청 탐색 실패
            return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage());
        } catch (Exception e) {
            // 400, 잘못된 요청
            return ResponseEntity.badRequest().body(e.getMessage());
        }
    }

    @PostMapping("/login/social")
    @Operation(summary = "로그인", description = "Social 로그인")
    public ResponseEntity login(@RequestBody SocialLoginRequestDTO request) {
        try {
            
            JwtToken tokens = userService.SocialLogin(request);

            // 200, 클라이언트 요청 성공
            return ResponseEntity.ok(tokens);
        } catch (Exception e) {
            // 400, 잘못된 요청
            return ResponseEntity.badRequest().body(e.getMessage());
        }
    }

    @GetMapping("/info")
    @Operation(summary = "회원 정보 조회", description = "액세스 토큰을 사용해 회원 정보 조회")
    public ResponseEntity info(Authentication authentication) {
        try {
            UserInfoResponseDTO response = userService.info(authentication.getName());

            // 200, 클라이언트 요청 성공
            return ResponseEntity.ok(response);
        } catch (UsernameNotFoundException e) {
            // 401, 클라이언트 인증 실패
            return ResponseEntity.status(HttpStatus.UNAUTHORIZED).body(e.getMessage());
        } catch (Exception e) {
            // 400, 잘못된 요청
            return ResponseEntity.badRequest().body(e.getMessage());
        }
    }

    @PostMapping("/img")
    @Operation(summary = "회원 이미지 수정", description = "마이 페이지에서 회원 이미지 수정")
    public ResponseEntity updateImg
            (Authentication authentication, @RequestPart(value = "image") MultipartFile image) {
        try {
            userService.updateImg(authentication.getName(), image);

            return ResponseEntity.ok("이미지 수정 성공");
        } catch (UsernameNotFoundException e) {
            // 401, 클라이언트 인증 실패
            return ResponseEntity.status(HttpStatus.UNAUTHORIZED).body(e.getMessage());
        } catch (DataIntegrityViolationException e) {
            // 409, 리소스 간의 충돌이 발생했을 때
            return ResponseEntity.status(HttpStatus.CONFLICT).body(e.getMessage());
        } catch (Exception e) {
            // 500, 서버 오류가 발생했을 때
            return ResponseEntity.status(HttpStatus.INTERNAL_SERVER_ERROR).body(e.getMessage());
        }
    }

    @DeleteMapping("/img")
    @Operation(summary = "회원 이미지 삭제", description = "마이 페이지에서 회원 이미지 삭제")
    public ResponseEntity deleteImg
            (Authentication authentication) {
        try {
            userService.deleteImg(authentication.getName());

            return ResponseEntity.ok("이미지 삭제 성공");
        } catch (UsernameNotFoundException e) {
            // 401, 클라이언트 인증 실패
            return ResponseEntity.status(HttpStatus.UNAUTHORIZED).body(e.getMessage());
        } catch (DataIntegrityViolationException e) {
            // 409, 리소스 간의 충돌이 발생했을 때
            return ResponseEntity.status(HttpStatus.CONFLICT).body(e.getMessage());
        } catch (Exception e) {
            // 500, 서버 오류가 발생했을 때
            return ResponseEntity.status(HttpStatus.INTERNAL_SERVER_ERROR).body(e.getMessage());
        }
    }

    @PostMapping("/nickname")
    @Operation(summary = "회원 닉네임 수정", description = "마이 페이지에서 회원 닉네임 수정")
    public ResponseEntity updateNickname
            (Authentication authentication, @RequestBody String nickname) {
        try {
            userService.updateNickname(authentication.getName(), nickname);

            return ResponseEntity.ok("닉네임 수정 성공");
        } catch (UsernameNotFoundException e) {
            // 401, 클라이언트 인증 실패
            return ResponseEntity.status(HttpStatus.UNAUTHORIZED).body(e.getMessage());
        } catch (UserNicknameDuplicateException e) {
            // 409, 리소스 간의 충돌이 발생했을 때
            return ResponseEntity.status(HttpStatus.CONFLICT).body(false);
        } catch (Exception e) {
            // 500, 서버 오류가 발생했을 때
            return ResponseEntity.status(HttpStatus.INTERNAL_SERVER_ERROR).body(e.getMessage());
        }
    }

}
