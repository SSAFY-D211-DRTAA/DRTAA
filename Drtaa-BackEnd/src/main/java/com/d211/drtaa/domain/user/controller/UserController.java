package com.d211.drtaa.domain.user.controller;


import com.d211.drtaa.domain.user.dto.request.*;
import com.d211.drtaa.domain.user.dto.response.UserInfoResponseDTO;
import com.d211.drtaa.domain.user.service.CustomUserDetailsService;
import com.d211.drtaa.domain.user.service.UserService;
import com.d211.drtaa.global.exception.auth.InvalidTokenException;
import com.d211.drtaa.global.util.jwt.JwtToken;
import com.d211.drtaa.global.exception.user.UserCreationException;
import com.d211.drtaa.global.exception.user.UserNicknameDuplicateException;
import io.jsonwebtoken.ExpiredJwtException;
import io.swagger.v3.oas.annotations.Operation;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.dao.DataIntegrityViolationException;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.security.access.AccessDeniedException;
import org.springframework.security.authentication.BadCredentialsException;
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
            @RequestPart(value = "image", required = false) MultipartFile image) {
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

    @GetMapping("/signup/{userProviderId}")
    @Operation(summary = "아이디 중복 체크", description = "회원가입 시 아이디 중복 체크(소셜 로그인은 중복 위험 미존재)")
    public ResponseEntity checkNickname(@PathVariable("userProviderId") String userProviderId) {
        try {
            boolean chk = userService.chkuserProviderId(userProviderId);

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
    @Operation(summary = "폼 로그인", description = "폼 로그인")
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
    @Operation(summary = "소셜 로그인", description = "소셜 로그인")
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

    @PostMapping("/token")
    @Operation(summary = "토큰 재발급", description = "유효기간 만료로 인한 JWT 토큰 재발급")
    public ResponseEntity<?> updateToken(@RequestParam String userRefreshToken) {
        try {
            JwtToken tokens = userService.updateToken(userRefreshToken);

            return ResponseEntity.ok(tokens);
        } catch (InvalidTokenException e) {
            // 리프레시 토큰이 만료된 경우
            return ResponseEntity.status(HttpStatus.UNAUTHORIZED).body("리프레시 토큰이 만료되었습니다. 다시 로그인해주세요.");
        } catch (UsernameNotFoundException e) {
            // 사용자 없음
            return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage());
        } catch (Exception e) {
            // 기타 예외
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
            // 404, 사용자 정보가 없음
            return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage());
        } catch (ExpiredJwtException e) {
            // 400, 잘못된 요청
            return ResponseEntity.badRequest().body(e.getMessage());
        } catch (Exception e) {
            // 400, 잘못된 요청
            return ResponseEntity.badRequest().body(e.getMessage());
        }
    }

    @PatchMapping(value = "/img", consumes = MediaType.MULTIPART_FORM_DATA_VALUE)
    @Operation(summary = "회원 이미지 수정", description = "마이 페이지에서 회원 이미지 수정")
    public ResponseEntity updateImg
            (Authentication authentication, @RequestPart(value = "image", required = false) MultipartFile image) {
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
            // 400, 잘못된 요청
            return ResponseEntity.status(HttpStatus.BAD_REQUEST).body(e.getMessage());
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
            // 400, 잘못된 요청
            return ResponseEntity.status(HttpStatus.BAD_REQUEST).body(e.getMessage());
        }
    }

    @PatchMapping("/nickname")
    @Operation(summary = "회원 닉네임 수정", description = "마이 페이지에서 회원 닉네임 수정")
    public ResponseEntity updateNickname
            (Authentication authentication, @RequestParam String nickname) {
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
            // 400, 잘못된 요청
            return ResponseEntity.status(HttpStatus.BAD_REQUEST).body(e.getMessage());
        }
    }

    @PatchMapping("/password")
    @Operation(summary = "회원 비밀번호 수정", description = "마이 페이지에서 회원 비밀번호 수정")
    public ResponseEntity updatePassword
            (Authentication authentication, @RequestBody PasswordChangeRequestDTO passwordChangeRequestDTO) {
        try {
            userDetailsService.changePassword(passwordChangeRequestDTO.getOldPassword(), passwordChangeRequestDTO.getNewPassword());
            
            return ResponseEntity.ok("비밀번호 수정 성공");
        } catch (AccessDeniedException e) {
            // 401, 클라이언트 인증 실패
            return ResponseEntity.status(HttpStatus.UNAUTHORIZED).body(e.getMessage());
        } catch (BadCredentialsException e) {
            // 409, 리소스 간의 충돌이 발생했을 때
            return ResponseEntity.status(HttpStatus.CONFLICT).body(e.getMessage());
        } catch (Exception e) {
            // 400, 잘못된 요청
            return ResponseEntity.status(HttpStatus.BAD_REQUEST).body(e.getMessage());
        }
    }

    @DeleteMapping
    @Operation(summary = "회원 탈퇴", description = "회월 탈퇴 시행")
    public ResponseEntity deleteUser(Authentication authentication) {
        try {
            userService.delete(authentication.getName());

            return ResponseEntity.ok("회원 탈퇴 성공");
        } catch (Exception e) {
            return ResponseEntity.status(HttpStatus.BAD_REQUEST).body(e.getMessage());
        }
    }
}
