package com.d211.drtaa.domain.rent.controller.history;

import com.d211.drtaa.domain.rent.dto.response.UserDetailHistoryResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.UserHistoryResponseDTO;
import com.d211.drtaa.domain.rent.service.history.RentHistoryService;
import com.d211.drtaa.global.exception.rent.RentHistoryNotFoundException;
import com.d211.drtaa.global.exception.rent.RentNotFoundException;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.security.core.Authentication;
import org.springframework.security.core.AuthenticationException;
import org.springframework.security.core.userdetails.UsernameNotFoundException;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@RequestMapping("/rent-history")
@RequiredArgsConstructor
@Log4j2
@Tag(name = "렌트 기록 컨트롤러", description = "렌트 기록 관련 기능 수행")
public class RentHistoryController {

    private final RentHistoryService rentHistoryService;
    
    @GetMapping
    @Operation(summary = "렌트 기록 조회", description = "사용자의 전체 렌트 기록을 조회")
    public ResponseEntity myHistory(Authentication authentication) {
        try{
            List<UserHistoryResponseDTO> responseDTO = rentHistoryService.getHistory(authentication.getName());

            return ResponseEntity.ok().body(responseDTO); // 200
        } catch (UsernameNotFoundException e) {
            return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage()); // 404
        } catch (AuthenticationException e) {
            return ResponseEntity.status(HttpStatus.UNAUTHORIZED).body("권한 인증에 실패하였습니다."); // 401
        } catch (Exception e) {
            return ResponseEntity.badRequest().body(e.getMessage()); // 400
        }
    }

    @GetMapping("/{rentHistoryId}")
    @Operation(summary = "렌트 기록 상세 조회", description = "사용자의 렌트 기록 상세 조회")
    public ResponseEntity myDetailHistory(@PathVariable("rentHistoryId") Long rentHistoryId) {
        try {
            UserDetailHistoryResponseDTO responseDTO = rentHistoryService.getDetailHistory(rentHistoryId);

            return ResponseEntity.ok().body(responseDTO); // 200
        } catch (RentHistoryNotFoundException e) {
            return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage()); // 404
        } catch (Exception e) {
            return ResponseEntity.badRequest().body(e.getMessage()); // 400
        }
    }

    @PostMapping("/{rentId}")
    @Operation(summary = "렌트 기록 생성",  description = "사용자의 새로운 렌트 기록 생성")
    public ResponseEntity createHistory
            (Authentication authentication, @PathVariable("rentId") Long rentId) {
        try {
            UserDetailHistoryResponseDTO responseDTO = rentHistoryService.createHistory(authentication.getName(), rentId);

            return ResponseEntity.ok().body(responseDTO); // 200
        } catch (UsernameNotFoundException | RentNotFoundException e) {
            return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage()); // 404
        } catch (AuthenticationException e) {
            return ResponseEntity.status(HttpStatus.UNAUTHORIZED).body("권한 인증에 실패하였습니다."); // 401
        } catch (Exception e) {
            return ResponseEntity.status(HttpStatus.BAD_REQUEST).body(e.getMessage()); // 400
        }
    }
}
