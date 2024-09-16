package com.d211.drtaa.domain.rent.controller;

import com.d211.drtaa.domain.rent.dto.response.RentDetailResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentResponseDTO;
import com.d211.drtaa.domain.rent.service.RentService;
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
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import java.util.List;

@RestController
@RequestMapping("/rent")
@RequiredArgsConstructor
@Log4j2
@Tag(name = "렌트 컨트롤러", description = "렌트 이용 관련 기능 수행")
public class RentController {

    private final RentService rentService;

    @GetMapping
    @Operation(summary = "렌트 조회", description = "회원의 전체 렌트 조회")
    public ResponseEntity getAllRent(Authentication authentication) {
        try {
            List<RentResponseDTO> response = rentService.getAllRent(authentication.getName());

            return ResponseEntity.ok(response); // 200
        } catch (UsernameNotFoundException e) {
            return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage()); // 404
        } catch (AuthenticationException e) {
            return ResponseEntity.status(HttpStatus.UNAUTHORIZED).body("권한 인증에 실패하였습니다."); // 401
        } catch (Exception e) {
            return ResponseEntity.status(HttpStatus.BAD_REQUEST).body(e.getMessage()); // 400
        }
    }

    @GetMapping("/{rentId}")
    @Operation(summary = "렌트 상세 조회", description = "회원의 렌트 상세 조회")
    public ResponseEntity getAllRent(@PathVariable("rentId") Long rentId) {
        try {
            RentDetailResponseDTO response = rentService.getDetailRent(rentId);

            return ResponseEntity.ok(response); //200
        } catch(RentNotFoundException e) {
            return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage()); // 404
        } catch (Exception e) {
            return ResponseEntity.status(HttpStatus.BAD_REQUEST).body(e.getMessage()); // 400
        }
    }
}
