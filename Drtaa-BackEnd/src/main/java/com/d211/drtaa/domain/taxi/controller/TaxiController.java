package com.d211.drtaa.domain.taxi.controller;


import com.d211.drtaa.domain.taxi.dto.request.TaxiCreateRequestDTO;
import com.d211.drtaa.domain.taxi.dto.response.TaxiDetailResponseDTO;
import com.d211.drtaa.domain.taxi.service.TaxiService;
import com.d211.drtaa.global.exception.rent.NoAvailableRentCarException;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.http.HttpStatus;
import org.springframework.security.core.Authentication;
import org.springframework.http.ResponseEntity;
import org.springframework.security.core.AuthenticationException;
import org.springframework.security.core.userdetails.UsernameNotFoundException;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequestMapping("/taxi")
@RequiredArgsConstructor
@Log4j2
@Tag(name = "택시 컨트롤러", description = "텍시 이용 관련 기능 수행")
public class TaxiController {
    
    private final TaxiService taxiService;
    
    @PostMapping
    @Operation(summary = "택시 요청", description = "택시 요청")
    public ResponseEntity createTaxi
            (Authentication authentication, @RequestBody TaxiCreateRequestDTO taxiCreateRequestDTO) {
        try {
            TaxiDetailResponseDTO response = taxiService.createTaxi(authentication.getName(), taxiCreateRequestDTO);
            return ResponseEntity.ok(response); // 200
        } catch (UsernameNotFoundException | NoAvailableRentCarException e) {
            return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage()); // 404
        } catch (AuthenticationException e) {
            return ResponseEntity.status(HttpStatus.UNAUTHORIZED).body("권한 인증에 실패하였습니다."); // 401
        } catch (Exception e) {
            return ResponseEntity.status(HttpStatus.BAD_REQUEST).body(e.getMessage()); // 400
        }
    }

}
