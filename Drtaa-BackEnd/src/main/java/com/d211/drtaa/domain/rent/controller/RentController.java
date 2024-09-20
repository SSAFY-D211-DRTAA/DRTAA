package com.d211.drtaa.domain.rent.controller;

import com.d211.drtaa.domain.rent.dto.request.RentStatusRequestDTO;
import com.d211.drtaa.domain.rent.dto.request.RentCreateRequestDTO;
import com.d211.drtaa.domain.rent.dto.request.RentEditRequestDTO;
import com.d211.drtaa.domain.rent.dto.request.RentTimeRequestDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarLocationResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentDetailResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentResponseDTO;
import com.d211.drtaa.domain.rent.service.RentService;
import com.d211.drtaa.global.exception.rent.NoAvailableRentCarException;
import com.d211.drtaa.global.exception.rent.RentCarNotFoundException;
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
    public ResponseEntity getAllRent(@PathVariable("rentId") long rentId) {
        try {
            RentDetailResponseDTO response = rentService.getDetailRent(rentId);

            return ResponseEntity.ok(response); //200
        } catch(RentNotFoundException e) {
            return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage()); // 404
        } catch (Exception e) {
            return ResponseEntity.status(HttpStatus.BAD_REQUEST).body(e.getMessage()); // 400
        }
    }

    @GetMapping("/current")
    @Operation(summary = "진행중인 렌트 상세 조회", description = "회원의 현재 진행중인 렌트 상세 조회")
    public ResponseEntity getCurrentRent(Authentication authentication) {
        try {
            RentDetailResponseDTO response = rentService.getCurrentRent(authentication.getName());

            return ResponseEntity.ok(response); //200
        } catch(RentNotFoundException e) {
            return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage()); // 404
        } catch (Exception e) {
            return ResponseEntity.status(HttpStatus.BAD_REQUEST).body(e.getMessage()); // 400
        }
    }

    @PostMapping
    @Operation(summary = "렌트 요청", description = "렌트 요청")
    public ResponseEntity createRent
            (Authentication authentication, @RequestBody RentCreateRequestDTO rentCreateRequestDTO) {
        try {
            RentDetailResponseDTO response = rentService.createRent(authentication.getName(), rentCreateRequestDTO);

            return ResponseEntity.ok(response); // 200
        } catch (UsernameNotFoundException | NoAvailableRentCarException e) {
            return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage()); // 404
        } catch (AuthenticationException e) {
            return ResponseEntity.status(HttpStatus.UNAUTHORIZED).body("권한 인증에 실패하였습니다."); // 401
        } catch (Exception e) {
            return ResponseEntity.status(HttpStatus.BAD_REQUEST).body(e.getMessage()); // 400
        }
    }

    @PatchMapping
    @Operation(summary = "렌트 변경", description = "렌트 인원, 장소 변경")
    public ResponseEntity updateRent(@RequestBody RentEditRequestDTO rentEditRequestDTO) {
        try {
            rentService.updateRent(rentEditRequestDTO);

            return ResponseEntity.ok("Success"); //200
        } catch(RentNotFoundException e) {
            log.error(e.getMessage());
            return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage()); // 404
        } catch (Exception e) {
            log.error(e.getMessage());
            return ResponseEntity.status(HttpStatus.BAD_REQUEST).body(e.getMessage()); // 400
        }
    }

    @PatchMapping("/status/{rentId}/in-progress")
    @Operation(summary = "렌트 상태 변경(탑승)", description = "렌트 취소")
    public ResponseEntity rentStatusInProgress(@PathVariable("rentId") Long rentId) {
        try {
            rentService.rentStatusInProgress(rentId);

            return ResponseEntity.ok("Success");
        } catch(RentNotFoundException e) {
            log.error(e.getMessage());
            return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage()); // 404
        } catch (Exception e) {
            log.error(e.getMessage());
            return ResponseEntity.status(HttpStatus.BAD_REQUEST).body(e.getMessage()); // 400
        }
    }

    @PatchMapping("/status/completed")
    @Operation(summary = "렌트 상태 변경(반납)", description = "렌트 취소")
    public ResponseEntity rentStatusCompleted(@RequestBody RentStatusRequestDTO requestDTO) {
        try {
            rentService.rentStatusCompleted(requestDTO);

            return ResponseEntity.ok("Success");
        } catch(RentNotFoundException | RentCarNotFoundException e) {
            log.error(e.getMessage());
            return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage()); // 404
        } catch (Exception e) {
            log.error(e.getMessage());
            return ResponseEntity.status(HttpStatus.BAD_REQUEST).body(e.getMessage()); // 400
        }
    }

    @PatchMapping("/status/canceled")
    @Operation(summary = "렌트 상태 변경(취소)", description = "렌트 취소")
    public ResponseEntity rentStatusCanceld(@RequestBody RentStatusRequestDTO requestDTO) {
        try {
            rentService.rentStatusCanceld(requestDTO);

            return ResponseEntity.ok("Success");
        } catch(RentNotFoundException | RentCarNotFoundException e) {
            log.error(e.getMessage());
            return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage()); // 404
        } catch (Exception e) {
            log.error(e.getMessage());
            return ResponseEntity.status(HttpStatus.BAD_REQUEST).body(e.getMessage()); // 400
        }
    }

    @PatchMapping("/time")
    @Operation(summary = "렌트 시간 연장", description = "렌트 연장된 이용 시간 변경")
    public ResponseEntity updateRentTime(@RequestBody RentTimeRequestDTO rentTimeRequestDTO) {
        try {
            rentService.updateRentTime(rentTimeRequestDTO);

            return ResponseEntity.ok("Success"); //200
        } catch(RentNotFoundException e) {
            log.error(e.getMessage());
            return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage()); // 404
        } catch (Exception e) {
            log.error(e.getMessage());
            return ResponseEntity.status(HttpStatus.BAD_REQUEST).body(e.getMessage()); // 400
        }
    }
}
