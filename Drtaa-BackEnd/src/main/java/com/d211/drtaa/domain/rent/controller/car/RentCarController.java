package com.d211.drtaa.domain.rent.controller.car;

import com.d211.drtaa.domain.rent.dto.request.RentCarCallRequestDTO;
import com.d211.drtaa.domain.rent.dto.request.RentCarDriveStatusRequestDTO;
import com.d211.drtaa.domain.rent.dto.request.RentCarUnassignedDispatchStatusRequestDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarDriveStatusResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarLocationResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarResponseDTO;
import com.d211.drtaa.domain.rent.entity.car.RentDrivingStatus;
import com.d211.drtaa.domain.rent.service.car.RentCarService;
import com.d211.drtaa.global.exception.rent.NoAvailableRentCarException;
import com.d211.drtaa.global.exception.rent.RentCarNotFoundException;
import com.d211.drtaa.global.exception.rent.RentNotFoundException;
import com.d211.drtaa.global.exception.websocket.WebSocketDisConnectedException;
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
@RequestMapping("/rent-car")
@RequiredArgsConstructor
@Log4j2
@Tag(name = "렌트 차량 컨트롤러", description = "렌트 차량 관련 기능 수행")
public class RentCarController {

    private final RentCarService rentCarService;

    @GetMapping("/dispatch")
    @Operation(summary = "전체 배차 상태 조회", description = "전체 렌트 차량의 배차 상태 조회")
    public ResponseEntity getAllDispatchStatus() {
        try {
            List<RentCarResponseDTO> response = rentCarService.getAllDispatchStatus();

            return ResponseEntity.ok(response); // 200
        } catch (Exception e) {
            return ResponseEntity.badRequest().body(e.getMessage()); // 400
        }
    }

    @PostMapping("/dispatch")
    @Operation(summary = "미배차 차량 조회", description = "미배차 상태인 렌트 차량 조회")
    public ResponseEntity getUnassignedDispatchStatus(@RequestBody RentCarUnassignedDispatchStatusRequestDTO rentCarUnassignedDispatchStatusRequestDTO) {
        try {
            RentCarResponseDTO response = rentCarService.getUnassignedDispatchStatus(rentCarUnassignedDispatchStatusRequestDTO);

            return ResponseEntity.ok(response); // 200
        } catch (NoAvailableRentCarException e) {
            RentCarResponseDTO response =  RentCarResponseDTO.builder()
                    .isAvailable(false)
                    // 빈 객체로 응답
                    .rentCarId(0)
                    .rentCarNumber("")
                    .rentCarManufacturer("")
                    .rentCarModel("")
                    .rentCarImg("")
                    .rentCarDrivingStatus(RentDrivingStatus.parked)
                    .build();

            return ResponseEntity.ok(response); // 200
        } catch (Exception e) {
            return ResponseEntity.badRequest().body(e.getMessage()); // 400
        }
    }

    @PatchMapping("/drive")
    @Operation(summary = "주행 상태 수정", description = "rentCarId의 맞는 렌트 차량의 주행 상태 수정")
    public ResponseEntity updateDriveStatus(@RequestBody RentCarDriveStatusRequestDTO rentCarDriveStatusRequestDTO) {
        try {
            rentCarService.updateDriveStatus(rentCarDriveStatusRequestDTO);

            return ResponseEntity.ok(rentCarDriveStatusRequestDTO); // 200
        } catch (RentCarNotFoundException e) {
            return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage()); // 404
        } catch (Exception e) {
            return ResponseEntity.badRequest().body(e.getMessage()); // 400
        }
    }

    @GetMapping("/drive/{rentCarId}")
    @Operation(summary = "주행 상태 조회", description = "rentCarId의 맞는 렌트 차량의 주행 상태 조회")
    public ResponseEntity getDriveStatus(@PathVariable("rentCarId") Long rentCarId) {
        try {
            RentCarDriveStatusResponseDTO response = rentCarService.getDriveStatus(rentCarId);

            return ResponseEntity.ok(response); // 200
        } catch (RentCarNotFoundException e) {
            return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage()); // 404
        } catch (Exception e) {
            return ResponseEntity.badRequest().body(e.getMessage()); // 400
        }
    }

    @PostMapping("/call/{rentId}")
    @Operation(summary = "렌트 차량 첫호출", description = "회원의 진행할 렌트 차량 첫호출(렌트 요청 시 입력했던 탑승 위치 전송)")
    public ResponseEntity callRentCar(@PathVariable("rentId") long rentId) {
        try{
            RentCarLocationResponseDTO response = rentCarService.callRentCar(rentId);

            return ResponseEntity.ok(response); //200
        } catch (RentNotFoundException | RentCarNotFoundException e) {
            return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage()); // 404
        } catch (AuthenticationException e) {
            return ResponseEntity.status(HttpStatus.UNAUTHORIZED).body("권한 인증에 실패하였습니다."); // 401
        } catch (WebSocketDisConnectedException e) {
            return ResponseEntity.status(HttpStatus.INTERNAL_SERVER_ERROR).body(e.getMessage()); // 500
        } catch (Exception e) {
            return ResponseEntity.status(HttpStatus.BAD_REQUEST).body(e.getMessage()); // 400
        }
    }

    @PostMapping("/call")
    @Operation(summary = "렌트 차량 재호출", description = "회원의 진행중인 렌트 차량 재호출(회원 위치 전송)")
    public ResponseEntity reCallRentCar(@RequestBody RentCarCallRequestDTO rentCarCallRequestDTO) {
        try{
            RentCarLocationResponseDTO response = rentCarService.reCallRentCar(rentCarCallRequestDTO);

            return ResponseEntity.ok(response); //200
        } catch (RentNotFoundException | RentCarNotFoundException e) {
            return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage()); // 404
        } catch (AuthenticationException e) {
            return ResponseEntity.status(HttpStatus.UNAUTHORIZED).body("권한 인증에 실패하였습니다."); // 401
        } catch (WebSocketDisConnectedException e) {
            return ResponseEntity.status(HttpStatus.INTERNAL_SERVER_ERROR).body(e.getMessage()); // 500
        } catch (Exception e) {
            return ResponseEntity.status(HttpStatus.BAD_REQUEST).body(e.getMessage()); // 400
        }
    }
}
