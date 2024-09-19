package com.d211.drtaa.domain.rent.controller.car;

import com.d211.drtaa.domain.rent.dto.request.RentCarDriveStatusRequestDTO;
import com.d211.drtaa.domain.rent.dto.request.RentCarUnassignedDispatchStatusRequestDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarDriveStatusResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarResponseDTO;
import com.d211.drtaa.domain.rent.entity.car.RentCar;
import com.d211.drtaa.domain.rent.entity.car.RentDrivingStatus;
import com.d211.drtaa.domain.rent.service.car.RentCarService;
import com.d211.drtaa.global.exception.rent.NoAvailableRentCarException;
import com.d211.drtaa.global.exception.rent.RentCarNotFoundException;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
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

            String response = "차 (rentCarId: " + rentCarDriveStatusRequestDTO.getRentCarId() + ")의 배차 상태가 " + rentCarDriveStatusRequestDTO.getRentCarDrivingStatus() + "로 변경되었습니다.";
            return ResponseEntity.ok(response); // 200
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
}
