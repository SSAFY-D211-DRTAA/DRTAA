package com.d211.drtaa.domain.rent.controller.car;

import com.d211.drtaa.domain.rent.dto.response.RentCarDispatchStatusResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarDriveStatusResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarResponseDTO;
import com.d211.drtaa.domain.rent.service.car.RentCarService;
import com.d211.drtaa.global.exception.rent.RentCarNotFoundException;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.http.HttpStatus;
import org.springframework.http.HttpStatusCode;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

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
            List<RentCarDispatchStatusResponseDTO> response = rentCarService.getAllDispatchStatus();

            return ResponseEntity.ok(response); // 200
        } catch (Exception e) {
            return ResponseEntity.badRequest().body(e.getMessage()); // 400
        }
    }

    @GetMapping("/dispatch/{rentCarId}")
    @Operation(summary = "특정 차량 배차 상태 조회", description = "rentCarId의 맞는 렌트 차량의 배차 상태 조회")
    public ResponseEntity getDispatchStatus(@PathVariable("rentCarId") Long rentCarId) {
        try {
            RentCarDispatchStatusResponseDTO response = rentCarService.getDispatchStatus(rentCarId);

            return ResponseEntity.ok(response); // 200
        } catch (RentCarNotFoundException e) {
            return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage()); // 404
        } catch (Exception e) {
            return ResponseEntity.badRequest().body(e.getMessage()); // 400
        }
    }

    @GetMapping("/dispatch/unassigned")
    @Operation(summary = "미배차 차량 조회", description = "미배차 상태인 렌트 차량 조회")
    public ResponseEntity getUnassignedDispatchStatus() {
        try {
            List<RentCarResponseDTO> response = rentCarService.getUnassignedDispatchStatus();

            return ResponseEntity.ok(response); // 200
        } catch (Exception e) {
            return ResponseEntity.badRequest().body(e.getMessage()); // 400
        }
    }

    @GetMapping("/dispatch/assigned")
    @Operation(summary = "배차 차량 조회", description = "배차 상태인 렌트 차량 조회")
    public ResponseEntity getAssignedDispatchStatus() {
        try {
            List<RentCarResponseDTO> response = rentCarService.getAssignedDispatchStatus();

            return ResponseEntity.ok(response); // 200
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
