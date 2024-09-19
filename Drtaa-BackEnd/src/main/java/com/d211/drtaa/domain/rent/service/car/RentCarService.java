package com.d211.drtaa.domain.rent.service.car;

import com.d211.drtaa.domain.rent.dto.request.RentCarDispatchStatusRequestDTO;
import com.d211.drtaa.domain.rent.dto.request.RentCarDriveStatusRequestDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarDispatchStatusResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarDriveStatusResponseDTO;
import com.d211.drtaa.domain.rent.entity.car.RentCar;

import java.util.List;

public interface RentCarService {
    // 전체 렌트 차량 배차 상태 조회
    List<RentCar> getAllDispatchStatus();

    // rentCarId의 맞는 렌트 차량 배차 상태 조회
    RentCarDispatchStatusResponseDTO getDispatchStatus(Long rentCarId);

    // 미배차 상태 렌트 차량 조회
    List<RentCar> getUnassignedDispatchStatus();

    // 배차 상태 렌트 차량 조회
    List<RentCar> getAssignedDispatchStatus();

    // rentCarId의 맞는 렌트 차량 주행 상태 조회
    RentCarDriveStatusResponseDTO getDriveStatus(Long rentCarId);

    // rentCarId의 맞는 렌트 차량 배차 상태 수정
    void updateDispatchStatus(RentCarDispatchStatusRequestDTO rentCarDispatchStatusRequestDTO);

    // rentCarId의 맞는 렌트 차량 주행 상태 수정
    void updateDriveStatus(RentCarDriveStatusRequestDTO rentCarDriveStatusRequestDTO);
}
