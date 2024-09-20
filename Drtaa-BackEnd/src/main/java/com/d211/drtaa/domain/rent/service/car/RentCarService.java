package com.d211.drtaa.domain.rent.service.car;

import com.d211.drtaa.domain.rent.dto.request.RentCarDriveStatusRequestDTO;
import com.d211.drtaa.domain.rent.dto.request.RentCarUnassignedDispatchStatusRequestDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarDriveStatusResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarLocationResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarResponseDTO;

import java.util.List;

public interface RentCarService {
    // 전체 렌트 차량 배차 상태 조회
    List<RentCarResponseDTO> getAllDispatchStatus();

    // 미배차 상태 렌트 차량 조회
    RentCarResponseDTO getUnassignedDispatchStatus(RentCarUnassignedDispatchStatusRequestDTO rentCarUnassignedDispatchStatusRequestDTO);

    // rentCarId의 맞는 렌트 차량 주행 상태 조회
    RentCarDriveStatusResponseDTO getDriveStatus(Long rentCarId);

    // rentCarId의 맞는 렌트 차량 주행 상태 수정
    void updateDriveStatus(RentCarDriveStatusRequestDTO rentCarDriveStatusRequestDTO);

    // rentId의 맞는 렌트 차량 호출
    RentCarLocationResponseDTO callRentCar(String name, long rentId);
}
