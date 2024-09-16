package com.d211.drtaa.domain.rent.service.car;

import com.d211.drtaa.domain.rent.dto.response.RentCarDispatchStatusResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarResponseDTO;

import java.util.List;

public interface RentCarService {
    // 전체 렌트 차량 배차 상태 조회
    List<RentCarDispatchStatusResponseDTO> getAllDispatchStatus();

    // rentCarId의 맞는 렌트 차량 배차 상태 조회
    RentCarDispatchStatusResponseDTO getDispatchStatus(Long rentCarId);

    // 미배차 상태 렌트 차량 조회
    List<RentCarResponseDTO> getUnassignedDispatchStatus();

    // 배차 상태 렌트 차량 조회
    List<RentCarResponseDTO> getAssignedDispatchStatus();
}
