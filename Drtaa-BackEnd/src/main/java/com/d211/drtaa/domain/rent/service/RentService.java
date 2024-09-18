package com.d211.drtaa.domain.rent.service;

import com.d211.drtaa.domain.rent.dto.request.RentRequestDTO;
import com.d211.drtaa.domain.rent.dto.request.RentStatusRequestDTO;
import com.d211.drtaa.domain.rent.dto.request.RentTimeRequestDTO;
import com.d211.drtaa.domain.rent.dto.response.RentDetailResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentResponseDTO;

import java.util.List;

public interface RentService {
    // 전체 렌트 조회
    List<RentResponseDTO> getAllRent(String userProviderId);

    // rentId의 렌트 상세 조회
    RentDetailResponseDTO getDetailRent(Long rentId);

    // rentId의 렌트 변경
    void updateRent(RentRequestDTO rentRequestDTO);

    // rentId의 렌트 상태 변경
    void updateRentStatus(RentStatusRequestDTO rentStatusRequestDTO);

    // rentId의 렌트 시간 변경
    void updateRentTime(RentTimeRequestDTO rentTimeRequestDTO);
}
