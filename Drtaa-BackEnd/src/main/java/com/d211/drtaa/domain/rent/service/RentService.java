package com.d211.drtaa.domain.rent.service;

import com.d211.drtaa.domain.rent.dto.response.RentDetailResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentResponseDTO;

import java.util.List;

public interface RentService {
    // 전체 렌트 조회
    List<RentResponseDTO> getAllRent(String userProviderId);

    // rentId의 렌트 상세 조회
    RentDetailResponseDTO getDetailRent(Long rentId);
}
