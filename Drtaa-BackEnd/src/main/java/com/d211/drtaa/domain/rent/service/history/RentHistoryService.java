package com.d211.drtaa.domain.rent.service.history;

import com.d211.drtaa.domain.rent.dto.response.UserDetailHistoryResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.UserHistoryResponseDTO;

public interface RentHistoryService {
    // 회원 전체 렌트 기록 조회
    UserHistoryResponseDTO getHistory(String userProviderId);

    // rentHistoryId의 해당하는 상세 렌트 기록 조회
    UserDetailHistoryResponseDTO getDetailHistory(Long rentHistoryId);
}
