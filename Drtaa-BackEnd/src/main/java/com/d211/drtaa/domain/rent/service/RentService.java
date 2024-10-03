package com.d211.drtaa.domain.rent.service;

import com.d211.drtaa.domain.rent.dto.request.*;
import com.d211.drtaa.domain.rent.dto.response.RentCarManipulateResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentDetailResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentResponseDTO;

import java.util.List;

public interface RentService {
    // 전체 렌트 조회
    List<RentResponseDTO> getAllRent(String userProviderId);

    // 완료된 전체 렌트 조회
    List<RentResponseDTO> getCompletedRent(String userProviderId);

    // 진행중 & 예약된 렌트 전체 조회
    List<RentResponseDTO> getActiveRent(String userProviderId);

    // rentId의 렌트 상세 조회
    RentDetailResponseDTO getDetailRent(long rentId);

    // 현재 진행중인 렌트 조회
    RentDetailResponseDTO getCurrentRent(String userProviderId);

    // 시작, 종료 날짜의 렌트가 존재하는지 확인
    boolean chkRent(String userProviderId, RentCheckRequestDTO rentCheckRequestDTO);

    // 렌트 요청
    RentDetailResponseDTO createRent(String userProviderId, RentCreateRequestDTO rentCreateRequestDTO);

    // rentId의 렌트 변경
    void updateRent(RentEditRequestDTO rentEditRequestDTO);

    // rentId의 렌트 상태: 반납
    void rentStatusCompleted(RentStatusRequestDTO requestDTO);

    // rentId의 렌트 상태: 취소
    void rentStatusCanceld(RentStatusRequestDTO requestDTO);

    // rentId의 렌트 시간 변경
    void updateRentTime(RentTimeRequestDTO rentTimeRequestDTO);

    // 오늘에 해당하는 렌트 만료 처리
    RentCarManipulateResponseDTO todayRentIsDone(RentCarManipulateRequestDTO rentCarManipulateRequestDTO);
}
