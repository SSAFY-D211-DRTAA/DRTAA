package com.d211.drtaa.domain.rent.service.car;

import com.d211.drtaa.domain.rent.dto.request.*;
import com.d211.drtaa.domain.rent.dto.response.RentCarDriveStatusResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarDrivingResponseDTO;
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

    // rentId의 맞는 렌트 차량 첫호출
    RentCarLocationResponseDTO callRentCar(long rentId);

    // rentId의 맞는 렌트 차량 재호출
    RentCarLocationResponseDTO reCallRentCar(RentCarCallRequestDTO rentCarCallRequestDTO);

    // rentId의 맞는 렌트 차량 탑승(driving) 상태로 변경
    RentCarDrivingResponseDTO updateRentCarDriveStatustoDriving(RentCarParkingRequestDTO rentCarParkingRequestDTO);

    // rentId의 맞는 렌트 차량 하차(parking) 상태로 변경
    RentCarDrivingResponseDTO updateRentCarDriveStatustoParking(RentCarParkingRequestDTO rentCarParkingRequestDTO);

    // rentCarId의 맞는 렌트를 찾아 사용자에게 차량 상태 알림 전송
    void alarmToAndroid(RentCarDriveStatusRequestDTO rentCarDriveStatusRequestDTO);

    // rentCarId의 맞는 렌트를 찾아 사용자에게 차량 도착 예상 시간 알림 전송
    void arrivalToAndroid(RentCarArriveStatusRequestDTO rentCarArriveStatusRequestDTO);

}
