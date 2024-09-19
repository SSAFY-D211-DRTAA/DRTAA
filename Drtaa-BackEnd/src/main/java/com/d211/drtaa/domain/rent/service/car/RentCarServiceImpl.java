package com.d211.drtaa.domain.rent.service.car;

import com.d211.drtaa.domain.rent.dto.request.RentCarDispatchStatusRequestDTO;
import com.d211.drtaa.domain.rent.dto.request.RentCarDriveStatusRequestDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarDispatchStatusResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarDriveStatusResponseDTO;
import com.d211.drtaa.domain.rent.entity.car.RentCar;
import com.d211.drtaa.domain.rent.repository.car.RentCarRepository;
import com.d211.drtaa.global.exception.rent.RentCarNotFoundException;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.stereotype.Service;

import java.util.ArrayList;
import java.util.List;

@Service
@RequiredArgsConstructor
@Log4j2
public class RentCarServiceImpl implements RentCarService {

    private final RentCarRepository rentCarRepository;

    @Override
    public List<RentCar> getAllDispatchStatus() {
        // 모든 렌트 차량 검색
        List<RentCar> rentCarsList = rentCarRepository.findAll();

        return rentCarsList;
    }

    @Override
    public RentCarDispatchStatusResponseDTO getDispatchStatus(Long rentCarId) {
        // rentCarId에 해당하는 렌트 차량 찾기
        RentCar car = rentCarRepository.findByRentCarId(rentCarId)
                .orElseThrow(() -> new RentCarNotFoundException("해당 rentCarId의 맞는 차량을 찾을 수 없습니다."));

        RentCarDispatchStatusResponseDTO response = RentCarDispatchStatusResponseDTO.builder()
                .rentCarId(car.getRentCarId())
                .rentCarNumber(car.getRentCarNumber())
                .rentCarIsDispatch(car.isRentCarIsDispatch())
                .build();

        return response;
    }

    @Override
    public List<RentCar> getUnassignedDispatchStatus() {
        // 미배차 상태 렌트 차량 검색
        List<RentCar> rentCarsList = rentCarRepository.findByRentCarIsDispatchFalse();

        return rentCarsList;
    }

    @Override
    public List<RentCar> getAssignedDispatchStatus() {
        // 배차 상태 렌트 차량 검색
        List<RentCar> rentCarsList = rentCarRepository.findByRentCarIsDispatchTrue();

        return rentCarsList;
    }

    @Override
    public RentCarDriveStatusResponseDTO getDriveStatus(Long rentCarId) {
        // rentCarId에 해당하는 렌트 차량 찾기
        RentCar car = rentCarRepository.findByRentCarId(rentCarId)
                .orElseThrow(() -> new RentCarNotFoundException("해당 rentCarId의 맞는 차량을 찾을 수 없습니다."));

        RentCarDriveStatusResponseDTO response = RentCarDriveStatusResponseDTO.builder()
                .rentCarId(car.getRentCarId())
                .rentCarNumber(car.getRentCarNumber())
                .rentCarDrivingStatus(car.getRentCarDrivingStatus())
                .build();

        return response;
    }

    @Override
    public void updateDispatchStatus(RentCarDispatchStatusRequestDTO rentCarDispatchStatusRequestDTO) {
        // rentCarId에 해당하는 렌트 차량 찾기
        RentCar car = rentCarRepository.findByRentCarId(rentCarDispatchStatusRequestDTO.getRentCarId())
                .orElseThrow(() -> new RentCarNotFoundException("해당 rentCarId의 맞는 차량을 찾을 수 없습니다."));

        // 상태 변경
        car.setRentCarIsDispatch(rentCarDispatchStatusRequestDTO.isRentCarIsDispatch());

        // 저장
        rentCarRepository.save(car);
    }

    @Override
    public void updateDriveStatus(RentCarDriveStatusRequestDTO rentCarDriveStatusRequestDTO) {
        // rentCarId에 해당하는 렌트 차량 찾기
        RentCar car = rentCarRepository.findByRentCarId(rentCarDriveStatusRequestDTO.getRentCarId())
                .orElseThrow(() -> new RentCarNotFoundException("해당 rentCarId의 맞는 차량을 찾을 수 없습니다."));

        // 상태 변경
        car.setRentCarDrivingStatus(rentCarDriveStatusRequestDTO.getRentCarDrivingStatus());

        // 저장
        rentCarRepository.save(car);
    }
}
