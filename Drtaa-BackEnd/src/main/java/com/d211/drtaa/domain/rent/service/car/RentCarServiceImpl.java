package com.d211.drtaa.domain.rent.service.car;

import com.d211.drtaa.domain.rent.dto.response.RentCarDispatchStatusResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarDriveStatusResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarResponseDTO;
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
    public List<RentCarDispatchStatusResponseDTO> getAllDispatchStatus() {
        // 모든 렌트 차량 검색
        List<RentCar> rentCarsList = rentCarRepository.findAll();

        List<RentCarDispatchStatusResponseDTO> response = new ArrayList<>();
        for(RentCar rentCar : rentCarsList) {
            RentCarDispatchStatusResponseDTO dto = RentCarDispatchStatusResponseDTO.builder()
                    .rentCarId(rentCar.getRentCarId())
                    .rentCarIsDispatch(rentCar.isRentCarIsDispatch())
                    .build();

            response.add(dto);
        }

        return response;
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
    public List<RentCarResponseDTO> getUnassignedDispatchStatus() {
        // 미배차 상태 렌트 차량 검색
        List<RentCar> rentCarsList = rentCarRepository.findByRentCarIsDispatchFalse();

        List<RentCarResponseDTO> response = new ArrayList<>();
        for(RentCar rentCar : rentCarsList) {
            RentCarResponseDTO dto = RentCarResponseDTO.builder()
                    .rentCarId(rentCar.getRentCarId())
                    .rentCarNumber(rentCar.getRentCarNumber())
                    .rentCarManufacturer(rentCar.getRentCarManufacturer())
                    .rentCarModel(rentCar.getRentCarModel())
                    .build();

            response.add(dto);
        }

        return response;
    }

    @Override
    public List<RentCarResponseDTO> getAssignedDispatchStatus() {
        // 배차 상태 렌트 차량 검색
        List<RentCar> rentCarsList = rentCarRepository.findByRentCarIsDispatchTrue();

        List<RentCarResponseDTO> response = new ArrayList<>();
        for(RentCar rentCar : rentCarsList) {
            RentCarResponseDTO dto = RentCarResponseDTO.builder()
                    .rentCarId(rentCar.getRentCarId())
                    .rentCarNumber(rentCar.getRentCarNumber())
                    .rentCarManufacturer(rentCar.getRentCarManufacturer())
                    .rentCarModel(rentCar.getRentCarModel())
                    .build();

            response.add(dto);
        }

        return response;
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
}
