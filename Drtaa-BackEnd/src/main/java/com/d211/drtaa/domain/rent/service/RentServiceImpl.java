package com.d211.drtaa.domain.rent.service;

import com.d211.drtaa.domain.rent.dto.request.RentCreateRequestDTO;
import com.d211.drtaa.domain.rent.dto.request.RentEditRequestDTO;
import com.d211.drtaa.domain.rent.dto.request.RentStatusRequestDTO;
import com.d211.drtaa.domain.rent.dto.request.RentTimeRequestDTO;
import com.d211.drtaa.domain.rent.dto.response.RentDetailResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentResponseDTO;
import com.d211.drtaa.domain.rent.entity.Rent;
import com.d211.drtaa.domain.rent.entity.RentStatus;
import com.d211.drtaa.domain.rent.entity.car.RentCar;
import com.d211.drtaa.domain.rent.entity.car.RentDrivingStatus;
import com.d211.drtaa.domain.rent.repository.RentRepository;
import com.d211.drtaa.domain.rent.repository.car.RentCarRepository;
import com.d211.drtaa.domain.user.entity.User;
import com.d211.drtaa.domain.user.repository.UserRepository;
import com.d211.drtaa.global.exception.rent.NoAvailableRentCarException;
import com.d211.drtaa.global.exception.rent.RentNotFoundException;
import jakarta.transaction.Transactional;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.security.core.userdetails.UsernameNotFoundException;
import org.springframework.stereotype.Service;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;

@Service
@RequiredArgsConstructor
@Log4j2
public class RentServiceImpl implements RentService{

    private final UserRepository userRepository;
    private final RentRepository rentRepository;
    private final RentCarRepository rentCarRepository;

    @Override
    public List<RentResponseDTO> getAllRent(String userProviderId) {
        // 사용자 찾기
        User user = userRepository.findByUserProviderId(userProviderId)
                .orElseThrow(() -> new UsernameNotFoundException("해당 userProviderId의 맞는 회원을 찾을 수 없습니다."));

        // 사용자의 렌트 찾기
        List<Rent> rents = rentRepository.findByUser(user);

        List<RentResponseDTO> response = new ArrayList<>();
        for(Rent rent: rents) {
            RentResponseDTO dto = RentResponseDTO.builder()
                    .rentStatus(rent.getRentStatus())
                    .rentHeadCount(rent.getRentHeadCount())
                    .rentTime(rent.getRentTime())
                    .rentStartTime(rent.getRentStartTime())
                    .build();

            response.add(dto);
        }

        return response;
    }

    @Override
    public RentDetailResponseDTO getDetailRent(Long rentId) {
        Rent rent = rentRepository.findByRentId(rentId)
                .orElseThrow(() -> new RentNotFoundException("해당 rentId의 맞는 렌트를 찾을 수 없습니다."));

        RentCar rentCar = rent.getRentCar();

        RentDetailResponseDTO response = RentDetailResponseDTO.builder()
                .rentStatus(rent.getRentStatus())
                .rentHeadCount(rent.getRentHeadCount())
                .rentPrice(rent.getRentPrice())
                .rentTime(rent.getRentTime())
                .rentStartTime(rent.getRentStartTime())
                .rentEndTime(rent.getRentEndTime())
                .rentDptLat(rent.getRentDptLat())
                .rentDptLon(rent.getRentDptLon())
                .rentCreatedAt(rent.getRentCreatedAt())
                // rent-car
                .rentCarId(rentCar.getRentCarId())
                .rentCarNumber(rentCar.getRentCarNumber())
                .rentCarManufacturer(rentCar.getRentCarManufacturer())
                .rentCarModel(rentCar.getRentCarModel())
                .build();

        return response;
    }

    @Override
    @Transactional
    public RentDetailResponseDTO createRent(String userProviderId, RentCreateRequestDTO rentCreateRequestDTO) {
        // 사용자 찾기
        User user = userRepository.findByUserProviderId(userProviderId)
                .orElseThrow(() -> new UsernameNotFoundException("해당 userProviderId의 맞는 회원을 찾을 수 없습니다."));

        // 미배차 상태 렌트 차량 탐색
        RentCar availableCar = rentCarRepository.findFirstByRentCarIsDispatch(false)
                .orElseThrow(() -> new NoAvailableRentCarException("현재 배정 가능한 차량이 없습니다."));

        // 렌트 종료 시간
        LocalDateTime rentEndTime = rentCreateRequestDTO.getRentStartTime().plusMinutes(rentCreateRequestDTO.getRentTime());

        // 렌트 생성
        Rent rent = Rent.builder()
                .user(user)
                .rentCar(availableCar)
                .rentStatus(RentStatus.RESERVED)
                .rentHeadCount(rentCreateRequestDTO.getRentHeadCount())
                .rentPrice(rentCreateRequestDTO.getRentPrice())
                .rentTime(rentCreateRequestDTO.getRentTime())
                .rentStartTime(rentCreateRequestDTO.getRentStartTime())
                .rentEndTime(rentEndTime)
                .rentDptLat(rentCreateRequestDTO.getRentDptLat())
                .rentDptLon(rentCreateRequestDTO.getRentDptLon())
                .build();

        // 생성된 렌트 저장
        rentRepository.save(rent);

        // 렌트 차량 상태 변경
        availableCar.setRentCarIsDispatch(true); // 배차 상태
        availableCar.setRentCarDrivingStatus(RentDrivingStatus.PARKED);// 주행 상태

        // 변경된 렌트 차량 상태 저장
        rentCarRepository.save(availableCar);
        
        // 결과
        RentDetailResponseDTO response = RentDetailResponseDTO.builder()
                // rent
                .rentStatus(rent.getRentStatus())
                .rentHeadCount(rent.getRentHeadCount())
                .rentPrice(rent.getRentPrice())
                .rentTime(rent.getRentTime())
                .rentStartTime(rent.getRentStartTime())
                .rentEndTime(rent.getRentEndTime())
                .rentDptLat(rent.getRentDptLat())
                .rentDptLon(rent.getRentDptLon())
                .rentCreatedAt(rent.getRentCreatedAt())
                // rent-car
                .rentCarId(rent.getRentCar().getRentCarId())
                .rentCarNumber(rent.getRentCar().getRentCarNumber())
                .rentCarManufacturer(rent.getRentCar().getRentCarManufacturer())
                .rentCarModel(rent.getRentCar().getRentCarModel())
                .build();

        return response;
    }

    @Override
    @Transactional
    public void updateRent(RentEditRequestDTO rentEditRequestDTO) {
        // 렌트 찾기
        Rent rent = rentRepository.findByRentId(rentEditRequestDTO.getRentId())
                .orElseThrow(() -> new RentNotFoundException("해당 rentId의 맞는 렌트를 찾을 수 없습니다."));

        // 상태 변경
        rent.setRentHeadCount(rentEditRequestDTO.getRentHeadCount() != null ? rentEditRequestDTO.getRentHeadCount() : rent.getRentHeadCount());
        rent.setRentDptLat(rentEditRequestDTO.getRentDptLat() != null ? rentEditRequestDTO.getRentDptLat() : rent.getRentDptLat());
        rent.setRentDptLon(rentEditRequestDTO.getRentDptLon() != null ? rentEditRequestDTO.getRentDptLon() : rent.getRentDptLon());

        // 변경 상태 저장
        rentRepository.save(rent);
    }

    @Override
    @Transactional
    public void updateRentStatus(RentStatusRequestDTO rentStatusRequestDTO) {
        // 렌트 찾기
        Rent rent = rentRepository.findByRentId(rentStatusRequestDTO.getRentId())
                .orElseThrow(() -> new RentNotFoundException("해당 rentId의 맞는 렌트를 찾을 수 없습니다."));

        // 상태 변경
        rent.setRentStatus(rentStatusRequestDTO.getRentStatus());

        // 변경 상태 저장
        rentRepository.save(rent);
    }

    @Override
    @Transactional
    public void updateRentTime(RentTimeRequestDTO rentTimeRequestDTO) {
        // 렌트 찾기
        Rent rent = rentRepository.findByRentId(rentTimeRequestDTO.getRentId())
                .orElseThrow(() -> new RentNotFoundException("해당 rentId의 맞는 렌트를 찾을 수 없습니다."));

        // 상태 변경
        rent.setRentTime(rentTimeRequestDTO.getRentTime());

        // 변경 상태 저장
        rentRepository.save(rent);
    }

    @Override
    @Transactional
    public void deleteRent(Long rentId) {
        // 렌트 찾기
        Rent rent = rentRepository.findByRentId(rentId)
                .orElseThrow(() -> new RentNotFoundException("해당 rentId의 맞는 렌트를 찾을 수 없습니다."));

        // 1. DB에서 삭제하기
        // rentRepository.delete(rent);

        // 2. 상태만 변경하기
        // 2-1. 상태 변경
        rent.setRentStatus(RentStatus.CANCELLED);

        // 2-2. 변경 상태 저장
        rentRepository.save(rent);
    }
}
