package com.d211.drtaa.domain.taxi.service;

import com.d211.drtaa.domain.rent.entity.Rent;
import com.d211.drtaa.domain.rent.entity.RentStatus;
import com.d211.drtaa.domain.rent.entity.car.RentCar;
import com.d211.drtaa.domain.rent.entity.car.RentCarSchedule;
import com.d211.drtaa.domain.rent.entity.car.RentDrivingStatus;
import com.d211.drtaa.domain.rent.repository.RentRepository;
import com.d211.drtaa.domain.rent.repository.car.RentCarRepository;
import com.d211.drtaa.domain.rent.repository.car.RentCarScheduleRepository;
import com.d211.drtaa.domain.taxi.dto.request.TaxiCreateRequestDTO;
import com.d211.drtaa.domain.taxi.dto.response.TaxiDetailResponseDTO;
import com.d211.drtaa.domain.taxi.repository.TaxiRepository;
import com.d211.drtaa.domain.user.entity.User;
import com.d211.drtaa.domain.user.repository.UserRepository;
import com.d211.drtaa.global.util.fcm.FcmMessage;
import com.d211.drtaa.global.util.fcm.FcmUtil;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

import java.time.LocalDate;
import java.time.LocalDateTime;
import java.util.List;
import java.util.stream.Collectors;

@Service
@RequiredArgsConstructor
@Log4j2
public class TaxiServiceImpl implements TaxiService {

    private final UserRepository userRepository;
    private final RentRepository rentRepository;
    private final RentCarRepository rentCarRepository;
    private final FcmUtil fcmUtil;
    private final RentCarScheduleRepository rentCarScheduleRepository;

    @Override
    @Transactional
    public TaxiDetailResponseDTO createTaxi(String userProviderId, TaxiCreateRequestDTO taxiCreateRequestDTO) {
        // 사용자 찾기
        User user = userRepository.findByUserProviderId(userProviderId)
                .orElseThrow(() -> new RuntimeException("해당 userProviderId의 맞는 회원을 찾을 수 없습니다."));

        // 시작, 종료 시간
        LocalDateTime startDateTime = taxiCreateRequestDTO.getTaxiStartTime();
        LocalDateTime endDateTime = taxiCreateRequestDTO.getTaxiEndTime();
        // ** 결제 **
        List<RentCar> availableCars = rentCarRepository.findAll().stream()
                .filter(car -> {
                    // 해당 차량의 일정 가져오기 (완료되지 않은 일정만 가져오기)
                    List<RentCarSchedule> schedules = rentCarScheduleRepository.findByRentCarAndRentCarScheduleIsDoneFalse(car);

                    // 일정이 없으면 바로 사용 가능하다고 판단
                    if (schedules.isEmpty()) {
                        return true;
                    }

                    // 일정이 있는 경우, 해당 기간에 겹치는 일정이 있는지 확인
                    boolean isAvailable = schedules.stream().allMatch(schedule -> {
                        return !(startDateTime.toLocalDate().isBefore(schedule.getRentCarScheduleEndDate()) &&
                                endDateTime.toLocalDate().isAfter(schedule.getRentCarScheduleStartDate()));
                    });

                    return isAvailable;
                })
                .collect(Collectors.toList());

        // 배차 가능한 차량이 없는 경우
        if (availableCars.isEmpty())
            throw new RuntimeException("해당 기간에 사용 가능한 차량이 없습니다.");

        // 배차 가능한 차량이 있는 경우
        RentCar availableCar = availableCars.get(0); // 가장 첫번째 차량 선택

        // 렌트 생성
        Rent taxi = Rent.builder()
                .user(user)
                .rentCar(availableCar)
                .travel(null)
                .rentStatus(RentStatus.reserved)
                .rentHeadCount(1)
                .rentPrice(taxiCreateRequestDTO.getTaxiPrice())
                .rentTime(taxiCreateRequestDTO.getTaxiTime())
                .rentStartTime(startDateTime)
                .rentEndTime(endDateTime)
                .rentDptLat(taxiCreateRequestDTO.getTaxiStartLat())
                .rentDptLon(taxiCreateRequestDTO.getTaxiStartLon())
                .build();

        rentRepository.save(taxi);

        // 택시 차량 상태 변경
        availableCar.setRentCarDrivingStatus(RentDrivingStatus.parking); // 주차(기본값)

        // 택시 차량 변경 상태 저장
        rentCarRepository.save(availableCar);

        RentCar taxiCar = taxi.getRentCar();

        // 응답 DTO 생성
        TaxiDetailResponseDTO response = TaxiDetailResponseDTO.builder()
                .taxiId(taxi.getRentId())
                .taxiStatus(taxi.getRentStatus())
                .taxiPrice(taxiCreateRequestDTO.getTaxiPrice())
                .taxiTime(taxiCreateRequestDTO.getTaxiTime())
                .taxiStartTime(taxiCreateRequestDTO.getTaxiStartTime())
                .taxiEndTime(taxiCreateRequestDTO.getTaxiEndTime())
                .taxiStartLat(taxiCreateRequestDTO.getTaxiStartLat())
                .taxiStartLon(taxiCreateRequestDTO.getTaxiStartLon())
                .taxiEndLat(taxiCreateRequestDTO.getTaxiEndLat())
                .taxiEndLon(taxiCreateRequestDTO.getTaxiEndLon())
                .taxiCreatedAt(taxi.getRentCreatedAt())
                .taxiCarId(taxiCar.getRentCarId())
                .taxiCarNumber(taxiCar.getRentCarNumber())
                .taxiCarManufacturer(taxiCar.getRentCarManufacturer())
                .taxiCarModel(taxiCar.getRentCarModel())
                .taxiCarImg(taxiCar.getRentCarImg())
                .startAddress(taxiCreateRequestDTO.getStartAddress())
                .endAddress(taxiCreateRequestDTO.getEndAddress())
                .build();

        // FCM 알림 전송
        FcmMessage.FcmDTO fcmDTO = fcmUtil.makeFcmDTO("🚕 택시 예약 완료", "택시가 예약되었습니다. 출발 시간을 확인해주세요.");
        fcmUtil.singleFcmSend(user, fcmDTO);

        return response;
    }
}