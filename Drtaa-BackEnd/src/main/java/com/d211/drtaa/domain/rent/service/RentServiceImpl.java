package com.d211.drtaa.domain.rent.service;

import com.d211.drtaa.domain.rent.dto.request.*;
import com.d211.drtaa.domain.rent.dto.response.RentCarManipulateResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentDetailResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentStatusResponseDTO;
import com.d211.drtaa.domain.rent.entity.Rent;
import com.d211.drtaa.domain.rent.entity.RentStatus;
import com.d211.drtaa.domain.rent.entity.car.RentCar;
import com.d211.drtaa.domain.rent.entity.car.RentCarSchedule;
import com.d211.drtaa.domain.rent.entity.car.RentDrivingStatus;
import com.d211.drtaa.domain.rent.entity.history.RentHistory;
import com.d211.drtaa.domain.rent.repository.RentRepository;
import com.d211.drtaa.domain.rent.repository.car.RentCarRepository;
import com.d211.drtaa.domain.rent.repository.car.RentCarScheduleRepository;
import com.d211.drtaa.domain.rent.repository.history.RentHistoryRepository;
import com.d211.drtaa.domain.travel.entity.DatePlaces;
import com.d211.drtaa.domain.travel.entity.Travel;
import com.d211.drtaa.domain.travel.entity.TravelDates;
import com.d211.drtaa.domain.travel.repository.DatePlacesRepository;
import com.d211.drtaa.domain.travel.repository.TravelDatesRepository;
import com.d211.drtaa.domain.travel.repository.TravelRepository;
import com.d211.drtaa.domain.user.entity.User;
import com.d211.drtaa.domain.user.repository.UserRepository;
import com.d211.drtaa.global.config.websocket.MyMessage;
import com.d211.drtaa.global.config.websocket.WebSocketConfig;
import com.d211.drtaa.global.exception.rent.RentCarNotFoundException;
import com.d211.drtaa.global.exception.rent.RentCarScheduleNotFoundException;
import com.d211.drtaa.global.exception.rent.RentNotFoundException;
import com.d211.drtaa.global.exception.travel.TravelAllPlacesVisitedException;
import com.d211.drtaa.global.exception.travel.TravelNotFoundException;
import com.d211.drtaa.global.exception.websocket.WebSocketDisConnectedException;
import com.d211.drtaa.global.util.fcm.FcmMessage;
import com.d211.drtaa.global.util.fcm.FcmUtil;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.google.firebase.messaging.Message;
import jakarta.transaction.Transactional;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.security.core.userdetails.UsernameNotFoundException;
import org.springframework.stereotype.Service;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.client.standard.StandardWebSocketClient;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import java.time.LocalDate;
import java.time.LocalDateTime;
import java.time.LocalTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

@Service
@RequiredArgsConstructor
@Log4j2
public class RentServiceImpl implements RentService{

    private final UserRepository userRepository;
    private final RentRepository rentRepository;
    private final RentCarRepository rentCarRepository;
    private final RentCarScheduleRepository rentCarScheduleRepository;
    private final RentHistoryRepository rentHistoryRepository;
    private final TravelRepository travelRepository;
    private final TravelDatesRepository travelDatesRepository;
    private final FcmUtil fcmUtil;
    private final WebSocketConfig webSocketConfig;
    private final ObjectMapper objectMapper = new ObjectMapper();
    private final DatePlacesRepository datePlacesRepository;

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
                    .rentId(rent.getRentId())
                    .travelName(rent.getTravel().getTravelName())
                    .rentStatus(rent.getRentStatus())
                    .rentTime(rent.getRentTime())
                    .rentHeadCount(rent.getRentHeadCount())
                    .rentPrice(rent.getRentPrice())
                    .rentStartTime(rent.getRentStartTime().toLocalDate())
                    .rentEndTime(rent.getRentEndTime().toLocalDate())
                    .build();

            response.add(dto);
        }

        return response;
    }

    @Override
    public List<RentResponseDTO> getCompletedRent(String userProviderId) {
        // 사용자 찾기
        User user = userRepository.findByUserProviderId(userProviderId)
                .orElseThrow(() -> new UsernameNotFoundException("해당 userProviderId의 맞는 회원을 찾을 수 없습니다."));

        // 사용자의 완료된 렌트 찾기
        List<Rent> rents = rentRepository.findByUserAndRentStatusCompleted(user);

        List<RentResponseDTO> response = new ArrayList<>();
        for(Rent rent: rents) {
            RentResponseDTO dto = RentResponseDTO.builder()
                    .rentId(rent.getRentId())
                    .rentStatus(rent.getRentStatus())
                    .rentTime(rent.getRentTime())
                    .rentTime(rent.getRentTime())
                    .rentHeadCount(rent.getRentHeadCount())
                    .rentPrice(rent.getRentPrice())
                    .rentStartTime(rent.getRentStartTime().toLocalDate())
                    .rentEndTime(rent.getRentEndTime().toLocalDate())
                    .build();

            response.add(dto);
        }

        return response;
    }

    @Override
    public List<RentResponseDTO> getActiveRent(String userProviderId) {
        // 사용자 찾기
        User user = userRepository.findByUserProviderId(userProviderId)
                .orElseThrow(() -> new UsernameNotFoundException("해당 userProviderId의 맞는 회원을 찾을 수 없습니다."));

        // 사용자의 예약중인 렌트 찾기
        List<Rent> rents = rentRepository.findByUserAndRentStatus(user, RentStatus.reserved);

        List<RentResponseDTO> response = new ArrayList<>();
        for(Rent rent: rents) {
            RentResponseDTO dto = RentResponseDTO.builder()
                    .rentId(rent.getRentId())
                    .rentStatus(rent.getRentStatus())
                    .rentTime(rent.getRentTime())
                    .rentTime(rent.getRentTime())
                    .rentHeadCount(rent.getRentHeadCount())
                    .rentPrice(rent.getRentPrice())
                    .rentStartTime(rent.getRentStartTime().toLocalDate())
                    .rentEndTime(rent.getRentEndTime().toLocalDate())
                    .build();

            response.add(dto);
        }

        return response;
    }

    @Override
    public RentDetailResponseDTO getDetailRent(long rentId) {
        // 렌트 찾기
        Rent rent = rentRepository.findByRentId(rentId)
                .orElseThrow(() -> new RentNotFoundException("해당 rentId의 맞는 렌트를 찾을 수 없습니다."));
        RentCar rentCar = rent.getRentCar();
        Travel travel = rent.getTravel();

        // 렌트 차량 스케즐 가져오기
        RentCarSchedule carSchedule = rentCarScheduleRepository.findByRentRentId(rent.getRentId())
                .orElseThrow(() -> new RentCarScheduleNotFoundException("해당 rentId의 맞는 렌트 차량 스케줄을 찾을 수 없습니다."));

        // 렌트 첫 날 일정 가져오기
        TravelDates firstDate = travelDatesRepository.findFirstByTravel(travel);

        // 첫날 첫 장소 가져오기
        DatePlaces firstPlace = datePlacesRepository.findFirstByTravelDates(firstDate);

        RentDetailResponseDTO response = RentDetailResponseDTO.builder()
                // rent
                .rentId(rent.getRentId())
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
                .rentCarImg(rentCar.getRentCarImg())
                .rentCarScheduleId(carSchedule.getRentCarScheduleId())
                // travel
                .travelId(travel.getTravelId())
                .datePlacesName(firstPlace.getDatePlacesName())
                .build();

        return response;
    }

    @Override
    public RentDetailResponseDTO getCurrentRent(String userProviderId) {
        // 현재 날짜에 진행중인 렌트 기록 찾기
        Rent rent = rentRepository.findCurrentRentByUserProviderId(userProviderId)
                .orElseThrow(() -> new RentNotFoundException("현재 진행 중인 렌트가 없습니다."));
        RentCar rentCar = rent.getRentCar();
        Travel travel = rent.getTravel();

        // 렌트 차량 스케즐 가져오기
        RentCarSchedule carSchedule = rentCarScheduleRepository.findByRentRentId(rent.getRentId())
                .orElseThrow(() -> new RentCarScheduleNotFoundException("해당 rentId의 맞는 렌트 차량 스케줄을 찾을 수 없습니다."));

        RentDetailResponseDTO response = RentDetailResponseDTO.builder()
                // rent
                .rentId(rent.getRentId())
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
                .rentCarImg(rentCar.getRentCarImg())
                .rentCarScheduleId(carSchedule.getRentCarScheduleId())
                // travel
                .travelId(travel.getTravelId())
                .build();

        return response;
    }

    @Override
    public RentStatusResponseDTO getRentStatusAndRentCarStatus(String userProviderId) {
        // 사용자 찾기
        User user = userRepository.findByUserProviderId(userProviderId)
                .orElseThrow(() -> new UsernameNotFoundException("해당 userProviderId의 맞는 회원을 찾을 수 없습니다."));

        // 응답
        RentStatusResponseDTO response = RentStatusResponseDTO.builder().build();

        try {
            // 오늘 날짜 기준 사용자의 렌트 찾기
            LocalDateTime startDate = LocalDateTime.now().plusDays(0).withHour(0).withMinute(0).withSecond(0);
            LocalDateTime endDate = LocalDateTime.now().plusDays(0).withHour(23).withMinute(59).withSecond(59);
            Rent rent = rentRepository.findByUserAndRentStartTimeBetween(user, startDate, endDate)
                    .orElseThrow(() -> new RentNotFoundException("오늘 날짜 기준 사용자의 렌트가 없습니다."));

            response.setRentStatus(rent.getRentStatus());
            response.setRentCarDrivingStatus(rent.getRentCar().getRentCarDrivingStatus());

            return response;
        } catch (RentNotFoundException e) {
            return response;
        }
    }

    @Override
    public boolean chkRent(String userProviderId, RentCheckRequestDTO rentCheckRequestDTO) {
        // 사용자 찾기
        User user = userRepository.findByUserProviderId(userProviderId)
                .orElseThrow(() -> new UsernameNotFoundException("해당 userProviderId의 맞는 회원을 찾을 수 없습니다."));

        // 시작 날짜, 종료 날짜
        LocalDate startDate = rentCheckRequestDTO.getRentStartTime();
        LocalDate endDate = rentCheckRequestDTO.getRentEndTime();

        // 해당 날짜의 rentStatus가 reserved인 렌트가 존재하는지 확인하기
        boolean isRentExist = rentRepository.existsByUserAndRentStatusAndRentStartTimeBetweenAndRentEndTimeBetween(
                user,                           // 조회하려는 대상 사용자
                RentStatus.reserved,            // 렌트 상태가 reserved인지 확인
                startDate.atStartOfDay(),       // 렌트 시작 시간을 해당 시작일의 00:00:00로 설정
                endDate.atTime(LocalTime.MAX),  // 렌트 종료 시간을 해당 종료일의 23:59:59로 설정
                startDate.atStartOfDay(),       // 렌트 종료 시간이 해당 시작일의 00:00:00과 비교
                endDate.atTime(LocalTime.MAX)   // 렌트 종료 시간이 해당 종료일의 23:59:59와 비교
        );

        // 이미 렌트가 존재하면 true, 없으면 false 반환
        return isRentExist;
    }

    @Override
    @Transactional
    public RentDetailResponseDTO createRent(String userProviderId, RentCreateRequestDTO rentCreateRequestDTO) {
        // 사용자 찾기
        User user = userRepository.findByUserProviderId(userProviderId)
                .orElseThrow(() -> new UsernameNotFoundException("해당 userProviderId의 맞는 회원을 찾을 수 없습니다."));

        // 시작, 종료 시간
        LocalDateTime startDateTime = rentCreateRequestDTO.getRentStartTime();
        LocalDateTime endDateTime = rentCreateRequestDTO.getRentEndTime();
        LocalDate startDate = startDateTime.toLocalDate();
        LocalDate endDate = endDateTime.toLocalDate();

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

        // 여행 생성
        Travel travel = Travel.builder()
                .travelName(startDate + " 여행")
                .travelStartDate(startDate)
                .travelEndDate(endDate)
                .build();

        // 여행 저장
        travelRepository.save(travel);

        // 여행별 일정 생성
        while (!startDate.isAfter(endDate)) {
            TravelDates travelDates = TravelDates.builder()
                    .travel(travel)
                    .travelDatesDate(startDate)
                    .travelDatesIsExpired(false)
                    .build();

            // 여행별 일정 저장
            travelDatesRepository.save(travelDates);

            // 각 일정 별 첫번째 장소(탑승 장소) 추가하기
            DatePlaces dptPlace = DatePlaces.builder()
                    .travel(travel)
                    .travelDates(travelDates)  // 현재 일정에 대한 장소 추가
                    .datePlacesOrder(1)
                    .datePlacesName(rentCreateRequestDTO.getDatePlacesName())
                    .datePlacesCategory(rentCreateRequestDTO.getDatePlacesCategory())
                    .datePlacesAddress(rentCreateRequestDTO.getDatePlacesAddress())
                    .datePlacesLat(rentCreateRequestDTO.getRentDptLat())
                    .datePlacesLon(rentCreateRequestDTO.getRentDptLon())
                    .datePlacesIsVisited(false)
                    .datePlacesIsExpired(false)
                    .build();

            // 추가한 장소 저장
            datePlacesRepository.save(dptPlace);

            // 다음날로 이동
            startDate = startDate.plusDays(1);
        }

        // 렌트 생성
        Rent rent = Rent.builder()
                .user(user)
                .rentCar(availableCar)
                .travel(travel)
                .rentStatus(RentStatus.reserved)
                .rentHeadCount(rentCreateRequestDTO.getRentHeadCount())
                .rentPrice(rentCreateRequestDTO.getRentPrice())
                .rentTime(rentCreateRequestDTO.getRentTime())
                .rentStartTime(startDateTime)
                .rentEndTime(endDateTime)
                .rentDptLat(rentCreateRequestDTO.getRentDptLat())
                .rentDptLon(rentCreateRequestDTO.getRentDptLon())
                .build();

        // 렌트 저장
        rentRepository.save(rent);

        // 렌트 차량 상태 변경
        availableCar.setRentCarDrivingStatus(RentDrivingStatus.parking); // 주차(기본값)

        // 렌트 차량 변경 상태 저장
        rentCarRepository.save(availableCar);

        // 렌트 차량 일정 생성
        RentCarSchedule rentCarSchedule = RentCarSchedule.builder()
                .rent(rent)
                .rentCar(availableCar)
                .rentCarScheduleStartDate(startDateTime.toLocalDate())
                .rentCarScheduleEndDate(endDateTime.toLocalDate())
                .rentCarScheduleIsDone(false)
                .build();

        // 렌트 차량 일정 저장
        rentCarScheduleRepository.save(rentCarSchedule);

        RentCar rentCar = rent.getRentCar();
        RentDetailResponseDTO response = RentDetailResponseDTO.builder()
                // rent
                .rentId(rent.getRentId())
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
                .rentCarImg(rentCar.getRentCarImg())
                .rentCarScheduleId(rentCarSchedule.getRentCarScheduleId())
                // travel
                .travelId(travel.getTravelId())
                .build();

        // 일정 추가 유도 알림 보내기
        FcmMessage.FcmDTO fcmDTO = fcmUtil.makeFcmDTO("📆"+ travel.getTravelName(), "해당 여행 일정들의 장소를 추가해주세요 !");
        fcmUtil.singleFcmSend(user, fcmDTO); // 비동기로 전송

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
    public void rentStatusCompleted(RentStatusRequestDTO requestDTO) {
        // 렌트 찾기
        Rent rent = rentRepository.findByRentId(requestDTO.getRentId())
                .orElseThrow(() -> new RentNotFoundException("해당 rentId의 맞는 렌트를 찾을 수 없습니다."));

        // 렌트 차량 탐색
        RentCar car = rentCarRepository.findByRentCarId(rent.getRentCar().getRentCarId())
                .orElseThrow(() -> new RentCarNotFoundException("해당 rentCarId의 맞는 차량을 찾을 수 없습니다."));

        // 렌트 차량 일정 탐색
        RentCarSchedule carSchedule = rentCarScheduleRepository.findByRentCarScheduleId(requestDTO.getRentCarScheduleId())
                .orElseThrow(() -> new RentCarScheduleNotFoundException("해당 렌트에 맞는 차량 일정을 찾을 수 없습니다."));

        // 상태 변경
        rent.setRentStatus(RentStatus.completed); // 완료
        car.setRentCarDrivingStatus(RentDrivingStatus.idling); // 무상태
        carSchedule.setRentCarScheduleIsDone(true); // 완료

        // 렌트 기록 생성
        RentHistory history = RentHistory.builder()
                .user(rent.getUser())
                .rent(rent)
                .build();

        // 변경 상태 저장
        rentHistoryRepository.save(history);
        rentRepository.save(rent);
        rentCarRepository.save(car);
        rentCarScheduleRepository.save(carSchedule);

        // 자율주행 서버로 반납 상태 전송
        try {
            StandardWebSocketClient client = new StandardWebSocketClient();
            WebSocketSession session = client.execute(new TextWebSocketHandler() {
                @Override
                protected void handleTextMessage(WebSocketSession session, TextMessage message) {
                    // 서버로부터 받은 메시지를 처리하는 로직
                    try {
                        // JSON 메시지를 JsonNode로 파싱
                        JsonNode jsonNode = objectMapper.readTree(message.getPayload());
                        log.info("Received message: {}", jsonNode);

                    } catch (Exception e) {
                        log.error("Error processing received message: ", e);
                    }
                }
            }, webSocketConfig.getUrl()).get();

            // 상태와 렌트 탑승 위치 전송
            MyMessage message = new MyMessage("vehicle_return", rent.getRentCar().getRentCarId());
            String jsonMessage = objectMapper.writeValueAsString(message);
            session.sendMessage(new TextMessage(jsonMessage));
            log.info("Sent message: {}", jsonMessage);
            Thread.sleep(1000); // 필요에 따라 대기 시간 조절

        } catch (Exception e) {
            log.error("Error during WebSocket communication: ", e);
            throw new WebSocketDisConnectedException("WebSocket이 네트워크 연결을 거부했습니다.");
        }
    }

    @Override
    @Transactional
    public void rentStatusCanceld(RentStatusRequestDTO requestDTO) {
        // 렌트 찾기
        Rent rent = rentRepository.findByRentId(requestDTO.getRentId())
                .orElseThrow(() -> new RentNotFoundException("해당 rentId의 맞는 렌트를 찾을 수 없습니다."));

        // 렌트 차량 탐색
        RentCar car = rentCarRepository.findByRentCarId(rent.getRentCar().getRentCarId())
                .orElseThrow(() -> new RentCarNotFoundException("해당 rentCarId의 맞는 차량을 찾을 수 없습니다."));

        // 렌트 차량 일정 탐색
        RentCarSchedule carSchedule = rentCarScheduleRepository.findByRentCarScheduleId(requestDTO.getRentCarScheduleId())
                .orElseThrow(() -> new RentCarScheduleNotFoundException("해당 렌트에 맞는 차량 일정을 찾을 수 없습니다."));

        // 상태 변경
        rent.setRentStatus(RentStatus.canceled); // 취소
        carSchedule.setRentCarScheduleIsDone(true); // 완료

        // 변경 상태 저장
        rentRepository.save(rent);
        rentCarScheduleRepository.save(carSchedule);
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
    public RentCarManipulateResponseDTO todayRentIsDone(RentCarManipulateRequestDTO rentCarManipulateRequestDTO) {
        // rentId에 해당하는 렌트 찾기
        Rent rent = rentRepository.findByRentId(rentCarManipulateRequestDTO.getRentId())
                .orElseThrow(() -> new RentNotFoundException("해당 rentId의 맞는 렌트를 찾을 수 없습니다."));

        // 해당 렌트에 맞는 여행 = travelId에 해당하는 여행
        Travel travel = rent.getTravel();

        // travelDatesId에 해당하는 일정 찾기
        TravelDates date = travelDatesRepository.findByTravelDatesId(rentCarManipulateRequestDTO.getTravelDatesId())
                .orElseThrow(() -> new TravelNotFoundException("해당 travelDatesId에 맞는 일정을 찾을 수 없습니다."));

        // 일정 만료 처리
        date.setTravelDatesIsExpired(true);

        // 변경 상태 저장
        travelDatesRepository.save(date);

        // 해당 일정에 맞는 장소들 isExpired false인 것들 찾기
        List<DatePlaces> expirePlaces = datePlacesRepository.findByTravelDatesAndDatePlacesIsExpiredFalse(date);

        // 장소들 모두 isExpired true 처리
        for(DatePlaces place : expirePlaces) {
            place.setDatePlacesIsExpired(true);

            // 변경 상태 저장
            datePlacesRepository.save(place);
        }

        // 다음 날 일정 찾기
        TravelDates nextDate = travelDatesRepository.findByTravelDatesId(date.getTravelDatesId() + 1)
                .orElseThrow(() -> new TravelAllPlacesVisitedException("해당 travelDatesId의 다음날 일정을 찾을 수 없습니다. 렌트 일정이 완료되었습니다."));

        // 다음날 첫번째 여행지 찾기
        DatePlaces nextDayPlace = datePlacesRepository.findByTravelDatesAndDatePlacesOrder(nextDate, 1)
                .orElseThrow(() -> new TravelNotFoundException("다음 날의 첫번째 장소를 찾을 수 없습니다. 다음날 장소를 추가해주세요."));

        // 응답 생성
        RentCarManipulateResponseDTO response = RentCarManipulateResponseDTO.builder()
                .travelId(travel.getTravelId())
                .travelDatesId(nextDate.getTravelDatesId())
                .datePlacesId(nextDayPlace.getDatePlacesId())
                .build();

        return response;
    }
}
