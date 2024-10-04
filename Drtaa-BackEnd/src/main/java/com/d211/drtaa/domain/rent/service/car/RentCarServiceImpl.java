package com.d211.drtaa.domain.rent.service.car;

import com.d211.drtaa.domain.rent.dto.request.*;
import com.d211.drtaa.domain.rent.dto.response.RentCarDriveStatusResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarManipulateResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarLocationResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarResponseDTO;
import com.d211.drtaa.domain.rent.entity.Rent;
import com.d211.drtaa.domain.rent.entity.RentStatus;
import com.d211.drtaa.domain.rent.entity.car.RentCar;
import com.d211.drtaa.domain.rent.entity.car.RentCarSchedule;
import com.d211.drtaa.domain.rent.entity.car.RentDrivingStatus;
import com.d211.drtaa.domain.rent.repository.RentRepository;
import com.d211.drtaa.domain.rent.repository.car.RentCarRepository;
import com.d211.drtaa.domain.rent.repository.car.RentCarScheduleRepository;
import com.d211.drtaa.domain.travel.entity.DatePlaces;
import com.d211.drtaa.domain.travel.entity.Travel;
import com.d211.drtaa.domain.travel.entity.TravelDates;
import com.d211.drtaa.domain.travel.repository.DatePlacesRepository;
import com.d211.drtaa.domain.travel.repository.TravelDatesRepository;
import com.d211.drtaa.global.config.websocket.MyMessage;
import com.d211.drtaa.global.config.websocket.WebSocketConfig;
import com.d211.drtaa.global.exception.rent.NoAvailableRentCarException;
import com.d211.drtaa.global.exception.rent.RentCarNotFoundException;
import com.d211.drtaa.global.exception.rent.RentNotFoundException;
import com.d211.drtaa.global.exception.travel.TravelAllPlacesVisitedException;
import com.d211.drtaa.global.exception.travel.TravelDateNotMatchException;
import com.d211.drtaa.global.exception.travel.TravelNotFoundException;
import com.d211.drtaa.global.exception.websocket.WebSocketDisConnectedException;
import com.d211.drtaa.global.util.fcm.FcmMessage;
import com.d211.drtaa.global.util.fcm.FcmUtil;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import jakarta.transaction.Transactional;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.stereotype.Service;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.client.standard.StandardWebSocketClient;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import java.time.LocalDate;
import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

@Service
@RequiredArgsConstructor
@Log4j2
public class RentCarServiceImpl implements RentCarService {

    private final RentRepository rentRepository;
    private final RentCarRepository rentCarRepository;
    private final RentCarScheduleRepository rentCarScheduleRepository;
    private final WebSocketConfig webSocketConfig;
    private final ObjectMapper objectMapper = new ObjectMapper();
    private final FcmUtil fcmUtil;
    private final DatePlacesRepository datePlacesRepository;
    private final TravelDatesRepository travelDatesRepository;

    @Override
    public List<RentCarResponseDTO> getAllDispatchStatus() {
        // 모든 렌트 차량 검색
        List<RentCar> rentCarsList = rentCarRepository.findAll();

        List<RentCarResponseDTO> responseDTOList = new ArrayList<>();
        for (RentCar rentCar : rentCarsList) {
            RentCarResponseDTO dto = RentCarResponseDTO.builder()
                    .isAvailable(true)
                    .rentCarId(rentCar.getRentCarId())
                    .rentCarNumber(rentCar.getRentCarNumber())
                    .rentCarManufacturer(rentCar.getRentCarManufacturer())
                    .rentCarModel(rentCar.getRentCarModel())
                    .rentCarImg(rentCar.getRentCarImg())
                    .rentCarDrivingStatus(rentCar.getRentCarDrivingStatus())
                    .build();

            responseDTOList.add(dto);
        }

        return responseDTOList;
    }

    @Override
    public RentCarResponseDTO getUnassignedDispatchStatus(RentCarUnassignedDispatchStatusRequestDTO requestDTO) {
        LocalDate finalStartDate = requestDTO.getRentCarScheduleStartDate();
        LocalDate endDate = requestDTO.getRentCarScheduleEndDate();
        
        // 가능한 차량 확인
        List<RentCar> availableCars = rentCarRepository.findAll().stream()
                .filter(car -> {
                    List<RentCarSchedule> schedules = rentCarScheduleRepository.findByRentCar(car);
                    return schedules.stream().noneMatch(schedule ->
                            (finalStartDate.isBefore(schedule.getRentCarScheduleEndDate()) || finalStartDate.isEqual(schedule.getRentCarScheduleEndDate())) &&
                                    (endDate.isAfter(schedule.getRentCarScheduleStartDate()) || endDate.isEqual(schedule.getRentCarScheduleStartDate()))
                    );
                })
                .collect(Collectors.toList());

        if (availableCars.isEmpty())
            throw new NoAvailableRentCarException("해당 기간에 사용 가능한 차량이 없습니다.");
        
        // 가능한 차량중 가장 첫번째 선택
        RentCar availableCar = availableCars.get(0);

        RentCarResponseDTO responseDTO = RentCarResponseDTO.builder()
                .isAvailable(true)
                .rentCarId(availableCar.getRentCarId())
                .rentCarNumber(availableCar.getRentCarNumber())
                .rentCarManufacturer(availableCar.getRentCarManufacturer())
                .rentCarModel(availableCar.getRentCarModel())
                .rentCarImg(availableCar.getRentCarImg())
                .rentCarDrivingStatus(availableCar.getRentCarDrivingStatus())
                .build();

        return responseDTO;
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
    @Transactional
    public RentCarLocationResponseDTO callRentCar(long rentId) {
        // rentId에 해당하는 렌트 찾기
        Rent rent = rentRepository.findByRentId(rentId)
                .orElseThrow(() -> new RentNotFoundException("해당 rentId의 맞는 렌트를 찾을 수 없습니다."));

        // rentCarId에 해당하는 렌트 차량 찾기
        RentCar car = rentCarRepository.findByRentCarId(rent.getRentCar().getRentCarId())
                .orElseThrow(() -> new RentCarNotFoundException("해당 rentCarId의 맞는 차량을 찾을 수 없습니다."));

        // 응답 DTO 초기화
        final RentCarLocationResponseDTO[] response = {null};
        // 자율주행 서버로 메시지 전송
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

                        // null 체크 추가
                        JsonNode rentCarId = jsonNode.get("rentCarId");
                        JsonNode latNode = jsonNode.get("latitude");
                        JsonNode lonNode = jsonNode.get("longitude");
                        log.info("latitude: {}", latNode);
                        log.info("longitude: {}", lonNode);
                        log.info("rentCarId: {}", rentCarId);

                        if (latNode != null && lonNode != null && !latNode.isNull() && !lonNode.isNull()) {
                            // DTO 생성
                            response[0] = RentCarLocationResponseDTO.builder()
                                    .rentCarId(car.getRentCarId())
                                    .rentCarLat(latNode.asDouble()) // rentCarLat을 double로 변환
                                    .rentCarLon(lonNode.asDouble()) // rentCarLon을 double로 변환
                                    .build();
                        } else {
                            log.warn("Received null for rentCarLat or rentCarLon");
                        }

                    } catch (Exception e) {
                        log.error("Error processing received message: ", e);
                    }
                }
            }, webSocketConfig.getUrl()).get();

            // 상태와 렌트 탑승 위치 전송
            MyMessage message = new MyMessage("vehicle_dispatch", rent.getRentDptLat(), rent.getRentDptLon(), rent.getRentCar().getRentCarId());
            String jsonMessage = objectMapper.writeValueAsString(message);
            session.sendMessage(new TextMessage(jsonMessage));
            log.info("Sent message: {}", jsonMessage);
            Thread.sleep(1000); // 필요에 따라 대기 시간 조절

        } catch (Exception e) {
            log.error("Error during WebSocket communication: ", e);
            throw new WebSocketDisConnectedException("WebSocket이 네트워크 연결을 거부했습니다.");
        }

        // 렌트 차량 상태 변경
        car.setRentCarDrivingStatus(RentDrivingStatus.calling);

        // 렌트 차량 변경 상태 저장
        rentCarRepository.save(car);

        // 렌트 상태 진행중으로 변경
        rent.setRentStatus(RentStatus.in_progress);

        // 렌트 변경 상태 저장
        rentRepository.save(rent);

        if(response[0] != null) {
            // 현재 렌트에 해당하는 여행 id
            response[0].setTravelId(rent.getTravel().getTravelId());
            // 현재 렌트에 해당하는 첫째날 id
            TravelDates date = travelDatesRepository.findFirstByTravel(rent.getTravel());
            response[0].setTravelDatesId(date.getTravelDatesId());
            // 첫째날의 첫 장소(탑승 장소) id
            DatePlaces place = datePlacesRepository.findFirstByTravelDatesOrderByDatePlacesOrderAsc(date);
            response[0].setDatePlacesId(place.getDatePlacesId());

            // 알림 보낼 내용
            String content = "차량이 여행 첫번째 탑승 위치( "+ place.getDatePlacesName() +")로 이동중입니다.\n 위치를 확인해 주세요 !!";

            // 사용자에게 알림 전송
            FcmMessage.FcmDTO fcmDTO = fcmUtil.makeFcmDTO("렌트 차량 위치", content);
            log.info("Message: {}", content);
            fcmUtil.singleFcmSend(rent.getUser(), fcmDTO);

            // 응답 반환
            return response[0];
        } else {
            // 응답이 없으면 빈 DTO 반환
            return RentCarLocationResponseDTO.builder().build();
        }
    }

    @Override
    @Transactional
    public RentCarLocationResponseDTO reCallRentCar(RentCarCallRequestDTO rentCarCallRequestDTO) {
        // rentId에 해당하는 렌트 찾기
        Rent rent = rentRepository.findByRentId(rentCarCallRequestDTO.getRentId())
                .orElseThrow(() -> new RentNotFoundException("해당 rentId의 맞는 렌트를 찾을 수 없습니다."));

        // rentCarId에 해당하는 렌트 차량 찾기
        RentCar car = rentCarRepository.findByRentCarId(rent.getRentCar().getRentCarId())
                .orElseThrow(() -> new RentCarNotFoundException("해당 rentCarId의 맞는 차량을 찾을 수 없습니다."));

        // datePlacesId에 해당하는 여행 장소 찾기
        DatePlaces requestPlace = datePlacesRepository.findByDatePlacesId(rentCarCallRequestDTO.getDatePlacesId())
                .orElseThrow(() -> new TravelNotFoundException("해당 datePlacesId에 맞는 장소를 찾을 수 없습니다."));

        // 응답 DTO 초기화
        final RentCarLocationResponseDTO[] response = {null};
        // 자율주행 서버로 메시지 전송
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

                        // null 체크 추가
                        JsonNode rentCarId = jsonNode.get("rentCarId");
                        JsonNode latNode = jsonNode.get("latitude");
                        JsonNode lonNode = jsonNode.get("longitude");
                        log.info("rentCarId: {}", rentCarId);
                        log.info("latitude: {}", latNode);
                        log.info("longitude: {}", lonNode);

                        if (latNode != null && lonNode != null && !latNode.isNull() && !lonNode.isNull()) {
                            // DTO 생성
                            response[0] = RentCarLocationResponseDTO.builder()
                                    .rentCarId(car.getRentCarId())
                                    .rentCarLat(latNode.asDouble()) // rentCarLat을 double로 변환
                                    .rentCarLon(lonNode.asDouble()) // rentCarLon을 double로 변환
                                    .build();
                        } else {
                            log.warn("Received null for rentCarLat or rentCarLon");
                        }

                    } catch (Exception e) {
                        log.error("Error processing received message: ", e);
                    }
                }
            }, webSocketConfig.getUrl()).get();

            // 상태와 사용자 탑승 호출 위치 전송
            MyMessage message = new MyMessage("vehicle_dispatch", rentCarCallRequestDTO.getUserLat(), rentCarCallRequestDTO.getUserLon(), car.getRentCarId());
            String jsonMessage = objectMapper.writeValueAsString(message);
            session.sendMessage(new TextMessage(jsonMessage));
            log.info("Sent message: {}", jsonMessage);
            Thread.sleep(1000); // 필요에 따라 대기 시간 조절

        } catch (Exception e) {
            log.error("Error during WebSocket communication: ", e);
            throw new WebSocketDisConnectedException("WebSocket이 네트워크 연결을 거부했습니다.");
        }

        // 렌트 차량 상태 변경
        car.setRentCarDrivingStatus(RentDrivingStatus.calling);

        // 렌트 차량 변경 상태 저장
        rentCarRepository.save(car);

        if(response[0] != null) {
            // 호출한 장소 그대로 반환
            response[0].setRentId(rent.getRentId());
            response[0].setTravelId(rentCarCallRequestDTO.getTravelId());
            response[0].setTravelDatesId(rentCarCallRequestDTO.getTravelDatesId());
            response[0].setDatePlacesId(rentCarCallRequestDTO.getDatePlacesId());

            // 응답 반환
            return response[0];
        } else {
            // 응답이 없으면 빈 DTO 반환
            return RentCarLocationResponseDTO.builder().build();
        }
    }

    @Override
    @Transactional
    public RentCarManipulateResponseDTO updateRentCarDriveStatustoDriving(RentCarManipulateRequestDTO rentCarManipulateRequestDTO) {
        // rentId에 해당하는 렌트 찾기
        Rent rent = rentRepository.findByRentId(rentCarManipulateRequestDTO.getRentId())
                .orElseThrow(() -> new RentNotFoundException("해당 rentId의 맞는 렌트를 찾을 수 없습니다."));

        // rentCarId에 해당하는 렌트 차량 찾기
        RentCar car = rentCarRepository.findByRentCarId(rent.getRentCar().getRentCarId())
                .orElseThrow(() -> new RentCarNotFoundException("해당 rentCarId의 맞는 차량을 찾을 수 없습니다."));

        // datePlacesId에 해당하는 장소 찾기
        DatePlaces currentPlace = datePlacesRepository.findByDatePlacesId(rentCarManipulateRequestDTO.getDatePlacesId())
                .orElseThrow(() -> new TravelNotFoundException("해당 datePlacesId의 맞는 장소를 찾을 수 없습니다."));

        // 서버로 보낼 위도, 경도
        double latitude = currentPlace.getDatePlacesLat();
        double longitude = currentPlace.getDatePlacesLon();
        log.info("이동 위도: {}, 경도: {}, 목적지: {}", latitude, longitude, currentPlace.getDatePlacesName());

        // 이동할 목적지 서버로 전송
        try {
            StandardWebSocketClient client = new StandardWebSocketClient();
            WebSocketSession session = client.execute(new TextWebSocketHandler() {
                @Override
                protected void handleTextMessage(WebSocketSession session, TextMessage message) {
                    log.info("자율주행 서버로 탑승 메시지 전송중...");
                    // 서버로부터 받은 메시지를 처리하는 로직
                    try {
                        // JSON 메시지를 JsonNode로 파싱
                        JsonNode jsonNode = objectMapper.readTree(message.getPayload());
                        log.info("Received message: {}", jsonNode);

                        // null 체크 추가
                        JsonNode latNode = jsonNode.get("latitude");
                        JsonNode lonNode = jsonNode.get("longitude");
                        JsonNode rentCarId = jsonNode.get("rentCarId");
                        JsonNode placeNameNode = jsonNode.get("destinationName");
                        log.info("latitude: {}", latNode);
                        log.info("longitude: {}", lonNode);
                        log.info("rentCarId: {}", rentCarId);
                        log.info("placeName: {}", placeNameNode);

                    } catch (Exception e) {
                        log.error("Error processing received message: ", e);
                    }
                }
            }, webSocketConfig.getUrl()).get();

            // 상태와 렌트 탑승 위치 전송
            MyMessage message = new MyMessage("vehicle_drive", latitude, longitude, rent.getRentCar().getRentCarId(), currentPlace.getDatePlacesName());
            String jsonMessage = objectMapper.writeValueAsString(message);
            session.sendMessage(new TextMessage(jsonMessage));
            log.info("Sent message: {}", jsonMessage);
            Thread.sleep(1000); // 필요에 따라 대기 시간 조절

        } catch (Exception e) {
            log.error("Error during WebSocket communication: ", e);
            throw new WebSocketDisConnectedException("WebSocket이 네트워크 연결을 거부했습니다.");
        }

        // 렌트 차량 상태 변경
        car.setRentCarDrivingStatus(RentDrivingStatus.driving);

        // 변경 상태 저장
        rentCarRepository.save(car);

        RentCarManipulateResponseDTO response = RentCarManipulateResponseDTO.builder()
                .travelId(rentCarManipulateRequestDTO.getTravelId())
                .travelDatesId(rentCarManipulateRequestDTO.getTravelDatesId())
                .datePlacesId(currentPlace.getDatePlacesId())
                .build();

        // 알림 보낼 내용
        String content = "차량이 요청 위치(" + currentPlace.getDatePlacesName() + ")로 이동중입니다.";

        // 사용자에게 알림 전송
        FcmMessage.FcmDTO fcmDTO = fcmUtil.makeFcmDTO("렌트 차량 위치", content);
        log.info("Message: {}", content);
        fcmUtil.singleFcmSend(rent.getUser(), fcmDTO);

        return response;
    }

    @Override
    @Transactional
    public RentCarManipulateResponseDTO updateRentCarDriveStatustoParking(RentCarManipulateRequestDTO rentCarManipulateRequestDTO) {
        // rentId에 해당하는 렌트 찾기
        Rent rent = rentRepository.findByRentId(rentCarManipulateRequestDTO.getRentId())
                .orElseThrow(() -> new RentNotFoundException("해당 rentId의 맞는 렌트를 찾을 수 없습니다."));

        // rentCarId에 해당하는 렌트 차량 찾기
        RentCar car = rentCarRepository.findByRentCarId(rent.getRentCar().getRentCarId())
                .orElseThrow(() -> new RentCarNotFoundException("해당 rentCarId의 맞는 차량을 찾을 수 없습니다."));

        // 해당 렌트에 맞는 여행 = travelId에 해당하는 여행
        Travel travel = rent.getTravel();

        // travelDatesId에 해당하는 일정 찾기
        TravelDates date = travelDatesRepository.findByTravelDatesId(rentCarManipulateRequestDTO.getTravelDatesId())
                .orElseThrow(() -> new TravelNotFoundException("해당 travelDatesId에 맞는 일정을 찾을 수 없습니다."));

        // datePlacesId에 해당하는 장소 찾기 -> 도착한 장소
        DatePlaces arrivedPlace = datePlacesRepository.findByDatePlacesId(rentCarManipulateRequestDTO.getDatePlacesId())
                .orElseThrow(() -> new TravelDateNotMatchException("해당 datePlacesId에 맞는 장소를 찾을 수 없습니다."));

        // 찾은 장소 = 도착한 장소 방문으로 상태 변경
        arrivedPlace.setDatePlacesIsVisited(true);

        // 찾은 장소 = 도착한 장소 만료로 상태 변경
        arrivedPlace.setDatePlacesIsExpired(true);

        // 응답
        RentCarManipulateResponseDTO response = RentCarManipulateResponseDTO.builder()
                .travelId(travel.getTravelId())
                // 아래에서 일정 설정
                // 아래에서 장소 설정
                .build();

       // 찾은 여행 일정의 마지막 장소 찾기
       DatePlaces lastPlace = datePlacesRepository.findFirstByTravelDatesOrderByDatePlacesOrderDesc(date)
               .orElseThrow(() -> new TravelNotFoundException("해당 datePlacesId에 맞는 마지막 장소를 찾을 수 없습니다."));
       log.info("찾은 일정의 마지막 장소: {}", lastPlace.getDatePlacesId());

       // 찾은 장소가 여행 일정의 마지막 장소와 같을 경우
       if(arrivedPlace.getDatePlacesId() == lastPlace.getDatePlacesId()) {
           // 해당 일정이 렌트 일정의 마지막인 경우
           if (date.getTravelDatesDate().equals(rent.getRentEndTime().toLocalDate())) {
                log.info("오늘이 렌트 마지막 일정");
               throw new TravelAllPlacesVisitedException("모든 여행지를 방문했고 렌트의 마지막날 입니다. 반납을 안내해주세요.");
           }

           // 해당 일정이 렌트 일정의 마지막 일정이 아닌 경우 -> 다음 날 여행지 반환하기
           // 다음날 일정 찾기
           TravelDates nextDate = travelDatesRepository.findByTravelDatesId(date.getTravelDatesId() + 1)
                   .orElseThrow(() -> new TravelNotFoundException("해당 travelDatesId의 다음날 일정을 찾을 수 없습니다."));
           log.info("다음날 일정: {}", nextDate.getTravelDatesId());

           // 다음날 첫번째 여행지 찾기
           DatePlaces nextDayPlace = datePlacesRepository.findByTravelDatesAndDatePlacesOrder(nextDate, 1)
                           .orElseThrow(() -> new TravelNotFoundException("다음 날의 첫번째 장소를 찾을 수 없습니다. 다음날 장소를 추가해주세요."));
           log.info("다음 날 첫번째 일정: {}", nextDayPlace.getDatePlacesId());

           // 다음 날 첫번째 장소 응답 반환
           response.setTravelDatesId(nextDate.getTravelDatesId());
           response.setDatePlacesId(nextDayPlace.getDatePlacesId());
       }

        // 다음 순서 장소 찾기
        DatePlaces nextPlace = datePlacesRepository.findByTravelDatesAndDatePlacesOrder(date, arrivedPlace.getDatePlacesOrder() + 1)
                .orElseThrow(() -> new TravelNotFoundException("다음 장소를 찾을 수 없습니다. 오늘 예정된 모든 여행지를 방문했습니다."));
        log.info("다음 장소 id: {}", nextPlace.getDatePlacesId());
        log.info("다음 장소 이름: {}", nextPlace.getDatePlacesName());

        // 다음 순서 장소 응답 반환
        response.setTravelDatesId(date.getTravelDatesId());
        response.setDatePlacesId(nextPlace.getDatePlacesId());

        // 자율주행 서버로 하차 메시지 전송
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
            MyMessage message = new MyMessage("vehicle_wait", rent.getRentCar().getRentCarId());
            String jsonMessage = objectMapper.writeValueAsString(message);
            session.sendMessage(new TextMessage(jsonMessage));
            log.info("Sent message: {}", jsonMessage);
            Thread.sleep(1000); // 필요에 따라 대기 시간 조절

        } catch (Exception e) {
            log.error("Error during WebSocket communication: ", e);
            throw new WebSocketDisConnectedException("WebSocket이 네트워크 연결을 거부했습니다.");
        }

        // 렌트 차량 상태 변경
        car.setRentCarDrivingStatus(RentDrivingStatus.parking);

        // 변경 상태 저장
        datePlacesRepository.save(arrivedPlace);
        rentCarRepository.save(car);

        // Android에게 알림 보내기
        FcmMessage.FcmDTO fcmDTO = fcmUtil.makeFcmDTO("렌트 일정", "렌트 일정을 꼭 확인해주세요 !!\n마지막 장소 이후에는 다음날로 넘어가거나 반납이 안내됩니다.");
        log.info("Message: {}", fcmDTO.getBody());
        fcmUtil.singleFcmSend(rent.getUser(), fcmDTO); // 비동기로 전송

       return response;
    }

    @Override
    public void alarmToAndroid(RentCarDriveStatusRequestDTO rentCarDriveStatusRequestDTO) {
        // rentCarId의 맞는 Rent를 찾기
        Long rentCarId = rentCarDriveStatusRequestDTO.getRentCarId();
        LocalDateTime now = LocalDateTime.now(); // 현재 시간

        // 렌트 조회
        Rent rent = rentRepository.findFirstByRentCar_RentCarIdAndRentStatusAndRentStartTimeLessThanEqualAndRentEndTimeGreaterThanEqual(
                rentCarId,
                RentStatus.in_progress,
                now,
                now
        ).orElseThrow(() -> new RentNotFoundException("rentCarId의 맞는 진행중인 렌트를 찾지 못했습니다."));

        log.info("RentId: {}", rent.getRentId());

        // Android에게 알림 보내기
        FcmMessage.FcmDTO fcmDTO = fcmUtil.makeFcmDTO("렌트 차량 상태", rentCarDriveStatusRequestDTO.getContents());
        log.info("Message: {}", fcmDTO.getBody());
        fcmUtil.singleFcmSend(rent.getUser(), fcmDTO); // 비동기로 전송
    }

    @Override
    public void arrivalToAndroid(RentCarArriveStatusRequestDTO rentCarArriveStatusRequestDTO) {
        // rentCarId의 맞는 Rent를 찾기
        Long rentCarId = rentCarArriveStatusRequestDTO.getRentCarId();
        LocalDateTime now = LocalDateTime.now(); // 현재 시간

        // 렌트 조회
        Rent rent = rentRepository.findFirstByRentCar_RentCarIdAndRentStatusAndRentStartTimeLessThanEqualAndRentEndTimeGreaterThanEqual(
                rentCarId,
                RentStatus.in_progress,
                now,
                now
        ).orElseThrow(() -> new RentNotFoundException("rentCarId의 맞는 진행중인 렌트를 찾지 못했습니다."));

        log.info("RentId: {}", rent.getRentId());

        // Android에게 알림 보내기
        FcmMessage.FcmDTO fcmDTO = fcmUtil.makeFcmDTO("렌트 차량 위치", rentCarArriveStatusRequestDTO.getContents());
        log.info("Message: {}", fcmDTO.getBody());
        fcmUtil.singleFcmSend(rent.getUser(), fcmDTO); // 비동기로 전송
    }
}
