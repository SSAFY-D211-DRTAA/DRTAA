package com.d211.drtaa.domain.rent.service.car;

import com.d211.drtaa.domain.rent.dto.request.RentCarArriveStatusRequestDTO;
import com.d211.drtaa.domain.rent.dto.request.RentCarCallRequestDTO;
import com.d211.drtaa.domain.rent.dto.request.RentCarDriveStatusRequestDTO;
import com.d211.drtaa.domain.rent.dto.request.RentCarUnassignedDispatchStatusRequestDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarDriveStatusResponseDTO;
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
import com.d211.drtaa.domain.travel.repository.TravelRepository;
import com.d211.drtaa.global.config.websocket.MyMessage;
import com.d211.drtaa.global.config.websocket.WebSocketConfig;
import com.d211.drtaa.global.exception.rent.NoAvailableRentCarException;
import com.d211.drtaa.global.exception.rent.RentCarNotFoundException;
import com.d211.drtaa.global.exception.rent.RentNotFoundException;
import com.d211.drtaa.global.exception.travel.TravelAllPlacesVisitedException;
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
import java.util.Date;
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
            // 알림 보낼 내용
            String content = "차량이 여행 첫번째 탑승 위치(" + rent.getTravel().getTravelName() + ")로 이동중입니다.\n 위치를 확인해 주세요 !!";

            // 사용자에게 알림 전송
            FcmMessage.FcmDTO fcmDTO = fcmUtil.makeFcmDTO("렌트 차량 위치", content);
            log.info("Message: {}", content);
            fcmUtil.singleFcmSend(rent.getUser(), fcmDTO);

            // 현재 렌트에 해당하는 여행 id
            response[0].setTravelId(rent.getTravel().getTravelId());
            // 현재 렌트에 해당하는 첫째날 id
            List<TravelDates> travelDates = travelDatesRepository.findByTravel(rent.getTravel());
            response[0].setTravelDatesId(travelDates.get(0).getTravelDatesId());
            // 첫째날의 첫 장소(탑승 장소) id
            List<DatePlaces> datePlaces = datePlacesRepository.findByTravelDatesId(travelDates.get(0).getTravelDatesId());
            response[0].setDatePlacesId(datePlaces.get(0).getDatePlacesId());

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
            MyMessage message = new MyMessage("vehicle_dispatch", rentCarCallRequestDTO.getUserLat(), rentCarCallRequestDTO.getUserLon(), rent.getRentCar().getRentCarId());
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
            // 알림 보낼 내용
            String content = "차량이 사용자의 위치로 이동중입니다.\n 조금만 기다려주세요 !!";

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
    public void updateRentCarDriveStatustoDriving(long rentId) {
        // rentId에 해당하는 렌트 찾기
        Rent rent = rentRepository.findByRentId(rentId)
                .orElseThrow(() -> new RentNotFoundException("해당 rentId의 맞는 렌트를 찾을 수 없습니다."));

        // rentCarId에 해당하는 렌트 차량 찾기
        RentCar car = rentCarRepository.findByRentCarId(rent.getRentCar().getRentCarId())
                .orElseThrow(() -> new RentCarNotFoundException("해당 rentCarId의 맞는 차량을 찾을 수 없습니다."));

        // ** 방문하지 않은 여행 일정 장소 찾기 로직 **
        // 자율주행 서버로 보낼 위도, 경도
        double lattitude = 37.576636819990284;
        double longitude = 126.89879021208397;

        // 해당 렌트의 여행
        Travel travel = rent.getTravel();
        // 현재 날짜
        LocalDate now = LocalDate.now();
        log.info("현재 날짜: {}", now);
        //
        boolean chk = true;

        // 현재 날짜 기준 여행 일정 찾기
        TravelDates dates = travelDatesRepository.findByTravelAndTravelDatesDate(travel, now)
                .orElseThrow(() -> new TravelNotFoundException("현재 날짜에 진행중인 여행을 찾을 수 없습니다."));

        // 현재 날짜 기준 여행 일정의 여행 장소 찾기
        List<DatePlaces> placesList = datePlacesRepository.findByTravelDatesId(dates.getTravelDatesId());
        for(DatePlaces places : placesList) {
            // 방문하지 않은 여행 일정 찾기
            if(!places.getDatePlacesIsVisited()) {
                lattitude = places.getDatePlacesLat();
                longitude = places.getDatePlacesLon();
                chk = false;

                break; // 반복 종료
            }
        }

        // ** 방문하지 않은 여행 일정 장소가 없는 경우 로직 **
        if(chk) {
            FcmMessage.FcmDTO fcmDTO = fcmUtil.makeFcmDTO("렌트 일정", "렌트 일정의 모든 여행지를 방문했습니다.\n 반납 예정이 아니라면 여행지를 추가해주세요.");
            log.info("Message: {}", fcmDTO.getBody());
            fcmUtil.singleFcmSend(rent.getUser(), fcmDTO); // 비동기로 전송

            new TravelAllPlacesVisitedException("렌트 일정의 모든 여행지를 방문했습니다. 반납 예정이 아니라면 여행지를 추가해주세요.");
            return;
        }

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
                        JsonNode latNode = jsonNode.get("latitude");
                        JsonNode lonNode = jsonNode.get("longitude");
                        JsonNode rentCarId = jsonNode.get("rentCarId");
                        log.info("latitude: {}", latNode);
                        log.info("longitude: {}", lonNode);
                        log.info("rentCarId: {}", rentCarId);

                    } catch (Exception e) {
                        log.error("Error processing received message: ", e);
                    }
                }
            }, webSocketConfig.getUrl()).get();

            // 상태와 렌트 탑승 위치 전송
            MyMessage message = new MyMessage("vehicle_drive", lattitude, longitude, rent.getRentCar().getRentCarId());
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
    }

    @Override
    @Transactional
    public void updateRentCarDriveStatustoParking(long rentId) {
        // rentId에 해당하는 렌트 찾기
        Rent rent = rentRepository.findByRentId(rentId)
                .orElseThrow(() -> new RentNotFoundException("해당 rentId의 맞는 렌트를 찾을 수 없습니다."));

        // rentCarId에 해당하는 렌트 차량 찾기
        RentCar car = rentCarRepository.findByRentCarId(rent.getRentCar().getRentCarId())
                .orElseThrow(() -> new RentCarNotFoundException("해당 rentCarId의 맞는 차량을 찾을 수 없습니다."));

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
        rentCarRepository.save(car);

        // 해당 일정 방문 여부 변경 -> travelId와 travelDatesId를 서버로 보내야 함
//        Travel travel = rent.getTravel(); // 해당 렌트의 여행
//        LocalDate now = LocalDate.now(); // 현재 날짜
//        // 현재 날짜 기준 여행 일정 찾기
//        TravelDates dates = travelDatesRepository.findByTravelAndTravelDatesDate(travel, now)
//                .orElseThrow(() -> new TravelNotFoundException("현재 날짜에 진행중인 여행을 찾을 수 없습니다."));

        // ** 마지막 일정인 경우 사용자에게 알림 로직 **
        // ** 마지막 일정의 마지막 장소 전인 경우 사용자에게 알림 로직 **
//        log.info("현재 날짜: {}", now);
//        // 현재 날짜와 렌트 종료 날짜가 같은지 확인
//        if(now.equals(rent.getRentEndTime())) {
//            // 현재 날짜 기준 여행 일정의 여행 장소 찾기
//            List<DatePlaces> placesList = datePlacesRepository.findByTravelDatesId(dates.getTravelDatesId());
//
//            // 여행 장소의 마지막인 경우
//            if(placesList.get(placesList.size() - 1).getDatePlacesIsVisited()) {
//                FcmMessage.FcmDTO fcmDTO = fcmUtil.makeFcmDTO("렌트 일정", "렌트 일정의 모든 여행지를 방문했습니다.\n 반납 예정이 아니라면 여행지를 추가해주세요.");
//                log.info("Message: {}", fcmDTO.getBody());
//                fcmUtil.singleFcmSend(rent.getUser(), fcmDTO); // 비동기로 전송
//            }
//
//            // 여행 장소 중 마지막 장소 전이 true인 경우
//            if(placesList.size() > 2 && placesList.get(placesList.size() - 2).getDatePlacesIsVisited()) {
//                FcmMessage.FcmDTO fcmDTO = fcmUtil.makeFcmDTO("렌트 일정", "해당 여행지 이후 마지막 여행지만 남았습니다.\n 마지막 여행지에서 반납 예정이 아니라면 여행지를 추가해주세요.");
//                log.info("Message: {}", fcmDTO.getBody());
//                fcmUtil.singleFcmSend(rent.getUser(), fcmDTO); // 비동기로 전송
//            }
//        }
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
