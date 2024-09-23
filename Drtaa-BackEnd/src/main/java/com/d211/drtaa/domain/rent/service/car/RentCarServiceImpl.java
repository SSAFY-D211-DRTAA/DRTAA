package com.d211.drtaa.domain.rent.service.car;

import com.d211.drtaa.domain.rent.dto.request.RentCarCallRequestDTO;
import com.d211.drtaa.domain.rent.dto.request.RentCarDriveStatusRequestDTO;
import com.d211.drtaa.domain.rent.dto.request.RentCarUnassignedDispatchStatusRequestDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarDriveStatusResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarLocationResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarResponseDTO;
import com.d211.drtaa.domain.rent.entity.Rent;
import com.d211.drtaa.domain.rent.entity.car.RentCar;
import com.d211.drtaa.domain.rent.entity.car.RentCarSchedule;
import com.d211.drtaa.domain.rent.entity.car.RentDrivingStatus;
import com.d211.drtaa.domain.rent.repository.RentRepository;
import com.d211.drtaa.domain.rent.repository.car.RentCarRepository;
import com.d211.drtaa.domain.rent.repository.car.RentCarScheduleRepository;
import com.d211.drtaa.global.config.websocket.MyMessage;
import com.d211.drtaa.global.config.websocket.WebSocketConfig;
import com.d211.drtaa.global.exception.rent.NoAvailableRentCarException;
import com.d211.drtaa.global.exception.rent.RentCarNotFoundException;
import com.d211.drtaa.global.exception.rent.RentNotFoundException;
import com.d211.drtaa.global.exception.websocket.WebSocketDisConnectedException;
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
    public void updateDriveStatus(RentCarDriveStatusRequestDTO rentCarDriveStatusRequestDTO) {
        // rentCarId에 해당하는 렌트 차량 찾기
        RentCar car = rentCarRepository.findByRentCarId(rentCarDriveStatusRequestDTO.getRentCarId())
                .orElseThrow(() -> new RentCarNotFoundException("해당 rentCarId의 맞는 차량을 찾을 수 없습니다."));

        // 상태 변경
        car.setRentCarDrivingStatus(rentCarDriveStatusRequestDTO.getRentCarDrivingStatus());

        // 저장
        rentCarRepository.save(car);
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

                        // DTO 생성
                        response[0] = RentCarLocationResponseDTO.builder()
                                .rentCarId(car.getRentCarId())
                                .rentCarLat(jsonNode.get("rentCarLat").asDouble()) // rentCarLat을 double로 변환
                                .rentCarLon(jsonNode.get("rentCarLon").asDouble()) // rentCarLon을 double로 변환
                                .build();

                    } catch (Exception e) {
                        log.error("Error processing received message: ", e);
                    }
                }
            }, webSocketConfig.getUrl()).get();

            // 상태와 렌트 차량 탑승 호출 위치 전달
            MyMessage message = new MyMessage("vehicle_dispatch", rent.getRentDptLat(), rent.getRentDptLon());
            String jsonMessage = objectMapper.writeValueAsString(message);
            session.sendMessage(new TextMessage(jsonMessage));
            log.info("Sent message: {}", jsonMessage);
            Thread.sleep(1000); // 필요에 따라 대기 시간 조절

        } catch (Exception e) {
            log.error("Error during WebSocket communication: ", e);
            throw new WebSocketDisConnectedException("WebSocket이 네트워크 연결을 거부했습니다.");
        }

        // 렌트 차량 상태 변경
        car.setRentCarDrivingStatus(RentDrivingStatus.call);

        // 렌트 차량 변경 상태 저장
        rentCarRepository.save(car);

        // 응답이 없으면 빈 DTO 반환
        return response[0] != null ? response[0] : RentCarLocationResponseDTO.builder().build();
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
                        JsonNode latNode = jsonNode.get("rentCarLat");
                        JsonNode lonNode = jsonNode.get("rentCarLon");
                        log.info("lat: {}", latNode);
                        log.info("lon: {}", lonNode);

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
            MyMessage message = new MyMessage("vehicle_dispatch", rentCarCallRequestDTO.getUserLat(), rentCarCallRequestDTO.getUserLon());
            String jsonMessage = objectMapper.writeValueAsString(message);
            session.sendMessage(new TextMessage(jsonMessage));
            log.info("Sent message: {}", jsonMessage);
            Thread.sleep(1000); // 필요에 따라 대기 시간 조절

        } catch (Exception e) {
            log.error("Error during WebSocket communication: ", e);
            throw new WebSocketDisConnectedException("WebSocket이 네트워크 연결을 거부했습니다.");
        }

        // 렌트 차량 상태 변경
        car.setRentCarDrivingStatus(RentDrivingStatus.call);

        // 렌트 차량 변경 상태 저장
        rentCarRepository.save(car);

        // 응답이 없으면 빈 DTO 반환
        return response[0] != null ? response[0] : RentCarLocationResponseDTO.builder().build();
    }


}
