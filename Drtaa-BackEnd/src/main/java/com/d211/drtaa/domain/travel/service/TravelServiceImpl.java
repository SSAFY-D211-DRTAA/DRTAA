package com.d211.drtaa.domain.travel.service;

import com.d211.drtaa.domain.rent.dto.response.RentCarDrivingResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.RentCarLocationResponseDTO;
import com.d211.drtaa.domain.rent.entity.Rent;
import com.d211.drtaa.domain.rent.entity.RentStatus;
import com.d211.drtaa.domain.rent.repository.RentRepository;
import com.d211.drtaa.domain.travel.dto.request.*;
import com.d211.drtaa.domain.travel.dto.response.*;
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
import com.d211.drtaa.global.exception.rent.RentNotFoundException;
import com.d211.drtaa.global.exception.travel.TravelNotFoundException;
import com.d211.drtaa.global.exception.websocket.WebSocketDisConnectedException;
import com.d211.drtaa.global.service.weather.WeatherService;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import jakarta.persistence.EntityManager;
import jakarta.persistence.PersistenceContext;
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
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;


@Service
@RequiredArgsConstructor
@Log4j2
public class TravelServiceImpl implements TravelService {

    @PersistenceContext
    private EntityManager entityManager;

    private final WeatherService weatherService;
    private final TravelRepository travelRepository;
    private final TravelDatesRepository travelDatesRepository;
    private final DatePlacesRepository datePlacesRepository;
    private final UserRepository userRepository;
    private final RentRepository rentRepository;
    private final WebSocketConfig webSocketConfig;
    private final ObjectMapper objectMapper = new ObjectMapper();

    @Override
    public List<TravelResponseDTO> getAllTravels(String userProviderId) {
        // 사용자 찾기
        User user = userRepository.findByUserProviderId(userProviderId)
                .orElseThrow(() -> new UsernameNotFoundException("해당 userProviderId의 맞는 회원을 찾을 수 없습니다."));

        // 사용자의 렌트 찾기
        List<Rent> rents = rentRepository.findByUser(user);

        // 사용자의 여행 찾기
        List<TravelResponseDTO> travels = new ArrayList<>();
        for(Rent rent : rents) {
            // 여행 찾기
            Travel travel = rent.getTravel();

            TravelResponseDTO dto = TravelResponseDTO.builder()
                    .rentId(rent.getRentId())
                    .rentStatus(rent.getRentStatus())
                    .travelId(travel.getTravelId())
                    .travelName(travel.getTravelName())
                    .travelStartDate(travel.getTravelStartDate())
                    .travelEndDate(travel.getTravelEndDate())
                    .build();

            travels.add(dto);
        }

        return travels;
    }

    @Override
    public List<TravelResponseDTO> getAllTravelsCompleted(String userProviderId) {
        // 사용자 찾기
        User user = userRepository.findByUserProviderId(userProviderId)
                .orElseThrow(() -> new UsernameNotFoundException("해당 userProviderId의 맞는 회원을 찾을 수 없습니다."));

        // 사용자의 완료된 렌트 찾기
        List<Rent> rents = rentRepository.findByUserAndRentStatusCompleted(user);

        // 사용자의 여행 찾기
        List<TravelResponseDTO> travels = new ArrayList<>();
        for(Rent rent : rents) {
            // 여행 찾기
            Travel travel = rent.getTravel();

            TravelResponseDTO dto = TravelResponseDTO.builder()
                    .rentId(rent.getRentId())
                    .rentStatus(rent.getRentStatus())
                    .travelId(travel.getTravelId())
                    .travelName(travel.getTravelName())
                    .travelStartDate(travel.getTravelStartDate())
                    .travelEndDate(travel.getTravelEndDate())
                    .build();

            travels.add(dto);
        }

        return travels;
    }

    @Override
    public List<TravelResponseDTO> getAllTravelsActive(String userProviderId) {
        // 사용자 찾기
        User user = userRepository.findByUserProviderId(userProviderId)
                .orElseThrow(() -> new UsernameNotFoundException("해당 userProviderId의 맞는 회원을 찾을 수 없습니다."));

        // 사용자의 진행중 & 예약된 렌트 찾기
        List<Rent> rents = rentRepository.findByUserAndRentStatusInOrderByRentStatusDesc(
                user, Arrays.asList(RentStatus.in_progress, RentStatus.reserved));

        // 사용자의 여행 찾기
        List<TravelResponseDTO> travels = new ArrayList<>();
        for(Rent rent : rents) {
            // 여행 찾기
            Travel travel = rent.getTravel();

            TravelResponseDTO dto = TravelResponseDTO.builder()
                    .rentId(rent.getRentId())
                    .rentStatus(rent.getRentStatus())
                    .travelId(travel.getTravelId())
                    .travelName(travel.getTravelName())
                    .travelStartDate(travel.getTravelStartDate())
                    .travelEndDate(travel.getTravelEndDate())
                    .build();

            travels.add(dto);
        }

        return travels;
    }

    @Override
    public TravelDetailResponseDTO getTravel(Long travelId) {
        // 여행 찾기
        Travel travel = travelRepository.findByTravelId(travelId)
                .orElseThrow(() -> new TravelNotFoundException("해당 travelId에 맞는 여행을 찾을 수 없습니다."));

        // 여행 일정 리스트 찾기
        List<TravelDates> datesList = travelDatesRepository.findByTravel(travel);

        // 각 일정에 대해 일정 장소 리스트를 찾고, DTO로 변환
        List<DatesDetailResponseDTO> datesDtoList = datesList.stream().map(dates -> {
            List<DatePlaces> placesList = datePlacesRepository.findByTravelDatesId(dates.getTravelDatesId());

            // 일정 장소 DTO 리스트 생성
            List<PlacesDetailResponseDTO> placesDtoList = placesList.stream().map(places ->
                    PlacesDetailResponseDTO.builder()
                            .travelDatesId(places.getTravelDates().getTravelDatesId())
                            .datePlacesId(places.getDatePlacesId())
                            .datePlacesOrder(places.getDatePlacesOrder())
                            .datePlacesName(places.getDatePlacesName())
                            .datePlacesCategory(places.getDatePlacesCategory())
                            .datePlacesAddress(places.getDatePlacesAddress())
                            .datePlacesLat(places.getDatePlacesLat())
                            .datePlacesLon(places.getDatePlacesLon())
                            .datePlacesIsVisited(places.getDatePlacesIsVisited())
                            .build()
            ).collect(Collectors.toList());

            // 일정 DTO 생성
            return DatesDetailResponseDTO.builder()
                    .travelId(dates.getTravel().getTravelId())
                    .travelDatesId(dates.getTravelDatesId())
                    .travelDatesDate(dates.getTravelDatesDate())
                    .placesDetail(placesDtoList) // 일정 장소 리스트를 설정
                    .build();
        }).collect(Collectors.toList());

        // 최종 여행 DTO 생성
        TravelDetailResponseDTO dto = TravelDetailResponseDTO.builder()
                .travelId(travel.getTravelId())
                .travelName(travel.getTravelName())
                .travelStartDate(travel.getTravelStartDate())
                .travelEndDate(travel.getTravelEndDate())
                .datesDetail(datesDtoList) // 일정 리스트를 설정
                .build();

        return dto;
    }

    @Override
    @Transactional
    public void createTravelDatesPlaces(PlacesAddRequestDTO placesAddRequestDTO) {
        // 여행 일정 찾기
        TravelDates dates = travelDatesRepository.findByTravelDatesId(placesAddRequestDTO.getTravelDatesId())
                .orElseThrow(() -> new TravelNotFoundException("해당 travelDatesId의 맞는 일정을 찾을 수 없습니다."));

        // 일정 장소 생성
        DatePlaces places = DatePlaces.builder()
                .travelDates(dates)
                .datePlacesName(placesAddRequestDTO.getDatePlacesName())
                .datePlacesCategory(placesAddRequestDTO.getDatePlacesCategory())
                .datePlacesAddress(placesAddRequestDTO.getDatePlacesAddress())
                .datePlacesLat(placesAddRequestDTO.getDatePlacesLat())
                .datePlacesLon(placesAddRequestDTO.getDatePlacesLon())
                .datePlacesIsVisited(false)
                .build();

        // 생성한 일정 장소 저장
        datePlacesRepository.save(places);
    }

    @Override
    @Transactional
    public RentCarDrivingResponseDTO addTravelDatesPlace(PlaceAddRequestDTO placeAddRequestDTO) {
        // 여행 찾기
        Travel travel = travelRepository.findByTravelId(placeAddRequestDTO.getTravelId())
                .orElseThrow(() -> new TravelNotFoundException("해당 travelId의 맞는 여행을 찾을 수 없습니다."));

        // 여행 일정 찾기
        TravelDates date = travelDatesRepository.findByTravelDatesId(placeAddRequestDTO.getTravelDatesId())
                .orElseThrow(() -> new TravelNotFoundException("해당 travelDatesId의 맞는 여행 일정을 찾을 수 없습니다."));

        // 일정의 원래 목적지 장소 찾기
        DatePlaces destinationPlace = datePlacesRepository.findByDatePlacesId(placeAddRequestDTO.getDatePlacesId())
                .orElseThrow(() -> new TravelNotFoundException("해당 datePlacesId의 맞는 일정 장소를 찾을 수 없습니다."));

        // 입력받은 일정(원래 목적지) 이후 장소들 찾기
        List<DatePlaces> afterPlaces = datePlacesRepository.findByTravelDatesAndDatePlacesOrderGreaterThan(date, destinationPlace.getDatePlacesOrder());

        // 추가로 등록할 장소 만들기
        DatePlaces newPlace = DatePlaces.builder()
                .travel(travel)
                .travelDates(date)
                // 순서는 아래에서 저장
                .datePlacesName(placeAddRequestDTO.getDatePlacesName())
                .datePlacesCategory(placeAddRequestDTO.getDatePlacesCategory())
                .datePlacesAddress(placeAddRequestDTO.getDatePlacesAddress())
                .datePlacesLat(placeAddRequestDTO.getDatePlacesLat())
                .datePlacesLon(placeAddRequestDTO.getDatePlacesLon())
                .datePlacesIsVisited(false)
                .build();

        // 응답할 빌더 생성
        RentCarDrivingResponseDTO response = RentCarDrivingResponseDTO.builder()
                .travelId(travel.getTravelId())
                .travelDatesId(date.getTravelDatesId())
                // 현재 이동할 장소
                .build();
        
        // 목적지 이전에 이동
        if(placeAddRequestDTO.getIsBefore()) {
            // 추가로 등록할 장소 순서 조정 -> 원래 목적지 순서로 들어감
            newPlace.setDatePlacesOrder(destinationPlace.getDatePlacesOrder());

            // 입력받은 일정(원래 목적지) 순서 조정 -> 하나 뒤로 밀려남
            destinationPlace.setDatePlacesOrder(destinationPlace.getDatePlacesOrder() + 1);
            datePlacesRepository.save(destinationPlace); // 상태 변경 저장

            // 추가한 장소로 응답
            response.setDatePlacesId(newPlace.getDatePlacesId());
        }

        // 목적지 이후에 이동
        else {
            // 추가로 등록할 장소 순서 조정 -> 원래 목적지 순서 뒤로 들어감
            newPlace.setDatePlacesOrder(destinationPlace.getDatePlacesOrder() + 1);

            // 원래 장소로 응답
            response.setDatePlacesId(destinationPlace.getDatePlacesId());
        }

        // 추가된 장소 상태 변경 저장
        datePlacesRepository.save(newPlace);

        // 이후 일정 순서 조정 -> 순서 하나씩 뒤로 밀기
        for(DatePlaces afterPlace: afterPlaces) {
            afterPlace.setDatePlacesOrder(destinationPlace.getDatePlacesOrder() + 1);
            datePlacesRepository.save(afterPlace);  // 순서 저장
        }

        return response;
    }

    @Override
    @Transactional
    public void updateTravelName(TravelNameRequestDTO travelNameRequestDTO) {
        // 여행 찾기
        Travel travel = travelRepository.findByTravelId(travelNameRequestDTO.getTravelId())
                .orElseThrow(() -> new TravelNotFoundException("해당 travelId의 맞는 여행을 찾을 수 없습니다."));

        // 여행 이름 변경
        travel.setTravelName(travelNameRequestDTO.getTravelName());

        // 변경 상태 저장
        travelRepository.save(travel);
    }

    @Override
    @Transactional
    public void updateTravelDatesPlaces(TravelDetailRequestDTO travelDetailRequestDTO) {
        // 여행 찾기
        Travel travel = travelRepository.findByTravelId(travelDetailRequestDTO.getTravelId())
                .orElseThrow(() -> new TravelNotFoundException("해당 travelId의 맞는 여행을 찾을 수 없습니다."));

        // 여행 일정 리스트 찾기
        List<DatesDetailRequestDTO> datesList = travelDetailRequestDTO.getDatesDetail();

        // 여행 일정 별 반복
        for(DatesDetailRequestDTO date : datesList) {
            // 일정 찾기
            TravelDates dates = travelDatesRepository.findByTravelDatesId(date.getTravelDatesId())
                    .orElseThrow(() -> new TravelNotFoundException("TravelDatesId에 해당하는 일정이 없습니다."));
            log.info("dates: {}", dates);

            // travelId와 travelDatesId에 해당하는 일정들 삭제
            datePlacesRepository.deleteAllByTravelAndTravelDates(travel, dates);
            log.info("일정 삭제 완료");

            // 입력받은 여행 일정 별 장소
            List<PlacesDetailRequestDTO> placesDetail = date.getPlacesDetail();

            if (placesDetail != null && !placesDetail.isEmpty()) {
                // 일정 장소 별 반복해 리스트에 추가
                for (int i = 1; i <= placesDetail.size(); i++) {
                    PlacesDetailRequestDTO placeDetail = placesDetail.get(i - 1);

                    DatePlaces datePlaces = DatePlaces.builder()
                            .travel(travel)
                            .travelDates(dates)
                            .datePlacesOrder(i)
                            .datePlacesName(placeDetail.getDatePlacesName())
                            .datePlacesCategory(placeDetail.getDatePlacesCategory())
                            .datePlacesAddress(placeDetail.getDatePlacesAddress())
                            .datePlacesLat(placeDetail.getDatePlacesLat())
                            .datePlacesLon(placeDetail.getDatePlacesLon())
                            .datePlacesIsVisited(placeDetail.getDatePlacesIsVisited())
                            .build();

                    log.info("{} 일정 추가", placeDetail.getDatePlacesName());
                    datePlacesRepository.saveAndFlush(datePlaces);

                    log.info("저장 완료");
                }
            }
        }

        // 첫번째 날 첫번째 장소는 탑승장소이므로 렌트의 탑승 장소 변경하기
        DatesDetailRequestDTO date = datesList.get(0);
        PlacesDetailRequestDTO placeDetail = date.getPlacesDetail().get(0);
        double rentDptLat = placeDetail.getDatePlacesLat();
        double rentDptLon = placeDetail.getDatePlacesLon();

        // 렌트 찾기
        Rent rent = rentRepository.findByTravel(travel)
                .orElseThrow(() -> new RentNotFoundException("해당 travelId의 맞는 렌트를 찾을 수 없습니다."));

        // 렌트 탑승 장소 위도, 경도 변경
        rent.setRentDptLat(rentDptLat);
        rent.setRentDptLon(rentDptLon);

        // 렌트 변경 상태 저장
        rentRepository.save(rent);
    }

    @Override
    public List<WeatherResponseDTO> getWeather(double datePlacesLat, double datePlacesLon) throws Exception {
        double latitude = datePlacesLat;
        double longitude = datePlacesLon;

        Map<String, Object> weatherData = weatherService.getWeekWeather(latitude, longitude);
        List<Map<String, Object>> weatherList = (List<Map<String, Object>>) weatherData.get("list");

        String[] dayOfWeek = {"일", "월", "화", "수", "목", "금", "토"};
        LocalDate now = LocalDate.now();
        int dayOfWeekValue = now.getDayOfWeek().getValue();
        int dayOfMonth = now.getDayOfMonth();
        List<WeatherResponseDTO> weeklyWeather = new ArrayList<>();

        for (int i = 0; i < 5; i++) {
            WeatherResponseDTO dayDto = new WeatherResponseDTO(dayOfWeek[(dayOfWeekValue + i) % 7], dayOfMonth + i);
            double minOfDay = Double.MAX_VALUE;
            double maxOfDay = Double.MIN_VALUE;
            double totalFeelsLike = 0;
            int totalHumidity = 0;
            double totalPop = 0;

            for (int j = 0; j < 8; j++) {
                int index = i * 8 + j;
                Map<String, Object> dayWeather = weatherList.get(index);
                Map<String, Object> main = (Map<String, Object>) dayWeather.get("main");
                double min = ((Number) main.get("temp_min")).doubleValue();
                double max = ((Number) main.get("temp_max")).doubleValue();
                double feelsLike = ((Number) main.get("feels_like")).doubleValue();
                int humidity = ((Number) main.get("humidity")).intValue();
                double pop = ((Number) dayWeather.get("pop")).doubleValue();

                if (min < minOfDay) minOfDay = min;
                if (max > maxOfDay) maxOfDay = max;
                totalFeelsLike += feelsLike;
                totalHumidity += humidity;
                totalPop += pop;

                if (j == 0) {
                    List<Map<String, Object>> weather = (List<Map<String, Object>>) dayWeather.get("weather");
                    Map<String, Object> weatherObj = weather.get(0);
                    dayDto.setDescription((String) weatherObj.get("icon"));
                }
            }

            dayDto.setMin(minOfDay);
            dayDto.setMax(maxOfDay);
            dayDto.setFeelsLike(totalFeelsLike / 8);
            dayDto.setHumidity(totalHumidity / 8);
            dayDto.setPop(totalPop / 8);
            weeklyWeather.add(dayDto);
        }

        return weeklyWeather;
    }
}
