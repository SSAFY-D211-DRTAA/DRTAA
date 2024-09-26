package com.d211.drtaa.domain.travel.service;

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
import com.d211.drtaa.global.exception.travel.TravelNotFoundException;
import com.d211.drtaa.global.service.weather.WeatherService;
import jakarta.persistence.EntityManager;
import jakarta.persistence.PersistenceContext;
import jakarta.transaction.Transactional;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.security.core.userdetails.UsernameNotFoundException;
import org.springframework.stereotype.Service;

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
                            .datePlacesId(places.getDatePlacesId())
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
                    .travelDatesId(dates.getTravelDatesId())
                    .travelDatesDate(dates.getTravelDatesDate())
                    .placesDetail(placesDtoList) // 일정 장소 리스트를 설정
                    .build();
        }).collect(Collectors.toList());

        // 최종 여행 DTO 생성
        TravelDetailResponseDTO dto = TravelDetailResponseDTO.builder()
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

            // 여행 일정 별 장소 테이블 비우기
            datePlacesRepository.deleteAllByTravelDates(dates);

            // 입력받은 여행 일정 별 장소
            List<PlacesDetailRequestDTO> placesDetail = date.getPlacesDetail();

            if (placesDetail != null && !placesDetail.isEmpty()) {
                // 일정 장소 저장할 리스트
                List<DatePlaces> datePlacesList = new ArrayList<>();

                // 일정 장소 별 반복해 리스트에 추가
                for (PlacesDetailRequestDTO places : placesDetail) {
                    DatePlaces datePlaces = DatePlaces.builder()
                            .travelDates(dates)
                            .datePlacesName(places.getDatePlacesName())
                            .datePlacesCategory(places.getDatePlacesCategory())
                            .datePlacesAddress(places.getDatePlacesAddress())
                            .datePlacesLat(places.getDatePlacesLat())
                            .datePlacesLon(places.getDatePlacesLon())
                            .datePlacesIsVisited(places.getDatePlacesIsVisited())
                            .build();
                    datePlacesList.add(datePlaces);
                }

                // DB에 한 번에 저장
                datePlacesRepository.saveAll(datePlacesList);
            }
        }
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
