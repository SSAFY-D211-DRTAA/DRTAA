package com.d211.drtaa.domain.travel.service;

import com.d211.drtaa.domain.travel.dto.request.PlacesAddRequestDTO;
import com.d211.drtaa.domain.travel.dto.request.PlacesUpdateRequestDTO;
import com.d211.drtaa.domain.travel.dto.request.TravelNameRequestDTO;
import com.d211.drtaa.domain.travel.dto.response.DatesDetailResponseDTO;
import com.d211.drtaa.domain.travel.dto.response.PlacesDetailResponseDTO;
import com.d211.drtaa.domain.travel.dto.response.TravelDetailResponseDTO;
import com.d211.drtaa.domain.travel.entity.DatePlaces;
import com.d211.drtaa.domain.travel.entity.Travel;
import com.d211.drtaa.domain.travel.entity.TravelDates;
import com.d211.drtaa.domain.travel.repository.DatePlacesRepository;
import com.d211.drtaa.domain.travel.repository.TravelDatesRepository;
import com.d211.drtaa.domain.travel.repository.TravelRepository;
import com.d211.drtaa.global.exception.travel.TravelNotFoundException;
import jakarta.persistence.EntityManager;
import jakarta.persistence.PersistenceContext;
import jakarta.transaction.Transactional;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.stereotype.Service;

import java.util.List;
import java.util.stream.Collectors;


@Service
@RequiredArgsConstructor
@Log4j2
public class TravelServiceImpl implements TravelService {

    @PersistenceContext
    private EntityManager entityManager;

    private final TravelRepository travelRepository;
    private final TravelDatesRepository travelDatesRepository;
    private final DatePlacesRepository datePlacesRepository;

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
    public void updateTravelDatesPlaces(PlacesUpdateRequestDTO placesUpdateRequestDTO) {
        // 여행 찾기
        Travel travel = travelRepository.findByTravelId(placesUpdateRequestDTO.getTravelId())
                .orElseThrow(() -> new TravelNotFoundException("해당 travelId의 맞는 여행을 찾을 수 없습니다."));

        // 여행 일정 찾기
        TravelDates dates = travelDatesRepository.findByTravelDatesId(placesUpdateRequestDTO.getTravelDatesId())
                .orElseThrow(() -> new TravelNotFoundException("해당 travelDatesId의 맞는 일정을 찾을 수 없습니다."));

        // 이전 일정 모두 지우기
        datePlacesRepository.deleteAllByTravelDates(dates);

        // 기본값 auto_increment를 1로 설정해야 하는 쿼리문 수행
        entityManager.createNativeQuery("ALTER TABLE date_places AUTO_INCREMENT = 1").executeUpdate();

        // 전달 받은 placesUpdateRequestDTO내 places리스트 자료들 저장
        for(PlacesDetailResponseDTO places: placesUpdateRequestDTO.getPlaces()) {
            // 생성
            DatePlaces place = DatePlaces.builder()
                    .travelDates(dates)
                    .datePlacesName(places.getDatePlacesName())
                    .datePlacesCategory(places.getDatePlacesCategory())
                    .datePlacesAddress(places.getDatePlacesAddress())
                    .datePlacesLat(places.getDatePlacesLat())
                    .datePlacesLon(places.getDatePlacesLon())
                    .datePlacesIsVisited(places.getDatePlacesIsVisited())
                    .build();

            // 저장
            datePlacesRepository.save(place);
        }
    }
}
