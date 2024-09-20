package com.d211.drtaa.domain.travel.service;

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
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.stereotype.Service;

import java.util.List;

@Service
@RequiredArgsConstructor
@Log4j2
public class TravelServiceImpl implements TravelService {

    private final TravelRepository travelRepository;
    private final TravelDatesRepository travelDatesRepository;
    private final DatePlacesRepository datePlacesRepository;

    @Override
    public TravelDetailResponseDTO getTravel(Long travelId) {
        // 여행 찾기
        Travel travel = travelRepository.findByTravelId(travelId)
                .orElseThrow(() -> new TravelNotFoundException("해당 travelId의 맞는 여행을 찾을 수 없습니다."));

        // 여행 일정 찾기
        TravelDates dates = travelDatesRepository.findByTravelId(travelId)
                .orElseThrow(() -> new TravelNotFoundException("해당 travelId의 맞는 여행을 찾을 수 없습니다."));

        // 일정 장소 찾기
        DatePlaces places = datePlacesRepository.findByTravelDatesId(dates.getTravelDatesId())
                .orElseThrow(() -> new TravelNotFoundException("해당 travelDatesId의 맞는 여행 일정을 찾을 수 없습니다."));

        PlacesDetailResponseDTO placesDto = PlacesDetailResponseDTO.builder()
                .travelDatesId(places.getTravelDatesId())
                .datePlacesName(places.getDatePlacesName())
                .datePlacesLat(places.getDatePlacesLat())
                .datePlacesLon(places.getDatePlacesLon())
                .build();

        DatesDetailResponseDTO datesDto = DatesDetailResponseDTO.builder()
                .travelDatesId(dates.getTravelDatesId())
                .travelDatesDate(dates.getTravelDatesDate())
                .placesDetail(placesDto)
                .build();

        TravelDetailResponseDTO dto = TravelDetailResponseDTO.builder()
                .travelName(travel.getTravelName())
                .travelStartDate(travel.getTravelStartDate())
                .travelEndDate(travel.getTravelEndDate())
                .datesDetail(datesDto)
                .build();
        
        return dto;
    }
}
