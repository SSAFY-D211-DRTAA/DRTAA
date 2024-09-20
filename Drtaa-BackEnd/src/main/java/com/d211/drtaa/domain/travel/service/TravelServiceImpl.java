package com.d211.drtaa.domain.travel.service;

import com.d211.drtaa.domain.travel.dto.response.TravelDetailResponseDTO;
import com.d211.drtaa.domain.travel.entity.Travel;
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
    public List<TravelDetailResponseDTO> getTravel(Long travelId) {
        Travel travel = travelRepository.findByTravelId(travelId)
                .orElseThrow(() -> new TravelNotFoundException("해당 travelId의 맞는 렌트를 찾을 수 없습니다."));

        return List.of();
    }
}
