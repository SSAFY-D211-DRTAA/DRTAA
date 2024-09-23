package com.d211.drtaa.domain.travel.service;

import com.d211.drtaa.domain.travel.dto.request.PlacesRequestDTO;
import com.d211.drtaa.domain.travel.dto.request.TravelNameRequestDTO;
import com.d211.drtaa.domain.travel.dto.response.TravelDetailResponseDTO;

import java.util.List;

public interface TravelService {
    // travelId의 해당하는 여행 일정, 장소 상세 조회
    TravelDetailResponseDTO getTravel(Long travelId);

    // travelId의 해당하는 여행 중 travelDatesId의 해당하는 일정에 장소 추가
    void createTravelDatesPlaces(PlacesRequestDTO placesRequestDTO);

    // travelId의 해당하는 여행 이름 변경
    void updateTravelName(TravelNameRequestDTO travelNameRequestDTO);
}
