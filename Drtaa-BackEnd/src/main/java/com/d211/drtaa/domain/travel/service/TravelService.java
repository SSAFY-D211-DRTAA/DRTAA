package com.d211.drtaa.domain.travel.service;

import com.d211.drtaa.domain.travel.dto.request.PlacesAddRequestDTO;
import com.d211.drtaa.domain.travel.dto.request.PlacesUpdateRequestDTO;
import com.d211.drtaa.domain.travel.dto.request.TravelNameRequestDTO;
import com.d211.drtaa.domain.travel.dto.response.TravelDetailResponseDTO;
import com.d211.drtaa.domain.travel.dto.response.WeatherResponseDTO;

import java.util.List;

public interface TravelService {
    // travelId의 해당하는 여행 일정, 장소 상세 조회
    TravelDetailResponseDTO getTravel(Long travelId);

    // travelId의 해당하는 여행 중 travelDatesId의 해당하는 일정에 장소 추가
    void createTravelDatesPlaces(PlacesAddRequestDTO placesAddRequestDTO);

    // travelId의 해당하는 여행 이름 변경
    void updateTravelName(TravelNameRequestDTO travelNameRequestDTO);

    // travelId의 해당하고 travelDatesId의 해당하는 여행 장소들 변경
    void updateTravelDatesPlaces(PlacesUpdateRequestDTO placesUpdateRequestDTO);

    // datePlaceLat과 datePlacesLon에 맞는 일주일치 날씨 조회
    List<WeatherResponseDTO> getWeather(double datePlacesLat, double datePlacesLon) throws Exception;
}
