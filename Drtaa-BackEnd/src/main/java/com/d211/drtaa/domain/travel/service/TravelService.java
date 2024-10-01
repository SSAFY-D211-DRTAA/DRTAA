package com.d211.drtaa.domain.travel.service;

import com.d211.drtaa.domain.travel.dto.request.PlaceAddRequestDTO;
import com.d211.drtaa.domain.travel.dto.request.PlacesAddRequestDTO;
import com.d211.drtaa.domain.travel.dto.request.TravelDetailRequestDTO;
import com.d211.drtaa.domain.travel.dto.request.TravelNameRequestDTO;
import com.d211.drtaa.domain.travel.dto.response.TravelDetailResponseDTO;
import com.d211.drtaa.domain.travel.dto.response.TravelResponseDTO;
import com.d211.drtaa.domain.travel.dto.response.WeatherResponseDTO;

import java.util.List;

public interface TravelService {
    // 회원의 모든 여행 일정 조회
    List<TravelResponseDTO> getAllTravels(String userProviderId);

    // 해당 회원의 완료된 렌트의 여행 조회
    List<TravelResponseDTO> getAllTravelsCompleted(String userProviderId);

    // 해당 회원의 진행중인 렌트의 여행 한개와 예약된 렌트의 여행 전체 조회
    List<TravelResponseDTO> getAllTravelsActive(String userProviderId);

    // travelId의 해당하는 여행 일정, 장소 상세 조회
    TravelDetailResponseDTO getTravel(Long travelId);

    // travelId의 해당하는 여행 중 travelDatesId의 해당하는 일정에 장소 추가
    void createTravelDatesPlaces(PlacesAddRequestDTO placesAddRequestDTO);

    // 여행 일정 장소 이전 또는 이후에 추가
    void addTravelDatesPlace(PlaceAddRequestDTO placeAddRequestDTO);

    // travelId의 해당하는 여행 이름 변경
    void updateTravelName(TravelNameRequestDTO travelNameRequestDTO);

    // travelId의 해당하고 travelDatesId의 해당하는 여행 장소들 변경
    void updateTravelDatesPlaces(TravelDetailRequestDTO travelDetailRequestDTO);

    // datePlaceLat과 datePlacesLon에 맞는 일주일치 날씨 조회
    List<WeatherResponseDTO> getWeather(double datePlacesLat, double datePlacesLon) throws Exception;
}
