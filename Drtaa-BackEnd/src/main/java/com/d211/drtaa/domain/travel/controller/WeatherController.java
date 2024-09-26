package com.d211.drtaa.domain.travel.controller;

import com.d211.drtaa.domain.travel.dto.response.WeatherResponseDTO;
import com.d211.drtaa.domain.travel.service.TravelService;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.security.core.parameters.P;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@RequestMapping("/travel")
@RequiredArgsConstructor
@CrossOrigin("*")
@Tag(name = "날씨 컨트롤러", description = "OpenWeatherAPI 사용")
@Log4j2
public class WeatherController {

    private final TravelService travelService;

    @GetMapping("/weather")
    @Operation(summary = "여행지 날씨 조회", description = "여행지 위도, 경도를 사용해 일주일치 날씨 조회")
    public ResponseEntity getWeather(@RequestParam("datePlacesLat")double datePlacesLat, @RequestParam("datePlacesLon")double datePlacesLon) {
        try {
            List<WeatherResponseDTO> response = travelService.getWeather(datePlacesLat, datePlacesLon);

            return ResponseEntity.ok(response);
        } catch (Exception e) {
            return ResponseEntity.status(HttpStatus.BAD_REQUEST).body(e.getMessage()); // 400
        }
    }
}
