package com.d211.drtaa.domain.travel.controller;

import com.d211.drtaa.domain.travel.dto.request.PlacesRequestDTO;
import com.d211.drtaa.domain.travel.dto.response.TravelDetailResponseDTO;
import com.d211.drtaa.domain.travel.service.TravelService;
import com.d211.drtaa.global.exception.travel.TravelNotFoundException;
import io.swagger.v3.oas.annotations.Operation;
import io.swagger.v3.oas.annotations.tags.Tag;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@RequestMapping("/travel")
@RequiredArgsConstructor
@Log4j2
@Tag(name = "여행 컨트롤러", description = "여행 일정 관련 기능 수행")
public class TravelController {

    private final TravelService travelService;

    @GetMapping("/{travelId}")
    @Operation(summary = "여행 상세 조회", description = "travelId의 해당하는 여행 일정 전체를 조회")
    public ResponseEntity getTravel(@PathVariable Long travelId) {
        try {
            TravelDetailResponseDTO response = travelService.getTravel(travelId);

            return ResponseEntity.ok(response); // 200
        } catch(TravelNotFoundException e) {
            return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage()); // 404
        } catch (Exception e) {
            return ResponseEntity.badRequest().body(e.getMessage()); // 400
        }
    }

    @PostMapping
    @Operation(summary = "장소 추가", description = "travelId의 해당하는 여행 중 travelDatesId의 해당하는 일정에 장소 추가")
    public ResponseEntity createTravelDatesPlaces(@RequestBody PlacesRequestDTO placesRequestDTO) {
        try {
            travelService.createTravelDatesPlaces(placesRequestDTO);

            return ResponseEntity.ok("Success");
        } catch (TravelNotFoundException e) {
            return ResponseEntity.status(HttpStatus.NOT_FOUND).body(e.getMessage()); // 404
        } catch (Exception e) {
            return ResponseEntity.badRequest().body(e.getMessage()); // 400
        }
    }
}
