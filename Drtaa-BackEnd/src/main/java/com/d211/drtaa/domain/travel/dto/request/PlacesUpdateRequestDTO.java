package com.d211.drtaa.domain.travel.dto.request;

import com.d211.drtaa.domain.travel.dto.response.PlacesDetailResponseDTO;
import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Getter;

import java.util.ArrayList;
import java.util.List;

@Data
@AllArgsConstructor
@Getter
public class PlacesUpdateRequestDTO {
    @Schema(description = "여행 고유 번호", example = "1")
    private Long travelId;
    @Schema(description = "여행 일정 고유번호", example = "1")
    private Long travelDatesId;

    List<PlacesDetailResponseDTO> places = new ArrayList<>();
}
