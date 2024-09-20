package com.d211.drtaa.domain.travel.dto.response;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;

import java.time.LocalDate;

@Data
@AllArgsConstructor
@Builder
public class DatesDetailResponseDTO {
    @Schema(description = "여행 일정 고유번호", example = "1")
    private Long travelDatesId;
    @Schema(name = "여행 일정 날짜", example = "2024/01/01")
    private LocalDate travelDatesDate;

    private PlacesDetailResponseDTO placesDetail;
}
