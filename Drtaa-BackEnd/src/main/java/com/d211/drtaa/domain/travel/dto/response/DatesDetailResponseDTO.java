package com.d211.drtaa.domain.travel.dto.response;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;

import java.time.LocalDate;
import java.util.ArrayList;
import java.util.List;

@Data
@AllArgsConstructor
@Builder
public class DatesDetailResponseDTO {
    @Schema(description = "여행 고유 번호", example = "1")
    private Long travelId;
    @Schema(description = "여행 일정 고유번호", example = "1")
    private Long travelDatesId;
    @Schema(description = "여행 일정 날짜", example = "2024/01/01")
    private LocalDate travelDatesDate;
    @Schema(description = "여행 일정 만료 여부", example = "false")
    private Boolean travelDatesIsExpired;

    private List<PlacesDetailResponseDTO> placesDetail = new ArrayList<>();
}
