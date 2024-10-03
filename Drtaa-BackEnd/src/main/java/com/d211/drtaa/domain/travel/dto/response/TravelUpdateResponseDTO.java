package com.d211.drtaa.domain.travel.dto.response;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;

import java.time.LocalDate;

@Data
@AllArgsConstructor
@Builder
public class TravelUpdateResponseDTO {
    @Schema(description = "여행 고유 번호", example = "1")
    private Long travelId;
    @Schema(description = "여행 일정 고유번호", example = "1")
    private Long travelDatesId;
    @Schema(description = "여행 일정 날짜, 해당 날짜를 기준으로 안드로이드에 저장할 것인지 판별하기", example = "2024/01/01")
    private LocalDate travelDatesDate;
    @Schema(description = "일정 장소 고유번호", example = "1")
    private long datePlacesId;
}
