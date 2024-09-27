package com.d211.drtaa.domain.travel.dto.request;

import com.d211.drtaa.domain.travel.dto.response.PlacesDetailResponseDTO;
import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Getter;

import java.time.LocalDate;
import java.util.ArrayList;
import java.util.List;

@Data
@AllArgsConstructor
@Getter
public class DatesDetailRequestDTO {
    @Schema(description = "여행 고유번호", example = "1")
    private long travelId;
    @Schema(description = "여행 일정 고유번호", example = "1")
    private Long travelDatesId;
    @Schema(description = "여행 일정 날짜", example = "2024/01/01")
    private LocalDate travelDatesDate;

    private List<PlacesDetailRequestDTO> placesDetail = new ArrayList<>();
}
