package com.d211.drtaa.domain.travel.dto.request;

import com.d211.drtaa.domain.travel.dto.response.DatesDetailResponseDTO;
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
public class TravelDetailRequestDTO {
    @Schema(description = "여행 고유번호", example = "1")
    private long travelId;
    @Schema(description = "여행 이름", example = "서울 여행")
    private String travelName;
    @Schema(description = "여행 시작 일정", example = "2024/01/01")
    private LocalDate travelStartDate;
    @Schema(description = "여행 종료 일정", example = "2024/01/02")
    private LocalDate travelEndDate;

    private List<DatesDetailRequestDTO> datesDetail = new ArrayList<>();
}
