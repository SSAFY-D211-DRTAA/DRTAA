package com.d211.drtaa.domain.travel.dto.response;

import com.d211.drtaa.domain.rent.entity.RentStatus;
import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;

import java.time.LocalDate;

@Data
@AllArgsConstructor
@Builder
public class TravelResponseDTO {
    @Schema(description = "렌트 고유 번호", example = "1")
    private long rentId;
    @Schema(description = "렌트 상태", example = "reserved")
    private RentStatus rentStatus;
    @Schema(description = "여행 고유 번호", example = "1")
    private Long travelId;
    @Schema(description = "여행 이름", example = "서울 여행")
    private String travelName;
    @Schema(description = "여행 시작 일정", example = "2024/01/01")
    private LocalDate travelStartDate;
    @Schema(description = "여행 종료 일정", example = "2024/01/02")
    private LocalDate travelEndDate;
}
