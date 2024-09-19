package com.d211.drtaa.domain.rent.dto.request;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Getter;

import java.time.LocalDate;

@Data
@AllArgsConstructor
@Getter
public class RentCarUnassignedDispatchStatusRequestDTO {
    @Schema(description = "렌트 차량 일정 시작 일자", example = "2024/01/01")
    private LocalDate rentCarScheduleStartDate;
    @Schema(description = "렌트 차량 종료 시작 일자", example = "2024/01/02")
    private LocalDate rentCarScheduleEndDate;
}
