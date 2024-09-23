package com.d211.drtaa.domain.rent.dto.request;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Getter;

import java.time.LocalDate;

@Data
@AllArgsConstructor
@Getter
public class RentCheckRequestDTO {
    @Schema(description = "렌트 이용 시작 시간", example = "2024/01/01 00:00")
    private LocalDate rentStartTime;
    @Schema(description = "렌트 이용 종료 시간", example = "2024/01/01 02:00")
    private LocalDate rentEndTime;
}
