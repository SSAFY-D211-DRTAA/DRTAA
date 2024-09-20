package com.d211.drtaa.domain.rent.dto.request;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Getter;

@Data
@AllArgsConstructor
@Getter
public class RentStatusRequestDTO {
    @Schema(description = "렌트 고유 번호", example = "1")
    private Long rentId;
    @Schema(description = "렌트 차량 일정 고유 번호", example = "1")
    private long rentCarScheduleId;
}
