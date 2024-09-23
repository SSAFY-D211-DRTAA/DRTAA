package com.d211.drtaa.domain.rent.dto.response;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;

@Data
@AllArgsConstructor
@Builder
public class RentCarLocationResponseDTO {
    @Schema(description = "렌트 차량 고유 번호", example = "1")
    private long rentCarId;
    @Schema(description = "렌트 차량 위도", example = "0.0")
    private double rentCarLat;
    @Schema(description = "렌트 차량 경도", example = "0.0")
    private double rentCarLon;
}
