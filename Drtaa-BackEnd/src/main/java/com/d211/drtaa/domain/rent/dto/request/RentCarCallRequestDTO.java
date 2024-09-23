package com.d211.drtaa.domain.rent.dto.request;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Getter;

@Data
@AllArgsConstructor
@Getter
public class RentCarCallRequestDTO {
    @Schema(description = "렌트 고유 번호", example = "1")
    private long rentId;
    @Schema(description = "회원 현재 위치 위도", example = "0.0")
    private double userLat;
    @Schema(description = "회원 현재 위치 경도", example = "0.0")
    private double userLon;
}
