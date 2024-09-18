package com.d211.drtaa.domain.rent.dto.request;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Getter;

@Data
@AllArgsConstructor
@Getter
public class RentRequestDTO {
    @Schema(description = "렌트 고유 번호", example = "1")
    private Long rentId;
    @Schema(description = "렌트 이용 인원 수", example = "1(단위: 명)")
    private Integer rentHeadCount;
    @Schema(description = "렌트 탑승 장소 위도", example = "0.0")
    private Double rentDptLat;
    @Schema(description = "렌트 탑승 장소 경도", example = "0.0")
    private Double rentDptLon;
}
