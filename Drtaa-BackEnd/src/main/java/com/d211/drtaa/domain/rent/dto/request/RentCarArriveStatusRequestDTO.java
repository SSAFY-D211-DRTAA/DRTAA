package com.d211.drtaa.domain.rent.dto.request;

import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Getter;

@Data
@AllArgsConstructor
@Getter
public class RentCarArriveStatusRequestDTO {
    @Schema(description = "렌트 차량 고유 번호", example = "1")
    private long rentCarId;
    @Schema(description = "렌트 차량 도착 여부", example = "false")
    private boolean isArrived;
    @Schema(description = "렌트 차량 도착 예상 분", example = "5")
    private int expectedMinutes;

    public boolean isArrived() {
        return isArrived;
    }
}
