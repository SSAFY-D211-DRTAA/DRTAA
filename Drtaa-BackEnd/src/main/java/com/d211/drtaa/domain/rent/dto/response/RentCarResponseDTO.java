package com.d211.drtaa.domain.rent.dto.response;

import com.d211.drtaa.domain.rent.entity.car.RentDrivingStatus;
import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;

@Data
@AllArgsConstructor
@Builder
public class RentCarResponseDTO {
    @Schema(description = "렌트 차량 고유 번호", example = "1")
    private long rentCarId;
    @Schema(description = "렌트 차량 번호", example = "123가1234")
    private String rentCarNumber;
    @Schema(description = "렌트 차량 제조사", example = "KIA")
    private String rentCarManufacturer;
    @Schema(description = "렌트 차량 모델 이름", example = "K5")
    private String rentCarModel;
}
