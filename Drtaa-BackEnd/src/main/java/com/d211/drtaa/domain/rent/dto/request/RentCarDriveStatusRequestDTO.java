package com.d211.drtaa.domain.rent.dto.request;

import com.d211.drtaa.domain.rent.entity.car.RentDrivingStatus;
import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Data;
import lombok.Getter;

@Data
@AllArgsConstructor
@Getter
public class RentCarDriveStatusRequestDTO {
    @Schema(description = "렌트 차량 고유 번호", example = "1")
    private long rentCarId;
    @Schema(description = "렌트 차량 주행 상태", example = "parked")
    private RentDrivingStatus rentCarDrivingStatus;
}
