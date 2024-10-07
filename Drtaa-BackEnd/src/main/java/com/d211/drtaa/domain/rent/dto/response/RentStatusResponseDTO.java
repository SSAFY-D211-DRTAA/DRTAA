package com.d211.drtaa.domain.rent.dto.response;

import com.d211.drtaa.domain.rent.entity.RentStatus;
import com.d211.drtaa.domain.rent.entity.car.RentDrivingStatus;
import io.swagger.v3.oas.annotations.media.Schema;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;

@Data
@AllArgsConstructor
@Builder
public class RentStatusResponseDTO {
    @Schema(description = "렌트 상태", example = "reserved")
    private RentStatus rentStatus;
    @Schema(description = "렌트 차량 주행 상태", example = "idling")
    private RentDrivingStatus rentCarDrivingStatus;
}
