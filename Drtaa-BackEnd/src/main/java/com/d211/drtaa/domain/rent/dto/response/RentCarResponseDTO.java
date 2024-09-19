package com.d211.drtaa.domain.rent.dto.response;

import com.d211.drtaa.domain.rent.entity.car.RentCar;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;

@Data
@AllArgsConstructor
@Builder
public class RentCarResponseDTO {
    private boolean isAvailable;
    private RentCar rentCar;
}
