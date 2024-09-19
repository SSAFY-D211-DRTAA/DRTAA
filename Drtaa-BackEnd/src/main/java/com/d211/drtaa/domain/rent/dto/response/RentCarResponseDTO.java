package com.d211.drtaa.domain.rent.dto.response;

import com.d211.drtaa.domain.rent.entity.car.RentCar;
import com.d211.drtaa.domain.rent.entity.car.RentDrivingStatus;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;

import java.util.List;

@Data
@AllArgsConstructor
@NoArgsConstructor
@Builder
public class RentCarResponseDTO {
    private boolean isAvailable;
    private long rentCarId;
    private String rentCarNumber;
    private String rentCarManufacturer;
    private String rentCarModel;
    private String rentCarImg;
    private RentDrivingStatus rentCarDrivingStatus;
}