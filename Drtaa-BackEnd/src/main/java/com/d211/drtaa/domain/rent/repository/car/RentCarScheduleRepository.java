package com.d211.drtaa.domain.rent.repository.car;

import com.d211.drtaa.domain.rent.entity.car.RentCar;
import com.d211.drtaa.domain.rent.entity.car.RentCarSchedule;
import org.springframework.data.jpa.repository.JpaRepository;

import java.time.LocalDate;
import java.util.List;
import java.util.Optional;

public interface RentCarScheduleRepository extends JpaRepository<RentCarSchedule, Long> {
    List<RentCarSchedule> findByRentCar(RentCar car);
    Optional<RentCarSchedule> findByRentCarScheduleId(long rentCarScheduleId);
    List<RentCarSchedule> findByRentCarAndRentCarScheduleIsDoneFalse(RentCar car);
    Optional<RentCarSchedule> findByRentRentId(long rentId);
}
