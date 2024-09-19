package com.d211.drtaa.domain.rent.repository.car;

import com.d211.drtaa.domain.rent.entity.car.RentCar;
import org.springframework.data.jpa.repository.JpaRepository;

import java.time.LocalDate;
import java.util.List;
import java.util.Optional;

public interface RentCarRepository extends JpaRepository<RentCar, Long> {
    // find
    Optional<RentCar> findByRentCarId(Long rentCarId);
}
