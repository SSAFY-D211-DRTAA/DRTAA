package com.d211.drtaa.domain.travel.repository;

import com.d211.drtaa.domain.travel.entity.Travel;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.Optional;

public interface TravelRepository extends JpaRepository<Travel, Long> {
    Optional<Travel> findByTravelId(Long travelId);
}
