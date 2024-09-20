package com.d211.drtaa.domain.travel.repository;

import com.d211.drtaa.domain.travel.entity.DatePlaces;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.Optional;

public interface DatePlacesRepository extends JpaRepository<DatePlaces, Long> {
    Optional<DatePlaces> findByTravelDatesId(Long travelDatesId);
}
