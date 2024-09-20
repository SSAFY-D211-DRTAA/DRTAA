package com.d211.drtaa.domain.travel.repository;

import com.d211.drtaa.domain.travel.entity.TravelDates;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.Optional;

public interface TravelDatesRepository extends JpaRepository<TravelDates, Long> {
    Optional<TravelDates> findByTravelId(Long travelId);
}