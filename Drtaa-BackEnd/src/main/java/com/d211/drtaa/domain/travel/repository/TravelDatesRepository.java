package com.d211.drtaa.domain.travel.repository;

import com.d211.drtaa.domain.travel.entity.TravelDates;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.List;
import java.util.Optional;

public interface TravelDatesRepository extends JpaRepository<TravelDates, Long> {
    List<TravelDates> findByTravelId(Long travelId);
    Optional<TravelDates> findByTravelDatesId(Long travelId);
}