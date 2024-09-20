package com.d211.drtaa.domain.travel.repository;

import com.d211.drtaa.domain.travel.entity.DatePlaces;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.List;

public interface DatePlacesRepository extends JpaRepository<DatePlaces, Long> {
    List<DatePlaces> findByTravelDatesId(Long travelDatesId);
}
