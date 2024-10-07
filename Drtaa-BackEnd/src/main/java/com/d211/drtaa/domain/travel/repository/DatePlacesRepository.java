package com.d211.drtaa.domain.travel.repository;

import com.d211.drtaa.domain.travel.entity.DatePlaces;
import com.d211.drtaa.domain.travel.entity.Travel;
import com.d211.drtaa.domain.travel.entity.TravelDates;
import io.lettuce.core.dynamic.annotation.Param;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.Query;

import java.util.List;
import java.util.Optional;

public interface DatePlacesRepository extends JpaRepository<DatePlaces, Long> {
    // find
    Optional<DatePlaces> findByDatePlacesId(Long datePlacesId);
    Optional<DatePlaces> findFirstByTravelDatesOrderByDatePlacesOrderDesc(TravelDates date);
    Optional<DatePlaces> findByTravelDatesAndDatePlacesOrder(TravelDates date, int datePlacesOrder);
    Optional<DatePlaces> findFirstByTravelAndDatePlacesIsExpiredFalseOrderByDatePlacesIdAsc(Travel travel);
    DatePlaces findFirstByTravelDatesOrderByDatePlacesOrderAsc(TravelDates travelDates);
    DatePlaces findFirstByTravelDates(TravelDates travelDates);
    List<DatePlaces> findByTravelDates(TravelDates travelDates);
    List<DatePlaces> findByTravelDatesAndDatePlacesOrderGreaterThan(TravelDates date, int datePlacesOrder);
    List<DatePlaces> findByTravelDatesAndDatePlacesIsExpiredFalse(TravelDates date);
    List<DatePlaces> findByTravelDatesOrderByDatePlacesOrderAsc(TravelDates date);

    // delete
    void deleteAllByTravelAndTravelDates(Travel travel, TravelDates dates);

}
