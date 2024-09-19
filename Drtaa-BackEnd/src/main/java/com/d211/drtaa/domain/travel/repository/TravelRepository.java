package com.d211.drtaa.domain.travel.repository;

import com.d211.drtaa.domain.travel.entity.Travel;
import org.springframework.data.jpa.repository.JpaRepository;

public interface TravelRepository extends JpaRepository<Travel, Long> {
}
