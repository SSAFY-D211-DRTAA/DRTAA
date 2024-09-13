package com.d211.drtaa.domain.rent.repository;

import com.d211.drtaa.domain.rent.entity.Rent;
import org.springframework.data.jpa.repository.JpaRepository;

public interface RentRepository extends JpaRepository<Rent, Long> {
}
