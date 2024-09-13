package com.d211.drtaa.domain.rent.repository.history;

import com.d211.drtaa.domain.rent.entity.history.RentHistory;
import org.springframework.data.jpa.repository.JpaRepository;

public interface RentHistoryRepository extends JpaRepository<RentHistory, Long> {
}
