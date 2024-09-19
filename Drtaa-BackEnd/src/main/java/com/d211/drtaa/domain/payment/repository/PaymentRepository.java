package com.d211.drtaa.domain.payment.repository;

import com.d211.drtaa.domain.payment.entity.Payment;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

import java.util.List;
import java.util.Optional;

@Repository
public interface PaymentRepository extends JpaRepository<Payment,Long> {

    Optional<Payment> findByReceiptId(String receiptId);

    List<Payment> findByUserId(Long userId);
}
