package com.d211.drtaa.domain.rent.entity.history;

import com.d211.drtaa.domain.rent.entity.Rent;
import com.d211.drtaa.domain.user.entity.User;
import io.swagger.v3.oas.annotations.media.Schema;
import jakarta.persistence.*;
import lombok.*;

@Entity
@Table(name = "rent_history")
@NoArgsConstructor
@AllArgsConstructor
@Getter
@Setter
@Builder
public class RentHistory {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    @Column(name = "rent_history_id", nullable = false)
    @Schema(description = "렌트 기록 고유 번호", example = "1")
    private long renHistoryId;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "user_id", nullable = false)
    @Schema(description = "렌트한 회원 고유 번호", example = "1")
    private User user;

    @OneToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "rent_id", nullable = false)
    @Schema(description = "렌트 고유 번호", example = "1")
    private Rent rent;
}
