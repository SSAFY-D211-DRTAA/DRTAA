package com.d211.study.repository;

import com.d211.study.domain.Member;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.Optional;

public interface MemberRepository extends JpaRepository<Member, Long> {
    Optional<Member> findByMemberEmail(String memberEmail);
    Optional<String> findMemberRefreshTokenByMemberEmail(String memberEmail);
    boolean existsByMemberEmail(String email);
    boolean deleteByMemberEmail(String email);
}
