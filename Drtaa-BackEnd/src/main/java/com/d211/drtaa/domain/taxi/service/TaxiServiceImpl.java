package com.d211.drtaa.domain.taxi.service;


import com.d211.drtaa.domain.taxi.dto.request.TaxiCreateRequestDTO;
import com.d211.drtaa.domain.taxi.dto.response.TaxiDetailResponseDTO;
import com.d211.drtaa.domain.user.entity.User;
import com.d211.drtaa.domain.user.repository.UserRepository;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.security.core.userdetails.UsernameNotFoundException;
import org.springframework.stereotype.Service;
import org.springframework.transaction.annotation.Transactional;

@Service
@RequiredArgsConstructor
@Log4j2
public class TaxiServiceImpl implements TaxiService {

    private final UserRepository userRepository;

    @Override
    @Transactional
    public TaxiDetailResponseDTO createTaxi(String userProviderId, TaxiCreateRequestDTO rentCreateRequestDTO) {
        User user = userRepository.findByUserProviderId(userProviderId)
                .orElseThrow(() -> new UsernameNotFoundException("해당 userProviderId의 맞는 회원을 찾을 수 없습니다."));


    }

}
