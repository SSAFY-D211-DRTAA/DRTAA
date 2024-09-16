package com.d211.drtaa.domain.rent.service.history;

import com.d211.drtaa.domain.rent.dto.response.UserDetailHistoryResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.UserHistoryResponseDTO;
import com.d211.drtaa.domain.rent.entity.Rent;
import com.d211.drtaa.domain.rent.entity.car.RentCar;
import com.d211.drtaa.domain.rent.entity.history.RentHistory;
import com.d211.drtaa.domain.rent.repository.history.RentHistoryRepository;
import com.d211.drtaa.domain.user.entity.User;
import com.d211.drtaa.domain.user.repository.UserRepository;
import com.d211.drtaa.global.exception.rent.RentHistoryNotFoundException;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.security.core.userdetails.UsernameNotFoundException;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
@Log4j2
public class RentHistoryServiceImpl implements RentHistoryService{

    private final UserRepository userRepository;
    private final RentHistoryRepository rentHistoryRepository;

    @Override
    public UserHistoryResponseDTO getHistory(String userProviderId) {
        // 사용자 찾기
        User user = userRepository.findByUserProviderId(userProviderId)
                .orElseThrow(() -> new UsernameNotFoundException("해당 userProviderId의 맞는 회원을 찾을 수 없습니다."));

        // 해당 사용자의 렌트 기록 찾기
        RentHistory history = rentHistoryRepository.findByUserId(user.getUserId())
                .orElseThrow(() -> new RentHistoryNotFoundException("해당 userId의 맞는 기록을 찾을 수 없습니다."));

        UserHistoryResponseDTO response = UserHistoryResponseDTO.builder()
                .renHistoryId(history.getRenHistoryId())
                .rentStartTime(history.getRent().getRentStartTime())
                .rentPrice(history.getRent().getRentPrice())
                .build();

        return response;
    }

    @Override
    public UserDetailHistoryResponseDTO getDetailHistory(Long rentHistoryId) {
        // rentHistoryId의 해당하는 렌트 기록 찾기
        RentHistory history = rentHistoryRepository.findByRentHistoryId(rentHistoryId)
                .orElseThrow(() -> new RentHistoryNotFoundException("해당 rentHistoryId의 맞는 기록을 찾을 수 없습니다."));

        // rent
        Rent rent = history.getRent();

        // rent-car
        RentCar rentCar = history.getRent().getRentCar();

        UserDetailHistoryResponseDTO response = UserDetailHistoryResponseDTO.builder()
                // rent
                .rentHeadCount(rent.getRentHeadCount())
                .rentPrice(rent.getRentPrice())
                .rentTime(rent.getRentTime())
                .rentStartTime(rent.getRentStartTime())
                .rentEndTime(rent.getRentEndTime())
                .rentCreatedAt(rent.getRentCreatedAt())

                // rent-car
                .rentCarNumber(rentCar.getRentCarNumber())
                .rentCarManufacturer(rentCar.getRentCarManufacturer())
                .rentCarModel(rentCar.getRentCarModel())

                // travel
                .build();

        return response;
    }
}
