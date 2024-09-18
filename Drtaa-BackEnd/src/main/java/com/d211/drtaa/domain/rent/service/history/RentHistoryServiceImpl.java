package com.d211.drtaa.domain.rent.service.history;

import com.d211.drtaa.domain.rent.dto.response.UserDetailHistoryResponseDTO;
import com.d211.drtaa.domain.rent.dto.response.UserHistoryResponseDTO;
import com.d211.drtaa.domain.rent.entity.Rent;
import com.d211.drtaa.domain.rent.entity.car.RentCar;
import com.d211.drtaa.domain.rent.entity.history.RentHistory;
import com.d211.drtaa.domain.rent.repository.RentRepository;
import com.d211.drtaa.domain.rent.repository.history.RentHistoryRepository;
import com.d211.drtaa.domain.user.entity.User;
import com.d211.drtaa.domain.user.repository.UserRepository;
import com.d211.drtaa.global.exception.rent.RentHistoryNotFoundException;
import com.d211.drtaa.global.exception.rent.RentNotFoundException;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.security.core.userdetails.UsernameNotFoundException;
import org.springframework.stereotype.Service;

import java.util.ArrayList;
import java.util.List;

@Service
@RequiredArgsConstructor
@Log4j2
public class RentHistoryServiceImpl implements RentHistoryService{

    private final UserRepository userRepository;
    private final RentHistoryRepository rentHistoryRepository;
    private final RentRepository rentRepository;

    @Override
    public List<UserHistoryResponseDTO> getHistory(String userProviderId) {
        // 사용자 찾기
        User user = userRepository.findByUserProviderId(userProviderId)
                .orElseThrow(() -> new UsernameNotFoundException("해당 userProviderId의 맞는 회원을 찾을 수 없습니다."));

        // 해당 사용자의 렌트 기록 찾기
        List<RentHistory> rentHistories = rentHistoryRepository.findByUser(user);

        List<UserHistoryResponseDTO> response = new ArrayList<>();
        for(RentHistory rentHistory : rentHistories){
            UserHistoryResponseDTO dto = UserHistoryResponseDTO.builder()
                    .renHistoryId(rentHistory.getRentHistoryId())
                    .rentStartTime(rentHistory.getRent().getRentStartTime())
                    .rentPrice(rentHistory.getRent().getRentPrice())
                    .build();

            response.add(dto);
        }

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

    @Override
    public UserDetailHistoryResponseDTO createHistory(String userProviderId, Long rentId) {
        // 사용자 찾기
        User user = userRepository.findByUserProviderId(userProviderId)
                .orElseThrow(() -> new UsernameNotFoundException("해당 userProviderId의 맞는 회원을 찾을 수 없습니다."));

        // 렌트 찾기
        Rent rent = rentRepository.findByRentId(rentId)
                .orElseThrow(() -> new RentNotFoundException("해당 rentId의 맞는 렌트를 찾을 수 없습니다."));

        // 기록 생성
        RentHistory history = RentHistory.builder()
                .user(user)
                .rent(rent)
                .build();

        // 생성된 기록 저장
        rentHistoryRepository.save(history);

        UserDetailHistoryResponseDTO response = UserDetailHistoryResponseDTO.builder()
                // rent
                .rentHeadCount(history.getRent().getRentHeadCount())
                .rentPrice(history.getRent().getRentPrice())
                .rentTime(history.getRent().getRentTime())
                .rentStartTime(history.getRent().getRentStartTime())
                .rentEndTime(history.getRent().getRentEndTime())
                .rentCreatedAt(history.getRent().getRentCreatedAt())
                // rent-car
                .rentCarNumber(history.getRent().getRentCar().getRentCarNumber())
                .rentCarManufacturer(history.getRent().getRentCar().getRentCarManufacturer())
                .rentCarModel(history.getRent().getRentCar().getRentCarModel())
                //travel

                .build();

        return response;
    }
}
