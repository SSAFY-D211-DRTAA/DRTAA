package com.d211.drtaa.domain.rent.job;

import com.d211.drtaa.domain.rent.entity.Rent;
import com.d211.drtaa.domain.rent.repository.RentRepository;
import com.d211.drtaa.domain.user.entity.User;
import com.d211.drtaa.global.util.fcm.FcmMessage;
import com.d211.drtaa.global.util.fcm.FcmUtil;
import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.batch.core.Job;
import org.springframework.batch.core.Step;
import org.springframework.batch.core.job.builder.JobBuilder;
import org.springframework.batch.core.launch.support.RunIdIncrementer;
import org.springframework.batch.core.repository.JobRepository;
import org.springframework.batch.core.step.builder.StepBuilder;
import org.springframework.batch.core.step.tasklet.Tasklet;
import org.springframework.batch.repeat.RepeatStatus;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.transaction.PlatformTransactionManager;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;

@Configuration
@RequiredArgsConstructor
@Log4j2
public class RentNotification {

    private final RentRepository rentRepository; // Rent 테이블을 위한 레포지토리
    private final FcmUtil fcmUtil; // FCM 알림 서비스

    @Bean
    public Job rentNotificationJob(JobRepository jobRepository, Step rentNotificationStep) {
        log.info("RentNotification"); // 로그에 RentNotification 출력

        return new JobBuilder("rentNotification", jobRepository) // 새로운 작업을 생성하는 빌더
                .preventRestart() // 작업을 재시작할 수 없도록 설정
                .incrementer(new RunIdIncrementer()) // 작업 실행 시 고유한 ID를 증가시켜주는 설정
                .start(rentNotificationStep) // 작업이 시작할 스텝을 지정
                .build();
    }

    @Bean
    public Step rentNotificationStep(JobRepository jobRepository, Tasklet tasklet, PlatformTransactionManager transactionManager) {
        log.info("RentNotificationStep");

        return new StepBuilder("rentNotificationStep", jobRepository) // 새로운 스텝을 생성하는 빌더
                .tasklet(tasklet, transactionManager) // 실행할 태스크렛과 트랜잭션 매니저를 설정
                .build(); // 스텝 객체를 생성
    }

    @Bean
    public Tasklet tasklet() {
        return ((contribution, chunkContext) -> {
            log.info(">>>>> Tasklet(1일, 3일 후 렌트 알림)");

            // 하루 뒤 예약된 렌트 정보를 조회
            log.info("하루 뒤 예약된 렌트 정보를 조회");
            LocalDateTime startDate = LocalDateTime.now().plusDays(1).withHour(0).withMinute(0).withSecond(0);
            LocalDateTime endDate = LocalDateTime.now().plusDays(1).withHour(23).withMinute(59).withSecond(59);
            List<Rent> tomorrowRents = rentRepository.findByRentStartTimeBetween(startDate, endDate);

            // 3일 뒤 예약된 렌트 정보를 조회
            log.info("3일 뒤 예약된 렌트 정보를 조회");
            startDate = LocalDateTime.now().plusDays(3).withHour(0).withMinute(0).withSecond(0);
            endDate = LocalDateTime.now().plusDays(3).withHour(23).withMinute(59).withSecond(59);
            List<Rent> threeDaysLaterRents = rentRepository.findByRentStartTimeBetween(startDate, endDate);

            // 알림을 보낼 렌트 목록 생성
            List<Rent> allRents = new ArrayList<>();
            allRents.addAll(tomorrowRents);
            allRents.addAll(threeDaysLaterRents);

            // 각 사용자에게 FCM 알림 전송
            for (Rent rent : allRents) {
                User user = rent.getUser(); // 사용자 정보 가져오기

                // FCM DTO 생성
                FcmMessage.FcmDTO fcmDTO = fcmUtil.makeFcmDTO("렌트 알림", rent.getTravel().getTravelName() + "의 렌트 시작이 " + (rent.getRentStartTime().isBefore(LocalDateTime.now().plusDays(3)) ? "하루" : "3일") + " 남았습니다.");
                // FCM 알림 전송
                fcmUtil.singleFcmSend(user, fcmDTO);
            }

            // 태스크렛 실행 완료 상태 반환
            return RepeatStatus.FINISHED;
        });
    }
}
