package com.d211.drtaa.domain.rent.service;

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

@Configuration
@RequiredArgsConstructor
@Log4j2
public class RentNotification {
    @Bean
    public Job rentNotificationJob(JobRepository jobRepository, Step rentNotificationStep) {
        log.info("RentNotification");

        return new JobBuilder("rentNotification", jobRepository)
                .preventRestart()
                .incrementer(new RunIdIncrementer())
                .start(rentNotificationStep)
                .build();
    }

    @Bean
    public Step rentNotificationStep(JobRepository jobRepository, Tasklet tasklet, PlatformTransactionManager transactionManager) {
        log.info("RentNotificationStep");

        return new StepBuilder("rentNotificationStep", jobRepository)
                .tasklet(tasklet, transactionManager)
                .build();
    }

    @Bean
    public Tasklet tasklet() {
        return ((contribution, chunkContext) -> {
            log.info(">>>>> Tasklet");
            return RepeatStatus.FINISHED;
        });
    }
}
