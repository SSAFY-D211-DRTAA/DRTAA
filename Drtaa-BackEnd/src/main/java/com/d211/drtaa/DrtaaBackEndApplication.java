package com.d211.drtaa;

import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.scheduling.annotation.EnableScheduling;

@SpringBootApplication
@EnableScheduling
public class DrtaaBackEndApplication {

    public static void main(String[] args) {
        SpringApplication.run(DrtaaBackEndApplication.class, args);
    }

}
