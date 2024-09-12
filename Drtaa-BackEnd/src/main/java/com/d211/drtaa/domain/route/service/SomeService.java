package com.d211.drtaa.domain.route.service;


import com.d211.drtaa.domain.route.mqtt.Publisher;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
public class SomeService {
    private static final String TAG = SomeService.class.getSimpleName();

    private final Publisher publisher;

    public void publishMessage(String topic, String message) {
        publisher.publishLocations(topic, message);
    }
}
