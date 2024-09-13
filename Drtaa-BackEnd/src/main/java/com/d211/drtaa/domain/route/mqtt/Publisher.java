package com.d211.drtaa.domain.route.mqtt;


import lombok.RequiredArgsConstructor;
import lombok.extern.log4j.Log4j2;
import org.springframework.messaging.MessageChannel;
import org.springframework.messaging.support.MessageBuilder;
import org.springframework.stereotype.Component;

import java.nio.charset.StandardCharsets;

@Log4j2
@Component
@RequiredArgsConstructor
public class Publisher {
    private static final String TAG = Publisher.class.getSimpleName();

    private final MessageChannel mqttOutputChannel;

    public void publishLocations(String topic, String payload){
        try {
            log.info("{} publish : {}", topic,payload);

            byte[] bytePayload = payload.getBytes(StandardCharsets.UTF_8);
            mqttOutputChannel.send(MessageBuilder.withPayload(bytePayload)
                    .setHeader("mqtt_topic", topic).build());
        } catch (Exception e){
            log.error("publish 중 에러 발생{}",e);
        }
    }
}
