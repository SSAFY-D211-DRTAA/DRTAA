//package com.d211.drtaa.domain.route.mqtt;
//
//
//import com.d211.drtaa.domain.route.service.SomeService;
//import lombok.RequiredArgsConstructor;
//import lombok.extern.log4j.Log4j2;
//import org.springframework.context.annotation.Bean;
//import org.springframework.context.annotation.Configuration;
//import org.springframework.integration.annotation.ServiceActivator;
//import org.springframework.messaging.MessageHandler;
//import org.springframework.messaging.MessageHeaders;
//
//@Log4j2
//@Configuration
//@RequiredArgsConstructor
//public class Subscriber {
//    private static final String TAG = Subscriber.class.getSimpleName();
//    private final SomeService someService;
//    @Bean
//    @ServiceActivator(inputChannel = "mqttInputChannel")
//    public MessageHandler messageHandler(){
//
//
//
//        return message -> {
//            log.info("메시지 수신! Subscriber");
//            MessageHeaders headers = message.getHeaders();
//            String payload = message.getPayload().toString();
//            String topic = headers.get("mqtt_receivedTopic", String.class);
//            log.info("topic : {}, payload : {}", topic, payload);
//
//            someService.publishMessage("sub/topic","hi d211");
//
//            switch (topic){ // 나중에 분기처리 하기
//                case "test1":
//                    break;
//                case "test2":
//                    break;
//                default:
//                    log.info("들어왓어 {}",topic);
//            }
//        };
//    }
//
//}
