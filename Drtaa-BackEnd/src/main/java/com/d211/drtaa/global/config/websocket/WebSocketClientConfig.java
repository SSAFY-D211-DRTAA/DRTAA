package com.d211.drtaa.global.config.websocket;

import lombok.extern.log4j.Log4j2;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.boot.CommandLineRunner;
import org.springframework.context.annotation.Bean;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.client.standard.StandardWebSocketClient;
import org.springframework.web.socket.handler.TextWebSocketHandler;
import com.fasterxml.jackson.databind.ObjectMapper;

@Component
@Log4j2
public class WebSocketClientConfig {

    private final ObjectMapper objectMapper = new ObjectMapper();

    @Autowired
    private WebSocketConfig webSocketConfig;

    @Bean
    public CommandLineRunner runWebSocketClient() {
        return args -> {
            StandardWebSocketClient client = new StandardWebSocketClient();
            WebSocketSession session = client.execute(new TextWebSocketHandler() {
                @Override
                protected void handleTextMessage(WebSocketSession session, TextMessage message) {
                    log.info(message.getPayload());
                }
            }, webSocketConfig.getUrl()).get();

            MyMessage myMessage = new MyMessage("Spring Boot Connect");
            String jsonMessage = objectMapper.writeValueAsString(myMessage);
            session.sendMessage(new TextMessage(jsonMessage));
        };
    }
}