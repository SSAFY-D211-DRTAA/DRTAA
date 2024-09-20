package com.d211.drtaa.global.config.websocket;

import lombok.Getter;
import lombok.Setter;
import org.springframework.boot.context.properties.ConfigurationProperties;
import org.springframework.context.annotation.Configuration;

@Configuration
@ConfigurationProperties(prefix = "websocket.server")
@Getter
@Setter
public class WebSocketConfig {
    private String url;
}
