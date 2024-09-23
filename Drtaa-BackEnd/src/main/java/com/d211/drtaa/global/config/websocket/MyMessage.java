package com.d211.drtaa.global.config.websocket;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;
import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

@JsonIgnoreProperties(ignoreUnknown = true)
@NoArgsConstructor
@AllArgsConstructor
@Getter
@Setter
public class MyMessage {
    private String action;
    private double latitude;
    private double longitude;

    public MyMessage(String action) {
        this.action = action;
    }
}