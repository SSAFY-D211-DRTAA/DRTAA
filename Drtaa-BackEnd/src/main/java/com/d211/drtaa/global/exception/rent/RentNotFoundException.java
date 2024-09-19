package com.d211.drtaa.global.exception.rent;

import lombok.Getter;

@Getter
public class RentNotFoundException extends RuntimeException {
    public RentNotFoundException(String message) {
        super(message);
    }

    public RentNotFoundException(String message, Throwable cause) {
        super(message, cause);
    }
}