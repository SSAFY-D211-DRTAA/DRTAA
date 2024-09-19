package com.d211.drtaa.global.exception.rent;

import lombok.Getter;

@Getter
public class RentCarNotFoundException extends RuntimeException {
    public RentCarNotFoundException(String message) {
        super(message);
    }

    public RentCarNotFoundException(String message, Throwable cause) {
        super(message, cause);
    }
}