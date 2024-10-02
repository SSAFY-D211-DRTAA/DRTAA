package com.d211.drtaa.global.exception.travel;

public class TravelDateNotMatchException extends RuntimeException {
    public TravelDateNotMatchException(String message) {
        super(message);
    }
    public TravelDateNotMatchException(String message, Throwable cause) {
        super(message, cause);
    }
}