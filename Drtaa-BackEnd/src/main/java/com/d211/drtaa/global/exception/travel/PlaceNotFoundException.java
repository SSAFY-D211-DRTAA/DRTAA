package com.d211.drtaa.global.exception.travel;

public class PlaceNotFoundException extends RuntimeException {
    public PlaceNotFoundException(String message) {
        super(message);
    }
    public PlaceNotFoundException(String message, Throwable cause) {
        super(message, cause);
    }
}
