package com.d211.drtaa.global.exception.travel;

public class TravelNotFoundException extends RuntimeException {
    public TravelNotFoundException(String message) {
        super(message);
    }
    public TravelNotFoundException(String message, Throwable cause) {
    super(message, cause);
  }
}
