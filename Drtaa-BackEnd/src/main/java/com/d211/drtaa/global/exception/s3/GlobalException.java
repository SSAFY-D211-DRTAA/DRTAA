package com.d211.drtaa.global.exception.s3;

import lombok.Getter;

@Getter
public class GlobalException extends RuntimeException {
    public GlobalException(String message) {
        super(message);
    }

    public GlobalException(String message, Throwable cause) {
        super(message, cause);
    }
}

