package com.d211.drtaa.global.exception.s3;

import lombok.Getter;

@Getter
public class S3FileNotFoundException extends RuntimeException {
    public S3FileNotFoundException(String message) {
        super(message);
    }

    public S3FileNotFoundException(String message, Throwable cause) {
        super(message, cause);
    }
}

