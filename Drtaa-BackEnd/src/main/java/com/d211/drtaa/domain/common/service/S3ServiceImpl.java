package com.d211.drtaa.domain.common.service;

import com.d211.drtaa.domain.common.util.S3Uploader;
import jakarta.transaction.Transactional;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Component;
import org.springframework.stereotype.Service;
import org.springframework.web.multipart.MultipartFile;

import java.io.File;
import java.io.IOException;
import java.util.Objects;

@Service
@RequiredArgsConstructor
@Component
@Slf4j
public class S3ServiceImpl implements S3Service {

    private final S3Uploader s3Uploader;

    @Override
    @Transactional
    public String uploadS3(MultipartFile file, String type) throws IOException {
        String storedFileName = "";

        if(file != null)
            storedFileName = s3Uploader.uploadFileToS3(file, type);

        return storedFileName;
    }

    @Override
    @Transactional
    public String uploadS3(File file, String type) throws IOException {
        String storedFileName = "";
        if(file != null) {
            storedFileName = s3Uploader.uploadFileToS3(file, type);
        }
        return storedFileName;

    }

    @Override
    @Transactional
    public void deleteS3(String fileUrl) throws Exception {
        if(!Objects.equals(fileUrl, ""))
            s3Uploader.deleteS3(fileUrl);
    }
}
