package com.d211.drtaa.global.exception.travel;

import com.d211.drtaa.domain.rent.dto.response.RentCarManipulateResponseDTO;
import com.d211.drtaa.domain.rent.entity.Rent;
import com.d211.drtaa.global.util.fcm.FcmMessage;
import com.d211.drtaa.global.util.fcm.FcmUtil;
import lombok.extern.log4j.Log4j2;

@Log4j2
public class TravelNextDayResponseException extends RuntimeException {

    private FcmUtil fcmUtil;
    private RentCarManipulateResponseDTO response;

    public TravelNextDayResponseException(String message, Rent rent, RentCarManipulateResponseDTO response) {
        super(message);
        // Android에게 알림 보내기
        FcmMessage.FcmDTO fcmDTO = fcmUtil.makeFcmDTO("렌트 일정", "오늘 예정된 모든 여행지를 방문했습니다.\n 이동하려면 여행지 장소 추가를 해주세요 !!");
        log.info("Message: {}", fcmDTO.getBody());
        fcmUtil.singleFcmSend(rent.getUser(), fcmDTO); // 비동기로 전송

        this.response = response;
    }

    public TravelNextDayResponseException(String message) {
        super(message);
    }

    public TravelNextDayResponseException(String message, Throwable cause) {
        super(message, cause);
    }

    public RentCarManipulateResponseDTO getResponse() {
        return response;
    }
}
