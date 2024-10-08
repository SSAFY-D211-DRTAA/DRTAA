package com.d211.drtaa.global.exception.travel;

import com.d211.drtaa.domain.rent.dto.response.RentCarManipulateResponseDTO;
import com.d211.drtaa.domain.rent.entity.Rent;
import com.d211.drtaa.global.util.fcm.FcmMessage;
import com.d211.drtaa.global.util.fcm.FcmUtil;
import lombok.extern.log4j.Log4j2;

@Log4j2
public class TravelAllPlacesVisitedException extends RuntimeException {

    FcmUtil fcmUtil;

    public TravelAllPlacesVisitedException(String message, Rent rent) {
        super(message);
        // Android에게 알림 보내기
        FcmMessage.FcmDTO fcmDTO = fcmUtil.makeFcmDTO("렌트 일정", "오늘 예정된 모든 여행지를 방문했습니다.\n 이동하려면 여행지 장소 추가를 해주세요 !!");
        log.info("Message: {}", fcmDTO.getBody());
        fcmUtil.singleFcmSend(rent.getUser(), fcmDTO); // 비동기로 전송
    }

    public TravelAllPlacesVisitedException(String message) {
        super(message);
    }

    public TravelAllPlacesVisitedException(String message, Throwable cause) {
        super(message, cause);
    }
}
