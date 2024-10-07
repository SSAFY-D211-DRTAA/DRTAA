const functions = require('firebase-functions/v1');
const admin = require("firebase-admin");

admin.initializeApp();

exports.detectRecommend = functions.firestore
  .document("/PLACES/{documentId}")
  .onUpdate((change, context) => {
    // documentId는 알아서 detect한다.
    const documentId = context.params.documentId;

    // 문서가 삭제된 경우는 처리하지 않음
    if (!change.after.exists) {
      functions.logger.log("Document deleted", documentId);
      return null;
    }
    const newValue = change.after.data();
    functions.logger.log("Detected write", newValue);

    // topic으로 fcm 전송
    const topic = documentId; // car id 1로 고정

    const datePlacesName = newValue.datePlacesName || "알 수 없는 장소";
    const datePlacesCategory = newValue.category || "알 수 없는 카테고리";
    const datePlacesAddress = newValue.address || "알 수 없는 주소";
    const datePlacesLat = newValue.datePlacesLat != null ? newValue.datePlacesLat.toString() : "0.0";
    const datePlacesLon = newValue.datePlacesLon != null ? newValue.datePlacesLon.toString() : "0.0";

    // FCM 메시지 생성 (JSON 데이터 포함)
    const message = {
        notification: {
            title: `여행지를 추천합니다!`,
            body: `현재 위치를 기준으로 추천합니다.`
        },
        data: {
            datePlacesName: datePlacesName,
            datePlacesCategory: datePlacesCategory,
            datePlacesAddress: datePlacesAddress,
            datePlacesLat: datePlacesLat,
            datePlacesLon: datePlacesLon
        },
        topic: topic
    };

      return admin.messaging().send(message)
      .then(response => {
        console.log('Successfully sent message:', response);
        return null;
      })
      .catch(error => {
        console.error('Error sending message:', error);
        return null;
      });

    return null;
  });