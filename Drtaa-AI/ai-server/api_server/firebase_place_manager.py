import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore

# Firebase 프로젝트 초기화 (처음 한 번만 실행)
cred = credentials.Certificate("serviceAccountKey.json")
firebase_admin.initialize_app(cred)

# Firestore 클라이언트 초기화
db = firestore.client()

# 데이터 쓰기
def write_place(address, category, datePlacesLat, datePlacesLon, datePlacesName):
    # 'cities' 컬렉션의 'LA' 문서에 데이터 추가
    doc_ref = db.collection('PLACES').document('1')
    doc_ref.set({
        'address': address,
        'category': category,
        'datePlacesLat': datePlacesLat,
        'datePlacesLon': datePlacesLon,
        'datePlacesName': datePlacesName
    })
    print("Document successfully written!")