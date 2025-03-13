import firebase_admin
from firebase_admin import credentials
from firebase_admin import firestore, messaging

from google.cloud.firestore import ArrayUnion

import json

# Firebase 인증 정보를 제공하는 서비스 계정 키 파일을 다운로드하고 경로를 설정합니다.
cred = credentials.Certificate('/home/morai/catkin_ws/src/ssafy_3/key/ifind-firebase-adminsdk.json')
# firebase_admin.initialize_app(cred)

# Firebase 앱 초기화
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://ifind.firebaseio.com'
})

# Firestore 데이터베이스를 가져옵니다.
db = firestore.client()

# 메시지 보내기
message = messaging.Message(
    notification=messaging.Notification(
        title='아이파인',
        body='학원에 도착했습니다.',
    ),
    token='exMLvsouRdOGLlODTVSdfx:APA91bHIRgFPM_U7uv4VgvyWr1v2oxI4FhJPWx269zuHvg_fhufU-496EsSUVo5sV7znV09spcQG1ewRV-NnhP9AS-Mev8fl4s5YL3Sk6EIbySgnaKLqLbpu_W6qynb5SbKRM_BG5NpX',
)
response = messaging.send(message)

print("메시지 보냈지롱", response)

# # 예제 경로 데이터
# path_data = [
#     {'latitude': 37.5665, 'longitude': 126.9780},
#     {'latitude': 37.5675, 'longitude': 126.9790},
#     {'latitude': 37.5675, 'longitude': 126.9790},
#     {'latitude': 37.5675, 'longitude': 126.9790},

# ]

# # morai/path 문서에 쓸 데이터를 정의합니다.
# path_document_data = {
#     'path': path_data
# }

# # morai/path 문서에 데이터를 쓰거나, 이미 문서가 있다면 덮어씁니다.
# db.collection('morai').document('path').set(path_document_data)

# # 데이터를 추가할 컬렉션과 문서 ID를 설정합니다.
# collection_name = 'morai'
# document_id = 'path'

# # 추가할 데이터를 딕셔너리 형태로 작성합니다.
# data = {
#     'type': 2,
# }

# # 데이터를 컬렉션에 추가합니다.
# doc_ref = db.collection(collection_name).document(document_id)

# doc_ref.update({'alert': ArrayUnion([data])})

# print('데이터가 성공적으로 추가되었습니다.')

# doc = db.collection('morai').document('node').get()

# if doc.exists:
#     # 문서의 데이터를 딕셔너리 형태로 변환
#     data = doc.to_dict()
#     print("Document data:", data)

#     # 변환된 데이터를 리스트로 재구성
#     data_list = []
#     for key, value in data.items():
#         node_data = {'number': key}
#         node_data.update(value)
#         data_list.append(node_data)
    
#     # 결과 출력
#     for item in data_list:
#         print(item)
# else:
#     print("저장된 하차 지점이 없습니다.")