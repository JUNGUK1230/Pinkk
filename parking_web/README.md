# parking_web

상단 카메라 영상을 받아 카메라 왜곡 보정과 BEV 변환을 적용한 뒤
Flask 웹페이지에 실시간으로 출력하는 최소 구성입니다.

## 폴더 구조

parking_web/
├── app.py
├── requirements.txt
└── templates/
    └── index.html

## 1. 파일 경로 수정

app.py에서 아래 경로를 실제 파일 위치에 맞게 수정하세요.

CALIBRATION_FILE = Path("/home/kukjiho/project/camera_calibration.npz")
HOMOGRAPHY_FILE = Path("/home/kukjiho/project/bev_homography.npz")

## 2. 설치

pip install -r requirements.txt

## 3. 실행

python3 app.py

## 4. 접속

http://localhost:5000

다른 PC에서 접속할 경우 서버 PC의 IP를 확인한 뒤 아래처럼 접속합니다.

http://서버IP:5000
