FROM python:3.12-slim

# 작업 디렉토리 설정
WORKDIR /app

# api_server 디렉토리의 내용만 복사
COPY api_server /app

# 필요한 패키지 설치
RUN pip install --no-cache-dir -r requirements.txt

# 애플리케이션 실행
CMD ["python", "app.py"]
