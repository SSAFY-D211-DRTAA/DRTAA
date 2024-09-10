FROM openjdk:17-jdk-alpine

# 작업 디렉토리 설정
WORKDIR = /app

COPY . .

RUN ls -al

RUN pwd
# Gradle 빌드
RUN rm -rf .gradle

RUN chmod +x ./gradlew && ./gradlew clean build

ARG JAR_FILE=./build/libs/Drtaa-0.0.1-SNAPSHOT.jar
COPY ${JAR_FILE} ./app.jar
ENV TZ=Asia/Seoul
ENTRYPOINT ["java", "-jar", "./app.jar"]