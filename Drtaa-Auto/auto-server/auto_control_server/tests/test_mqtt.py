import pytest
import paho.mqtt.client as mqtt
import time

# MQTT 브로커 설정
BROKER_ADDRESS = "localhost"
PORT = 1883
TOPIC = "test/topic"
MESSAGE = "Hello, MQTT!"

@pytest.fixture(scope="module")
def mqtt_client():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, protocol=mqtt.MQTTv5)
    client.connect(BROKER_ADDRESS, PORT)
    client.loop_start()
    yield client
    client.loop_stop()
    client.disconnect()

def test_mqtt_publish(mqtt_client):
    # 메시지 발행 테스트
    result = mqtt_client.publish(TOPIC, MESSAGE)
    assert result.rc == mqtt.MQTT_ERR_SUCCESS

def test_mqtt_subscribe(mqtt_client):
    # 메시지 구독 테스트
    received_messages = []

    def on_message(client, userdata, message):
        received_messages.append(message.payload.decode())

    mqtt_client.subscribe(TOPIC)
    mqtt_client.on_message = on_message

    # 메시지 발행
    mqtt_client.publish(TOPIC, MESSAGE)

    # 메시지 수신 대기
    time.sleep(2)

    assert len(received_messages) == 1
    assert received_messages[0] == MESSAGE

def test_mqtt_multiple_messages(mqtt_client):
    # 여러 메시지 발행 및 구독 테스트
    received_messages = []

    def on_message(client, userdata, message):
        received_messages.append(message.payload.decode())

    mqtt_client.subscribe(TOPIC)
    mqtt_client.on_message = on_message

    messages = ["Message 1", "Message 2", "Message 3"]
    for msg in messages:
        mqtt_client.publish(TOPIC, msg)

    # 메시지 수신 대기
    time.sleep(2)

    assert len(received_messages) == len(messages)
    assert received_messages == messages

def test_mqtt_qos(mqtt_client):
    # QoS 레벨 테스트
    result = mqtt_client.publish(TOPIC, MESSAGE, qos=1)
    assert result.rc == mqtt.MQTT_ERR_SUCCESS

    # QoS 1에서는 메시지 전달 확인을 기다려야 함
    result.wait_for_publish()
    assert result.is_published()
