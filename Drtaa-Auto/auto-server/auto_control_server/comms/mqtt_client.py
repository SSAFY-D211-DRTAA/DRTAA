import paho.mqtt.client as mqtt
from .message_handler import MessageHandler
from utils.logger import setup_logger
import logging
import json

# 스크립트 실행 관련 로거
main_logger = setup_logger(__name__)
# MQTT 메시지 처리 관련 로거
message_logger = setup_logger('message_logger', 'mqtt_messages.log', logging.INFO)

class MQTTClient:
    def __init__(self, broker='mqtt-broker', port=1883, topic="vehicle_control"):
        self.broker = broker
        self.port = port
        self.topic = topic

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, protocol=mqtt.MQTTv5)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_subscribe = self.on_subscribe
        self.client.on_publish = self.on_publish

        self.sub_topic = "sub/topic"
        self.pub_topic = "pub/topic"

        self.sub_gps_topic = "gps/data/v1/subscribe"
        self.pub_gps_topic = "gps/data/v1/publish"

        self.sub_path_topic = "path/data/v1/subscribe"
        self.pub_path_topic = "path/data/v1/publish"

        self.sub_cmd_topic = "cmd/data/v1/subscribe"
        self.pub_cmd_topic = "cmd/data/v1/publish"

        self.message_handler = MessageHandler()

    def on_connect(self, client, userdata, flags, rc, properties=None):
        client.subscribe(self.topic, qos=2)

        client.subscribe(self.sub_topic, qos=2)
        client.subscribe(self.sub_gps_topic, qos=2)
        client.subscribe(self.sub_path_topic, qos=2)
        client.subscribe(self.sub_cmd_topic, qos=2)

        main_logger.info(f"Connected with result code {rc}")

    def on_message(self, client, userdata, msg):
        message_logger.debug(f"Received message on topic {msg.topic}: {msg.payload}")

        response = self.message_handler.handle_message(msg.payload.decode(), "MQTT")
        
        if msg.topic == self.pub_gps_topic:
            self.client.publish(f"{self.pub_gps_topic}", json.dumps(response))
        elif msg.topic == self.pub_path_topic:
            self.client.publish(f"{self.pub_path_topic}", json.dumps(response))

    def on_subscribe(self, client, userdata, mid, granted_qos, properties=None):
        main_logger.info(f"Subscribed: {mid} {granted_qos}")

    def on_publish(self, client, userdata, mid, granted_qos, ties=None):
        message_logger.debug(f"Message Published: {mid}")

    def start(self):
        self.client.connect(self.broker, self.port, 60)
        self.client.loop_start()
        main_logger.info(f"MQTT broker {self.broker}:{self.port}")
        main_logger.info("MQTT client started")

    def stop(self):
        self.client.loop_stop()
        self.client.disconnect()
        main_logger.info("MQTT client stopped")

def run_mqtt_client():
    client = MQTTClient()
    client.start()

if __name__ == "__main__":
    run_mqtt_client()
