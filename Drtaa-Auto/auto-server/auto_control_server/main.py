import asyncio
from comms.command_receiver_websocket_server import CommandReceiverWebSocketServer
from comms.command_distributor_websocket_server import CommandDistributorWebSocketServer
from comms.mqtt_client import MQTTClient
from utils.logger import setup_logger

logger = setup_logger(__name__)

async def main():
    mqtt_client = MQTTClient()
    command_receiver = CommandReceiverWebSocketServer(port=8765)
    command_distributor = CommandDistributorWebSocketServer(port=8766, mqtt_client=mqtt_client)

    # WebSocket 서버와 MQTT 클라이언트 시작
    await asyncio.gather(
        command_receiver.start(),
        command_distributor.start(),
        asyncio.to_thread(mqtt_client.start)
    )

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Server stopped")
