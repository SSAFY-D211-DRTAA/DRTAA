import asyncio
import json
import websockets

from utils.logger import setup_logger
from .mqtt_client import MQTTClient
from .event_system import event_system
from .message_handler import MessageHandler

logger = setup_logger(__name__)

class CommandDistributorWebSocketServer:
    def __init__(self, host='0.0.0.0', port: int=8766, mqtt_client: MQTTClient=None):
        self.host = host
        self.port = port
        self.mqtt_client: MQTTClient = mqtt_client

        # self.clients = set()
        self.client = None

        event_system.subscribe('recv_command', self.distribute_command)

    async def handle_client(self, websocket, path):
        self.client = websocket
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    tag = data.get("tag")
                    response = None

                    if tag == "gps":
                        logger.info(f"saved gps data")
                        response = "gps"
                        try:
                            with open('gps_data.json', 'w') as f:
                                json.dump(data, f)
                        except IOError as e:
                            logger.error(f"GPS file save error: {e}")
                    elif tag == "global_path":
                        logger.info(f"saved global path data")
                        response = "global path"
                        try:
                            with open('global_path.json', 'w') as f:
                                json.dump(data, f)
                        except IOError as e:
                            logger.error(f"Global Path file save error: {e}")
                        
                        self.mqtt_client.publish_global_path(data)
                    elif tag == "complete_drive":
                        logger.info(f"ack complete drive")
                        response = "complete drive"
                    elif tag == "connect":
                        logger.info(f"Local client connected")
                        response = data
                    else:
                        logger.warning(f"unknown tag: {tag}")
                    
                    await websocket.send(json.dumps({f"success": response}))
                except json.JSONDecodeError:
                    await websocket.send(json.dumps({"error": "Invalid JSON"}))
        except websockets.exceptions.ConnectionClosed:
            logger.info("Client disconnected from additional server")

    # async def handle_client(self, websocket, path):
    #     self.clients.add(websocket)
    #     try:
    #         await websocket.wait_closed()
    #     finally:
    #         self.clients.remove(websocket)

    async def distribute_command(self, command):

        # for client in self.clients:
        #     try:
        #         await client.send(message)
        #     except websockets.exceptions.ConnectionClosed:
        #         logger.info("Failed to send command to a client")
        
        if self.client:
            try:
                logger.info(f"Send command to local client: {command}")
                await self.client.send(command)
            except websockets.exceptions.ConnectionClosed:
                logger.info("Failed to send command to a client")

    async def start(self):
        server = await websockets.serve(self.handle_client, self.host, self.port)
        logger.info(f"Additional WebSocket server started on ws://{self.host}:{self.port}")
        await server.wait_closed()

def run_additional_websocket_server():
    server = CommandDistributorWebSocketServer()
    asyncio.run(server.start())

if __name__ == "__main__":
    run_additional_websocket_server()
