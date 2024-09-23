import asyncio
import json
import websockets

from utils.logger import setup_logger
from .event_system import event_system
from .message_handler import MessageHandler

logger = setup_logger(__name__)

class CommandDistributorWebSocketServer:
    def __init__(self, host='0.0.0.0', port=8766):
        self.host = host
        self.port = port
        # self.clients = set()
        self.client = None
        event_system.subscribe('new_command', self.distribute_command)

    async def handle_client(self, websocket, path):
        self.client = websocket
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    logger.info(f"Received message on additional server: {data}")
                    try:
                        with open('gps_data.json', 'w') as f:
                            json.dump(data, f)
                    except IOError as e:
                        logger.error(f"GPS file save error: {e}")
                    await websocket.send(json.dumps(data))
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
        logger.info(f"Distributing command: {command}")
        message = json.dumps(command)
        # for client in self.clients:
        #     try:
        #         await client.send(message)
        #     except websockets.exceptions.ConnectionClosed:
        #         logger.info("Failed to send command to a client")
        if self.client:
            try:
                await self.client.send(message)
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
