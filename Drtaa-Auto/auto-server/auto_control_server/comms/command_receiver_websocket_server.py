import asyncio
import json
import websockets

from utils.logger import setup_logger
from .event_system import event_system
from .message_handler import MessageHandler

logger = setup_logger(__name__)

class CommandReceiverWebSocketServer:
    def __init__(self, host='0.0.0.0', port=8765):
        self.host = host
        self.port = port
        self.message_handler = MessageHandler()

    async def handle_client(self, websocket, path):
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    # logger.debug(f"recv msg: {data}")
                    await event_system.publish('recv_command', message)
                    response = self.message_handler.handle_message(data, "WebSocket")
                    await websocket.send(json.dumps(response))
                except json.JSONDecodeError:
                    await websocket.send(json.dumps({"error": "Invalid JSON"}))
        except websockets.exceptions.ConnectionClosed:
            logger.info("Client disconnected")

    async def start(self):
        server = await websockets.serve(self.handle_client, self.host, self.port)
        logger.info(f"WebSocket server started on ws://{self.host}:{self.port}")
        await server.wait_closed()

def run_websocket_server():
    server = CommandReceiverWebSocketServer()
    asyncio.run(server.start())

if __name__ == "__main__":
    run_websocket_server()
