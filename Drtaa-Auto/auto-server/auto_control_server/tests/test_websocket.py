import pytest
import asyncio
import websockets
import json
from communication.websocket_server import WebSocketServer

@pytest.fixture
async def websocket_server():
    server = WebSocketServer('localhost', 8765)
    task = asyncio.create_task(server.start())
    await asyncio.sleep(0.1)  # 서버가 시작될 때까지 잠시 대기
    yield server
    task.cancel()
    await asyncio.sleep(0.1)  # 서버가 종료될 때까지 잠시 대기

@pytest.mark.asyncio
async def test_websocket_connection(websocket_server):
    uri = "ws://localhost:8765"
    async with websockets.connect(uri) as websocket:
        assert websocket.open

@pytest.mark.asyncio
async def test_vehicle_info_request(websocket_server):
    uri = "ws://localhost:8765"
    async with websockets.connect(uri) as websocket:
        await websocket.send(json.dumps({"action": "vehicle_info", "vehicle_id": "123"}))
        response = await websocket.recv()
        response_data = json.loads(response)
        assert "status" in response_data
        assert response_data["status"] == "success"
        assert "info" in response_data

@pytest.mark.asyncio
async def test_vehicle_dispatch_request(websocket_server):
    uri = "ws://localhost:8765"
    async with websockets.connect(uri) as websocket:
        await websocket.send(json.dumps({"action": "vehicle_dispatch", "location": "A"}))
        response = await websocket.recv()
        response_data = json.loads(response)
        assert "status" in response_data
        assert response_data["status"] == "success"
        assert "message" in response_data
        assert "dispatched" in response_data["message"]

@pytest.mark.asyncio
async def test_invalid_action(websocket_server):
    uri = "ws://localhost:8765"
    async with websockets.connect(uri) as websocket:
        await websocket.send(json.dumps({"action": "invalid_action"}))
        response = await websocket.recv()
        response_data = json.loads(response)
        assert "error" in response_data
        assert response_data["error"] == "Unknown action"

@pytest.mark.asyncio
async def test_invalid_json(websocket_server):
    uri = "ws://localhost:8765"
    async with websockets.connect(uri) as websocket:
        await websocket.send("Invalid JSON")
        response = await websocket.recv()
        response_data = json.loads(response)
        assert "error" in response_data
        assert response_data["error"] == "Invalid JSON"
