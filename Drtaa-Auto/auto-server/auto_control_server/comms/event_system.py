import asyncio
from utils.logger import setup_logger

logger = setup_logger(__name__)

class EventSystem:
    def __init__(self):
        self._subscribers = {}

    def subscribe(self, event, callback):
        if event not in self._subscribers:
            self._subscribers[event] = []
        self._subscribers[event].append(callback)

    async def publish(self, event, data):
        if event in self._subscribers:
            for callback in self._subscribers[event]:
                if asyncio.iscoroutinefunction(callback):
                    await callback(data)
                else:
                    callback(data)
        else:
            logger.warning(f"No subscribers for event: {event}")

event_system = EventSystem()
