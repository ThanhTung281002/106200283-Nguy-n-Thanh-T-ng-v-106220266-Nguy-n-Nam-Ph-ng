# drone/core/connection.py
import asyncio
from mavsdk import System

class DroneConnection:
    def __init__(self, uri: str = "udp://:14550", timeout: float = 10.0):
        self.uri = uri
        self.timeout = timeout
        self.drone = System()

    async def _wait_until_connected(self):
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("[connection] Connected to PX4")
                return

    async def connect(self) -> System:
        """Connect to PX4 SITL with timeout (Python 3.10 friendly)."""
        print(f"[connection] connecting to {self.uri} ...")
        await self.drone.connect(system_address=self.uri)

        # create async task to wait for connection
        wait_task = asyncio.create_task(self._wait_until_connected())

        try:
            await asyncio.wait_for(wait_task, timeout=self.timeout)
            return self.drone

        except asyncio.TimeoutError:
            wait_task.cancel()
            raise RuntimeError(
                f"Connection timeout after {self.timeout}s (URI={self.uri})"
            )
