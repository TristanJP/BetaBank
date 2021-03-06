import asyncio
import websockets
import json
import base64
import numpy as np

class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)

class Websocket:
  def __init__(self, state):
    self.state = state
    self.clients = set()

  def state_event(self):
    return json.dumps(self.state, cls=NumpyEncoder)

  async def notify_state(self):
    if self.clients:
      message = self.state_event()
      await asyncio.wait([client.send(message) for client in self.clients])

  async def register(self, websocket):
    self.clients.add(websocket)

  async def unregister(self, websocket):
    self.clients.remove(websocket)

  async def broadcast_state(self, sleep_time):
    while True:
      for ws in self.clients:
        await ws.send("state     " + self.state_event())
      await asyncio.sleep(sleep_time)

  async def push_state(self, websocket, path):
    await self.register(websocket)
    # self.state = current_state
    try:
      async for msg in websocket:
        pass
    finally:
      await self.unregister(websocket)