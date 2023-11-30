import asyncio
from queue import Queue
from telnetlib import Telnet

import websockets

# Cola para los mensajes entrantes del WebSocket
message_queue = Queue()

async def telnet_task(websocket):
    with Telnet('192.168.146.132', 23) as tn:
        while True:
            if not message_queue.empty():
                # Hay un mensaje para enviar a Telnet
                message = message_queue.get()
                tn.write(message.encode('ascii') + b"\n")
                await asyncio.sleep(0.1)

            # Leer de Telnet y enviar al WebSocket
            data = tn.read_very_eager().decode('utf-8')
            if data:
                await websocket.send(data)
            await asyncio.sleep(0.1)

async def echo(websocket, path):
    telnet_read_write_task = asyncio.create_task(telnet_task(websocket))
    try:
        async for message in websocket:
            if message != "request_data":
                print("Received message from client:", message)
                message_queue.put(message)
    finally:
        telnet_read_write_task.cancel()

# Iniciar el servidor WebSocket
start_server = websockets.serve(echo, "localhost", 5000)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
