import asyncio
from bleak import BleakScanner, BleakClient

TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

def callback(sender, data):
    print(data.decode())

async def main():
    print("Scanning...")

    devices = await BleakScanner.discover()

    device = next((d for d in devices if d.name == "GCM Micromouse"), None)

    if device is None:
        print("Not found")
        return

    async with BleakClient(device) as client:

        print("Connected:", client.is_connected)

        try:
            await client.start_notify(TX_UUID, callback)
            print("Notifications enabled")
        except Exception as e:
            print(e)
            return

        while True:
            await asyncio.sleep(1)

asyncio.run(main())