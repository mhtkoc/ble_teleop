#!/usr/bin/env python3
"""
BLE Test Client - Klavye girdilerini BLE üzerinden gönderir
Kullanım: python3 test_ble_client.py
WASD tuşları ile robot kontrolü
"""

import asyncio
import sys
from bleak import BleakClient, BleakScanner

UART_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
UART_RX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # Write to this
UART_TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # Read from this


async def find_device():
    """BLE cihazını bul"""
    devices = await BleakScanner.discover()
    for device in devices:
        if device.name and "autobot" in device.name.lower():
            return device.address
    return None


async def send_command(client, command):
    """Komut gönder"""
    try:
        await client.write_gatt_char(UART_RX_CHAR_UUID, command.encode())
        print(f"Gönderildi: {command}")
    except Exception as e:
        print(f"Hata: {e}")


async def main():
    print("BLE cihazı aranıyor...")
    device_address = await find_device()
    
    if not device_address:
        print("Autobot cihazı bulunamadı!")
        return
    
    print(f"Bağlanıyor: {device_address}")
    
    async with BleakClient(device_address) as client:
        print("Bağlandı!")
        print("\nKomutlar:")
        print("W = İleri (F)")
        print("S = Geri (B)")
        print("A = Sol (L)")
        print("D = Sağ (R)")
        print("Boşluk = Dur")
        print("Q = Çıkış")
        
        while True:
            try:
                command = input("\nKomut girin: ").strip().upper()
                
                if command == 'Q':
                    break
                elif command == 'W':
                    await send_command(client, 'F')
                elif command == 'S':
                    await send_command(client, 'B')
                elif command == 'A':
                    await send_command(client, 'L')
                elif command == 'D':
                    await send_command(client, 'R')
                elif command == ' ' or command == '':
                    await send_command(client, 'S')
                else:
                    print("Geçersiz komut!")
                    
            except KeyboardInterrupt:
                break
    
    print("Bağlantı kapatıldı.")


if __name__ == "__main__":
    # Bleak kütüphanesi için gerekli
    # pip3 install bleak
    try:
        import bleak
    except ImportError:
        print("Bleak kütüphanesi gerekli: pip3 install bleak")
        sys.exit(1)
    
    asyncio.run(main())
