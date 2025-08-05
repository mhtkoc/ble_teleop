# BLE Teleop Package

Bu paket Bluetooth Low Energy (BLE) GATT sunucusu üzerinden gelen komutları `cmd_vel` topicine dönüştürür.

## Özellikler

- BLE GATT sunucusu olarak çalışır
- F, B, L, R karakterlerini robotun hareket komutlarına dönüştürür
- ROS2 `cmd_vel` topic'ine `geometry_msgs/Twist` mesajları yayınlar

## Komutlar

- `F`: İleri hareket (forward)
- `B`: Geri hareket (backward) 
- `L`: Sola dönüş (left turn)
- `R`: Sağa dönüş (right turn)
- `S` veya boş: Dur (stop)

## Kurulum

```bash
cd /home/pi/ros2_ws
colcon build --packages-select ble_teleop
source install/setup.bash
```

## Kullanım

### Direkt node çalıştırma:
```bash
ros2 run ble_teleop ble_cmd_vel_node
```

### Launch dosyası ile çalıştırma:
```bash
ros2 launch ble_teleop ble_teleop.launch.py
```

## BLE Bağlantısı

1. Node çalıştıktan sonra, cihazınızın BLE ayarlarından "autobot-teleop" isimli cihazı bulun
2. UART service UUID: `6e400001-b5a3-f393-e0a9-e50e24dcca9e`
3. RX Characteristic UUID: `6e400002-b5a3-f393-e0a9-e50e24dcca9e` (write)
4. TX Characteristic UUID: `6e400003-b5a3-f393-e0a9-e50e24dcca9e` (notify)

## Parametreler

- `linear_speed`: Doğrusal hız (varsayılan: 0.5 m/s)
- `angular_speed`: Açısal hız (varsayılan: 1.0 rad/s)

## Gereksinimler

- Python3
- BlueZ (Linux Bluetooth stack)
- ROS2
- D-Bus Python bindings
- PyGObject

## Notlar

- Bu node root yetkilerine ihtiyaç duyabilir (BLE işlemleri için)
- Bluetooth adaptörünün etkin olduğundan emin olun
- Node çalışırken terminal çıktısından BLE bağlantı durumunu takip edebilirsiniz
