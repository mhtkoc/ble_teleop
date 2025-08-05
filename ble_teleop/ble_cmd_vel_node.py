#!/usr/bin/env python3

import sys
import dbus
import dbus.mainloop.glib
from gi.repository import GLib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import threading

from .example_advertisement import Advertisement
from .example_advertisement import register_ad_cb, register_ad_error_cb
from .example_gatt_server import Service, Characteristic
from .example_gatt_server import register_app_cb, register_app_error_cb

BLUEZ_SERVICE_NAME = 'org.bluez'
DBUS_OM_IFACE = 'org.freedesktop.DBus.ObjectManager'
LE_ADVERTISING_MANAGER_IFACE = 'org.bluez.LEAdvertisingManager1'
GATT_MANAGER_IFACE = 'org.bluez.GattManager1'
GATT_CHRC_IFACE = 'org.bluez.GattCharacteristic1'
UART_SERVICE_UUID = '6e400001-b5a3-f393-e0a9-e50e24dcca9e'
UART_RX_CHARACTERISTIC_UUID = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'
UART_TX_CHARACTERISTIC_UUID = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'
LOCAL_NAME = 'autobot-teleop'

mainloop = None
ros_node = None


class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('ble_cmd_vel_publisher')
        self.publisher_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.get_logger().info('BLE CMD_VEL Publisher node started')
        
        # Robot movement parameters
        self.linear_speed = 0.3  # m/s 
        self.angular_speed = 1.0  # rad/s
        
    def publish_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear_x
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = angular_z
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: linear.x={linear_x}, angular.z={angular_z}')
        
    def process_command(self, command):
        """Process received BLE command and publish corresponding cmd_vel"""
        command = command.strip().upper()
        
        if command == 'F':  # Forward
            self.publish_cmd_vel(linear_x=self.linear_speed)
        elif command == 'B':  # Backward
            self.publish_cmd_vel(linear_x=-self.linear_speed)
        elif command == 'L':  # Left turn
            self.publish_cmd_vel(angular_z=self.angular_speed)
        elif command == 'R':  # Right turn
            self.publish_cmd_vel(angular_z=-self.angular_speed)
        elif command == 'S' or command == '':  # Stop
            self.publish_cmd_vel()
        else:
            self.get_logger().warn(f'Unknown command: {command}')


class TxCharacteristic(Characteristic):
    def __init__(self, bus, index, service):
        Characteristic.__init__(self, bus, index, UART_TX_CHARACTERISTIC_UUID,
                                ['notify'], service)
        self.notifying = False

    def send_tx(self, s):
        if not self.notifying:
            return
        value = []
        for c in s:
            value.append(dbus.Byte(ord(c)))
        self.PropertiesChanged(GATT_CHRC_IFACE, {'Value': value}, [])

    def StartNotify(self):
        if self.notifying:
            return
        self.notifying = True

    def StopNotify(self):
        if not self.notifying:
            return
        self.notifying = False


class RxCharacteristic(Characteristic):
    def __init__(self, bus, index, service):
        Characteristic.__init__(self, bus, index, UART_RX_CHARACTERISTIC_UUID,
                                ['write'], service)

    def WriteValue(self, value, options):
        global ros_node
        received_data = bytearray(value).decode().strip()
        print(f'BLE received: {received_data}')
        
        if ros_node:
            ros_node.process_command(received_data)


class UartService(Service):
    def __init__(self, bus, index):
        Service.__init__(self, bus, index, UART_SERVICE_UUID, True)
        self.add_characteristic(TxCharacteristic(bus, 0, self))
        self.add_characteristic(RxCharacteristic(bus, 1, self))


class Application(dbus.service.Object):
    def __init__(self, bus):
        self.path = '/'
        self.services = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_service(self, service):
        self.services.append(service)

    @dbus.service.method(DBUS_OM_IFACE, out_signature='a{oa{sa{sv}}}')
    def GetManagedObjects(self):
        response = {}
        for service in self.services:
            response[service.get_path()] = service.get_properties()
            chrcs = service.get_characteristics()
            for chrc in chrcs:
                response[chrc.get_path()] = chrc.get_properties()
        return response


class UartApplication(Application):
    def __init__(self, bus):
        Application.__init__(self, bus)
        self.add_service(UartService(bus, 0))


class UartAdvertisement(Advertisement):
    def __init__(self, bus, index):
        Advertisement.__init__(self, bus, index, 'peripheral')
        self.add_service_uuid(UART_SERVICE_UUID)
        self.add_local_name(LOCAL_NAME)
        self.include_tx_power = True


def find_adapter(bus):
    remote_om = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, '/'),
                               DBUS_OM_IFACE)
    objects = remote_om.GetManagedObjects()
    for o, props in objects.items():
        if LE_ADVERTISING_MANAGER_IFACE in props and GATT_MANAGER_IFACE in props:
            return o
    return None


def ros_thread():
    """Run ROS2 in a separate thread"""
    global ros_node
    rclpy.init()
    ros_node = CmdVelPublisher()
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


def main():
    global mainloop
    
    # Start ROS2 in a separate thread
    ros_thread_handle = threading.Thread(target=ros_thread, daemon=True)
    ros_thread_handle.start()
    
    # Setup D-Bus main loop
    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()
    
    adapter = find_adapter(bus)
    if not adapter:
        print('BLE adapter not found')
        return

    service_manager = dbus.Interface(
        bus.get_object(BLUEZ_SERVICE_NAME, adapter),
        GATT_MANAGER_IFACE)
    ad_manager = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter),
                                LE_ADVERTISING_MANAGER_IFACE)

    app = UartApplication(bus)
    adv = UartAdvertisement(bus, 0)

    mainloop = GLib.MainLoop()

    service_manager.RegisterApplication(app.get_path(), {},
                                        reply_handler=register_app_cb,
                                        error_handler=register_app_error_cb)
    ad_manager.RegisterAdvertisement(adv.get_path(), {},
                                     reply_handler=register_ad_cb,
                                     error_handler=register_ad_error_cb)
    
    print("BLE GATT server started. Waiting for connections...")
    print("Commands: F=Forward, B=Backward, L=Left, R=Right, S=Stop")
    
    try:
        mainloop.run()
    except KeyboardInterrupt:
        print("\nShutting down...")
        adv.Release()
        if ros_node:
            ros_node.destroy_node()


if __name__ == '__main__':
    main()
