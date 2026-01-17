"""
ZMK Battery Monitor - Complete GUI Control Panel
Features: Connected Devices Detection, Power Control, Temperature Monitoring, Settings, Bond Management, Battery %, RSSI
Dependencies: pip install bleak tkinter
"""

import asyncio
import struct
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
from bleak import BleakClient, BleakScanner
from datetime import datetime
import threading
import platform

# ============================================================================
# UUIDs
# ============================================================================

SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
POWER_CONTROL_UUID = "12345678-1234-5678-1234-56789abcdef1"
TEMP_INTERNAL_UUID = "12345678-1234-5678-1234-56789abcdef2"
VOLTAGE_UUID = "12345678-1234-5678-1234-56789abcdef3"
TEMP_EXTERNAL_UUID = "12345678-1234-5678-1234-56789abcdef5"
AUTO_SETTINGS_UUID = "12345678-1234-5678-1234-56789abcdef6"
TEMP_SETTINGS_UUID = "12345678-1234-5678-1234-56789abcdef7"
BOND_MANAGEMENT_UUID = "12345678-1234-5678-1234-56789abcdef8"

# Standard Battery Service
BATTERY_SERVICE_UUID = "0000180f-0000-1000-8000-00805f9b34fb"
BATTERY_LEVEL_UUID = "00002a19-0000-1000-8000-00805f9b34fb"

# Commands
CMD_POWER_OFF = 0x00
CMD_POWER_ON = 0x01
CMD_POWER_TOGGLE = 0x02
CMD_RESET_DEVICE = 0x52

# ============================================================================
# BLE Manager with Connected Devices Detection
# ============================================================================

class BatteryMonitorBLE:
    def __init__(self, gui):
        self.gui = gui
        self.client = None
        self.connected = False
        self.device_address = None
        self.last_rssi = None  # Store RSSI from scan to use during connection
        
    async def get_connected_devices(self):
        """
        Lấy danh sách thiết bị đang kết nối với hệ thống
        Returns: List of (name, address, status) tuples
        """
        connected_devices = []
        system = platform.system()
        
        self.gui.log(f"🔍 Checking connected devices on {system}...")
        
        try:
            if system == "Windows":
                connected_devices = await self._get_windows_connected()
            elif system == "Darwin":  # macOS
                connected_devices = await self._get_macos_connected()
            elif system == "Linux":
                connected_devices = await self._get_linux_connected()
            else:
                self.gui.log(f"⚠️ Platform {system} not fully supported")
        except Exception as e:
            self.gui.log(f"❌ Error getting connected devices: {e}")
        
        return connected_devices
    
    async def _get_windows_connected(self):
        """Lấy connected devices trên Windows"""
        connected = []
        
        # Method 1: PowerShell - Get connected Bluetooth devices
        try:
            import subprocess
            import re
            
            # Query Windows for connected Bluetooth devices
            result = subprocess.run(
                ['powershell', '-Command', 
                 'Get-PnpDevice -Class Bluetooth | Where-Object {$_.Status -eq "OK"} | Select-Object FriendlyName, InstanceId | Format-List'],
                capture_output=True,
                text=True,
                timeout=8
            )
            
            if result.returncode == 0:
                # Parse output
                current_device = {}
                for line in result.stdout.split('\n'):
                    line = line.strip()
                    
                    if line.startswith('FriendlyName'):
                        name = line.split(':', 1)[1].strip()
                        current_device['name'] = name
                    
                    elif line.startswith('InstanceId'):
                        instance_id = line.split(':', 1)[1].strip()
                        current_device['instance_id'] = instance_id
                        
                        # Try to extract MAC address from InstanceId
                        # Format: BTHLE\DEV_XXXXXXXXXXXX or BTH\{GUID}\XXXXXXXXXXXX
                        mac_match = re.search(r'[_\\]([0-9A-F]{12})', instance_id, re.IGNORECASE)
                        if mac_match:
                            mac_hex = mac_match.group(1)
                            # Convert to MAC format XX:XX:XX:XX:XX:XX
                            mac = ':'.join([mac_hex[i:i+2] for i in range(0, 12, 2)])
                            current_device['address'] = mac.upper()
                            
                            # Add to connected list
                            if 'name' in current_device and 'address' in current_device:
                                # Skip generic services
                                if not any(skip in current_device['name'] for skip in [
                                    'Generic Access Profile',
                                    'Generic Attribute Profile', 
                                    'Bluetooth LE Generic',
                                    'Device Information Service',
                                    'Device Identification Service',
                                    'Microsoft Bluetooth',
                                    'Service Discovery',
                                    'Avrcp Transport',
                                    'RFCOMM Protocol',
                                    'Intel(R) Wireless',
                                    'Nefarius'
                                ]):
                                    # Check if not duplicate
                                    if not any(d['address'] == current_device['address'] for d in connected):
                                        connected.append({
                                            'name': current_device['name'],
                                            'address': current_device['address'],
                                            'status': '🟢 Connected',
                                            'rssi': None
                                        })
                                        self.gui.log(f"  ✓ {current_device['name']} ({current_device['address']})")
                        
                        current_device = {}
                    
        except Exception as e:
            self.gui.log(f"⚠️ PowerShell query failed: {e}")
        
        # Method 2: BLE Scan with high RSSI (likely connected)
        try:
            self.gui.log("📡 Scanning BLE for strong signals...")
            devices = await BleakScanner.discover(timeout=3.0, return_adv=True)
            
            for address, (device, adv_data) in devices.items():
                # RSSI > -60 usually means connected or very close
                if hasattr(device, 'rssi') and device.rssi and device.rssi > -60:
                    if device.name and not any(d['address'] == address for d in connected):
                        # Skip generic services
                        if not any(skip in device.name for skip in [
                            'Generic Access',
                            'Generic Attribute',
                            'Bluetooth LE Generic',
                            'Device Information',
                            'Device Identification'
                        ]):
                            connected.append({
                                'name': device.name,
                                'address': address,
                                'rssi': device.rssi,
                                'status': f'🟢 Active (RSSI: {device.rssi})'
                            })
                            self.gui.log(f"  ✓ {device.name} ({address}) RSSI: {device.rssi}")
        
        except Exception as e:
            self.gui.log(f"⚠️ BLE scan error: {e}")
        
        return connected
    
    async def _get_macos_connected(self):
        """Lấy connected devices trên macOS"""
        connected = []
        
        try:
            import subprocess
            
            # Sử dụng system_profiler
            result = subprocess.run(
                ['system_profiler', 'SPBluetoothDataType', '-json'],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            if result.returncode == 0:
                import json
                data = json.loads(result.stdout)
                
                # Parse devices (structure varies by macOS version)
                if 'SPBluetoothDataType' in data:
                    bt_data = data['SPBluetoothDataType'][0]
                    
                    # Connected devices
                    if 'device_connected' in bt_data:
                        for device in bt_data['device_connected']:
                            connected.append({
                                'name': device.get('device_name', 'Unknown'),
                                'address': device.get('device_address', 'N/A'),
                                'status': '🟢 Connected'
                            })
        
        except Exception as e:
            self.gui.log(f"⚠️ macOS query error: {e}")
        
        # Alternative: Scan with high RSSI filter
        try:
            devices = await BleakScanner.discover(timeout=3.0, return_adv=True)
            
            for address, (device, adv_data) in devices.items():
                if hasattr(device, 'rssi') and device.rssi and device.rssi > -60:
                    # Check if not already in list
                    if not any(d['address'] == address for d in connected):
                        connected.append({
                            'name': device.name or 'Unknown',
                            'address': address,
                            'rssi': device.rssi,
                            'status': '🟢 Active'
                        })
        except Exception as e:
            self.gui.log(f"⚠️ macOS scan error: {e}")
        
        return connected
    
    async def _get_linux_connected(self):
        """Lấy connected devices trên Linux"""
        connected = []
        
        try:
            import subprocess
            
            # Method 1: bluetoothctl info (shows connected devices)
            result = subprocess.run(
                ['bluetoothctl', 'devices', 'Connected'],
                capture_output=True,
                text=True,
                timeout=3
            )
            
            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if 'Device' in line:
                        parts = line.split()
                        if len(parts) >= 3:
                            address = parts[1]
                            name = ' '.join(parts[2:])
                            connected.append({
                                'name': name,
                                'address': address,
                                'status': '🟢 Connected'
                            })
            
            # Method 2: Check /sys/class/bluetooth
            import os
            bt_path = '/sys/class/bluetooth'
            if os.path.exists(bt_path):
                for controller in os.listdir(bt_path):
                    controller_path = os.path.join(bt_path, controller)
                    if os.path.isdir(controller_path):
                        # List connected devices
                        for device in os.listdir(controller_path):
                            if device.startswith('hci'):
                                continue
                            
                            connected_file = os.path.join(controller_path, device, 'connected')
                            if os.path.exists(connected_file):
                                with open(connected_file, 'r') as f:
                                    if f.read().strip() == '1':
                                        # Device is connected
                                        name_file = os.path.join(controller_path, device, 'name')
                                        name = 'Unknown'
                                        if os.path.exists(name_file):
                                            with open(name_file, 'r') as f:
                                                name = f.read().strip()
                                        
                                        # Convert device ID to MAC
                                        mac = device.replace('_', ':')
                                        
                                        if not any(d['address'] == mac for d in connected):
                                            connected.append({
                                                'name': name,
                                                'address': mac,
                                                'status': '🟢 Connected (sys)'
                                            })
        
        except Exception as e:
            self.gui.log(f"⚠️ Linux query error: {e}")
        
        return connected
    
    async def scan_devices(self, scan_mode='all'):
        """
        Scan for available devices
        scan_mode: 'all', 'connected', 'advertising'
        """
        devices_dict = {}
        
        # 1. Get connected devices first
        if scan_mode in ['all', 'connected']:
            self.gui.log("🔗 Checking connected devices...")
            connected = await self.get_connected_devices()
            
            for device in connected:
                addr = device['address']
                devices_dict[addr] = {
                    'name': device['name'],
                    'address': addr,
                    'source': device['status'],
                    'rssi': device.get('rssi')
                }
        
        # 2. BLE Advertisement Scan
        if scan_mode in ['all', 'advertising']:
            self.gui.log("📡 Scanning BLE advertisements...")
            discovered = await BleakScanner.discover(timeout=5.0, return_adv=True)
            
            for address, (device, adv_data) in discovered.items():
                if device.name:
                    rssi = device.rssi if hasattr(device, 'rssi') else None
                    
                    if address in devices_dict:
                        # Device found in both - update status and RSSI
                        devices_dict[address]['source'] = '✅ Connected + Advertising'
                        if rssi is not None:
                            devices_dict[address]['rssi'] = rssi
                    else:
                        # New device from advertising
                        devices_dict[address] = {
                            'name': device.name,
                            'address': address,
                            'source': '📡 Advertising',
                            'rssi': rssi
                        }
        
        # Convert to list and sort
        source_priority = {
            '✅ Connected + Advertising': 0,
            '🟢 Connected': 1,
            '🟢 Active': 1,
            '📡 Advertising': 2
        }
        
        devices_list = sorted(
            devices_dict.values(),
            key=lambda x: (source_priority.get(x['source'], 3), -(x['rssi'] or -100))
        )
        
        # Return devices with RSSI info stored
        return [(f"{d['name']} {d['source']}", d['address'], d['rssi']) for d in devices_list]
    
    async def connect(self, address, rssi_from_scan=None):
        """Connect to device"""
        try:
            # Store RSSI from scan
            if rssi_from_scan is not None:
                self.last_rssi = rssi_from_scan
                self.gui.update_rssi(rssi_from_scan)
                self.gui.log(f"📡 Initial RSSI from scan: {rssi_from_scan} dBm")
            
            self.client = BleakClient(address)
            await self.client.connect()
            self.connected = True
            self.device_address = address
            
            # Subscribe to notifications
            await self.client.start_notify(POWER_CONTROL_UUID, self.power_notification)
            await self.client.start_notify(TEMP_INTERNAL_UUID, self.temp_internal_notification)
            await self.client.start_notify(TEMP_EXTERNAL_UUID, self.temp_external_notification)
            await self.client.start_notify(VOLTAGE_UUID, self.voltage_notification)
            await self.client.start_notify(AUTO_SETTINGS_UUID, self.auto_settings_notification)
            await self.client.start_notify(TEMP_SETTINGS_UUID, self.temp_settings_notification)
            
            try:
                await self.client.start_notify(BATTERY_LEVEL_UUID, self.battery_level_notification)
            except Exception as e:
                self.gui.log(f"⚠️  Battery service not available: {e}")
            
            # Read initial values
            await self.read_all_data()
            
            self.gui.log(f"✅ Connected to {address}")
            return True
        except Exception as e:
            self.gui.log(f"❌ Connection failed: {e}")
            return False
    
    async def disconnect(self):
        """Disconnect from device"""
        if self.client and self.connected:
            await self.client.disconnect()
            self.connected = False
            self.gui.log("🔌 Disconnected")
    
    async def read_all_data(self):
        """Read all current values"""
        try:
            # Read power state
            data = await self.client.read_gatt_char(POWER_CONTROL_UUID)
            self.power_notification(None, data)
            
            # Read temperatures
            data = await self.client.read_gatt_char(TEMP_INTERNAL_UUID)
            self.temp_internal_notification(None, data)
            
            data = await self.client.read_gatt_char(TEMP_EXTERNAL_UUID)
            self.temp_external_notification(None, data)
            
            # Read voltage
            data = await self.client.read_gatt_char(VOLTAGE_UUID)
            self.voltage_notification(None, data)
            
            # Read battery level
            try:
                data = await self.client.read_gatt_char(BATTERY_LEVEL_UUID)
                self.battery_level_notification(None, data)
            except Exception as e:
                self.gui.log(f"⚠️  Battery level not available: {e}")
            
            # Read RSSI - Call after reading characteristics (similar to Android's onCharacteristicRead)
            await self.read_remote_rssi()
            
            # Read settings
            data = await self.client.read_gatt_char(AUTO_SETTINGS_UUID)
            self.auto_settings_notification(None, data)
            
            data = await self.client.read_gatt_char(TEMP_SETTINGS_UUID)
            self.temp_settings_notification(None, data)
            
            # Read bonds
            data = await self.client.read_gatt_char(BOND_MANAGEMENT_UUID)
            self.parse_bonds(data)
            
            self.gui.log("✅ All data refreshed")
            
        except Exception as e:
            self.gui.log(f"⚠️  Error reading data: {e}")
    
    async def read_remote_rssi(self):
        """
        Read remote RSSI - Equivalent to Android's gatt.readRemoteRssi()
        In Bleak, this accesses the platform-specific backend
        If no new RSSI available, keep the last known value
        """
        try:
            # The equivalent of Android's gatt.readRemoteRssi() in Bleak
            # is to access the device's RSSI through the backend
            
            # Method 1: Try Bleak's internal backend device property
            if hasattr(self.client, '_backend') and hasattr(self.client._backend, '_device'):
                device = self.client._backend._device
                
                # Windows: device has SignalStrength property
                if hasattr(device, 'SignalStrength'):
                    rssi = device.SignalStrength
                    self.last_rssi = rssi  # Update stored value
                    self.gui.update_rssi(rssi)
                    self.gui.log(f"📡 RSSI: {rssi} dBm")
                    return
                
                # macOS: peripheral has RSSI property
                elif hasattr(device, 'RSSI'):
                    rssi = int(device.RSSI())
                    self.last_rssi = rssi  # Update stored value
                    self.gui.update_rssi(rssi)
                    self.gui.log(f"📡 RSSI: {rssi} dBm")
                    return
            
            # Method 2: Linux BlueZ - Use D-Bus to read RSSI property
            if hasattr(self.client, '_backend') and hasattr(self.client._backend, '_device_path'):
                try:
                    import dbus
                    bus = dbus.SystemBus()
                    device_obj = bus.get_object('org.bluez', self.client._backend._device_path)
                    device_props = dbus.Interface(device_obj, 'org.freedesktop.DBus.Properties')
                    rssi = int(device_props.Get('org.bluez.Device1', 'RSSI'))
                    
                    self.last_rssi = rssi  # Update stored value
                    self.gui.update_rssi(rssi)
                    self.gui.log(f"📡 RSSI: {rssi} dBm")
                    return
                except Exception as e:
                    self.gui.log(f"⚠️  D-Bus RSSI read failed: {e}")
            
            # Method 3: Fallback - some Bleak versions have get_rssi()
            if hasattr(self.client, 'get_rssi'):
                rssi = await self.client.get_rssi()
                self.last_rssi = rssi  # Update stored value
                self.gui.update_rssi(rssi)
                self.gui.log(f"📡 RSSI: {rssi} dBm")
                return
            
            # If no new RSSI available, keep the last known value
            if self.last_rssi is not None:
                self.gui.log(f"📡 RSSI: {self.last_rssi} dBm (from scan, platform doesn't support live read)")
            else:
                self.gui.log(f"⚠️  RSSI not available")
            
        except Exception as e:
            # Keep last known RSSI on error
            if self.last_rssi is not None:
                self.gui.log(f"⚠️  RSSI read error: {e}, keeping last value: {self.last_rssi} dBm")
            else:
                self.gui.log(f"⚠️  RSSI read error: {e}")
    
    # Notification handlers
    def power_notification(self, sender, data):
        state = bool(data[0])
        self.gui.update_power_state(state)
        self.gui.log(f"🔋 Power: {'ON' if state else 'OFF'}")
    
    def temp_internal_notification(self, sender, data):
        temp = struct.unpack('<h', data)[0] / 100.0
        self.gui.update_temp_internal(temp)
        self.gui.log(f"🌡️  Internal: {temp:.2f}°C")
    
    def temp_external_notification(self, sender, data):
        temp = struct.unpack('<h', data)[0] / 100.0
        self.gui.update_temp_external(temp)
        self.gui.log(f"🌡️  External: {temp:.2f}°C")
    
    def voltage_notification(self, sender, data):
        voltage = struct.unpack('<H', data)[0]
        self.gui.update_voltage(voltage)
        self.gui.log(f"⚡ Voltage: {voltage}mV ({voltage/1000:.3f}V)")
    
    def battery_level_notification(self, sender, data):
        level = data[0]  # Battery percentage 0-100
        self.gui.update_battery_level(level)
        self.gui.log(f"🔋 Battery: {level}%")
    
    def auto_settings_notification(self, sender, data):
        settings = {
            'auto_on_enabled': bool(data[0]),
            'auto_off_enabled': bool(data[1]),
            'auto_on_percent': data[2],
            'auto_off_percent': data[3],
            'storage_percent': data[4],
            'reverse_off_enabled': bool(data[5]),
            'reverse_off_percent': data[6],
            'reverse_on_enabled': bool(data[7]),
            'reverse_on_percent': data[8],
        }
        self.gui.update_auto_settings(settings)
    
    def temp_settings_notification(self, sender, data):
        settings = {
            'int_high_enabled': bool(data[0]),
            'int_high_threshold': struct.unpack('>h', data[1:3])[0] / 100.0,
            'int_low_enabled': bool(data[3]),
            'int_low_threshold': struct.unpack('>h', data[4:6])[0] / 100.0,
            'ext_high_enabled': bool(data[6]),
            'ext_high_threshold': struct.unpack('>h', data[7:9])[0] / 100.0,
            'ext_low_enabled': bool(data[9]),
            'ext_low_threshold': struct.unpack('>h', data[10:12])[0] / 100.0,
        }
        self.gui.update_temp_settings(settings)
    
    def parse_bonds(self, data):
        """Parse bond list"""
        bond_count = data[0]
        bonds = []
        
        for i in range(bond_count):
            offset = 1 + (i * 40)
            addr = data[offset:offset+7]
            alias = data[offset+7:offset+39].decode('utf-8', errors='ignore').strip('\x00')
            is_connected = bool(data[offset+39])
            
            addr_str = ':'.join(f'{b:02X}' for b in addr[:6])
            bonds.append({
                'index': i,
                'address': addr_str,
                'alias': alias or f"Device {i}",
                'connected': is_connected
            })
        
        self.gui.update_bond_list(bonds)
    
    # Actions
    async def set_power(self, command):
        """Set power state"""
        await self.client.write_gatt_char(POWER_CONTROL_UUID, bytes([command]))
    
    async def save_auto_settings(self, settings):
        """Save auto settings"""
        data = bytes([
            1 if settings['auto_on_enabled'] else 0,
            1 if settings['auto_off_enabled'] else 0,
            settings['auto_on_percent'],
            settings['auto_off_percent'],
            settings['storage_percent'],
            1 if settings['reverse_off_enabled'] else 0,
            settings['reverse_off_percent'],
            1 if settings['reverse_on_enabled'] else 0,
            settings['reverse_on_percent'],
            0
        ])
        await self.client.write_gatt_char(AUTO_SETTINGS_UUID, data)
    
    async def save_temp_settings(self, settings):
        """Save temperature settings"""
        data = bytes([
            1 if settings['int_high_enabled'] else 0,
        ]) + struct.pack('>h', int(settings['int_high_threshold'] * 100)) + bytes([
            1 if settings['int_low_enabled'] else 0,
        ]) + struct.pack('>h', int(settings['int_low_threshold'] * 100)) + bytes([
            1 if settings['ext_high_enabled'] else 0,
        ]) + struct.pack('>h', int(settings['ext_high_threshold'] * 100)) + bytes([
            1 if settings['ext_low_enabled'] else 0,
        ]) + struct.pack('>h', int(settings['ext_low_threshold'] * 100))
        
        await self.client.write_gatt_char(TEMP_SETTINGS_UUID, data)
    
    async def set_bond_alias(self, bond_index, alias):
        """Set bond alias"""
        alias_bytes = alias.encode('utf-8')[:32].ljust(32, b'\x00')
        data = bytes([0x02, bond_index]) + alias_bytes
        await self.client.write_gatt_char(BOND_MANAGEMENT_UUID, data)
    
    async def delete_bond(self, bond_index):
        """Delete bond"""
        data = bytes([0x03, bond_index]) + (b'\x00' * 32)
        await self.client.write_gatt_char(BOND_MANAGEMENT_UUID, data)

# ============================================================================
# GUI Application
# ============================================================================

class BatteryMonitorGUI:
    def __init__(self):
        self._closing = False  # ← Khởi tạo TRƯỚC TIÊN
        self.loop = None
        self.scanned_devices = {}
        
        self.root = tk.Tk()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close) 
        self.root.title("ZMK Battery Monitor - Connected Devices Finder")
        self.root.geometry("900x700")
        
        self.ble = BatteryMonitorBLE(self)
        self.loop = None
        self.scanned_devices = {}  # Store {address: (name, rssi)} mapping
        
        self.create_widgets()
        self.start_async_loop()
    
    def create_widgets(self):
        """Create GUI widgets"""
        
        # Top frame - Connection với scan mode selector
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding=10)
        conn_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Row 0: Scan mode
        ttk.Label(conn_frame, text="Scan Mode:").grid(row=0, column=0, sticky=tk.W)
        self.scan_mode_var = tk.StringVar(value='all')
        mode_frame = ttk.Frame(conn_frame)
        mode_frame.grid(row=0, column=1, columnspan=3, sticky=tk.W, padx=5)
        
        ttk.Radiobutton(mode_frame, text="All Devices", variable=self.scan_mode_var, 
                       value='all').pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(mode_frame, text="Connected Only", variable=self.scan_mode_var, 
                       value='connected').pack(side=tk.LEFT, padx=5)
        ttk.Radiobutton(mode_frame, text="Advertising Only", variable=self.scan_mode_var, 
                       value='advertising').pack(side=tk.LEFT, padx=5)
        
        # Row 1: Device selection
        ttk.Label(conn_frame, text="Device:").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.device_var = tk.StringVar()
        self.device_combo = ttk.Combobox(conn_frame, textvariable=self.device_var, width=50)
        self.device_combo.grid(row=1, column=1, columnspan=2, padx=5, sticky=tk.EW)
        
        # Row 2: Buttons
        btn_frame = ttk.Frame(conn_frame)
        btn_frame.grid(row=2, column=0, columnspan=4, pady=5)
        
        self.scan_btn = ttk.Button(btn_frame, text="🔍 Scan", command=self.scan_devices)
        self.scan_btn.pack(side=tk.LEFT, padx=5)
        
        self.connect_btn = ttk.Button(btn_frame, text="🔗 Connect", command=self.connect_device)
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        self.disconnect_btn = ttk.Button(btn_frame, text="🔌 Disconnect", 
                                        command=self.disconnect_device, state=tk.DISABLED)
        self.disconnect_btn.pack(side=tk.LEFT, padx=5)
        
        # Notebook for tabs
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Tab 1: Monitoring
        monitor_tab = ttk.Frame(notebook)
        notebook.add(monitor_tab, text="📊 Monitoring")
        self.create_monitor_tab(monitor_tab)
        
        # Tab 2: Power Control
        power_tab = ttk.Frame(notebook)
        notebook.add(power_tab, text="🔋 Power Control")
        self.create_power_tab(power_tab)
        
        # Tab 3: Auto Settings
        auto_tab = ttk.Frame(notebook)
        notebook.add(auto_tab, text="⚙️  Auto Settings")
        self.create_auto_settings_tab(auto_tab)
        
        # Tab 4: Temperature Settings
        temp_tab = ttk.Frame(notebook)
        notebook.add(temp_tab, text="🌡️  Temperature")
        self.create_temp_settings_tab(temp_tab)
        
        # Tab 5: Bond Management
        bond_tab = ttk.Frame(notebook)
        notebook.add(bond_tab, text="🔗 Bonds")
        self.create_bond_tab(bond_tab)
        
        # Bottom frame - Log
        log_frame = ttk.LabelFrame(self.root, text="📝 Activity Log", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=8, state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True)
    
    def create_monitor_tab(self, parent):
        """Create monitoring tab"""
        
        # Status indicators
        status_frame = ttk.Frame(parent, padding=10)
        status_frame.pack(fill=tk.BOTH, expand=True)
        
        # Power State
        ttk.Label(status_frame, text="Power State:", font=('Arial', 12, 'bold')).grid(row=0, column=0, sticky=tk.W, pady=5)
        self.power_label = ttk.Label(status_frame, text="--", font=('Arial', 12))
        self.power_label.grid(row=0, column=1, sticky=tk.W, padx=10)
        
        # Internal Temperature
        ttk.Label(status_frame, text="Internal Temp:", font=('Arial', 12, 'bold')).grid(row=1, column=0, sticky=tk.W, pady=5)
        self.temp_int_label = ttk.Label(status_frame, text="--", font=('Arial', 12))
        self.temp_int_label.grid(row=1, column=1, sticky=tk.W, padx=10)
        
        # External Temperature
        ttk.Label(status_frame, text="External Temp:", font=('Arial', 12, 'bold')).grid(row=2, column=0, sticky=tk.W, pady=5)
        self.temp_ext_label = ttk.Label(status_frame, text="--", font=('Arial', 12))
        self.temp_ext_label.grid(row=2, column=1, sticky=tk.W, padx=10)
        
        # Voltage
        ttk.Label(status_frame, text="Battery Voltage:", font=('Arial', 12, 'bold')).grid(row=3, column=0, sticky=tk.W, pady=5)
        self.voltage_label = ttk.Label(status_frame, text="--", font=('Arial', 12))
        self.voltage_label.grid(row=3, column=1, sticky=tk.W, padx=10)
        
        # Battery Level
        ttk.Label(status_frame, text="Battery Level:", font=('Arial', 12, 'bold')).grid(row=4, column=0, sticky=tk.W, pady=5)
        self.battery_label = ttk.Label(status_frame, text="--", font=('Arial', 12))
        self.battery_label.grid(row=4, column=1, sticky=tk.W, padx=10)
        
        # RSSI Signal Strength
        ttk.Label(status_frame, text="Signal (RSSI):", font=('Arial', 12, 'bold')).grid(row=5, column=0, sticky=tk.W, pady=5)
        self.rssi_label = ttk.Label(status_frame, text="--", font=('Arial', 12))
        self.rssi_label.grid(row=5, column=1, sticky=tk.W, padx=10)
        
        # Read button
        ttk.Separator(status_frame, orient=tk.HORIZONTAL).grid(row=6, column=0, columnspan=2, sticky=tk.EW, pady=15)
        
        self.read_btn = ttk.Button(status_frame, text="🔄 Read All Data", 
                                   command=lambda: self.run_async(self.ble.read_all_data()),
                                   state=tk.DISABLED)
        self.read_btn.grid(row=7, column=0, columnspan=2, pady=10)
    
    def create_power_tab(self, parent):
        """Create power control tab"""
        
        frame = ttk.Frame(parent, padding=20)
        frame.pack(fill=tk.BOTH, expand=True)
        
        ttk.Label(frame, text="MOSFET Control", font=('Arial', 14, 'bold')).pack(pady=10)
        
        btn_frame = ttk.Frame(frame)
        btn_frame.pack(pady=20)
        
        ttk.Button(btn_frame, text="🔴 Power OFF", command=lambda: self.run_async(self.ble.set_power(CMD_POWER_OFF)), width=20).pack(pady=5)
        ttk.Button(btn_frame, text="🟢 Power ON", command=lambda: self.run_async(self.ble.set_power(CMD_POWER_ON)), width=20).pack(pady=5)
        ttk.Button(btn_frame, text="🔄 Toggle", command=lambda: self.run_async(self.ble.set_power(CMD_POWER_TOGGLE)), width=20).pack(pady=5)
        
        ttk.Separator(frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=20)
        
        ttk.Label(frame, text="Device Control", font=('Arial', 14, 'bold')).pack(pady=10)
        ttk.Button(frame, text="🔄 Reset Device", command=self.reset_device, width=20).pack(pady=5)
    
    def create_auto_settings_tab(self, parent):
        """Create auto settings tab"""
        
        frame = ttk.Frame(parent, padding=10)
        frame.pack(fill=tk.BOTH, expand=True)
        
        # Auto ON
        auto_on_frame = ttk.LabelFrame(frame, text="Auto ON (Enable charging when battery is low)", padding=10)
        auto_on_frame.pack(fill=tk.X, pady=5)
        
        self.auto_on_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(auto_on_frame, text="Enable Auto ON", variable=self.auto_on_var).grid(row=0, column=0, sticky=tk.W)
        
        ttk.Label(auto_on_frame, text="Turn ON when battery <").grid(row=1, column=0, sticky=tk.W, padx=20)
        self.auto_on_spin = ttk.Spinbox(auto_on_frame, from_=0, to=100, width=10)
        self.auto_on_spin.set(30)
        self.auto_on_spin.grid(row=1, column=1)
        ttk.Label(auto_on_frame, text="%").grid(row=1, column=2, sticky=tk.W)
        
        # Auto OFF
        auto_off_frame = ttk.LabelFrame(frame, text="Auto OFF (Stop charging when battery is full)", padding=10)
        auto_off_frame.pack(fill=tk.X, pady=5)
        
        self.auto_off_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(auto_off_frame, text="Enable Auto OFF", variable=self.auto_off_var).grid(row=0, column=0, sticky=tk.W)
        
        ttk.Label(auto_off_frame, text="Turn OFF when battery >").grid(row=1, column=0, sticky=tk.W, padx=20)
        self.auto_off_spin = ttk.Spinbox(auto_off_frame, from_=0, to=100, width=10)
        self.auto_off_spin.set(80)
        self.auto_off_spin.grid(row=1, column=1)
        ttk.Label(auto_off_frame, text="%").grid(row=1, column=2, sticky=tk.W)
        
        # Storage Mode
        storage_frame = ttk.LabelFrame(frame, text="Storage Mode (Always active)", padding=10)
        storage_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(storage_frame, text="Turn OFF when battery ≤").grid(row=0, column=0, sticky=tk.W)
        self.storage_spin = ttk.Spinbox(storage_frame, from_=0, to=100, width=10)
        self.storage_spin.set(40)
        self.storage_spin.grid(row=0, column=1)
        ttk.Label(storage_frame, text="%").grid(row=0, column=2, sticky=tk.W)
        
        # Reverse Controls
        reverse_frame = ttk.LabelFrame(frame, text="Reverse Controls (Opposite behavior)", padding=10)
        reverse_frame.pack(fill=tk.X, pady=5)
        
        self.reverse_off_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(reverse_frame, text="Reverse OFF: Turn OFF when battery <", variable=self.reverse_off_var).grid(row=0, column=0, sticky=tk.W)
        self.reverse_off_spin = ttk.Spinbox(reverse_frame, from_=0, to=100, width=10)
        self.reverse_off_spin.set(25)
        self.reverse_off_spin.grid(row=0, column=1, padx=5)
        ttk.Label(reverse_frame, text="%").grid(row=0, column=2, sticky=tk.W)
        
        self.reverse_on_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(reverse_frame, text="Reverse ON: Turn ON when battery >", variable=self.reverse_on_var).grid(row=1, column=0, sticky=tk.W)
        self.reverse_on_spin = ttk.Spinbox(reverse_frame, from_=0, to=100, width=10)
        self.reverse_on_spin.set(60)
        self.reverse_on_spin.grid(row=1, column=1, padx=5)
        ttk.Label(reverse_frame, text="%").grid(row=1, column=2, sticky=tk.W)
        
        # Save button
        ttk.Button(frame, text="💾 Save Auto Settings", command=self.save_auto_settings).pack(pady=10)
    
    def create_temp_settings_tab(self, parent):
        """Create temperature settings tab"""
        
        frame = ttk.Frame(parent, padding=10)
        frame.pack(fill=tk.BOTH, expand=True)
        
        # Internal
        int_frame = ttk.LabelFrame(frame, text="Internal Temperature Protection", padding=10)
        int_frame.pack(fill=tk.X, pady=5)
        
        self.int_high_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(int_frame, text="High: Turn OFF when >", variable=self.int_high_var).grid(row=0, column=0, sticky=tk.W)
        self.int_high_spin = ttk.Spinbox(int_frame, from_=-40, to=125, width=10, increment=0.5)
        self.int_high_spin.set(50)
        self.int_high_spin.grid(row=0, column=1, padx=5)
        ttk.Label(int_frame, text="°C").grid(row=0, column=2, sticky=tk.W)
        
        self.int_low_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(int_frame, text="Low: Turn ON when <", variable=self.int_low_var).grid(row=1, column=0, sticky=tk.W)
        self.int_low_spin = ttk.Spinbox(int_frame, from_=-40, to=125, width=10, increment=0.5)
        self.int_low_spin.set(10)
        self.int_low_spin.grid(row=1, column=1, padx=5)
        ttk.Label(int_frame, text="°C").grid(row=1, column=2, sticky=tk.W)
        
        # External
        ext_frame = ttk.LabelFrame(frame, text="External Temperature Protection (NTC)", padding=10)
        ext_frame.pack(fill=tk.X, pady=5)
        
        self.ext_high_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(ext_frame, text="High: Turn OFF when >", variable=self.ext_high_var).grid(row=0, column=0, sticky=tk.W)
        self.ext_high_spin = ttk.Spinbox(ext_frame, from_=-40, to=125, width=10, increment=0.5)
        self.ext_high_spin.set(60)
        self.ext_high_spin.grid(row=0, column=1, padx=5)
        ttk.Label(ext_frame, text="°C").grid(row=0, column=2, sticky=tk.W)
        
        self.ext_low_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(ext_frame, text="Low: Turn ON when <", variable=self.ext_low_var).grid(row=1, column=0, sticky=tk.W)
        self.ext_low_spin = ttk.Spinbox(ext_frame, from_=-40, to=125, width=10, increment=0.5)
        self.ext_low_spin.set(5)
        self.ext_low_spin.grid(row=1, column=1, padx=5)
        ttk.Label(ext_frame, text="°C").grid(row=1, column=2, sticky=tk.W)
        
        # Save button
        ttk.Button(frame, text="💾 Save Temperature Settings", command=self.save_temp_settings).pack(pady=10)
    
    def create_bond_tab(self, parent):
        """Create bond management tab"""
        
        frame = ttk.Frame(parent, padding=10)
        frame.pack(fill=tk.BOTH, expand=True)
        
        # Bond list
        list_frame = ttk.Frame(frame)
        list_frame.pack(fill=tk.BOTH, expand=True)
        
        columns = ('Index', 'Address', 'Alias', 'Status')
        self.bond_tree = ttk.Treeview(list_frame, columns=columns, show='headings', height=10)
        
        self.bond_tree.heading('Index', text='#')
        self.bond_tree.heading('Address', text='Address')
        self.bond_tree.heading('Alias', text='Alias')
        self.bond_tree.heading('Status', text='Status')
        
        self.bond_tree.column('Index', width=50)
        self.bond_tree.column('Address', width=200)
        self.bond_tree.column('Alias', width=200)
        self.bond_tree.column('Status', width=100)
        
        self.bond_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        scrollbar = ttk.Scrollbar(list_frame, orient=tk.VERTICAL, command=self.bond_tree.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.bond_tree.configure(yscrollcommand=scrollbar.set)
        
        # Buttons
        btn_frame = ttk.Frame(frame)
        btn_frame.pack(fill=tk.X, pady=10)
        
        ttk.Button(btn_frame, text="🏷️  Set Alias", command=self.set_bond_alias).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="🗑️  Delete Bond", command=self.delete_bond).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="🔄 Refresh", command=lambda: self.run_async(self.ble.read_all_data())).pack(side=tk.LEFT, padx=5)
    
    # Update methods
    def update_power_state(self, state):
        self.power_label.config(text=f"{'🟢 ON' if state else '🔴 OFF'}", 
                               foreground='green' if state else 'red')
    
    def update_temp_internal(self, temp):
        self.temp_int_label.config(text=f"{temp:.2f}°C")
    
    def update_temp_external(self, temp):
        self.temp_ext_label.config(text=f"{temp:.2f}°C")
    
    def update_voltage(self, voltage):
        self.voltage_label.config(text=f"{voltage}mV ({voltage/1000:.3f}V)")
    
    def update_battery_level(self, level):
        # Color code based on level
        if level > 80:
            color = 'green'
        elif level > 20:
            color = 'orange'
        else:
            color = 'red'
        
        self.battery_label.config(text=f"{level}%", foreground=color)
    
    def update_rssi(self, rssi):
        """Update RSSI signal strength"""
        # RSSI interpretation:
        # > -50: Excellent
        # -50 to -60: Good
        # -60 to -70: Fair
        # < -70: Weak
        
        if rssi > -50:
            color = 'green'
            quality = 'Excellent'
        elif rssi > -60:
            color = 'green'
            quality = 'Good'
        elif rssi > -70:
            color = 'orange'
            quality = 'Fair'
        else:
            color = 'red'
            quality = 'Weak'
        
        self.rssi_label.config(text=f"{rssi} dBm ({quality})", foreground=color)
    
    def update_auto_settings(self, settings):
        self.auto_on_var.set(settings['auto_on_enabled'])
        self.auto_on_spin.delete(0, tk.END)
        self.auto_on_spin.insert(0, str(settings['auto_on_percent']))
        
        self.auto_off_var.set(settings['auto_off_enabled'])
        self.auto_off_spin.delete(0, tk.END)
        self.auto_off_spin.insert(0, str(settings['auto_off_percent']))
        
        self.storage_spin.delete(0, tk.END)
        self.storage_spin.insert(0, str(settings['storage_percent']))
        
        self.reverse_off_var.set(settings['reverse_off_enabled'])
        self.reverse_off_spin.delete(0, tk.END)
        self.reverse_off_spin.insert(0, str(settings['reverse_off_percent']))
        
        self.reverse_on_var.set(settings['reverse_on_enabled'])
        self.reverse_on_spin.delete(0, tk.END)
        self.reverse_on_spin.insert(0, str(settings['reverse_on_percent']))
    
    def update_temp_settings(self, settings):
        self.int_high_var.set(settings['int_high_enabled'])
        self.int_high_spin.delete(0, tk.END)
        self.int_high_spin.insert(0, f"{settings['int_high_threshold']:.2f}")
        
        self.int_low_var.set(settings['int_low_enabled'])
        self.int_low_spin.delete(0, tk.END)
        self.int_low_spin.insert(0, f"{settings['int_low_threshold']:.2f}")
        
        self.ext_high_var.set(settings['ext_high_enabled'])
        self.ext_high_spin.delete(0, tk.END)
        self.ext_high_spin.insert(0, f"{settings['ext_high_threshold']:.2f}")
        
        self.ext_low_var.set(settings['ext_low_enabled'])
        self.ext_low_spin.delete(0, tk.END)
        self.ext_low_spin.insert(0, f"{settings['ext_low_threshold']:.2f}")
    
    def update_bond_list(self, bonds):
        # Clear existing items
        for item in self.bond_tree.get_children():
            self.bond_tree.delete(item)
        
        # Add bonds
        for bond in bonds:
            status = '🟢 Connected' if bond['connected'] else '⚪ Not Connected'
            self.bond_tree.insert('', tk.END, values=(
                bond['index'],
                bond['address'],
                bond['alias'],
                status
            ))
    
    def log(self, message):
        """Add message to log"""
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.log_text.config(state=tk.NORMAL)
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)
    
    # Action methods
    def scan_devices(self):
        """Scan for BLE devices based on selected mode"""
        scan_mode = self.scan_mode_var.get()
        mode_text = {
            'all': 'all devices',
            'connected': 'connected devices only',
            'advertising': 'advertising devices only'
        }
        
        self.log(f"🔍 Scanning for {mode_text[scan_mode]}...")
        self.scan_btn.config(state=tk.DISABLED)
        
        async def scan():
            devices = await self.ble.scan_devices(scan_mode=scan_mode)
            
            # Store devices with RSSI info
            self.scanned_devices.clear()
            device_display_list = []
            
            for name, addr, rssi in devices:
                self.scanned_devices[addr] = (name, rssi)
                device_display_list.append(f"{name} ({addr})")
                
                # Log with RSSI immediately
                rssi_text = f" RSSI: {rssi} dBm" if rssi is not None else " (no RSSI)"
                self.log(f"  {addr}: {name.split('(')[0].strip()}{rssi_text}")
            
            self.device_combo['values'] = device_display_list
            
            if devices:
                self.device_combo.current(0)
                self.log(f"✅ Found {len(devices)} device(s) with RSSI data")
            else:
                self.log(f"⚠️ No devices found in {mode_text[scan_mode]} mode")
            
            self.scan_btn.config(state=tk.NORMAL)
        
        self.run_async(scan())
    
    def connect_device(self):
        """Connect to selected device"""
        device_str = self.device_var.get()
        if not device_str:
            messagebox.showwarning("Warning", "Please select a device first")
            return
        
        # Extract address - handle both formats
        if '(' in device_str and ')' in device_str:
            address = device_str.split('(')[-1].strip(')')
        else:
            address = device_str
        
        # Get RSSI from scanned devices
        rssi_from_scan = None
        if address in self.scanned_devices:
            _, rssi_from_scan = self.scanned_devices[address]
            if rssi_from_scan is not None:
                self.log(f"📡 Using RSSI from scan: {rssi_from_scan} dBm")
        
        self.log(f"🔄 Connecting to {address}...")
        self.connect_btn.config(state=tk.DISABLED)
        
        async def connect():
            success = await self.ble.connect(address, rssi_from_scan)
            if success:
                self.disconnect_btn.config(state=tk.NORMAL)
                self.read_btn.config(state=tk.NORMAL)
            else:
                self.connect_btn.config(state=tk.NORMAL)
        
        self.run_async(connect())
    
    def disconnect_device(self):
        """Disconnect from device"""
        async def disconnect():
            await self.ble.disconnect()
            self.connect_btn.config(state=tk.NORMAL)
            self.disconnect_btn.config(state=tk.DISABLED)
            self.read_btn.config(state=tk.DISABLED)
        
        self.run_async(disconnect())
    
    def reset_device(self):
        """Reset the device"""
        if messagebox.askyesno("Confirm", "Are you sure you want to reset the device?"):
            async def reset():
                await self.ble.client.write_gatt_char(POWER_CONTROL_UUID, bytes([CMD_RESET_DEVICE]))
                self.log("🔄 Device reset command sent")
            
            self.run_async(reset())
    
    def save_auto_settings(self):
        """Save auto settings to device"""
        try:
            settings = {
                'auto_on_enabled': self.auto_on_var.get(),
                'auto_off_enabled': self.auto_off_var.get(),
                'auto_on_percent': int(self.auto_on_spin.get()),
                'auto_off_percent': int(self.auto_off_spin.get()),
                'storage_percent': int(self.storage_spin.get()),
                'reverse_off_enabled': self.reverse_off_var.get(),
                'reverse_off_percent': int(self.reverse_off_spin.get()),
                'reverse_on_enabled': self.reverse_on_var.get(),
                'reverse_on_percent': int(self.reverse_on_spin.get()),
            }
            
            # Validate
            if settings['auto_on_percent'] >= settings['auto_off_percent']:
                messagebox.showerror("Error", "Auto ON percent must be < Auto OFF percent")
                return
            
            if settings['reverse_off_percent'] >= settings['reverse_on_percent']:
                messagebox.showerror("Error", "Reverse OFF percent must be < Reverse ON percent")
                return
            
            self.run_async(self.ble.save_auto_settings(settings))
            self.log("✅ Auto settings saved")
            
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numbers")
    
    def save_temp_settings(self):
        """Save temperature settings to device"""
        try:
            settings = {
                'int_high_enabled': self.int_high_var.get(),
                'int_high_threshold': float(self.int_high_spin.get()),
                'int_low_enabled': self.int_low_var.get(),
                'int_low_threshold': float(self.int_low_spin.get()),
                'ext_high_enabled': self.ext_high_var.get(),
                'ext_high_threshold': float(self.ext_high_spin.get()),
                'ext_low_enabled': self.ext_low_var.get(),
                'ext_low_threshold': float(self.ext_low_spin.get()),
            }
            
            # Validate
            if settings['int_low_threshold'] >= settings['int_high_threshold']:
                messagebox.showerror("Error", "Internal low must be < Internal high")
                return
            
            if settings['ext_low_threshold'] >= settings['ext_high_threshold']:
                messagebox.showerror("Error", "External low must be < External high")
                return
            
            self.run_async(self.ble.save_temp_settings(settings))
            self.log("✅ Temperature settings saved")
            
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numbers")
    
    def set_bond_alias(self):
        """Set alias for selected bond"""
        selection = self.bond_tree.selection()
        if not selection:
            messagebox.showwarning("Warning", "Please select a bond first")
            return
        
        item = self.bond_tree.item(selection[0])
        bond_index = int(item['values'][0])
        current_alias = item['values'][2]
        
        # Dialog for alias input
        dialog = tk.Toplevel(self.root)
        dialog.title("Set Bond Alias")
        dialog.geometry("300x120")
        dialog.transient(self.root)
        dialog.grab_set()
        
        ttk.Label(dialog, text="Enter new alias:").pack(pady=10)
        alias_entry = ttk.Entry(dialog, width=30)
        alias_entry.insert(0, current_alias)
        alias_entry.pack(pady=5)
        
        def save_alias():
            new_alias = alias_entry.get()
            if new_alias:
                self.run_async(self.ble.set_bond_alias(bond_index, new_alias))
                self.log(f"✅ Alias set for bond {bond_index}: {new_alias}")
                dialog.destroy()
        
        ttk.Button(dialog, text="Save", command=save_alias).pack(pady=10)
    
    def delete_bond(self):
        """Delete selected bond"""
        selection = self.bond_tree.selection()
        if not selection:
            messagebox.showwarning("Warning", "Please select a bond first")
            return
        
        item = self.bond_tree.item(selection[0])
        bond_index = int(item['values'][0])
        status = item['values'][3]
        
        if '🟢' in status:
            messagebox.showerror("Error", "Cannot delete active bond")
            return
        
        if messagebox.askyesno("Confirm", f"Delete bond {bond_index}?"):
            self.run_async(self.ble.delete_bond(bond_index))
            self.log(f"🗑️  Bond {bond_index} deleted")
    
    # Async event loop management
    def start_async_loop(self):
        """Start async event loop in separate thread"""
        def run_loop():
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.loop.run_forever()
        
        thread = threading.Thread(target=run_loop, daemon=True)
        thread.start()
    
    def run_async(self, coro):
        """Run async coroutine"""
        if self.loop:
            asyncio.run_coroutine_threadsafe(coro, self.loop)
    
    def run(self):
        """Start GUI"""
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.mainloop()
    
    def on_close(self):
        if not self._closing:
            self._closing = True
            #self.log("🔄 Closing application...")
            
            # Disconnect if connected
            if self.ble.connected:
                # Create new event loop for synchronous context
                disconnect_loop = asyncio.new_event_loop()
                try:
                    disconnect_loop.run_until_complete(
                        asyncio.wait_for(self.ble.disconnect(), timeout=5.0)
                    )
                except asyncio.TimeoutError:
                    #self.log("⚠️ Disconnect timeout during shutdown")
                    pass
                except Exception as e:
                    #self.log(f"⚠️ Error during disconnect: {e}")
                    pass
                finally:
                    disconnect_loop.close()
            
            # Stop async loop
            if self.loop:
                self.loop.call_soon_threadsafe(self.loop.stop)
            
            # Destroy window
            self.root.destroy()
            #self.Log("✅ Application closed")

# ============================================================================
# Main
# ============================================================================

if __name__ == "__main__":
    app = BatteryMonitorGUI()
    app.run()