"""
ZMK Battery Monitor - Complete GUI Control Panel
Features: Power Control, Temperature Monitoring, Settings, Bond Management, Battery %
Dependencies: pip install bleak
"""

import asyncio
import struct
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
from bleak import BleakClient, BleakScanner
from datetime import datetime
import threading

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
# BLE Manager
# ============================================================================

class BatteryMonitorBLE:
    def __init__(self, gui):
        self.gui = gui
        self.client = None
        self.connected = False
        self.device_address = None
        
    async def scan_devices(self, include_paired=True):
        """Scan for available devices"""
        devices_dict = {}
        
        # 1. BLE Advertisement Scan - Tìm devices đang phát sóng
        self.gui.log("🔍 Scanning BLE advertisements...")
        discovered = await BleakScanner.discover(timeout=5.0)
        for d in discovered:
            if d.name:
                devices_dict[d.address] = {
                    'name': d.name,
                    'address': d.address,
                    'source': '📡 Advertising',
                    'rssi': d.rssi if hasattr(d, 'rssi') else None
                }
        
        # 2. Get paired/connected devices from OS
        if include_paired:
            self.gui.log("🔗 Checking OS paired devices...")
            try:
                import platform
                system = platform.system()
                
                if system == "Windows":
                    paired = await self._get_windows_paired_devices()
                elif system == "Darwin":  # macOS
                    paired = await self._get_macos_paired_devices()
                else:  # Linux
                    paired = await self._get_linux_paired_devices()
                
                # Merge with discovered devices
                for addr, info in paired.items():
                    if addr in devices_dict:
                        # Device found in both sources
                        devices_dict[addr]['source'] = '✅ Paired + Advertising'
                    else:
                        # Device only in paired list (not advertising)
                        devices_dict[addr] = {
                            'name': info['name'],
                            'address': addr,
                            'source': '🔗 Paired Only',
                            'rssi': None
                        }
            except Exception as e:
                self.gui.log(f"⚠️  Could not get paired devices: {e}")
        
        # Convert to list and sort by source priority
        source_priority = {'✅ Paired + Advertising': 0, '📡 Advertising': 1, '🔗 Paired Only': 2}
        devices_list = sorted(
            devices_dict.values(),
            key=lambda x: source_priority.get(x['source'], 3)
        )
        
        return [(f"{d['name']} {d['source']}", d['address']) for d in devices_list]
    
    async def _get_windows_paired_devices(self):
        """Get paired devices from Windows (requires winrt)"""
        devices = {}
        try:
            # Try using winrt if available
            from winrt.windows.devices.bluetooth import BluetoothDevice
            from winrt.windows.devices.enumeration import DeviceInformation
            
            # This is a simplified version - full implementation needs more code
            # For now, return empty dict
            pass
        except ImportError:
            # Fallback: Use registry (Windows only)
            try:
                import winreg
                key_path = r"SYSTEM\CurrentControlSet\Services\BTHPORT\Parameters\Devices"
                with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, key_path) as key:
                    i = 0
                    while True:
                        try:
                            device_key = winreg.EnumKey(key, i)
                            # Convert registry key to MAC address format
                            mac = ':'.join([device_key[j:j+2] for j in range(0, 12, 2)])
                            
                            with winreg.OpenKey(key, device_key) as device:
                                try:
                                    name = winreg.QueryValueEx(device, "Name")[0]
                                    devices[mac.upper()] = {'name': name}
                                except:
                                    devices[mac.upper()] = {'name': f"Device {mac}"}
                            i += 1
                        except OSError:
                            break
            except Exception as e:
                self.gui.log(f"⚠️  Registry read failed: {e}")
        
        return devices
    
    async def _get_macos_paired_devices(self):
        """Get paired devices from macOS"""
        devices = {}
        try:
            # Use system_profiler to get Bluetooth devices
            import subprocess
            result = subprocess.run(
                ['system_profiler', 'SPBluetoothDataType', '-json'],
                capture_output=True,
                text=True,
                timeout=3
            )
            
            if result.returncode == 0:
                import json
                data = json.loads(result.stdout)
                # Parse Bluetooth data (simplified - actual structure is complex)
                # This would need proper parsing based on macOS version
                pass
        except Exception as e:
            self.gui.log(f"⚠️  macOS query failed: {e}")
        
        return devices
    
    async def _get_linux_paired_devices(self):
        """Get paired devices from Linux (using bluetoothctl)"""
        devices = {}
        try:
            import subprocess
            # Get paired devices using bluetoothctl
            result = subprocess.run(
                ['bluetoothctl', 'devices', 'Paired'],
                capture_output=True,
                text=True,
                timeout=3
            )
            
            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if 'Device' in line:
                        parts = line.split()
                        if len(parts) >= 3:
                            addr = parts[1]
                            name = ' '.join(parts[2:])
                            devices[addr.upper()] = {'name': name}
        except Exception as e:
            self.gui.log(f"⚠️  bluetoothctl failed: {e}")
        
        return devices
    
    async def connect(self, address):
        """Connect to device"""
        try:
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
            await self.client.start_notify(BATTERY_LEVEL_UUID, self.battery_level_notification)
            
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
            
            # Read settings
            data = await self.client.read_gatt_char(AUTO_SETTINGS_UUID)
            self.auto_settings_notification(None, data)
            
            data = await self.client.read_gatt_char(TEMP_SETTINGS_UUID)
            self.temp_settings_notification(None, data)
            
            # Read bonds
            data = await self.client.read_gatt_char(BOND_MANAGEMENT_UUID)
            self.parse_bonds(data)
            
        except Exception as e:
            self.gui.log(f"⚠️  Error reading data: {e}")
    
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
        self.root = tk.Tk()
        self.root.title("ZMK Battery Monitor Control Panel")
        self.root.geometry("900x700")
        
        self.ble = BatteryMonitorBLE(self)
        self.loop = None
        
        self.create_widgets()
        self.start_async_loop()
    
    def create_widgets(self):
        """Create GUI widgets"""
        
        # Top frame - Connection
        conn_frame = ttk.LabelFrame(self.root, text="Connection", padding=10)
        conn_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(conn_frame, text="Device:").grid(row=0, column=0, sticky=tk.W)
        self.device_var = tk.StringVar()
        self.device_combo = ttk.Combobox(conn_frame, textvariable=self.device_var, width=40)
        self.device_combo.grid(row=0, column=1, padx=5)
        
        self.scan_btn = ttk.Button(conn_frame, text="Scan", command=self.scan_devices)
        self.scan_btn.grid(row=0, column=2, padx=5)
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.connect_device)
        self.connect_btn.grid(row=0, column=3, padx=5)
        
        self.disconnect_btn = ttk.Button(conn_frame, text="Disconnect", command=self.disconnect_device, state=tk.DISABLED)
        self.disconnect_btn.grid(row=0, column=4, padx=5)
        
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
        log_frame = ttk.LabelFrame(self.root, text="Log", padding=10)
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
        """Scan for BLE devices"""
        self.log("🔍 Scanning for devices...")
        self.scan_btn.config(state=tk.DISABLED)
        
        async def scan():
            devices = await self.ble.scan_devices(include_paired=True)
            self.device_combo['values'] = [f"{name} ({addr})" for name, addr in devices]
            if devices:
                self.device_combo.current(0)
                self.log(f"✅ Found {len(devices)} devices")
                
                # Log device sources
                for name, addr in devices:
                    if '✅' in name:
                        self.log(f"  {addr}: Paired + Active")
                    elif '📡' in name:
                        self.log(f"  {addr}: Advertising only")
                    elif '🔗' in name:
                        self.log(f"  {addr}: Paired but not advertising")
            else:
                self.log("⚠️  No devices found")
            self.scan_btn.config(state=tk.NORMAL)
        
        self.run_async(scan())
    
    def connect_device(self):
        """Connect to selected device"""
        device_str = self.device_var.get()
        if not device_str:
            messagebox.showwarning("Warning", "Please select a device first")
            return
        
        # Extract address from "Name (Address)" format
        address = device_str.split('(')[-1].strip(')')
        
        self.log(f"🔄 Connecting to {address}...")
        self.connect_btn.config(state=tk.DISABLED)
        
        async def connect():
            success = await self.ble.connect(address)
            if success:
                self.disconnect_btn.config(state=tk.NORMAL)
            else:
                self.connect_btn.config(state=tk.NORMAL)
        
        self.run_async(connect())
    
    def disconnect_device(self):
        """Disconnect from device"""
        async def disconnect():
            await self.ble.disconnect()
            self.connect_btn.config(state=tk.NORMAL)
            self.disconnect_btn.config(state=tk.DISABLED)
        
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
        """Handle window close"""
        if self.ble.connected:
            self.run_async(self.ble.disconnect())
        if self.loop:
            self.loop.call_soon_threadsafe(self.loop.stop)
        self.root.destroy()

# ============================================================================
# Main
# ============================================================================

if __name__ == "__main__":
    app = BatteryMonitorGUI()
    app.run()