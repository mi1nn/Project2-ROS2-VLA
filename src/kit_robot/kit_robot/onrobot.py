from pymodbus.client import ModbusTcpClient


def _connect_virtual_gripper(timeout_sec=3.0):
    try:
        import rclpy
        from onrobot_rg_msgs.srv import SetCommand
        from rclpy.executors import SingleThreadedExecutor
    except ImportError:
        return None

    if not rclpy.ok():
        return None

    node = rclpy.create_node("onrobot_virtual_gripper_client")
    client = node.create_client(SetCommand, "/onrobot/sendCommand")
    if not client.wait_for_service(timeout_sec=timeout_sec):
        node.destroy_node()
        return None

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def send(command):
        request = SetCommand.Request()
        request.command = str(command)
        future = client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=timeout_sec)

        if not future.done():
            future.cancel()
            print(f"[onrobot] command {command!r} timed out")
            return

        result = future.result()

        if result is None or not result.success:
            print(f"[onrobot] command {command!r} failed: {result}")

    print("[onrobot] using virtual gripper")
    return send


class RG:
    MAX_WIDTH = {
        "rg2": 1100,
        "rg6": 1600,
    }

    def __init__(self, gripper, ip, port):
        if gripper not in self.MAX_WIDTH:
            raise ValueError("gripper must be 'rg2' or 'rg6'")

        self.max_width = self.MAX_WIDTH[gripper]
        self.client = ModbusTcpClient(
            ip,
            port=port,
            stopbits=1,
            bytesize=8,
            parity="E",
            baudrate=115200,
            timeout=1,
        )

        self.connected = bool(self.client.connect())
        self._virtual_send = None

        if not self.connected:
            self._virtual_send = _connect_virtual_gripper()
            if self._virtual_send is None:
                print(
                    f"[onrobot] gripper unavailable at {ip}:{port}; "
                    "commands will be skipped"
                )

    def _read_register(self, address):
        if not self.connected:
            raise ConnectionError("RG gripper is not connected")

        result = self.client.read_holding_registers(
            address=address,
            count=1,
            slave=65,
        )
        if result.isError():
            raise ConnectionError(f"RG gripper read failed (reg {address}): {result}")
        return result.registers[0]

    def _move(self, width_val, force_val, virtual_command):
        if not self.connected:
            if self._virtual_send:
                self._virtual_send(virtual_command)
            else:
                print("[onrobot] command skipped: gripper is not connected")
            return

        result = self.client.write_registers(
            address=0,
            values=[force_val, width_val, 16],
            slave=65,
        )
        if result.isError():
            raise ConnectionError(f"RG gripper move failed: {result}")

    def get_width(self):
        if not self.connected:
            return 0.0
        return self._read_register(267) / 10.0

    def get_status(self):
        if not self.connected:
            return [0] * 7

        status = self._read_register(268)
        return [(status >> bit) & 1 for bit in range(7)]

    def close_gripper(self, force_val=400):
        self._move(0, force_val, "c")

    def open_gripper(self, force_val=400):
        self._move(self.max_width, force_val, "o")

    def move_gripper(self, width_val, force_val=400):
        self._move(width_val, force_val, int(width_val))

    def close(self):
        self.client.close()