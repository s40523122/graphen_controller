import serial
import struct
import time
import threading
import sys
import os

# ══════════════════════════════════════════
#  共用常數
# ══════════════════════════════════════════
DOF_NAMES = [
    "小拇指指根", "無名指指根", "中指指根",   "中指側擺",
    "食指指根",   "食指側擺",   "小拇指指中", "無名指指中",
    "中指指中",   "食指指中",   "大拇指側擺",
    "大拇指第一關節", "大拇指第二關節"
]

ANGLE_RANGE = {
    "小拇指指根":     (900,  1650),
    "無名指指根":     (900,  1650),
    "中指指根":       (900,  1650),
    "中指側擺":       (-150,  150),
    "食指指根":       (900,  1650),
    "食指側擺":       (-150,  150),
    "小拇指指中":     (1050, 1900),
    "無名指指中":     (1050, 1900),
    "中指指中":       (1050, 1900),
    "食指指中":       (1050, 1900),
    "大拇指側擺":     (650,  1750),
    "大拇指第一關節": (1250, 1600),
    "大拇指第二關節": (1500, 2040),
}

GESTURES = {
    "open":  [1650, 1650, 1650,    0, 1650,    0, 1900, 1900, 1900, 1900, 1750, 1600, 2040],
    "close": [ 970,  970,  900,    0,  920,    0, 1140, 1140, 1140, 1140, 1700, 1300, 1650],
    "ya":    [ 970,  970, 1650,   90, 1650, -150, 1140, 1140, 1900, 1900, 1300, 1281, 1800],
    "ok":    [1650, 1650, 1630,  -25, 1400,    0, 1900, 1900, 1900, 1610, 1200, 1325, 1990],
    "666":   [1650,  970,  970,    0,  970,    0, 1900, 1140, 1140, 1140, 1680, 1400, 1770],
    "rock":  [1650,  970,  970,    0, 1650,    0, 1900, 1140, 1140, 1900, 1160, 1300, 1800],
    "card":  [ 970,  970,  970,    0,  970,    0, 1140, 1140, 1140, 1900, 1130, 1390, 2040],
}

REG_ANGLE_SET = 0x0438
REG_ANGLE_ACT = 0x0470
REG_SPEED_SET = 0x0454

# ══════════════════════════════════════════
#  通訊工具（純函式）
# ══════════════════════════════════════════

def checksum(data: bytes) -> int:
    return sum(data) & 0xFF

def build_write_frame(hand_id: int, address: int, data: list) -> bytes:
    addr_l  = address & 0xFF
    addr_h  = (address >> 8) & 0xFF
    data_b  = bytes(data)
    payload = bytes([hand_id, len(data_b) + 3, 0x12, addr_l, addr_h]) + data_b
    return bytes([0xEB, 0x90]) + payload + bytes([checksum(payload)])

def build_read_frame(hand_id: int, address: int, byte_len: int) -> bytes:
    addr_l  = address & 0xFF
    addr_h  = (address >> 8) & 0xFF
    payload = bytes([hand_id, 0x04, 0x11, addr_l, addr_h, byte_len])
    return bytes([0xEB, 0x90]) + payload + bytes([checksum(payload)])

def int16_to_bytes(v: int) -> list:
    return list(struct.pack('<h', v))

def parse_int16_list(raw: bytes, count: int) -> list:
    return [struct.unpack_from('<h', raw, i * 2)[0] for i in range(count)]

# ══════════════════════════════════════════
#  DexterousHand 類別
# ══════════════════════════════════════════

class DexterousHand:
    def __init__(self, port: str, hand_id: int = 1,
                 baudrate: int = 115200, label: str = ""):
        self.port     = port
        self.hand_id  = hand_id
        self.baudrate = baudrate
        self.label    = label or port

        self.ser  = None
        self.serial_lock = threading.Lock()

        self.current_angles = None
        self.read_running   = False
        self._reader_thread = None

    # ── 連線 / 斷線 ───────────────────────────────────

    def connect(self) -> bool:
        try:
            self.ser = serial.Serial(
                port      = self.port,
                baudrate  = self.baudrate,
                bytesize  = serial.EIGHTBITS,
                parity    = serial.PARITY_NONE,
                stopbits  = serial.STOPBITS_ONE,
                timeout   = 0.5,
            )
            print(f"  ✅ [{self.label}] 串口已開啟：{self.port}")
            return True
        except serial.SerialException as e:
            print(f"  ❌ [{self.label}] 無法開啟串口：{e}")
            return False

    def disconnect(self):
        self.stop_reader()
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"  🔌 [{self.label}] 串口已關閉")

    # ── 底層讀寫 ──────────────────────────────────────

    def _write_registers(self, address: int, data: list) -> bool:
        if not self.ser or not self.ser.is_open:
            return False

        frame = build_write_frame(self.hand_id, address, data)

        with self.serial_lock:
            self.ser.reset_input_buffer()
            self.ser.write(frame)
            resp = self.ser.read(9)

        if len(resp) == 0:
            return False

        ok = (len(resp) >= 8 and
              resp[0] == 0x90 and
              resp[4] == 0x12 and
              resp[7] == 0x01)
        return ok

    def _read_angle_actual(self):
        if not self.ser or not self.ser.is_open:
            return None

        frame = build_read_frame(self.hand_id, REG_ANGLE_ACT, 26)

        acquired = self.serial_lock.acquire(timeout=0.05)
        if not acquired:
            return None
        try:
            self.ser.reset_input_buffer()
            self.ser.write(frame)
            resp = self.ser.read(7 + 26 + 1)
        finally:
            self.serial_lock.release()

        if len(resp) < 34 or resp[0] != 0x90 or resp[4] != 0x11:
            return None
        return parse_int16_list(resp[7:33], 13)

    # ── 控制函式 ──────────────────────────────────────

    def send_angles(self, angles_13: list) -> bool:
        if len(angles_13) != 13:
            return False

        validated = []
        for i, (name, val) in enumerate(zip(DOF_NAMES, angles_13)):
            if val == -1:
                validated.append(-1)
                continue
            lo, hi = ANGLE_RANGE[name]
            if not (lo <= val <= hi):
                validated.append(-1)
            else:
                validated.append(val)

        data = []
        for v in validated:
            data.extend(int16_to_bytes(v))
        return self._write_registers(REG_ANGLE_SET, data)

    def send_gesture(self, name: str) -> bool:
        if name not in GESTURES:
            return False
        return self.send_angles(GESTURES[name])

    def set_speed(self, speed: int) -> bool:
        speed = max(400, min(2500, speed))
        data  = []
        for _ in range(13):
            data.extend(int16_to_bytes(speed))
        return self._write_registers(REG_SPEED_SET, data)

    # ── 背景角度讀取（加快到 0.05s）─────────────────────

    def start_reader(self, interval: float = 0.05):
        self.read_running   = True
        self._reader_thread = threading.Thread(
            target=self._reader_worker,
            args=(interval,),
            daemon=True
        )
        self._reader_thread.start()

    def _reader_worker(self, interval: float):
        while self.read_running:
            result = self._read_angle_actual()
            if result:
                self.current_angles = result
            time.sleep(interval)

    def stop_reader(self):
        self.read_running = False
        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1)

    def wait_for_angles(self, timeout: float = 2.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.current_angles:
                return True
            time.sleep(0.05)
        return False

# ══════════════════════════════════════════
#  即時顯示（ANSI 游標控制，原地刷新）
# ══════════════════════════════════════════

DISPLAY_LINES = 16  # header(2) + 欄位標題(1) + 分隔(1) + 13 關節

def render_angles(hand):
    """產生整個顯示區塊的字串，一次性輸出"""
    angles = hand.current_angles
    lines = []
    lines.append(f"  [{hand.label}] 📐 當前角度  [{time.strftime('%H:%M:%S')}]")
    lines.append(f"  {'#':<3} {'自由度':<14} {'原始值':>6}  進度")
    lines.append("  " + "─" * 48)

    if angles is None:
        for _ in range(13):
            lines.append("  ---")
    else:
        for i, (name, val) in enumerate(zip(DOF_NAMES, angles)):
            lo, hi = ANGLE_RANGE[name]
            ratio  = max(0.0, min(1.0, (val - lo) / (hi - lo) if hi != lo else 0))
            filled = int(ratio * 20)
            bar    = "█" * filled + "░" * (20 - filled)
            lines.append(f"  {i:<3} {name:<14} {val:>6}  [{bar}]")

    return "\n".join(lines)

def display_thread_fn(hand, stop_event):
    # 先印出佔位空行，讓游標有地方往上跳
    print("\n" * DISPLAY_LINES, end="")
    sys.stdout.flush()

    while not stop_event.is_set():
        block = render_angles(hand)
        # 游標上移 DISPLAY_LINES 行，覆寫整個區塊
        sys.stdout.write(f"\033[{DISPLAY_LINES}A")
        sys.stdout.write(block)
        sys.stdout.write("\n")   # 最後一行換行，讓輸入列保持在下方
        sys.stdout.flush()
        time.sleep(0.05)

# ══════════════════════════════════════════
#  指令解析
# ══════════════════════════════════════════

def parse_command(cmd: str, hand):
    parts = cmd.strip().split()
    if not parts:
        return

    token = parts[0].lower()

    if token == "help":
        print_help()

    elif token == "speed":
        if len(parts) == 2:
            try:
                spd = int(parts[1])
                ok  = hand.set_speed(spd)
                print(f"  {'✅' if ok else '❌'} [{hand.label}] 速度設為 {spd}")
            except ValueError:
                print("  ⚠️  speed 後請接數字，例如：speed 1500")
        else:
            print("  ⚠️  用法：speed <值>")

    elif token in GESTURES:
        ok = hand.send_gesture(token)
        print(f"  {'✅' if ok else '❌'} [{hand.label}] 手勢：{token}")

    else:
        try:
            angles = [int(x.strip()) for x in cmd.split(',')]
            if len(angles) != 13:
                print(f"  ⚠️  需要 13 個值，目前輸入 {len(angles)} 個")
                return
            ok = hand.send_angles(angles)
            print(f"  {'✅' if ok else '❌'} [{hand.label}] 角度指令已送出")
        except ValueError:
            print(f"  ⚠️  無法識別指令：{cmd}（輸入 help 查看說明）")

# ══════════════════════════════════════════
#  說明
# ══════════════════════════════════════════

def print_help():
    print("""
  ┌──────────────────────────────────────────────────────────────┐
  │  指令說明                                                     │
  │                                                              │
  │  角度控制（13 個值，逗號分隔，-1 = 不動）                     │
  │  > 1200,1200,1200,0,1200,0,1500,1500,1500,1500,900,1400,1600 │
  │                                                              │
  │  手勢控制                                                    │
  │  > open   張開    close  握拳    ya    比耶                   │
  │  > ok     OK      666    666     rock  搖滾    card  藏牌     │
  │                                                              │
  │  其他                                                        │
  │  > speed <值>   設置速度（400-2500）                          │
  │  > help         顯示此說明                                   │
  │  > quit         離開                                         │
  └──────────────────────────────────────────────────────────────┘
""")

# ══════════════════════════════════════════
#  主程式
# ══════════════════════════════════════════

def main():
    print("╔══════════════════════════════════════════╗")
    print("║   RH5DG2 靈巧手  控制系統                 ║")
    print("╚══════════════════════════════════════════╝\n")

    # ── 修改這裡的 port ──
    hand = DexterousHand(port='/dev/ttyUSB0', hand_id=1, label='右手')

    if not hand.connect():
        print("  ❌ 無法連線，結束")
        return

    hand.set_speed(1000)
    hand.start_reader()
    print(f"  ⚙️  速度預設：1000")

    print("  ⏳ 等待角度資料...", end="", flush=True)
    hand.wait_for_angles()
    print(" OK\n")

    print_help()

    stop_event = threading.Event()
    dt = threading.Thread(
        target=display_thread_fn,
        args=(hand, stop_event),
        daemon=True
    )
    dt.start()

    try:
        while True:
            cmd = input().strip()
            if not cmd:
                continue
            if cmd.lower() == "quit":
                break
            parse_command(cmd, hand)

    except KeyboardInterrupt:
        print("\n  👋 使用者中斷")
    finally:
        stop_event.set()
        hand.disconnect()

if __name__ == "__main__":
    main()
