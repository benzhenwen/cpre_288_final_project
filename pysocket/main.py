"""
CyBot TCP client with concurrent RX/TX and live 2D map rendering.

- Text commands you type are sent line-based (like "forward 100", "turn 90", "exit").
- Incoming text lines are echoed immediately, without blocking user input.
- When a binary packet beginning with b"DATA" arrives, we parse:
    pos_x, pos_y, pos_r, target_x, target_y, target_r (6 x float32, mm/deg)
  Then we read 0..N objects, each 13 bytes:
    float32 x_mm, float32 y_mm, float32 radius_mm, uint8 type_bool
  The sequence ends with a single zero byte (0x00) sentinel. The first byte of
  the next object’s x value is guaranteed never to be 0, so the sentinel is unambiguous.

- A Pygame window shows a ±5 m square field; updates on each DATA packet.

Usage:
    python cybot_client.py --host 192.168.1.1 --port 288
"""

import argparse
import math
import socket
import struct
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional

# --- Optional: install pygame first: pip install pygame ---
import pygame


# ----------------------------- Data Models -----------------------------

@dataclass
class Object2D:
    x_mm: float
    y_mm: float
    r_mm: float
    t: int   # 0: short, 1: tal, 2: dark ir, 3: light ir


@dataclass
class WorldState:
    # Robot pos
    pos_x: Optional[float] = None
    pos_y: Optional[float] = None
    pos_r_deg: Optional[float] = None
    # Target pos
    target_x: Optional[float] = None
    target_y: Optional[float] = None
    target_r_deg: Optional[float] = None
    # Movement approach data
    move_mode_flag: Optional[bool] = False  # 0 = moving, 1 = rotating
    apprach_distance_offset: Optional[float] = None
    # Objects
    objects: List[Object2D] = field(default_factory=list)
    # Timestamp of last update
    updated_at: float = 0.0

@dataclass
class Button:
    #button data
    name: str    # the text of the button
    command: str # the command to be sent over tx


ALL_BUTTONS = [
    Button("reset_field", "!"),
    Button("oi_free", "e"),
    Button("stop", "k"),
    Button("scan", "s"),
    Button("tile step", "g"),
    Button("full start", "a"),
    Button("basic auto", "p"),
    Button("servo cal", "c"),
    Button("ir cal", "i"),
    Button("reverse", "r100"),
    Button("align turn", "t0"),
    Button("success", "v")
]


# ----------------------------- Renderer -----------------------------

class MapRenderer:
    """
    Simple Pygame renderer showing a 10m x 10m world centered at the origin.
    """
    FIELD_HALF_MM = 5000  # ±2 meters
    ROBOT_RADIUS_MM = 160

    def __init__(self, shared_state: WorldState, state_lock: threading.Lock, on_move_command):
        self.state = shared_state
        self.lock = state_lock

        pygame.init()
        self.W, self.H = 1100, 1100
        self.screen = pygame.display.set_mode((self.W, self.H))
        pygame.display.set_caption("CyBot World View (±3 m)")
        self.clock = pygame.time.Clock()
        self.scale = self.W / (2 * self.FIELD_HALF_MM)  # px per mm

        # sending commands
        self.on_move_command = on_move_command

        # Colors
        self.bg = (20, 20, 24)
        self.grid = (50, 50, 60)
        self.axis = (80, 80, 100)
        self.robot_red = (220, 60, 60)
        self.target_green = (30, 212, 54)
        self.target_dark_green = (16, 148, 62)
        self.object_blue_dark = (70, 120, 220)   
        self.object_blue_light = (120, 170, 255) 
        self.object_black = (80, 80, 80)
        self.object_white = (210, 210, 210)
        self.white = (230, 230, 230)
        self.error_color = (245, 7, 209)

        self.font = pygame.font.SysFont("consolas", 14)

    def to_screen(self, x_mm: float, y_mm: float):

        # World (mm): origin center; +x right, +y up
        # Screen: origin top-left; +x right, +y down
        sx = int(((x_mm - 1000) + self.FIELD_HALF_MM) * self.scale)
        sy = int((self.FIELD_HALF_MM - y_mm) * self.scale)
        return sx, sy

    def screen_to_world_mm(self, sx: int, sy: int):
        # Inverse of to_screen(): sx = (x + H) * scale; sy = (H - y) * scale
        x_mm = (sx / self.scale) - self.FIELD_HALF_MM + 1000
        y_mm = self.FIELD_HALF_MM - (sy / self.scale)
        return x_mm, y_mm

    def format_move_command(self, x_mm: float, y_mm: float) -> str:
        """
        Returns 'm' + 10 digits: y(5) + x(5), each zero-padded.
        Example (500,500) -> 'm0050000500'
        Only supports non-negative mm (as per your example).
        """
        xi = int(round(x_mm) + 5000)
        yi = int(round(y_mm) + 5000)

        if xi > 9999 or yi > 9999 or xi < 0 or yi < 0:
            print(f"[click] Ignored (out of bounds): x={xi} y={yi}")
            return ""

        return f"m{yi:04d}{xi:04d}\n"

    def draw_grid(self):
        self.screen.fill(self.bg)
        # minor grid every 1 m, major every 5 m
        step_mm = 1000
        for d in range(-self.FIELD_HALF_MM, self.FIELD_HALF_MM * 2 + 1, step_mm):
            # vertical lines (x = d)
            x, _ = self.to_screen(d, 0)
            pygame.draw.line(self.screen, self.grid, (x, 0), (x, self.H), 1)
            # horizontal lines (y = d)
            _, y = self.to_screen(0, d)
            pygame.draw.line(self.screen, self.grid, (0, y), (self.W, y), 1)

        # axes
        x0, y0 = self.to_screen(0, 0)
        pygame.draw.line(self.screen, self.axis, (x0, 0), (x0, self.H), 2)
        pygame.draw.line(self.screen, self.axis, (0, y0), (self.W, y0), 2)

        # border
        pygame.draw.rect(self.screen, self.white, pygame.Rect(0, 0, self.W, self.H), 2)

    def draw_robot(self, x_mm: float, y_mm: float, r_deg: float, color_outline, color_dir):
        cx, cy = self.to_screen(x_mm, y_mm)
        rr = int(self.ROBOT_RADIUS_MM * self.scale)
        pygame.draw.circle(self.screen, color_outline, (cx, cy), rr, 2)

        # heading line from center to perimeter
        theta = math.radians(r_deg)
        hx = cx + int(rr * math.cos(theta))
        hy = cy - int(rr * math.sin(theta))  # minus because screen y grows downward
        pygame.draw.line(self.screen, color_dir, (cx, cy), (hx, hy), 2)

    def draw_movement_target(self, cx_mm: float, cy_mm: float, tx_mm: float, ty_mm: float, tr_deg: float, move_mode_flag: bool, approach_mm: float, color_outline, color_dir):
        cx, cy = self.to_screen(cx_mm, cy_mm)
        tx, ty = self.to_screen(tx_mm, ty_mm)
        ar = int(self.ROBOT_RADIUS_MM * self.scale) / 2
        tr = int(approach_mm * self.scale)

        if (move_mode_flag):
            # heading line from center to perimeter
            theta = math.radians(tr_deg)
            hx = cx + int(ar * math.cos(theta))
            hy = cy - int(ar * math.sin(theta))  # minus because screen y grows downward
            pygame.draw.line(self.screen, color_dir, (cx, cy), (hx, hy), 2)

        if (approach_mm > 0):
            # the approach circle     
            pygame.draw.circle(self.screen, color_outline, (tx, ty), tr, 2)

        # heading line from the start to the target
        theta = -math.atan2(ty - cy, tx - cx)
        hx = tx - int(tr) * math.cos(theta)
        hy = ty + int(tr) * math.sin(theta)  # minus because screen y grows downward
        pygame.draw.line(self.screen, color_outline, (cx, cy), (hx, hy), 2)

    def draw_objects(self, objects: List[Object2D]):
        for obj in objects:
            sx, sy = self.to_screen(obj.x_mm, obj.y_mm)
            rr = max(1, int(obj.r_mm * self.scale))     
            color = self.error_color
            if   (obj.t == 0):
                color = self.object_blue_dark  
            elif (obj.t == 1):
                color = self.object_blue_light   
            elif (obj.t == 2):
                color = self.object_black      
            elif (obj.t == 3):
                color = self.object_white 
            pygame.draw.circle(self.screen, color, (sx, sy), rr, width= (3 if (obj.t == 3) else 0))

    def draw_hud(self):
        with self.lock:
            updated = self.state.updated_at
            pos = (self.state.pos_x, self.state.pos_y, self.state.pos_r_deg)
            tgt = (self.state.target_x, self.state.target_y, self.state.target_r_deg)
            apr = (self.state.move_mode_flag, self.state.apprach_distance_offset)
            nobj = len(self.state.objects)

        lines = [
            f"Last DATA: {time.strftime('%H:%M:%S', time.localtime(updated)) if updated else '—'}",
            f"Robot: x={pos[0]:.0f}mm y={pos[1]:.0f}mm r={pos[2]:.1f}°" if pos[0] is not None else "Robot: —",
            f"Target: x={tgt[0]:.0f}mm y={tgt[1]:.0f}mm r={tgt[2]:.1f}°" if tgt[0] is not None else "Target: —",
            f"Approach: mmf_v={apr[0]:d} apr_d={apr[1]:.0f}" if apr[1] is not None else "Approach: —",
            f"Objects: {nobj}"
        ]
        y = 6
        for s in lines:
            surf = self.font.render(s, True, self.white)
            self.screen.blit(surf, (8, y))
            y += 18
    
    def draw_buttons(self):
        button_width = 100
        button_height = 25

        current_button_y = self.H - (button_height + 5)
        
        for button in ALL_BUTTONS:
            pygame.draw.rect(self.screen, self.white, [5, current_button_y, button_width, button_height])

            surf = self.font.render(button.name, True, (0, 0, 0))
            self.screen.blit(surf, (8, current_button_y + 5))
            
            current_button_y -= button_height + 5


    def tick(self) -> bool:
        """
        One frame. Returns False if window is closing.
        """
        # Copy state under lock
        with self.lock:
            pos_x, pos_y, pos_r = self.state.pos_x, self.state.pos_y, self.state.pos_r_deg
            tgt_x, tgt_y, tgt_r = self.state.target_x, self.state.target_y, self.state.target_r_deg
            mmf_v = self.state.move_mode_flag
            tgt_apr_dist = self.state.apprach_distance_offset
            objects = list(self.state.objects)

        # button press detection
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1: # move to (also overrides button presses)
                mx, my = event.pos

                # check for button press first
                button_width = 100
                button_height = 25

                current_button_y = self.H - (button_height + 5)

                button_pressed = False
                for button in range(len(ALL_BUTTONS)):
                    if (mx > 5 and mx < button_width and my > current_button_y and my < current_button_y + button_height):
                        try:
                            self.on_move_command(ALL_BUTTONS[button].command + "\n")
                            print(f"[button] Sent: {ALL_BUTTONS[button].command}")
                        except Exception as e:
                            print(f"[button] send failed: {e}")
                        
                        button_pressed = True
                        break
                        
                    current_button_y -= button_height + 5

                if not button_pressed: # movement
                    # turn it into a field move command
                    x_mm, y_mm = self.screen_to_world_mm(mx, my)

                    # Clamp to the field bounds (±2000 mm)
                    x_mm = max(-self.FIELD_HALF_MM, min(self.FIELD_HALF_MM, x_mm))
                    y_mm = max(-self.FIELD_HALF_MM, min(self.FIELD_HALF_MM, y_mm))

                    cmd = self.format_move_command(x_mm, y_mm)
                    if cmd:
                        try:
                            # Send exactly the 11-byte ASCII command (no newline unless your bot expects it)
                            # If your bot wants a newline, append '\n' here.
                            self.on_move_command(cmd)
                            print(f"[click] Sent move: {cmd}")
                        except Exception as e:
                            print(f"[click] send failed: {e}")
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 3: # rotate to
                mx, my = event.pos
                # turn it into a field move command
                x_mm, y_mm = self.screen_to_world_mm(mx, my)

                # Clamp to the field bounds (±2000 mm)
                x_mm = max(-self.FIELD_HALF_MM, min(self.FIELD_HALF_MM, x_mm))
                y_mm = max(-self.FIELD_HALF_MM, min(self.FIELD_HALF_MM, y_mm))

                target_angle = round(math.atan2(y_mm - pos_y, x_mm - pos_x)  * (180 / math.pi) - pos_r)
                cmd = f"t{target_angle}"
                try:
                    # Send the turn command
                    self.on_move_command(cmd + "\n")
                    print(f"[click] Sent rotate: {cmd}")
                except Exception as e:
                    print(f"[click] send failed: {e}")

        self.draw_grid()

        if tgt_x is not None:
            self.draw_movement_target(pos_x, pos_y, tgt_x, tgt_y, tgt_r or 0.0, mmf_v, tgt_apr_dist, self.target_green, self.target_dark_green)
        if pos_x is not None:
            self.draw_robot(pos_x, pos_y, pos_r or 0.0, self.robot_red, self.robot_red)

        self.draw_objects(objects)
        self.draw_hud()
        self.draw_buttons()

        pygame.display.flip()
        self.clock.tick(30)  # ~30 FPS
        return True

    def shutdown(self):
        pygame.quit()


# ----------------------------- Networking -----------------------------

class CyBotClient:
    """
    Handles the TCP socket, incoming stream parsing (text lines vs DATA packets),
    and user command sending.
    """

    def __init__(self, host: str, port: int, state: WorldState, state_lock: threading.Lock):
        self.host = host
        self.port = port
        self.state = state
        self.lock = state_lock

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5.0)
        self.sock.connect((host, port))
        # after connected, use blocking for simplicity
        self.sock.settimeout(None)

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._tx_thread = threading.Thread(target=self._tx_loop, daemon=True)
        self._stop = threading.Event()

        # Buffer for text parsing
        self._rx_buf = bytearray()

        print(f"[info] Connected to {host}:{port}")
        print("Type commands like: forward 100 | reverse 50 | turn 90 | exit")

    def start(self):
        self._rx_thread.start()
        self._tx_thread.start()

    def stop(self):
        self._stop.set()
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        self.sock.close()

    # ---------- TX (user input) ----------
    def _tx_loop(self):
        while not self._stop.is_set():
            try:
                user = input().strip()
            except EOFError:
                user = "exit"

            if not user:
                continue

            if user in ("exit", "quit"):
                # Optional: tell server we're exiting (as in your sample)
                try:
                    self.sock.sendall(b"e\n")
                except Exception:
                    pass
                self._stop.set()
                break

            # Map command header like your original program
            parts = user.split()
            header = parts[0].lower()
            if header in ("drive", "forward"):
                ch = "f"
            elif header == "reverse":
                ch = "r"
            elif header == "turn":
                ch = "t"
            elif header in ("terminate", "kill"):
                ch = "k"
            elif header == "auto":
                ch = "a"
            elif header == "scan":
                ch = "s"
            elif header in ("pathing", "fa", "fullauto"):
                ch = "p"
            elif header == "calibrate":
                ch = "c"
            elif header == "ping":
                ch = "*"
            else:
                ch = header[:1]

            value = "100"
            if len(parts) > 1 and parts[1].lstrip("+-").isdigit():
                value = parts[1]

            msg = f"{ch}{value}\n".encode()
            try:
                self.sock.sendall(msg)
                print(f"[tx] sent: {msg.decode().rstrip()}")
            except Exception as e:
                print(f"[tx] send failed: {e}")
                self._stop.set()
                break

    # ---------- RX (mixed text + DATA) ----------
    def _rx_loop(self):
        """
        Mixed-stream parser:
        - Text lines (ending with '\n'): print as [rx] ...
        - If buffer starts with b'DATA', switch to binary packet parser (blocking reads).
        """
        while not self._stop.is_set():
            try:
                chunk = self.sock.recv(4096)
                if not chunk:
                    print("[rx] connection closed by peer")
                    self._stop.set()
                    break
                self._rx_buf.extend(chunk)

                # Process as much as possible
                while True:
                    if self._rx_buf.startswith(b"DATA"):
                        # Consume header and parse binary packet directly from socket
                        del self._rx_buf[:4]
                        self._parse_data_packet()
                        # after parsing, continue to scan any remaining buffered text/data
                        continue

                    # Otherwise, look for a newline-delimited text line
                    nl = self._rx_buf.find(b"\n")
                    if nl != -1:
                        line = self._rx_buf[:nl + 1]
                        del self._rx_buf[:nl + 1]
                        try:
                            s = line.decode(errors="replace").rstrip()
                        except Exception:
                            s = repr(line)
                        # Show line without interfering with input; just print on its own row
                        print(f"\n[rx] {s}")
                    else:
                        # Need more bytes to decide
                        break

            except Exception as e:
                print(f"[rx] error: {e}")
                self._stop.set()
                break

    def _recv_exact(self, n: int) -> bytes:
        """
        Blocking read for exactly n bytes from socket.
        Raises if connection drops.
        """
        out = bytearray()
        # First use any buffered bytes we already have
        if self._rx_buf:
            take = min(n, len(self._rx_buf))
            out += self._rx_buf[:take]
            del self._rx_buf[:take]

        while len(out) < n:
            chunk = self.sock.recv(n - len(out))
            if not chunk:
                raise ConnectionError("socket closed while reading")
            out += chunk
        return bytes(out)

    def _recv_one(self) -> int:
        """
        Receive a single byte and return its int value (0..255).
        """
        b = self._recv_exact(1)
        return b[0]

    def _parse_data_packet(self):
        """
        After 'DATA' has already been consumed. Packet body:
          7 floats: pos_x, pos_y, pos_r, tgt_x, tgt_y, tgt_r, approach_dist (little-endian)
          1 char: move_mode_flag (bool)
          1 char: contains N - number of objects
          followed by N objects, each 13 bytes:
            4-byte float x, 4-byte float y, 4-byte float r, 1-byte bool type
          terminated by a single 0x00 sentinel byte. (First byte of next x is never 0.)
        """
        try:
            # 7 floats (robot + target + approach dist), little-endian
            head = self._recv_exact(7 * 4)
            pos_x, pos_y, pos_r, tgt_x, tgt_y, tgt_r, tgt_apr_dist = struct.unpack("<fffffff", head)

            # 1 char move mode flag
            move_mode_flag_byte = self._recv_exact(1)
            mmf_v = (move_mode_flag_byte != 0)

            # 1-byte object count (0..16)
            count_byte = self._recv_exact(1)
            obj_count = count_byte[0]

            if obj_count != 111:
                if obj_count > 64:
                    # Bail out of this packet to avoid desync.
                    raise ValueError(f"Protocol error: object_count={obj_count} (>16)")

                # Read the whole objects region in a single call
                block = self._recv_exact(obj_count * 13)

                # Parse objects
                objects: List[Object2D] = []
                offset = 0
                for _ in range(obj_count):
                    rec = block[offset:offset+13]
                    x, y, r = struct.unpack("<fff", rec[:12])
                    type_obj = rec[12]
                    objects.append(Object2D(x_mm=x, y_mm=y, r_mm=r, t=type_obj))
                    offset += 13

            # Commit to shared state
            with self.lock:
                self.state.pos_x = pos_x
                self.state.pos_y = pos_y
                self.state.pos_r_deg = pos_r
                self.state.target_x = tgt_x
                self.state.target_y = tgt_y
                self.state.target_r_deg = tgt_r
                self.state.move_mode_flag = mmf_v
                self.state.apprach_distance_offset = tgt_apr_dist
                if obj_count != 111:
                    self.state.objects = objects
                self.state.updated_at = time.time()

            # print(f"\n[data] robot=({pos_x:.1f},{pos_y:.1f},{pos_r:.1f}°)  "
            #       f"target=({tgt_x:.1f},{tgt_y:.1f},{tgt_r:.1f}°)  "
            #       f"objects={len(objects)}")

        except Exception as e:
            print(f"[data] parse error: {e}")


# ----------------------------- Main -----------------------------

def main():
    parser = argparse.ArgumentParser(description="CyBot TCP client with live map")
    parser.add_argument("--host", default="192.168.1.1")
    parser.add_argument("--port", type=int, default=288)
    args = parser.parse_args()

    state = WorldState()
    lock = threading.Lock()
    client = CyBotClient(args.host, args.port, state, lock)

    renderer = MapRenderer(state, lock, on_move_command=lambda s: client.sock.sendall(s.encode()))

    try:
        client.start()
        # GUI loop in main thread
        running = True
        while running and not client._stop.is_set():
            running = renderer.tick()
        client.stop()
    finally:
        renderer.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[info] Interrupted by user")