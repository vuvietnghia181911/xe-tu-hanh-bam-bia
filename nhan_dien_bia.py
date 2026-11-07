"""
XE BAM THEO TAM BIA (bia tron dong tam, IN DEN TRANG hoac in mau do/hong)
  Dien thoai (app DroidCam) --WiFi--> Laptop (OpenCV nhan dien bia) --UDP--> ESP32
  Dung chung code ESP32 "xe_ai_esp32.ino" (khong can nap lai)

Khong can AI, khong can train. Chi can:  pip install opencv-python numpy
Chay:
  - Dung HOTSPOT dien thoai (khuyen dung, it tre):   python nhan_dien_bia.py hotspot
  - Hoac go IP dien thoai trong DroidCam:            python nhan_dien_bia.py 192.168.4.2
  Xe tu bao dia chi cho laptop -> khong can go IP cua xe.

Cach nhan dien: tim cac vong tron (den, hoac do/hong) CO CHUNG MOT TAM.
  Vat thuong khong co nhieu vong tron dong tam nen it bi nham.
  Mac dinh: bia DEN TRANG (BIA_MAU = False). Dung bia mau thi doi BIA_MAU = True.

Phim:
  Chuot trai vao vong do cua bia : hoc lai mau bia (chi dung khi BIA_MAU = True)
  C     : ve lai mau mac dinh (chi dung khi BIA_MAU = True)
  M     : bat/tat cua so "Mask" (trang = phan mau bia)
  SPACE : tam dung / chay tiep
  R     : reset xe (khi xe CO LAP hoac da KHOA MUC TIEU)
  P     : kiem tra ket noi toi xe
  Q     : thoat (xe dung lai)
"""

import math
import socket
import subprocess
import sys
import threading
import time

import cv2
import numpy as np

# ================== CAU HINH ==================
CAM_URL = "http://192.168.4.2:4747/video"   # dung khi khong go gi trong lenh chay
ESP32_IP = "auto"       # "auto" = tu tim xe (xe tu bao dia chi) | hoac go IP co dinh, vd "192.168.4.1"
ESP32_PORT = 4210
HELLO_PORT = 4211       # cong xe bao dia chi, giong HELLO_PORT trong code ESP32

car_ip = None if ESP32_IP == "auto" else ESP32_IP

FLIP = False            # True neu dung camera truoc
FRAME_WIDTH = 640
SEND_HZ = 15
CENTER_TOL = 0.15       # chi de ve, giong CENTER_TOL trong code ESP32

BIA_MAU = False         # False = bia in DEN TRANG (mac dinh) | True = bia mau do/hong

# --- Bia den trang ---
DARK_MAX = 80           # diem toi hon muc nay (0..255) coi la mau den cua bia
ADAPT_BLOCK = 41        # vung so sanh sang/toi cuc bo (so le) -> chiu duoc anh sang khong deu
ADAPT_C = 12            # toi hon trung binh xung quanh bao nhieu thi coi la den

# --- Bia mau: do / hong (mau do nam o 2 dau thang mau H cua OpenCV) ---
DEFAULT_RANGES = [((0, 70, 40), (10, 255, 255)), ((150, 70, 40), (179, 255, 255))]
H_TOL, S_TOL, V_TOL = 12, 80, 80   # do rong khoang mau khi bam chuot hoc mau

# Can it nhat bao nhieu vong dong tam (cham giua cung tinh 1 vong).
# Bia den trang can 3 vong de khong nham banh xe, mat kinh, loa...
MIN_RINGS = 2 if BIA_MAU else 3
MIN_COMP_PX = 25        # bo qua vung mau nho hon (pixel) -> loc nhieu
RING_THICK = 0.15       # vong phai "mong": do lech khoang cach / khoang cach trung binh < muc nay
MIN_COVERAGE = 0.35     # vong (hoac cung tron) phai phu it nhat 35% vong quanh tam -> bi che 1 phan van nhan
HOLD_FRAMES = 3         # mat bia vai khung hinh van giu ket qua cu (chong nhap nhay)

ranges = list(DEFAULT_RANGES)
last_hsv = None


class FrameGrabber:
    """Doc video lien tuc o luong rieng, luon giu khung hinh moi nhat (tranh bi tre)."""

    def __init__(self, url):
        self.url = url
        self.cap = cv2.VideoCapture(url)
        self.frame = None
        self.ok = False
        self.lock = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while True:
            ok, frame = self.cap.read()
            if not ok:
                with self.lock:
                    self.ok = False
                time.sleep(1)
                self.cap.release()
                self.cap = cv2.VideoCapture(self.url)
                continue
            with self.lock:
                self.frame = frame
                self.ok = True

    def read(self):
        with self.lock:
            if not self.ok or self.frame is None:
                return False, None
            return True, self.frame.copy()


def default_gateway():
    """Dia chi 'cong mac dinh' cua laptop. Khi laptop vao hotspot thi day chinh la IP dien thoai."""
    try:
        out = subprocess.run(["route", "print", "0.0.0.0"], capture_output=True,
                             text=True, timeout=3).stdout
    except Exception:
        return None
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[0] == "0.0.0.0" and parts[1] == "0.0.0.0" and parts[2][0].isdigit():
            return parts[2]
    return None


def camera_source():
    if len(sys.argv) < 2:
        return CAM_URL
    arg = sys.argv[1]
    if arg.lower() == "hotspot":
        gw = default_gateway()
        if gw is None:
            print("Khong tim thay IP dien thoai. Laptop da vao hotspot chua?")
            return CAM_URL
        print("IP dien thoai (hotspot):", gw)
        return f"http://{gw}:4747/video"
    if arg.isdigit():
        return int(arg)
    if "://" in arg:
        return arg
    return f"http://{arg}:4747/video"


def open_hello_socket():
    """Mo cong nghe xe bao dia chi."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(("", HELLO_PORT))
    except OSError as e:
        print("Khong mo duoc cong tim xe:", e)
    s.setblocking(False)
    return s


def find_car(hello_sock):
    """Doc tin xe bao dia chi. Tra ve True neu vua tim thay xe (hoac xe doi dia chi)."""
    global car_ip
    found_new = False
    while True:
        try:
            data, addr = hello_sock.recvfrom(64)
        except OSError:
            return found_new
        if data.startswith(b"XE_AI_HELLO") and addr[0] != car_ip and ESP32_IP == "auto":
            car_ip = addr[0]
            print("Tim thay xe tai", car_ip)
            found_new = True


def pick_color(event, x, y, flags, param):
    """Bam chuot vao vong do cua bia -> hoc mau tai diem do."""
    global ranges
    if event != cv2.EVENT_LBUTTONDOWN or last_hsv is None:
        return
    if not BIA_MAU:
        print("Che do bia den trang: khong can hoc mau")
        return
    h, w = last_hsv.shape[:2]
    patch = last_hsv[max(0, y - 3):min(h, y + 4), max(0, x - 3):min(w, x + 4)].reshape(-1, 3)
    h0, s0, v0 = [int(v) for v in np.median(patch, axis=0)]
    smin, vmin = max(s0 - S_TOL, 50), max(v0 - V_TOL, 40)
    lo, hi = h0 - H_TOL, h0 + H_TOL
    if lo < 0:
        ranges = [((0, smin, vmin), (hi, 255, 255)), ((180 + lo, smin, vmin), (179, 255, 255))]
    elif hi > 179:
        ranges = [((lo, smin, vmin), (179, 255, 255)), ((0, smin, vmin), (hi - 180, 255, 255))]
    else:
        ranges = [((lo, smin, vmin), (hi, 255, 255))]
    print(f"Da hoc mau bia: H={h0} S={s0} V={v0}")


def dark_mask(hsv):
    """Bia den trang: lay cac diem den (toi han, hoac toi hon xung quanh)."""
    v = hsv[:, :, 2]                                   # do sang
    local = cv2.adaptiveThreshold(v, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                  cv2.THRESH_BINARY_INV, ADAPT_BLOCK, ADAPT_C)
    mask = cv2.bitwise_or(local, (v < DARK_MAX).astype(np.uint8) * 255)
    kernel = np.ones((3, 3), np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)


def color_mask(hsv):
    if not BIA_MAU:
        return dark_mask(hsv)
    mask = None
    for lo, hi in ranges:
        m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def fit_circle(xs, ys):
    """Tim duong tron khop nhat voi cac diem (dung duoc ca khi chi la mot cung tron)."""
    a = np.c_[2 * xs, 2 * ys, np.ones_like(xs)]
    b = xs * xs + ys * ys
    (cx, cy, c), *_ = np.linalg.lstsq(a, b, rcond=None)
    return float(cx), float(cy)


def get_components(mask):
    """Tach mask thanh tung vung mau rieng (moi vong do / cham giua la 1 vung)."""
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    rng = np.random.default_rng(0)
    img_h, img_w = mask.shape
    comps = []
    for i in range(1, n):
        area = stats[i, cv2.CC_STAT_AREA]
        if area < MIN_COMP_PX:
            continue
        x0, y0, w, h = stats[i, :4]
        touches_edge = x0 == 0 or y0 == 0 or x0 + w >= img_w or y0 + h >= img_h
        if touches_edge and area > 0.25 * img_w * img_h:
            continue                         # mang nen toi lon tran ra mep anh, khong phai vong bia
        ys, xs = np.nonzero(labels[y0:y0 + h, x0:x0 + w] == i)
        xs = xs.astype(np.float32) + x0
        ys = ys.astype(np.float32) + y0
        if len(xs) > 400:                    # lay mau bot cho nhanh
            idx = rng.choice(len(xs), 400, replace=False)
            xs, ys = xs[idx], ys[idx]
        comps.append((xs, ys, int(area)))
    return comps


def rings_around(cx, cy, comps):
    """Dem cac vong dong tam quanh diem (cx, cy). Tra ve (so_vong, ban_kinh, danh_sach_ban_kinh)."""
    radii = []
    for xs, ys, area in comps:
        dx, dy = xs - cx, ys - cy
        d = np.hypot(dx, dy)
        md = float(d.mean())
        own_r = math.sqrt(area / math.pi)
        if math.hypot(float(xs.mean()) - cx, float(ys.mean()) - cy) < max(4.0, 0.3 * own_r) \
                and float(d.max()) < 2.5 * own_r:
            radii.append((md, float(d.max())))           # cham tron dac o giua bia
            continue
        if md < 3 or float(d.std()) / md > RING_THICK:
            continue                                     # khong phai vong mong quanh tam nay
        bins = ((np.arctan2(dy, dx) + math.pi) / (2 * math.pi) * 24).astype(int) % 24
        if len(np.unique(bins)) / 24 < MIN_COVERAGE:
            continue                                     # cung qua ngan
        radii.append((md, float(d.max())))
    if not radii:
        return 0, 0.0, []
    radii.sort()
    rings, last = 1, radii[0][0]
    outer = [radii[0][1]]                                # mep ngoai cua tung vong (khac nhau)
    for md, rmax in radii[1:]:                           # cac manh cung 1 vong chi tinh 1 lan
        if md > last * 1.2 + 2:
            rings += 1
            last = md
            outer.append(rmax)
        else:
            outer[-1] = max(outer[-1], rmax)
    # Ban kinh bia = mep ngoai vong thu 3 (vong den ngoai cung cua bia).
    # Khong lay vong xa hon vi do co the la vien to giay -> lam bia trong to hon thuc te.
    return rings, outer[min(2, len(outer) - 1)], [md for md, _ in radii]


def detect_bia(hsv):
    """Tra ve (cx, cy, r, so_vong, cac_ban_kinh, mask) ; cx = None neu khong thay bia."""
    mask = color_mask(hsv)
    comps = get_components(mask)
    h, w = mask.shape
    candidates = []                                      # cac diem co the la tam bia
    for xs, ys, _ in comps:
        candidates.append((float(xs.mean()), float(ys.mean())))
        if len(xs) >= 20:
            fx, fy = fit_circle(xs, ys)
            if -w < fx < 2 * w and -h < fy < 2 * h:
                candidates.append((fx, fy))
    best, best_score = None, 0.0
    for cx, cy in candidates:
        rings, r, radii = rings_around(cx, cy, comps)
        if rings >= MIN_RINGS and r * rings > best_score:
            best_score = r * rings
            best = (cx, cy, r, rings, radii)
    if best is None:
        return None, None, None, 0, [], mask
    cx, cy, r, rings, radii = best
    return cx, cy, r, rings, radii, mask


def send(sock, msg):
    if car_ip is None:                       # chua tim thay xe
        return
    try:
        sock.sendto(msg.encode(), (car_ip, ESP32_PORT))
    except OSError as e:
        print("Loi gui lenh:", e)


def read_status(sock, status):
    while True:
        try:
            data, _ = sock.recvfrom(256)
        except OSError:
            return status
        text = data.decode(errors="ignore")
        if text == "SUCCESS":
            print("ESP32 tra loi: SUCCESS -> ket noi thanh cong")
        elif text == "RESET OK":
            print("ESP32: da reset")
        else:
            status = text


def draw_overlay(img, det, status, paused, fps):
    h, w = img.shape[:2]
    band = int(CENTER_TOL * w / 2)
    cv2.line(img, (w // 2, 0), (w // 2, h), (200, 200, 200), 1)
    cv2.line(img, (w // 2 - band, 0), (w // 2 - band, h), (120, 120, 120), 1)
    cv2.line(img, (w // 2 + band, 0), (w // 2 + band, h), (120, 120, 120), 1)

    if det is None:
        line1, c1 = "NO TARGET (khong thay bia)", (0, 200, 255)
    else:
        cx, cy, r, rings, radii, x, area = det
        for rr in radii:
            cv2.circle(img, (int(cx), int(cy)), int(rr), (0, 255, 0), 1)
        cv2.circle(img, (int(cx), int(cy)), int(r), (0, 255, 0), 3)
        cv2.drawMarker(img, (int(cx), int(cy)), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
        line1, c1 = f"TARGET: BIA  x={x:+.2f}  {rings} vong  {area * 100:.1f}%", (0, 255, 0)
    cv2.putText(img, line1, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, c1, 2)
    cv2.putText(img, f"Xe: {status}", (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    if paused:
        cv2.putText(img, "TAM DUNG (SPACE: chay)", (10, 79), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.putText(img, f"{fps:.0f} FPS", (w - 90, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


def main():
    global last_hsv, ranges
    cam = camera_source()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    hello_sock = open_hello_socket()
    send(sock, "PING")
    send(sock, "G")
    if car_ip is None:
        print("Dang tim xe... (xe phai cung WiFi voi laptop)")

    print("Dang ket noi camera:", cam)
    grabber = FrameGrabber(cam)
    cv2.namedWindow("Xe bam bia")
    cv2.setMouseCallback("Xe bam bia", pick_color)
    print("Chuot: hoc mau bia | C: mau mac dinh | M: mask | SPACE: dung/chay | R: reset | P: ping | Q: thoat")

    status = "dang tim xe..." if car_ip is None else "chua ket noi"
    paused = False
    show_mask = False
    last_send = 0.0
    last_det, lost_frames = None, 0
    fps, t_prev = 0.0, time.time()

    while True:
        if find_car(hello_sock):             # vua tim thay xe -> chao xe, cho xe chay
            send(sock, "PING")
            send(sock, "S" if paused else "G")
            status = "chua ket noi"
        status = read_status(sock, status)
        ok, frame = grabber.read()
        now = time.time()

        if not ok:
            blank = np.zeros((360, FRAME_WIDTH, 3), np.uint8)
            cv2.putText(blank, "Dang cho camera dien thoai...", (20, 170),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)
            cv2.putText(blank, f"Xe: {status}", (20, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.imshow("Xe bam bia", blank)
            if now - last_send >= 1 / SEND_HZ:
                send(sock, "N")
                last_send = now
            if (cv2.waitKey(100) & 0xFF) == ord("q"):
                break
            continue

        if FLIP:
            frame = cv2.flip(frame, 1)
        scale = FRAME_WIDTH / frame.shape[1]
        frame = cv2.resize(frame, (FRAME_WIDTH, int(frame.shape[0] * scale)))
        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
        last_hsv = hsv

        cx, cy, r, rings, radii, mask = detect_bia(hsv)
        if cx is not None:
            x = (cx - w / 2) / (w / 2)
            area = math.pi * r * r / (w * h)
            last_det, lost_frames = (cx, cy, r, rings, radii, x, area), 0
        else:
            lost_frames += 1
            if lost_frames > HOLD_FRAMES:
                last_det = None
        det = last_det

        if now - last_send >= 1 / SEND_HZ:
            send(sock, "N" if det is None else f"T,{det[5]:.3f},{det[6]:.4f}")
            last_send = now

        dt = now - t_prev
        t_prev = now
        if dt > 0:
            fps = fps * 0.9 + (1 / dt) * 0.1

        draw_overlay(frame, det, status, paused, fps)
        cv2.imshow("Xe bam bia", frame)
        if show_mask:
            cv2.imshow("Mask", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord(" "):
            paused = not paused
            send(sock, "S" if paused else "G")
            print("Xe:", "TAM DUNG" if paused else "CHAY")
        if key == ord("r"):
            send(sock, "R")
        if key == ord("p"):
            send(sock, "PING")
        if key == ord("c"):
            ranges = list(DEFAULT_RANGES)
            print("Da ve mau mac dinh")
        if key == ord("m"):
            show_mask = not show_mask
            if not show_mask:
                cv2.destroyWindow("Mask")

    for _ in range(5):
        send(sock, "S")
        time.sleep(0.05)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
