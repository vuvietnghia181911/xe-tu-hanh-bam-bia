/*
  =====================================================
   XE TỰ HÀNH KẾT HỢP AI NHẬN DIỆN VÀ BÁM THEO MỤC TIÊU
   (theo thuật toán trong video của HaiTech Lab)

   - ESP32 tự phát WiFi "XE_AI", điện thoại + laptop kết nối vào
   - Điện thoại (DroidCam) -> laptop (Python + YOLOv8) -> gửi lệnh UDP về xe
   - CHƯA THẤY MỤC TIÊU: chế độ tự hành mặc định (né vật cản)
       gặp vật -> dừng, quét 4 hướng (phải, chéo phải, chéo trái, trái)
               -> có hướng trống: quay đúng về hướng thoáng nhất
                  (2 hướng gần bằng nhau thì ưu tiên bên trái)
               -> bị chặn hết: lùi xa, quét lại; vẫn kín thì quay đầu
               -> quay đầu nhiều lần liền vẫn kẹt: dừng, báo CÔ LẬP (bấm R để reset)
   - THẤY MỤC TIÊU: laptop điều khiển
       quay cho tâm mục tiêu trùng tâm camera -> chạy thẳng tới
       còn cách 15 cm -> dừng, báo KHÓA MỤC TIÊU

   ESP32 DevKit V1 (đế mở rộng) + L298N + HC-SR04 + SG90
   Arduino IDE + "esp32 by Espressif" bản 3.x | Serial Monitor: 115200
  =====================================================
   Chân giữ nguyên như xe hiện tại:
   L298N:   ENA-D32  IN1-D33  IN2-D25  IN3-D26  IN4-D27  ENB-D14
   HC-SR04: Trig-D23  Echo-D22 (qua cầu phân áp 1k/2k)  VCC-V  GND-G
   Servo:   hàng D19 (nâu-G, đỏ-V, cam-S)

   Lệnh từ laptop (UDP cổng 4210):
     "T,x,area"  thấy mục tiêu (x: -1 trái .. 0 giữa .. +1 phải, area: 0..1)
     "N"         không thấy mục tiêu
     "PING"      kiểm tra kết nối -> xe trả lời "SUCCESS"
     "R"         reset khi xe bị cô lập / đã khóa mục tiêu
     "S" / "G"   tạm dừng / chạy tiếp
*/

#include <WiFi.h>
#include <WiFiUdp.h>

// ================== WIFI ==================
// true  = xe kết nối vào HOTSPOT của điện thoại (KHUYÊN DÙNG: hình mượt, ít trễ)
// false = xe tự phát WiFi "XE_AI" (video phải đi qua ESP32 nên dễ bị trễ)
const bool USE_PHONE_HOTSPOT = true;
const char* HOTSPOT_SSID = "XeAi";      // tên điểm phát WiFi của điện thoại
const char* HOTSPOT_PASS = "07022006";  // mật khẩu điểm phát

const char* AP_SSID = "XE_AI";     // dùng khi USE_PHONE_HOTSPOT = false (hoặc không vào được hotspot)
const char* AP_PASS = "12345678";  // ít nhất 8 ký tự
const int AP_CHANNEL = 6;          // bị nhiễu, giật thì đổi sang 1 hoặc 11
const int UDP_PORT = 4210;
const int HELLO_PORT = 4211;  // xe tự báo địa chỉ cho laptop qua cổng này

// ================== CHÂN ==================
const int ENA = 32, IN1 = 33, IN2 = 25;  // bên trái
const int IN3 = 26, IN4 = 27, ENB = 14;  // bên phải
const int TRIG_PIN = 23;
const int ECHO_PIN = 22;
const int SERVO_PIN = 19;
const int LED_PIN = 2;  // sáng = đang bám mục tiêu

// ================== TỐC ĐỘ (0..255) ==================
int SPEED_FORWARD = 100;    // chạy tự hành
int SPEED_APPROACH = 100;   // đi thẳng tới mục tiêu (sau khi đã ngắm xong)
int SPEED_SLOW = 90;        // gần mục tiêu (< 50 cm) thì chậm lại
int SPEED_ALIGN = 220;      // nhịp quay tại chỗ khi ngắm (xe 4 bánh quay tại chỗ cần lực mạnh)
int SPEED_TURN = 220;       // rẽ khi né vật cản (quay tại chỗ cần lực mạnh, thấp quá chỉ nhích)
int TURN_KICK_SPEED = 250;  // mồi riêng cho động tác rẽ (mạnh hơn mồi thường)
int TURN_KICK_MS = 120;
int SPEED_BACK = 140;  // lùi (thấp quá thì xe lùi rất ngắn hoặc đứng bánh)
int KICK_SPEED = 140;  // mồi lúc khởi động cho bánh dễ lăn
int KICK_MS = 90;

// ================== KHOẢNG CÁCH (cm) ==================
const int OBSTACLE_CM = 30;      // tự hành: vật gần hơn mức này -> dừng nhìn 2 bên
const int CLEAR_CM = 40;         // bên nào xa hơn mức này = trống
const int TARGET_STOP_CM = 20;   // bám mục tiêu: cảm biến thấy bia gần hơn mức này thì dừng, khóa mục tiêu
const float ARRIVE_AREA = 0.30;  // bia chiếm >= 30% khung hình = đã tới sát -> dừng (kể cả cảm biến không thấy bia)
const float SLOW_AREA = 0.10;    // bia chiếm >= 10% khung hình (khoảng 60 cm) -> chạy chậm
const int SLOW_ZONE_CM = 60;     // cảm biến thấy vật gần hơn mức này -> chạy chậm
const int BRAKE_MS = 120;        // phanh khi tới mục tiêu (không bị trôi)
const int LOCK_RELEASE_CM = 35;  // đã khóa mà mục tiêu đi xa hơn mức này -> bám tiếp

// ================== CAMERA ==================
const float CENTER_TOL = 0.15;                // dùng khi đã khóa mục tiêu
const unsigned long TARGET_TIMEOUT_MS = 700;  // lâu không có tin mục tiêu -> coi như mất
const unsigned long LOST_WAIT_MS = 1500;      // vừa mất mục tiêu -> DỪNG CHỜ (không chạy thẳng bừa)
const unsigned long LOCK_HOLD_MS = 2000;      // đã khóa mà mất mục tiêu lâu -> về tự hành

// ---- DỪNG - NGẮM - ĐI ----
const float AIM_TOL = 0.15;         // NGẮM: |x| nhỏ hơn mức này = bia đã ở giữa
const int AIM_CONFIRM = 2;          // NGẮM: bia ở giữa đủ 3 tin liên tiếp mới coi là ngắm xong
const int AIM_WAIT_MS = 350;        // NGẮM: sau mỗi nhịp quay đứng chờ cho camera gửi hình MỚI (trễ nhiều thì tăng)
const int AIM_PULSE_MIN_MS = 60;    // nhịp quay ngắn nhất
const int AIM_PULSE_GAIN_MS = 200;  // bia lệch càng nhiều nhịp quay càng dài
const float GO_TOL = 0.3;           // ĐI: bia lệch quá mức này -> dừng lại ngắm lại
const int GO_BAD_COUNT = 2;         // ĐI: lệch đủ 2 tin liên tiếp mới dừng (chống nhiễu)

// ---- VẬT CẢN KHI ĐANG ĐI TỚI BIA ----
const float LOCK_MIN_AREA = 0.08;   // bia chiếm >= 8% khung hình = bia ở gần -> vật trước mặt CHÍNH LÀ bia
const int TRACK_OBSTACLE_CM = 25;   // đang đi tới bia mà có vật gần hơn mức này (không phải bia) -> vòng qua
const int DETOUR_ANGLE = 60;        // vòng qua vật cản: quay lệch bao nhiêu độ
const int DETOUR_MS = 900;          // chạy bao lâu để vượt qua vật cản

// ---- TÌM LẠI MỤC TIÊU ----
const int SEARCH_STEP_MS = 60;      // mỗi nhịp quay tìm (cộng thêm thời gian mồi khi quay)
const int SEARCH_WAIT_MS = 450;     // sau mỗi nhịp đứng chờ camera gửi hình mới
const int SEARCH_MAX_STEPS = 10;    // quay khoảng 1 vòng vẫn không thấy bia -> về tự hành

// ================== NÉ VẬT CẢN ==================
const int TURN90_MS = 380;                      // thời gian quay ~90 độ (chỉnh trên sàn thật)
const int TURN_STEP_MS = 150;                   // rẽ xong mà phía trước chưa trống -> quay thêm từng nhịp này
const int TURN_EXTRA_STEPS = 6;                 // quay thêm tối đa mấy nhịp
const int BACK_SHORT_MS = 250;                  // lùi ngắn khi vật quá sát (lấy chỗ để quay)
const int BACK_LONG_MS = 1200;                  // bị chặn hết: lần lùi xa đầu tiên
const int BACK_STEP_MS = 500;                   // mỗi lần sau lùi xa thêm chừng này (1200 -> 1700 ...)
const int BACK_IF_CLOSE_CM = 20;                // vật gần hơn mức này mới lùi trước khi quay
const int MAX_BACK_TRIES = 2;                   // bị chặn hết: lùi xa + quét lại tối đa mấy lần rồi quay đầu
const int LOOK_MS = 350;                        // chờ servo quay tới nơi
const int TIE_CM = 15;                          // 2 hướng chênh nhau ít hơn mức này = coi như bằng nhau -> ưu tiên trái
const int ISOLATE_TURNS = 3;                    // quay đầu 3 lần...
const unsigned long ISOLATE_WINDOW_MS = 20000;  // ...trong 20 giây vẫn kẹt -> CÔ LẬP

const int SERVO_CENTER = 90;
// Các hướng quét khi gặp vật: phải, chéo phải, chéo trái, trái
const int SCAN_ANGLES[] = { 30, 60, 120, 150 };
const int SCAN_N = 4;

const bool SERVO_REVERSED = false;  // lệnh nhìn phải mà servo nhìn trái -> true
const bool INVERT_LEFT = false;     // bên trái quay ngược khi đi tiến -> true
const bool INVERT_RIGHT = false;    // bên phải quay ngược khi đi tiến -> true
const bool SWAP_SIDES = true;       // lệnh quay TRÁI mà xe quay PHẢI (đi thẳng vẫn đúng) -> true

// ================== TRẠNG THÁI ==================
enum Mode { PAUSED,
            AUTO,
            CONTROL,
            LOCKED,
            ISOLATED,
            SEARCH };
Mode mode = AUTO;

WiFiUDP udp;
IPAddress laptopIP;
uint16_t laptopPort = 0;
bool hasLaptop = false;
bool paused = false;
bool targetSeen = false;
float targetX = 0;
float targetArea = 0;
unsigned long lastTargetMsg = 0;
unsigned long lastSeenTime = 0;
bool moving = false;
bool lastTurnRight = false;
unsigned long turnAroundTimes[ISOLATE_TURNS] = { 0 };
int turnAroundIdx = 0;

// Trạng thái DỪNG - NGẮM - ĐI
enum AimState { AIMING,
                GOING };
AimState aimState = AIMING;
unsigned long msgCount = 0;      // số tin mục tiêu đã nhận (để biết có tin MỚI không)
unsigned long lastEvalMsg = 0;   // tin cuối cùng đã dùng để quyết định
unsigned long aimWaitUntil = 0;  // đang chờ hình mới đến thời điểm này
int centeredCount = 0;
int goBadCount = 0;

// Trạng thái TÌM LẠI MỤC TIÊU
bool searchRight = true;              // quay tìm về phía nào
int searchSteps = 0;                  // đã quay mấy nhịp
unsigned long searchWaitUntil = 0;    // đang chờ camera đến thời điểm này

// Bắt đầu (lại) bước NGẮM: dừng xe, chờ hình mới rồi mới tính
void resetFrontFast();
void startAiming() {
  resetFrontFast();
  aimState = AIMING;
  centeredCount = 0;
  goBadCount = 0;
  aimWaitUntil = millis() + AIM_WAIT_MS;
  lastEvalMsg = msgCount;
}

const char* modeName(Mode m) {
  switch (m) {
    case PAUSED: return "TAM DUNG";
    case AUTO: return "TU HANH (khong co muc tieu)";
    case CONTROL: return "BAM MUC TIEU";
    case LOCKED: return "KHOA MUC TIEU";
    case ISOLATED: return "CO LAP - bam R de reset";
    case SEARCH: return "TIM LAI MUC TIEU";
  }
  return "?";
}

// ================== GỬI TRẠNG THÁI CHO LAPTOP ==================
void sendStatus(const char* s) {
  if (!hasLaptop) return;
  udp.beginPacket(laptopIP, laptopPort);
  udp.print(s);
  udp.endPacket();
}

void setMode(Mode m) {
  if (m == mode) return;
  if (m == CONTROL) startAiming();  // vừa thấy mục tiêu -> bắt đầu bằng bước NGẮM
  mode = m;
  Serial.printf("[%s]\n", modeName(m));
  sendStatus(modeName(m));
  digitalWrite(LED_PIN, (m == CONTROL || m == LOCKED) ? HIGH : LOW);
}

// Bắt đầu TÌM LẠI MỤC TIÊU: quay từng nhịp về phía dự đoán, mỗi nhịp đứng chờ camera
void startSearch(bool toRight) {
  searchRight = toRight;
  searchSteps = 0;
  searchWaitUntil = millis() + SEARCH_WAIT_MS;   // nhìn trước 1 lần rồi mới quay
  setMode(SEARCH);
  Serial.printf("[TIM] Tim lai bia, quay ve phia %s\n", toRight ? "PHAI" : "TRAI");
}

// ================== ĐỘNG CƠ ==================
void setMotor(int inA, int inB, int pwmPin, int speed, bool invert) {
  if (invert) speed = -speed;
  speed = constrain(speed, -255, 255);
  digitalWrite(inA, speed > 0 ? HIGH : LOW);
  digitalWrite(inB, speed < 0 ? HIGH : LOW);
  ledcWrite(pwmPin, abs(speed));
}

void drive(int left, int right) {  // âm = lùi
  if (SWAP_SIDES) {
    int t = left;
    left = right;
    right = t;
  }  // dây động cơ 2 bên bị cắm đổi chỗ
  setMotor(IN1, IN2, ENA, left, INVERT_LEFT);
  setMotor(IN3, IN4, ENB, right, INVERT_RIGHT);
}

void startMove(int left, int right) {  // có mồi khởi động
  drive(left > 0 ? KICK_SPEED : (left < 0 ? -KICK_SPEED : 0),
        right > 0 ? KICK_SPEED : (right < 0 ? -KICK_SPEED : 0));
  delay(KICK_MS);
  drive(left, right);
  moving = true;
}

void go(int left, int right) {
  if (!moving) startMove(left, right);
  else drive(left, right);
}

void stopCar() {
  drive(0, 0);
  moving = false;
}

// Phanh chủ động: nối tắt 2 đầu động cơ qua L298N để xe dừng ngay, không trôi
void brake(int ms) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
  ledcWrite(ENA, 255);
  ledcWrite(ENB, 255);
  delay(ms);
  stopCar();
}

void goBack(int ms) {
  startMove(-SPEED_BACK, -SPEED_BACK);
  delay(ms);
  stopCar();
  delay(150);
}
// Quay tại chỗ, có mồi mạnh riêng để chắc chắn xoay được
void spin(bool toRight, int ms) {
  int s = toRight ? 1 : -1;
  drive(s * TURN_KICK_SPEED, -s * TURN_KICK_SPEED);
  delay(TURN_KICK_MS);
  drive(s * SPEED_TURN, -s * SPEED_TURN);
  delay(ms);
  stopCar();
  delay(150);
}
void turnRight(int ms) {
  spin(true, ms);
}
void turnLeft(int ms) {
  spin(false, ms);
}

// ================== SERVO ==================
void servoWrite(int angle) {
  angle = constrain(angle, 0, 180);
  if (SERVO_REVERSED) angle = 180 - angle;
  uint32_t us = map(angle, 0, 180, 500, 2400);
  ledcWrite(SERVO_PIN, us * 65535UL / 20000UL);
}

// ================== CẢM BIẾN SIÊU ÂM ==================
float pingOnce() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long t = pulseIn(ECHO_PIN, HIGH, 25000UL);
  if (t == 0) return 400;  // không có sóng dội -> xa
  return t * 0.0343 / 2.0;
}

float readDistance() {  // đo 3 lần, lấy giá trị giữa
  float a = pingOnce();
  delay(25);
  float b = pingOnce();
  delay(25);
  float c = pingOnce();
  if (a > b) {
    float x = a;
    a = b;
    b = x;
  }
  if (b > c) {
    float x = b;
    b = c;
    c = x;
  }
  if (a > b) {
    float x = a;
    a = b;
    b = x;
  }
  return b;
}

float lookAt(int angle) {
  servoWrite(angle);
  delay(LOOK_MS);
  return readDistance();
}

// ================== NHẬN LỆNH TỪ LAPTOP ==================
void readUdp() {
  char buf[64];
  while (udp.parsePacket() > 0) {
    int len = udp.read(buf, sizeof(buf) - 1);
    if (len <= 0) continue;
    buf[len] = 0;
    laptopIP = udp.remoteIP();
    laptopPort = udp.remotePort();
    hasLaptop = true;

    if (strcmp(buf, "PING") == 0) {
      sendStatus("SUCCESS");
      Serial.println("Laptop kiem tra ket noi -> SUCCESS");
    } else if (strcmp(buf, "R") == 0) {
      if (mode == ISOLATED || mode == LOCKED) setMode(AUTO);
      sendStatus("RESET OK");
    } else if (strcmp(buf, "S") == 0) {
      paused = true;
    } else if (strcmp(buf, "G") == 0) {
      paused = false;
    } else if (buf[0] == 'N') {
      targetSeen = false;
    } else if (buf[0] == 'T') {
      float x, area;
      if (sscanf(buf, "T,%f,%f", &x, &area) == 2) {
        targetSeen = true;
        targetX = x;
        targetArea = area;
        lastTargetMsg = millis();
        lastSeenTime = millis();
        msgCount++;
      }
    }
  }
}

// ================== CHẾ ĐỘ TỰ HÀNH (CHƯA THẤY MỤC TIÊU) ==================

// Quét 4 hướng, trả về góc của hướng thoáng nhất, hoặc -1 nếu bị chặn hết
int scanBestAngle() {
  int best = -1;
  float bestD = 0;
  for (int i = 0; i < SCAN_N; i++) {
    float d = lookAt(SCAN_ANGLES[i]);
    Serial.printf("   Goc %3d (%s): %.0f cm\n", SCAN_ANGLES[i],
                  SCAN_ANGLES[i] < 90 ? "phai" : "trai", d);
    if (d < CLEAR_CM) continue;
    bool farther = d > bestD + TIE_CM;                                                                    // xa hơn hẳn
    bool tieButLeft = d > bestD - TIE_CM && SCAN_ANGLES[i] > 90 && (best < 0 || SCAN_ANGLES[best] < 90);  // gần bằng nhau -> ưu tiên trái
    if (best < 0 || farther || tieButLeft) {
      best = i;
      bestD = d;
    }
  }
  servoWrite(SERVO_CENTER);
  delay(300);
  return best < 0 ? -1 : SCAN_ANGLES[best];
}

// Quay thân xe về đúng hướng đã chọn (dưới 90 = phải, trên 90 = trái)
void turnToAngle(int angle) {
  int offset = angle - SERVO_CENTER;
  bool toRight = offset < 0;
  int ms = abs(offset) * TURN90_MS / 90;
  Serial.printf("-> Quay %s %d do\n", toRight ? "PHAI" : "TRAI", abs(offset));
  spin(toRight, ms);
  lastTurnRight = toRight;

  // Kiểm tra lại: phía trước đã trống chưa? Chưa thì quay thêm từng nhịp
  // (xe quay thiếu do sàn trơn / pin yếu -> không bị kẹt quét đi quét lại)
  for (int i = 0; i < TURN_EXTRA_STEPS; i++) {
    float d = readDistance();
    if (d >= CLEAR_CM) return;
    Serial.printf("   Phia truoc con %.0f cm -> quay them\n", d);
    spin(toRight, TURN_STEP_MS);
  }
}

// Ghi nhận 1 lần quay đầu; quay đầu quá nhiều lần trong thời gian ngắn = bị kẹt thật
bool tooManyTurnArounds() {
  unsigned long now = millis();
  turnAroundTimes[turnAroundIdx] = now;
  turnAroundIdx = (turnAroundIdx + 1) % ISOLATE_TURNS;
  unsigned long oldest = turnAroundTimes[turnAroundIdx];
  return oldest != 0 && now - oldest < ISOLATE_WINDOW_MS;
}

void autoStep() {
  float d = readDistance();
  if (d > OBSTACLE_CM) {
    go(SPEED_FORWARD, SPEED_FORWARD);
    return;
  }

  Serial.printf("Vat can %.0f cm -> DUNG, quet 4 huong\n", d);
  stopCar();
  delay(200);

  for (int attempt = 0; attempt <= MAX_BACK_TRIES; attempt++) {
    int angle = scanBestAngle();
    if (angle >= 0) {
      if (readDistance() < BACK_IF_CLOSE_CM) goBack(BACK_SHORT_MS);  // quá sát thì lùi lấy chỗ quay
      turnToAngle(angle);
      return;
    }
    if (attempt < MAX_BACK_TRIES) {
      int ms = BACK_LONG_MS + attempt * BACK_STEP_MS;  // lần sau lùi xa hơn lần trước
      Serial.printf("-> Bi chan het, LUI XA %d ms roi quet lai (lan %d)\n", ms, attempt + 1);
      goBack(ms);
    }
  }

  // Lùi nhiều lần vẫn kín -> quay đầu (đổi chiều so với lần quay trước)
  if (tooManyTurnArounds()) {
    for (int i = 0; i < ISOLATE_TURNS; i++) turnAroundTimes[i] = 0;
    setMode(ISOLATED);  // thử hết cách vẫn kẹt -> báo cô lập
    return;
  }
  Serial.println("-> Van bi chan het, QUAY DAU");
  if (lastTurnRight) turnLeft(TURN90_MS * 2);
  else turnRight(TURN90_MS * 2);
  lastTurnRight = !lastTurnRight;
}

// ================== CHẾ ĐỘ BÁM MỤC TIÊU ==================
// Bước NGẮM: xe đứng yên, quay từng nhịp nhỏ cho tâm bia về giữa khung hình
void aimStep() {
  stopCar();
  if (millis() < aimWaitUntil) {
    delay(5);
    return;
  }  // đang chờ camera gửi hình mới
  if (msgCount == lastEvalMsg) {
    delay(5);
    return;
  }  // chưa có tin mới từ laptop
  lastEvalMsg = msgCount;

  if (fabs(targetX) <= AIM_TOL) {  // bia đã ở giữa
    centeredCount++;
    if (centeredCount >= AIM_CONFIRM) {
      Serial.printf("[NGAM] x=%+.2f -> NGAM XONG, DI THANG\n", targetX);
      aimState = GOING;
      goBadCount = 0;
    }
    return;
  }

  centeredCount = 0;  // bia còn lệch -> quay 1 nhịp
  int ms = AIM_PULSE_MIN_MS + (int)(fabs(targetX) * AIM_PULSE_GAIN_MS);
  Serial.printf("[NGAM] x=%+.2f -> quay %s %d ms\n", targetX, targetX > 0 ? "PHAI" : "TRAI", ms);
  if (targetX > 0) drive(SPEED_ALIGN, -SPEED_ALIGN);  // bia bên phải -> quay phải
  else drive(-SPEED_ALIGN, SPEED_ALIGN);              // bia bên trái -> quay trái
  delay(ms);
  stopCar();
  aimWaitUntil = millis() + AIM_WAIT_MS;  // đứng chờ hình mới
  lastEvalMsg = msgCount;                 // bỏ các tin cũ gửi trong lúc quay
}

// Bước TÌM: mỗi nhịp quay một chút rồi đứng chờ camera; thấy bia thì loop() tự chuyển sang NGẮM
void searchStep() {
  stopCar();
  if (millis() < searchWaitUntil) { delay(5); return; }
  if (searchSteps >= SEARCH_MAX_STEPS) {
    Serial.println("[TIM] Quay 1 vong khong thay bia -> ve tu hanh");
    setMode(AUTO);
    return;
  }
  searchSteps++;
  Serial.printf("[TIM] quay %s nhip %d/%d\n", searchRight ? "PHAI" : "TRAI", searchSteps, SEARCH_MAX_STEPS);
  spin(searchRight, SEARCH_STEP_MS);
  searchWaitUntil = millis() + SEARCH_WAIT_MS;
}

// Đang đi tới bia thì gặp vật cản (không phải bia): VÒNG QUA rồi TÌM LẠI bia
void detourObstacle() {
  Serial.println("[VONG] Vat can chan duong toi bia -> vong qua");
  stopCar();
  delay(150);
  bool targetWasRight = targetX > 0;

  int angle = scanBestAngle();                 // tìm bên trống để vòng
  if (angle < 0) {
    goBack(BACK_LONG_MS);
    angle = scanBestAngle();
  }
  if (angle < 0) {
    Serial.println("[VONG] Khong co duong vong -> tim lai bia");
    startSearch(targetWasRight);
    return;
  }

  bool toRight = angle < 90;
  int ms = DETOUR_ANGLE * TURN90_MS / 90;
  if (readDistance() < BACK_IF_CLOSE_CM) goBack(BACK_SHORT_MS);
  Serial.printf("[VONG] Re %s %d do, chay qua vat can\n", toRight ? "PHAI" : "TRAI", DETOUR_ANGLE);
  spin(toRight, ms);

  startMove(SPEED_FORWARD, SPEED_FORWARD);     // chạy vượt qua vật cản
  unsigned long t0 = millis();
  while (millis() - t0 < (unsigned long)DETOUR_MS) {
    if (readDistance() <= OBSTACLE_CM) break;  // gặp vật khác thì dừng sớm
  }
  stopCar();
  delay(150);

  Serial.println("[VONG] Quay lai huong cu");
  spin(!toRight, ms);
  startSearch(!toRight);                       // bia giờ lệch về phía ngược với hướng vừa vòng
}

// Đo nhanh phía trước khi đang đi tới bia: mỗi lần 1 phép đo, lấy giá trị giữa của 3 lần gần nhất
float frontBuf[3] = { 400, 400, 400 };
int frontIdx = 0;
float readFrontFast() {
  frontBuf[frontIdx] = pingOnce();
  frontIdx = (frontIdx + 1) % 3;
  float a = frontBuf[0], b = frontBuf[1], c = frontBuf[2];
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}
void resetFrontFast() {
  for (int i = 0; i < 3; i++) frontBuf[i] = 400;
}

// Tới nơi: phanh ngay, khóa mục tiêu
void arriveAtTarget(const char* why, float d) {
  brake(BRAKE_MS);
  setMode(LOCKED);
  Serial.printf("[TOI NOI] %s (cam bien %.0f cm, bia %.0f%% khung hinh) -> KHOA MUC TIEU\n",
                why, d, targetArea * 100);
}

// Bước ĐI: đi thẳng tới bia, bia lệch thì dừng lại ngắm lại
void goStep() {
  float d = readFrontFast();
  bool targetNear = targetArea >= LOCK_MIN_AREA;   // bia to trong hình = bia ở gần

  if (targetArea >= ARRIVE_AREA) {                  // bia đã to chiếm gần hết hình -> tới sát
    arriveAtTarget("Bia rat to trong hinh", d);
    return;
  }
  if (d <= TARGET_STOP_CM && targetNear) {          // vật trước mặt chính là bia -> tới nơi
    arriveAtTarget("Cam bien thay bia o gan", d);
    return;
  }
  if (d <= TRACK_OBSTACLE_CM && !targetNear) {      // bia còn xa mà có vật chắn -> vòng qua
    Serial.printf("[DI] Vat can %.0f cm (bia con xa, %.1f%% khung hinh)\n", d, targetArea * 100);
    detourObstacle();
    return;
  }

  if (msgCount != lastEvalMsg) {  // có tin mới -> kiểm tra bia còn ở giữa không
    lastEvalMsg = msgCount;
    if (fabs(targetX) > GO_TOL) {
      if (++goBadCount >= GO_BAD_COUNT) {
        Serial.printf("[DI] x=%+.2f -> bia lech, DUNG LAI NGAM LAI\n", targetX);
        stopCar();
        startAiming();
        return;
      }
    } else {
      goBadCount = 0;
    }
  }

  bool slow = d < SLOW_ZONE_CM || targetArea >= SLOW_AREA;   // gần bia -> chạy chậm
  int spd = slow ? SPEED_SLOW : SPEED_APPROACH;
  go(spd, spd);

  static unsigned long lastDbg = 0;
  if (millis() - lastDbg > 400) {
    Serial.printf("[DI] x=%+.2f  d=%.0fcm  bia=%.0f%%  toc do=%d\n", targetX, d, targetArea * 100, spd);
    lastDbg = millis();
  }
  delay(20);   // cho sóng siêu âm cũ tắt hẳn rồi mới đo tiếp
}

void controlStep() {
  if (aimState == AIMING) aimStep();
  else goStep();
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  ledcAttachChannel(ENA, 1000, 8, 0);
  ledcAttachChannel(ENB, 1000, 8, 1);
  ledcAttachChannel(SERVO_PIN, 50, 16, 4);

  stopCar();
  servoWrite(SERVO_CENTER);

  Serial.println();
  Serial.println("=== XE TU HANH + AI NHAN DIEN MUC TIEU ===");
  bool connected = false;
  if (USE_PHONE_HOTSPOT) {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);  // tắt tiết kiệm điện cho WiFi phản hồi nhanh
    WiFi.begin(HOTSPOT_SSID, HOTSPOT_PASS);
    Serial.printf("Dang ket noi hotspot \"%s\"", HOTSPOT_SSID);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
      delay(500);
      Serial.print(".");
    }
    connected = WiFi.status() == WL_CONNECTED;
    if (connected) {
      Serial.printf("\nDa vao hotspot! IP cua xe: %s\n", WiFi.localIP().toString().c_str());
    } else {
      Serial.println("\nKHONG vao duoc hotspot (kiem tra ten, mat khau, bang tan 2.4 GHz) -> tu phat WiFi");
    }
  }
  if (!connected) {
    WiFi.mode(WIFI_AP);
    if (WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, 4)) {
      Serial.printf("Xe phat WiFi: %s   mat khau: %s\n", AP_SSID, AP_PASS);
      Serial.printf("IP cua xe: %s\n", WiFi.softAPIP().toString().c_str());
    } else {
      Serial.println("LOI: khong phat duoc WiFi");
    }
  }
  udp.begin(UDP_PORT);

  Serial.println("Xe se chay tu hanh sau 3 giay...");
  delay(3000);
  Serial.printf("[%s]\n", modeName(mode));
}

// ================== LOOP ==================
void loop() {
  readUdp();
  bool target = targetSeen && millis() - lastTargetMsg < TARGET_TIMEOUT_MS;

  // Gửi trạng thái cho laptop 2 lần mỗi giây
  static unsigned long lastBeat = 0;
  if (millis() - lastBeat > 500) {
    sendStatus(modeName(mode));
    lastBeat = millis();
  }

  // Mỗi giây báo địa chỉ của xe cho laptop (laptop tự tìm xe, không cần gõ IP)
  static unsigned long lastHello = 0;
  if (millis() - lastHello > 1000) {
    IPAddress bc = (WiFi.getMode() == WIFI_AP) ? IPAddress(192, 168, 4, 255) : WiFi.broadcastIP();
    udp.beginPacket(bc, HELLO_PORT);
    udp.print("XE_AI_HELLO");
    udp.endPacket();
    lastHello = millis();
  }

  // Tạm dừng
  if (paused) {
    stopCar();
    setMode(PAUSED);
    return;
  }
  if (mode == PAUSED) setMode(AUTO);

  // Cô lập: đứng yên chờ reset (phím R trên laptop hoặc nút EN trên board)
  if (mode == ISOLATED) {
    stopCar();
    return;
  }

  // Đã khóa mục tiêu: ĐỨNG YÊN. Chỉ bám tiếp khi bia bị dời ra XA (thấy bia nhỏ lại, ở giữa hình).
  // Bấm R trên laptop (hoặc nút EN trên board) để xe chạy lại từ đầu.
  if (mode == LOCKED) {
    stopCar();
    if (target && targetArea < LOCK_MIN_AREA && fabs(targetX) <= CENTER_TOL * 2
        && readDistance() > LOCK_RELEASE_CM) {
      Serial.println("[KHOA] Bia da bi doi ra xa -> bam theo tiep");
      setMode(CONTROL);
    } else {
      delay(50);
    }
    return;
  }

  if (target) {
    setMode(CONTROL);                    // thấy bia (kể cả đang tìm lại) -> NGẮM rồi ĐI
    controlStep();
  } else if (mode == CONTROL && aimState == GOING && targetArea >= SLOW_AREA) {
    // Đang đi tới, bia đã to mà đột nhiên mất -> bia tràn ra ngoài khung hình vì quá gần -> coi như tới nơi
    arriveAtTarget("Bia qua gan, tran ra ngoai khung hinh", readDistance());
  } else if (mode == CONTROL && millis() - lastSeenTime < LOST_WAIT_MS) {
    // Vừa mất mục tiêu (hình bị trễ / giật) -> dừng chờ camera thấy lại, KHÔNG chạy thẳng bừa
    static unsigned long lastWaitMsg = 0;
    if (millis() - lastWaitMsg > 500) {
      Serial.println("[BAM] mat muc tieu tam thoi -> dung cho");
      lastWaitMsg = millis();
    }
    stopCar();
    startAiming();  // thấy lại thì phải ngắm lại từ đầu
    delay(20);
  } else if (mode == CONTROL) {
    startSearch(targetX > 0);            // mất bia lâu -> quay về phía thấy bia lần cuối để tìm
  } else if (mode == SEARCH) {
    searchStep();
  } else {
    setMode(AUTO);
    autoStep();
  }
}