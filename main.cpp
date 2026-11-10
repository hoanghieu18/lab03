/*
 * CSE3069 - Bài 3: Điều khiển thiết bị an toàn
 * Máy trạng thái DỪNG / CHẠY / LỖI + ngắt ngoài + timer phần cứng + PWM + giao diện web (tùy chọn)
 *
 * Phần cứng (theo sơ đồ board ESP32-DEVKITC-32E của nhóm):
 *   Tải    : LED_R trên board = GPIO26 (IO26) --[R14 1k]--> Base Q6 MMBT3904 (R15 10k kéo base xuống GND)
 *            Collector --> LED D3 --[R9 1k]--> +5V ; Emitter --> GND
 *            => GPIO26 = HIGH: LED sáng, LOW: LED tắt. Khi mới cấp nguồn base bị kéo xuống GND nên tải TẮT.
 *            (Đây là đường điều khiển qua transistor, GPIO không cấp dòng trực tiếp cho LED.)
 *   Nút    : FUNC_BUT = GPIO36 (SENSOR_VP), chân chỉ-đầu-vào, KHÔNG có pull-up nội => cần pull-up ngoài trên board.
 *   DHT11  : DATA_RH = GPIO14 (qua R28 22R, kéo lên +5V bằng R27 3.3k); đầu vào để tạo ca "dữ liệu lỗi" bằng cách rút cảm biến.
 *
 * Thư viện: "DHT sensor library" (Adafruit) + "Adafruit Unified Sensor"
 * (WiFi và WebServer có sẵn trong ESP32 Arduino core)
 *
 * Lệnh qua Serial Monitor (115200, kết thúc dòng "Newline") hoặc qua web:
 *   ON         -> chạy tải (chỉ khi đang DỪNG và cảm biến hợp lệ)
 *   OFF        -> dừng tải
 *   DUTY <n>   -> đặt duty cycle 0..100 (%) , vd: DUTY 25
 *   STATS      -> in số liệu ngắt ngoài + chu kỳ timer
 *   Lệnh khác  -> coi là "sai định dạng": nếu đang CHẠY thì chuyển sang LỖI
 *
 * Web: ESP32 tự phát WiFi (AP). Kết nối điện thoại/máy tính vào WiFi đó rồi mở http://192.168.4.1
 * Mọi lệnh từ web đi qua CÙNG hàm handleCommand() như lệnh Serial nên quy tắc an toàn không đổi:
 * LỖI / hết giờ luôn về DỪNG, kể cả khi mất kết nối web.
 */

#include <Arduino.h>
#include <DHT.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_timer.h"

// ===================== CẤU HÌNH =====================
#define LOAD_PIN      26      // LED_R: GPIO26 -> R14 -> transistor Q6 -> LED D3
#define BUTTON_PIN    36      // FUNC_BUT: GPIO36 (SENSOR_VP), chỉ-đầu-vào, không có pull-up nội
#define DHT_PIN       14      // DATA_RH: GPIO14
#define DHT_TYPE      DHT11   // board dùng DHT11

// Nút nhấn: true nếu nhấn = mức LOW (pull-up ngoài), false nếu nhấn = mức HIGH.
// Sơ đồ chưa cho thấy mạch nút, hãy kiểm tra trên board rồi chỉnh lại nếu cần.
#define BUTTON_ACTIVE_LOW  true

// WiFi do ESP32 tự phát (mật khẩu tối thiểu 8 ký tự)
const char* AP_SSID = "NhaKinh-TH03";
const char* AP_PASS = "12345678";

const uint32_t MAX_RUN_MS      = 20000;  // giới hạn thời gian chạy (10-30 s) - ghi vào README
const uint32_t ERROR_HOLD_MS   = 2000;   // giữ trạng thái LỖI trước khi về DỪNG
const uint32_t SENSOR_PERIOD_MS = 2000;  // chu kỳ đọc cảm biến

// Chống dội nút: đặt 0 để đo số liệu "TRƯỚC khi chống dội", 50 để đo "SAU".
#define DEBOUNCE_MS   0

// PWM
const uint32_t PWM_FREQ = 1000;   // 1 kHz
const uint8_t  PWM_BITS = 8;      // duty 0..255

// Timer phần cứng: tạo sự kiện mỗi 1 giây
const uint32_t TIMER_PERIOD_US = 1000000;

// ===================== MÁY TRẠNG THÁI =====================
enum State { ST_STOP, ST_RUN, ST_ERROR };
State state = ST_STOP;

uint32_t runStartMs   = 0;
uint32_t errorStartMs = 0;
uint8_t  dutyPercent  = 100;
String   lastMsg      = "San sang";      // thông báo gần nhất, hiển thị trên web

// ===================== CẢM BIẾN =====================
DHT dht(DHT_PIN, DHT_TYPE);
uint32_t lastSensorMs = 0;
bool     sensorOk     = false;
float    lastTemp = NAN, lastHum = NAN;

// ===================== NGẮT NGOÀI (nút nhấn) =====================
volatile uint32_t buttonCount  = 0;   // số lần ISR ghi nhận
volatile uint32_t lastIsrMs    = 0;
uint32_t printedButtonCount    = 0;

void IRAM_ATTR onButtonISR() {
  // ISR chỉ làm một việc: tăng biến đếm (có lọc thời gian nếu DEBOUNCE_MS > 0)
#if DEBOUNCE_MS > 0
  uint32_t now = millis();
  if (now - lastIsrMs < DEBOUNCE_MS) return;
  lastIsrMs = now;
#endif
  buttonCount++;
}

// ===================== TIMER PHẦN CỨNG =====================
volatile uint32_t timerTicks   = 0;
volatile int64_t  lastTickUs   = 0;
volatile int64_t  periodSumUs  = 0;   // tổng các chu kỳ đo được (từ tick thứ 2)
volatile int64_t  lastPeriodUs = 0;
hw_timer_t* hwTimer = nullptr;

void IRAM_ATTR onTimerISR() {
  int64_t now = esp_timer_get_time();   // micro giây, độc lập với millis()
  if (timerTicks > 0) {
    lastPeriodUs = now - lastTickUs;
    periodSumUs += lastPeriodUs;
  }
  lastTickUs = now;
  timerTicks++;
}

// ===================== WEB =====================
WebServer server(80);

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Nha kinh - TH03</title>
<style>
  body{font-family:system-ui,sans-serif;margin:0;padding:16px;background:#f3f5f7;color:#1c2630}
  .card{max-width:480px;margin:0 auto 12px;background:#fff;border-radius:12px;padding:16px;box-shadow:0 1px 3px #0002}
  h1{font-size:18px;margin:0 0 4px}
  .sub{color:#667;font-size:13px;margin-bottom:8px}
  #badge{display:inline-block;padding:8px 20px;border-radius:999px;font-size:22px;font-weight:700;color:#fff;background:#6b7785}
  .DUNG{background:#6b7785!important}.CHAY{background:#1f9d55!important}.LOI{background:#d64545!important}
  .row{display:flex;justify-content:space-between;padding:6px 0;border-bottom:1px solid #eef0f2;font-size:15px}
  .row:last-child{border:0}
  .bar{height:10px;background:#e3e7eb;border-radius:6px;overflow:hidden;margin:10px 0 2px}
  #fill{height:100%;width:0;background:#1f9d55;transition:width .4s}
  button,select{font-size:16px;padding:12px 14px;border-radius:8px;border:0;margin:4px 2px}
  button{color:#fff;background:#2f6fed;font-weight:600}
  #bOff{background:#6b7785}#bBad{background:#b5792a;font-size:13px}
  select{background:#e9edf1}
  #msg{font-size:13px;color:#445;margin-top:8px;min-height:18px}
</style>
</head>
<body>
<div class="card">
  <h1>Dieu khien thiet bi an toan</h1>
  <div class="sub">CSE3069 - Bai 3 | tai: LED_R (GPIO26)</div>
  <span id="badge">...</span>
  <div class="bar"><div id="fill"></div></div>
  <div class="sub" id="left"></div>
  <div>
    <button id="bOn" onclick="cmd('ON')">BAT (ON)</button>
    <button id="bOff" onclick="cmd('OFF')">TAT (OFF)</button>
  </div>
  <div>
    Duty:
    <select id="duty" onchange="cmd('DUTY '+this.value)">
      <option value="25">25%</option><option value="50">50%</option>
      <option value="90">90%</option><option value="100" selected>100%</option>
    </select>
    <button id="bBad" onclick="cmd('ABC')">Gui lenh sai (thu ca 3)</button>
  </div>
  <div id="msg"></div>
</div>
<div class="card">
  <div class="row"><span>Nhiet do</span><b id="t">-</b></div>
  <div class="row"><span>Do am</span><b id="h">-</b></div>
  <div class="row"><span>Cam bien</span><b id="s">-</b></div>
  <div class="row"><span>Gioi han chay</span><b id="max">-</b></div>
  <div class="row"><span>So lan ngat nut</span><b id="btn">-</b></div>
  <div class="row"><span>Timer: so tick</span><b id="tick">-</b></div>
  <div class="row"><span>Timer: chu ky TB</span><b id="avg">-</b></div>
</div>
<script>
let maxMs = 20000;
function $(i){return document.getElementById(i)}
function render(d){
  $('badge').textContent = d.state;
  $('badge').className = d.state;
  maxMs = d.maxRunMs;
  $('fill').style.width = d.state==='CHAY' ? (100*d.runLeftMs/maxMs)+'%' : '0';
  $('left').textContent = d.state==='CHAY' ? 'Con '+(d.runLeftMs/1000).toFixed(1)+' s truoc khi tu dung' : 'Tai dang tat';
  $('t').textContent = d.temp===null ? 'NaN' : d.temp.toFixed(1)+' °C';
  $('h').textContent = d.hum===null ? 'NaN' : d.hum.toFixed(0)+' %';
  $('s').textContent = d.sensorOk ? 'Hop le' : 'LOI / khong doc duoc';
  $('max').textContent = (d.maxRunMs/1000)+' s';
  $('btn').textContent = d.buttonCount;
  $('tick').textContent = d.timerTicks;
  $('avg').textContent = d.avgPeriodUs>0 ? (d.avgPeriodUs/1000).toFixed(3)+' ms' : '-';
  $('msg').textContent = d.msg;
  $('bOn').disabled = !(d.state==='DUNG' && d.sensorOk);
}
function poll(){
  fetch('/status').then(r=>r.json()).then(render).catch(()=>{$('msg').textContent='Mat ket noi voi ESP32';});
}
function cmd(c){
  fetch('/cmd?c='+encodeURIComponent(c)).then(r=>r.json()).then(render).catch(()=>{$('msg').textContent='Mat ket noi voi ESP32';});
}
poll(); setInterval(poll, 1000);
</script>
</body>
</html>
)rawliteral";

// ===================== ĐIỀU KHIỂN TẢI =====================
void pwmWriteLoad(uint8_t duty8) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(LOAD_PIN, duty8);
#else
  ledcWrite(0, duty8);
#endif
}

void loadOff() { pwmWriteLoad(0); }

void loadOn() {
  pwmWriteLoad((uint32_t)dutyPercent * 255 / 100);
}

// ===================== LOG =====================
void setLastMsg(const String& s) {
  lastMsg = s;
  lastMsg.replace("\"", "'");        // tránh làm hỏng JSON
}

void logEvent(const char* msg) {
  Serial.printf("[%8lu ms] %s\n", (unsigned long)millis(), msg);
  setLastMsg(msg);
}

const char* stateName(State s) {
  switch (s) {
    case ST_STOP:  return "DUNG";
    case ST_RUN:   return "CHAY";
    default:       return "LOI";
  }
}

void changeState(State next, const char* reason) {
  if (state == next) return;
  Serial.printf("[%8lu ms] %s -> %s | %s\n", (unsigned long)millis(),
                stateName(state), stateName(next), reason);
  setLastMsg(String(stateName(state)) + " -> " + stateName(next) + " | " + reason);
  state = next;
  if (next == ST_RUN) {
    runStartMs = millis();
    loadOn();
  } else {
    loadOff();                       // DỪNG và LỖI đều tắt tải ngay
    if (next == ST_ERROR) errorStartMs = millis();
  }
}

// ===================== ĐỌC CẢM BIẾN (không chặn) =====================
void readSensor() {
  if (millis() - lastSensorMs < SENSOR_PERIOD_MS) return;
  lastSensorMs = millis();
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    if (sensorOk) logEvent("Cam bien: doc ra NaN");
    sensorOk = false;
    lastTemp = NAN; lastHum = NAN;
  } else {
    sensorOk = true;
    lastTemp = t; lastHum = h;
  }
}

// ===================== HÀM CẬP NHẬT TRẠNG THÁI =====================
// Gọi mỗi vòng loop(), chỉ dùng millis(), không dùng delay().
void updateState() {
  switch (state) {
    case ST_STOP:
      // tải đã tắt; chờ lệnh ON
      break;

    case ST_RUN:
      // Ưu tiên cao nhất: kiểm tra lỗi và hết giờ TRƯỚC mọi lệnh khác
      if (!sensorOk) {
        changeState(ST_ERROR, "du lieu cam bien khong hop le (NaN)");
      } else if (millis() - runStartMs >= MAX_RUN_MS) {
        changeState(ST_STOP, "het thoi gian toi da");
      }
      break;

    case ST_ERROR:
      loadOff();                                   // đảm bảo luôn tắt
      if (millis() - errorStartMs >= ERROR_HOLD_MS) {
        changeState(ST_STOP, "het thoi gian giu LOI, ve trang thai an toan");
      }
      break;
  }
}

// ===================== XỬ LÝ LỆNH (Serial và web dùng chung) =====================
void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.length() == 0) return;
  Serial.printf("[%8lu ms] Lenh nhan: \"%s\"\n", (unsigned long)millis(), cmd.c_str());

  if (cmd == "ON") {
    if (state == ST_STOP && sensorOk) {
      changeState(ST_RUN, "lenh ON hop le");
    } else if (state == ST_STOP && !sensorOk) {
      logEvent("Tu choi ON: cam bien chua hop le");
    } else {
      logEvent("Bo qua ON: he khong o trang thai DUNG");
    }
  } else if (cmd == "OFF") {
    changeState(ST_STOP, "lenh OFF");
  } else if (cmd.startsWith("DUTY ")) {
    int d = cmd.substring(5).toInt();
    if (d >= 0 && d <= 100) {
      dutyPercent = d;
      Serial.printf("[%8lu ms] Duty = %d%%\n", (unsigned long)millis(), d);
      setLastMsg("Duty = " + String(d) + "%");
      if (state == ST_RUN) loadOn();
    } else {
      goto invalid;
    }
  } else if (cmd == "STATS") {
    uint32_t ticks = timerTicks;
    double avgUs = ticks > 1 ? (double)periodSumUs / (ticks - 1) : 0;
    Serial.printf("STATS: nut ISR=%lu | timer ticks=%lu, chu ky TB=%.1f us (dat %lu us), lech=%.1f us\n",
                  (unsigned long)buttonCount, (unsigned long)ticks, avgUs,
                  (unsigned long)TIMER_PERIOD_US, avgUs - TIMER_PERIOD_US);
  } else {
    goto invalid;
  }
  return;

invalid:
  if (state == ST_RUN) {
    changeState(ST_ERROR, "lenh sai dinh dang khi dang CHAY");
  } else {
    logEvent("Lenh khong hop le, bo qua");
  }
}

void pollSerial() {
  static String buf;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf.length()) { handleCommand(buf); buf = ""; }
    } else if (buf.length() < 32) {
      buf += c;
    } else {
      buf = "";                       // dòng quá dài -> coi là sai
      if (state == ST_RUN) changeState(ST_ERROR, "lenh qua dai");
    }
  }
}

// ===================== WEB HANDLERS =====================
void sendStatus() {
  uint32_t ticks = timerTicks;
  double avgUs = ticks > 1 ? (double)periodSumUs / (ticks - 1) : 0;

  uint32_t runLeft = 0;
  if (state == ST_RUN) {
    uint32_t elapsed = millis() - runStartMs;
    runLeft = elapsed >= MAX_RUN_MS ? 0 : MAX_RUN_MS - elapsed;
  }

  String j = "{";
  j += "\"state\":\"" + String(stateName(state)) + "\"";
  j += ",\"runLeftMs\":" + String(runLeft);
  j += ",\"maxRunMs\":" + String(MAX_RUN_MS);
  j += ",\"duty\":" + String(dutyPercent);
  j += ",\"sensorOk\":" + String(sensorOk ? "true" : "false");
  j += ",\"temp\":" + (isnan(lastTemp) ? String("null") : String(lastTemp, 1));
  j += ",\"hum\":"  + (isnan(lastHum)  ? String("null") : String(lastHum, 0));
  j += ",\"buttonCount\":" + String((unsigned long)buttonCount);
  j += ",\"timerTicks\":" + String((unsigned long)ticks);
  j += ",\"avgPeriodUs\":" + String(avgUs, 1);
  j += ",\"msg\":\"" + lastMsg + "\"";
  j += "}";
  server.send(200, "application/json", j);
}

void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleWebCmd() {
  if (!server.hasArg("c")) {
    server.send(400, "application/json", "{\"error\":\"thieu tham so c\"}");
    return;
  }
  String c = server.arg("c");
  if (c.length() > 32) {
    // lệnh quá dài = sai định dạng, xử lý như lệnh sai
    c = "?";
  }
  handleCommand(c);                  // cùng một đường xử lý với Serial
  sendStatus();
}

// ===================== SETUP / LOOP =====================
void setup() {
  // 1) Việc đầu tiên: đưa tải về trạng thái an toàn (tắt)
  pinMode(LOAD_PIN, OUTPUT);
  digitalWrite(LOAD_PIN, LOW);

  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== CSE3069 Bai 3: Dieu khien thiet bi an toan ===");

  // 2) PWM
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(LOAD_PIN, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(0, PWM_FREQ, PWM_BITS);
  ledcAttachPin(LOAD_PIN, 0);
#endif
  loadOff();

  // 3) Nút nhấn + ngắt ngoài
  pinMode(BUTTON_PIN, INPUT);                     // GPIO36 không có pull-up nội, dùng pull-up ngoài của board
#if BUTTON_ACTIVE_LOW
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), onButtonISR, FALLING);
#else
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), onButtonISR, RISING);
#endif

  // 4) Timer phần cứng (1 MHz, đếm lên, báo mỗi TIMER_PERIOD_US)
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  hwTimer = timerBegin(1000000);
  timerAttachInterrupt(hwTimer, &onTimerISR);
  timerAlarm(hwTimer, TIMER_PERIOD_US, true, 0);
#else
  hwTimer = timerBegin(0, 80, true);              // 80 MHz / 80 = 1 MHz
  timerAttachInterrupt(hwTimer, &onTimerISR, true);
  timerAlarmWrite(hwTimer, TIMER_PERIOD_US, true);
  timerAlarmEnable(hwTimer);
#endif

  // 5) Cảm biến
  dht.begin();
  delay(2000);                                    // DHT cần thời gian ổn định (tải đang tắt)
  lastSensorMs = 0;
  readSensor();

  // 6) WiFi AP + web server
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, sendStatus);
  server.on("/cmd", HTTP_GET, handleWebCmd);
  server.begin();
  Serial.printf("WiFi AP: \"%s\" | mat khau: %s | mo http://%s\n",
                AP_SSID, AP_PASS, WiFi.softAPIP().toString().c_str());

  Serial.printf("Gioi han chay toi da: %lu ms | Duty mac dinh: %u%%\n",
                (unsigned long)MAX_RUN_MS, dutyPercent);
  Serial.printf("Chong doi nut: %d ms (0 = tat)\n", DEBOUNCE_MS);
  Serial.println("Lenh: ON, OFF, DUTY <0-100>, STATS");
  logEvent("San sang, trang thai DUNG");
}

void loop() {
  pollSerial();
  readSensor();
  updateState();          // luôn được gọi mỗi vòng, web không chặn được
  server.handleClient();

  // In số lần nút được ghi nhận (xử lý chính nằm ngoài ISR)
  uint32_t c = buttonCount;
  if (c != printedButtonCount) {
    printedButtonCount = c;
    Serial.printf("[%8lu ms] Nut: ISR dem = %lu\n", (unsigned long)millis(), (unsigned long)c);
  }
}