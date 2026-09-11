#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ========== OLED配置 ==========
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_MOSI   19
#define OLED_CLK    21
#define OLED_DC     5
#define OLED_CS     17
#define OLED_RESET  18

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

// ========== 按钮配置 ==========
#define BUTTON_PIN  12
#define LONG_PRESS_MS 500

// ========== 参数设置 ==========
#define SCAN_DURATION  1
#define DEVICE_TIMEOUT 10000
#define MAX_MACS       50
#define RSSI_STRONG   -55
#define RSSI_MIDDLE   -65
#define RSSI_WEAK     -75
#define JUMP_THRESHOLD 8

// ========== 数据结构 ==========
struct Device {
  uint8_t mac[6];
  int rssi;
  unsigned long lastSeen;
};

Device devices[MAX_MACS];
int deviceCount = 0;
int peakRSSI = -100;
unsigned long peakStartTime = 0;

// 历史记录（用于检测突变）
int historyRSSI[3] = {-100, -100, -100};
int historyIndex = 0;

// 搜索动画状态
int dotCount = 0;
unsigned long lastDotUpdate = 0;
bool alertTriggered = false;

// ========== MAC列表状态 ==========
bool inMacListMode = false;
int macListPage = 0;
int macListTotalPages = 1;
#define MACS_PER_PAGE 4

// ========== 按钮检测变量 ==========
unsigned long buttonPressStart = 0;
bool buttonPressed = false;
bool longPressTriggered = false;

BLEScan* pBLEScan;

// ========== 函数声明 ==========
String getSignalLevel(int rssi);
void drawProgressBar(int rssi);
void bootAnimation();
void drawMacListPage();
void sortDevicesByRSSI();

// ========== 获取信号等级 ==========
String getSignalLevel(int rssi) {
  if (rssi < -90) return " NO SIGNAL";
  if (rssi >= RSSI_STRONG) return " >STRONG< ";
  if (rssi >= RSSI_MIDDLE) return "  MIDDLE  ";
  if (rssi >= RSSI_WEAK) return "   WEAK   ";
  return " NO SIGNAL";
}

// ========== 绘制进度条 ==========
void drawProgressBar(int rssi) {
  int barX = 0;
  int barY = 56;
  int barWidth = 128;
  int barHeight = 8;
  
  display.drawRect(barX, barY, barWidth, barHeight, SSD1306_WHITE);
  
  int fillWidth = 0;
  int clamped = rssi;
  if (clamped > -40) clamped = -40;
  if (clamped < -90) clamped = -90;
  
  int rangeMin = -90;
  int rangeMax = -40;
  int pxMin = 2;
  int pxMax = barWidth - 2;
  fillWidth = (clamped - rangeMin) * (pxMax - pxMin) / (rangeMax - rangeMin);
  fillWidth = constrain(fillWidth, 0, barWidth - 2);
  
  if (fillWidth > 0) {
    display.fillRect(barX + 1, barY + 1, fillWidth, barHeight - 2, SSD1306_WHITE);
  }
}

// ========== 按RSSI排序（从强到弱） ==========
void sortDevicesByRSSI() {
  if (deviceCount <= 1) return;
  
  for (int i = 0; i < deviceCount - 1; i++) {
    for (int j = i + 1; j < deviceCount; j++) {
      if (devices[j].rssi > devices[i].rssi) {
        Device temp = devices[i];
        devices[i] = devices[j];
        devices[j] = temp;
      }
    }
  }
}

// ========== BLE扫描回调 ==========
class MyBLEAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    uint8_t mac[6];
    memcpy(mac, advertisedDevice.getAddress().getNative(), 6);
    int rssi = advertisedDevice.getRSSI();
    
    unsigned long now = millis();
    
    int idx = -1;
    for (int i = 0; i < deviceCount; i++) {
      if (memcmp(devices[i].mac, mac, 6) == 0) {
        idx = i;
        break;
      }
    }
    
    if (idx == -1 && deviceCount < MAX_MACS) {
      idx = deviceCount;
      memcpy(devices[idx].mac, mac, 6);
      devices[idx].rssi = rssi;
      devices[idx].lastSeen = now;
      deviceCount++;
    } else if (idx != -1) {
      devices[idx].rssi = rssi;
      devices[idx].lastSeen = now;
    }
    
    // 更新峰值（1秒窗口）
    if (now - peakStartTime > 1000) {
      if (peakRSSI > -100) {
        historyRSSI[historyIndex] = peakRSSI;
        historyIndex = (historyIndex + 1) % 3;
      }
      peakRSSI = -100;
      peakStartTime = now;
    }
    if (rssi > peakRSSI) peakRSSI = rssi;
  }
};

// ========== 清理超时 ==========
void cleanDevices() {
  unsigned long now = millis();
  int newCount = 0;
  for (int i = 0; i < deviceCount; i++) {
    if (now - devices[i].lastSeen < DEVICE_TIMEOUT) {
      if (newCount != i) {
        memcpy(&devices[newCount], &devices[i], sizeof(Device));
      }
      newCount++;
    }
  }
  deviceCount = newCount;
}

// ========== 检测信号突变 ==========
bool detectJump() {
  int sum = 0;
  int count = 0;
  for (int i = 0; i < 3; i++) {
    if (historyRSSI[i] > -100) {
      sum += historyRSSI[i];
      count++;
    }
  }
  if (count == 0) return false;
  int avg = sum / count;
  
  if (peakRSSI > -90 && (peakRSSI - avg) > JUMP_THRESHOLD) {
    return true;
  }
  return false;
}

// ========== 进入MAC列表模式 ==========
void enterMacListMode() {
  inMacListMode = true;
  macListPage = 0;
  sortDevicesByRSSI();
  macListTotalPages = (deviceCount + MACS_PER_PAGE - 1) / MACS_PER_PAGE;
  if (macListTotalPages < 1) macListTotalPages = 1;
}

// ========== 退出MAC列表模式 ==========
void exitMacListMode() {
  inMacListMode = false;
}

// ========== 显示MAC地址列表 ==========
void drawMacListPage() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // 标题行
  display.setTextSize(1);
  display.setCursor(0, 0);
  char title[30];
  sprintf(title, "MAC List[%d/%d]Total:%d",
          macListPage + 1, macListTotalPages, deviceCount);
  display.print(title);
  
  display.drawLine(0, 10, 128, 10, SSD1306_WHITE);
  
  int startIdx = macListPage * MACS_PER_PAGE;
  int endIdx = startIdx + MACS_PER_PAGE;
  if (endIdx > deviceCount) endIdx = deviceCount;
  
  display.setTextSize(1);
  for (int i = startIdx; i < endIdx; i++) {
    int yPos = 12 + (i - startIdx) * 10;
    if (yPos > 58) break;
    
    char macStr[18];
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
            devices[i].mac[0], devices[i].mac[1], devices[i].mac[2],
            devices[i].mac[3], devices[i].mac[4], devices[i].mac[5]);
    
    display.setCursor(0, yPos);
    display.print(macStr);
    display.print("");
    display.print(devices[i].rssi);
  }
  
  // 底部提示
  display.setTextSize(1);
  display.setCursor(0, 56);
  if (deviceCount > MACS_PER_PAGE) {
    display.print("Click:Page  Hold:Exit");
  } else {
    display.print("Hold:Exit");
  }
  
  display.display();
}

// ========== 启动动画 ==========
void bootAnimation() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(25, 10);
  display.println("BLE");
  display.setTextSize(1);
  display.setCursor(20, 34);
  display.println("Patrol v3.5");
  display.display();
  delay(600);

  for (int i = 0; i <= 128; i += 4) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(25, 10);
    display.println("BLE");
    display.setTextSize(1);
    display.setCursor(20, 34);
    display.println("Patrol v3.5");
    
    display.drawRect(0, 54, 128, 8, SSD1306_WHITE);
    if (i > 2) {
      display.fillRect(2, 56, i - 4, 4, SSD1306_WHITE);
    }
    display.display();
    delay(12);
  }
  delay(150);
}

// ========== 更新OLED（主界面） ==========
void updateDisplay() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  display.setTextSize(1);
  display.setCursor(0, 0);
  
  bool jumped = detectJump();
  if (jumped && peakRSSI > -100) {
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    display.print(">>> >!TARGET!<  ");
    display.setTextColor(SSD1306_WHITE);
    alertTriggered = true;
  } else {
    alertTriggered = false;
    unsigned long now = millis();
    if (now - lastDotUpdate > 300) {
      lastDotUpdate = now;
      dotCount = (dotCount + 1) % 4;
    }
    display.print(">>>Scanning");
    if (dotCount == 0) display.print("   ");
    else if (dotCount == 1) display.print(".  ");
    else if (dotCount == 2) display.print(".. ");
    else display.print("...");
  }
  
  display.setCursor(90, 0);
  display.print(" [MAC]");
  
  display.setTextSize(3);
  display.setCursor(0, 8);
  char buf[20];
  if (peakRSSI > -90) {
    sprintf(buf, "%d dBm", peakRSSI);
  } else {
    sprintf(buf, "--- dBm");
  }
  display.println(buf);
  
  display.setTextSize(2);
  display.setCursor(0, 30);
  String level = getSignalLevel(peakRSSI);
  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
  display.print(level);
  display.setTextColor(SSD1306_WHITE);
  
  display.setTextSize(1);
  display.setCursor(0, 47);
  sprintf(buf, "Devices: %d  BLE", deviceCount);
  display.print(buf);
  
  drawProgressBar(peakRSSI);
  
  display.display();
}

// ========== 初始化 ==========
void setup() {
  Serial.begin(115200);
  
  if (!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println("OLED init failed!");
    for (;;);
  }
  
  bootAnimation();
  
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  // ===== 初始化BLE =====
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyBLEAdvertisedDeviceCallbacks(), false);
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(50);
  pBLEScan->setWindow(50);
  
  peakStartTime = millis();
  for (int i = 0; i < 3; i++) {
    historyRSSI[i] = -100;
  }
  Serial.println("=== BLE Sniffer v3.5 Started ===");
}

// ========== 主循环 ==========
void loop() {
  // ===== 按钮检测（单击 + 长按） =====
  bool currentState = digitalRead(BUTTON_PIN);
  
  if (currentState == LOW && !buttonPressed) {
    buttonPressed = true;
    buttonPressStart = millis();
    longPressTriggered = false;
  }
  
  if (currentState == LOW && buttonPressed && !longPressTriggered) {
    if (millis() - buttonPressStart > LONG_PRESS_MS) {
      longPressTriggered = true;
      if (inMacListMode) {
        exitMacListMode();
      } else {
        enterMacListMode();
      }
      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(20, 20);
      display.setTextColor(SSD1306_WHITE);
      display.print(inMacListMode ? "MAC List" : "Scanning");
      display.display();
      delay(300);
    }
  }
  
  if (currentState == HIGH && buttonPressed) {
    unsigned long pressDuration = millis() - buttonPressStart;
    buttonPressed = false;
    
    if (pressDuration < LONG_PRESS_MS && !longPressTriggered) {
      if (inMacListMode) {
        macListPage++;
        if (macListPage >= macListTotalPages) {
          macListPage = 0;
        }
        sortDevicesByRSSI();
        macListTotalPages = (deviceCount + MACS_PER_PAGE - 1) / MACS_PER_PAGE;
        if (macListTotalPages < 1) macListTotalPages = 1;
        if (macListPage >= macListTotalPages) macListPage = 0;
      } else {
        enterMacListMode();
      }
    }
  }
  
  // ===== BLE扫描（持续1秒） =====
  pBLEScan->start(SCAN_DURATION, false);
  
  cleanDevices();
  
  if (inMacListMode) {
    drawMacListPage();
  } else {
    updateDisplay();
  }
  
  Serial.printf("Dev:%d Peak:%d dBm Alert:%d\n", deviceCount, peakRSSI, alertTriggered);
}