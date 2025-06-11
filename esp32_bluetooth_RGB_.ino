#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "BluetoothSerial.h" // Bluetooth seri haberleşme kütüphanesi
// Wi-Fi ve OTA etkinlik kontrol değişkenleri
#define WIFI_ENABLED  0  // 1: WiFi açık,  0: WiFi kapalı

#if WIFI_ENABLED
#include <WiFi.h>
#include <WebServer.h>
#include <ElegantOTA.h>

// Wi-Fi ayarları
const char* ssid = "wifi ag adı yazın"; // Wi-Fi ağ adı
const char* password = "wifi şifrenizi yazın"; // Wi-Fi şifresi
// Sabit IP ayarları
IPAddress local_IP(192, 168, 1, 12);  // Sabit IP adresi
IPAddress gateway(192, 168, 1, 1);    // Ağ geçidi (genellikle router IP'si)
IPAddress subnet(255, 255, 255, 0);   // Alt ağ maskesi


WebServer server(80); // Web sunucusu 80 numaralı portt


#endif // WIFI_ENABLED


// === OLED ===
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_SDA 4
#define OLED_SCL 5
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// === PINLER ===
#define ENCODER_PIN_A 32
#define ENCODER_PIN_B 33
#define ENCODER_BUTTON 25
#define BUZZER_PIN 26
#define BATTERY_ADC_PIN 35
#define LOW_VOLTAGE_THRESHOLD 11.0 // Batarya tipinize göre güncelleyin
#define CHARGE_ADC_PIN 34
#define CHARGE_PRESENT_THRESHOLD 3.0 // Gerilim bölücüye göre ayarla (adaptörün voltajını böldükten sonra bu pindeki voltaj eşiği)

const uint8_t pwmPins[3][3] = {
  {16, 14, 13}, {19, 18, 17}, {23, 22, 21}
};
const uint8_t pwmChannels[3][3] = {
  {2, 1, 0}, {5, 4, 3}, {8, 7, 6}
};

// === EEPROM ===
#define EEPROM_SIZE 7
int hueAddr[3] = {0, 2, 4};
int brightnessAddr[3] = {1, 3, 5};
#define GROUP_ADDR 6

// === Kontrol Değişkenleri ===
int animationSpeed = 30; // Genel animasyon hızı
int hue[3] = {0, 0, 0};
float brightness[3] = {1.0, 1.0, 1.0};
int selectedGroup = -1; // -1: Tümü, 0-2: Grup 1-3, 3-7: Animasyonlar, 8: Bluetooth Renk Modu

volatile int encoderPos = 0;
int lastEncoderPos = 0;
int lastEncoded = 0;
unsigned long lastEncoderTime = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastRainbowUpdate = 0;
unsigned long buttonPressTime = 0;
bool buttonPressed = false;
bool longPressHandled = false;
bool longPressCandidate = false;

bool chargingDisplayActive = false; // Batarya şarj ekranı aktif mi?
bool chargingDisplaySuppressed = false; // Batarya şarj ekranı kullanıcı tarafından gizlendi mi?

// Animasyonlar için durum değişkenleri
unsigned long lastAnimationTime = 0;
int currentGroup = 0;         // Kayar animasyonlarda hangi grubun sırası
int currentColorPhase = 0; // Kayar animasyonlarda 0: Kırmızı, 1: Mavi, 2: Yeşil
const int ON = 255;         // On/Off modu için tam parlaklık
const int OFF = 0;          // On/Off modu için kapalı
const int MAX_BRIGHTNESS_VAL = 255; // İz bırakan animasyon için tam parlaklık
const int MIN_BRIGHTNESS_VAL = 0;   // İz bırakan animasyon için kapalı
const int FADE_STEP = 10;         // İz bırakan animasyonda sönme adım miktarı
const unsigned long FADE_INTERVAL = 3; // İz bırakma sönme hızı (ms)

// Her grubun o anki renk değerlerini tutan değişkenler (fade için önemli)
int currentRGB[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}; // [grup][kanal]

// İz bırakan animasyon için değişkenler
unsigned long lastFadeTime[3] = {0, 0, 0}; // Her grubun son sönme zamanı
bool fading[3] = {false, false, false};   // Her grup sönüyor mu?

// === Bluetooth ===
BluetoothSerial SerialBT;
bool isBluetoothConnected = false; // Bluetooth bağlantı durumu
bool bluetoothColorMode = false;   // Bluetooth ile renk ayarı yapılıp yapılmadığını takip et

// === Fonksiyon Prototipleri ===
void updateDisplay();
void applyAllColors();
void allGroupsOff();
void startFadeOut(int group);
void handleFading();
void setGroupColor(int group, int r, int g, int b);
void drawBatteryMiniIcon(int x, int y, float voltage, bool charging);
void drawBatteryWithFillBars(float voltage, bool charging); // Eğer charge_anim.h kullanmıyorsanız bunu yorum satırı yapın
void bluetoothCallback(const uint8_t *buffer, size_t len); // Bluetooth callback prototipi
void rgbToHsv(int r, int g, int b, int &h, float &s, float &v); // Yeni fonksiyon prototipi

// Pil Yüzdesi için Fonksiyon (Eksik olduğu için ekledim)
int getFillBarsFromVoltage(float voltage) {
  // Bu fonksiyonu kendi batarya voltaj aralıklarınıza göre doldurmanız gerekmektedir.
  // Örnek olarak NiMH için bir skala sunulmuştur:
  if (voltage >= 17.6) return 10; // Tam dolu (max 10 bar)
  else if (voltage >= 17.2) return 9;
  else if (voltage >= 17.0) return 8;
  else if (voltage >= 16.5) return 7;
  else if (voltage >= 15.0) return 6;
  else if (voltage >= 14.0) return 5;
  else if (voltage >= 13.0) return 4;
  else if (voltage >= 12.0) return 3;
  else if (voltage >= 11.0) return 2;
  else if (voltage >= 10.0) return 1; // Düşük pil
  else return 0; // Boş
}


void IRAM_ATTR readEncoder() {
  int MSB = digitalRead(ENCODER_PIN_A);
  int LSB = digitalRead(ENCODER_PIN_B);
  int encoded = (MSB << 1) | LSB;
  int transition = (lastEncoded << 2) | encoded;

  if (millis() - lastEncoderTime > 2) {
    if (transition == 0b0001 || transition == 0b0111 || transition == 0b1110 || transition == 0b1000)
      encoderPos++;
    else if (transition == 0b0010 || transition == 0b0100 || transition == 0b1101 || transition == 0b1011)
      encoderPos--;

    lastEncoded = encoded;
    lastEncoderTime = millis();
  }
}

int getNiMHBatteryPercentage(float voltage) {
  // Bu değerleri kendi batarya tipinize göre güncelleyin!
  // Örneğin, 4S NiMH için 4.8V nominal, 5.8V tam dolu
  // Bu aralıklar çok yüksek görünüyor, muhtemelen 12V kurşun asit veya çok hücreli NiMH.
  if (voltage >= 17.0) return 100;
  else if (voltage >= 16.5) return 90;
  else if (voltage >= 16.0) return 80;
  else if (voltage >= 15.5) return 70;
  else if (voltage >= 15.0) return 60;
  else if (voltage >= 14.5) return 50;
  else if (voltage >= 14.0) return 40;
  else if (voltage >= 13.5) return 30;
  else if (voltage >= 13.0) return 20;
  else return 10;
}

float readChargeVoltage() {
  int raw = analogRead(CHARGE_ADC_PIN);
  float voltage = raw / 4095.0 * 3.3;
  return voltage * (22.0 + 4.7) / 4.7; // Oran senin R1/R2 değerlerine göre (voltaj bölücü)
}

bool isChargingPluggedIn() {
  float v = readChargeVoltage();
  return v > CHARGE_PRESENT_THRESHOLD; // Bu eşiği adaptörünüze göre ayarlayın
}

float readBatteryVoltage() {
  int raw = analogRead(BATTERY_ADC_PIN);
  float voltage = raw / 4095.0 * 3.3;
  return voltage * (22.0 + 4.7) / 4.7; // Gerilim bölücü R1=22k, R2=4.7k için
}

void setupPWM(uint8_t pin, uint8_t channel) {
  ledcSetup(channel, 5000, 8); // 5 kHz frekans, 8 bit çözünürlük
  ledcAttachPin(pin, channel);
}



void hsvToRgb(int h, float v, int &r, int &g, int &b) {
  float s = 1.0; // Saturasyon genellikle tam dolu (1.0) tutulur
  float C = s * v;
  float X = C * (1 - abs(fmod(h / 60.0, 2) - 1));
  float m = v - C;
  float r1, g1, b1;
  if (h < 60)         { r1 = C; g1 = X; b1 = 0; }
  else if (h < 120){ r1 = X; g1 = C; b1 = 0; }
  else if (h < 180){ r1 = 0; g1 = C; b1 = X; }
  else if (h < 240){ r1 = 0; g1 = X; b1 = C; }
  else if (h < 300){ r1 = X; g1 = 0; b1 = C; }
  else           { r1 = C; g1 = 0; b1 = X; }
  r = (r1 + m) * 255;
  g = (g1 + m) * 255;
  b = (b1 + m) * 255;
}

// Yeni RGB'den HSV'ye dönüşüm fonksiyonu
void rgbToHsv(int r, int g, int b, int &h, float &s, float &v) {
  float fr = r / 255.0; // 0-1 aralığına normalize et
  float fg = g / 255.0;
  float fb = b / 255.0;

  float cmax = max(max(fr, fg), fb);
  float cmin = min(min(fr, fg), fb);
  float delta = cmax - cmin;

  if (delta == 0) {
    h = 0; // Renk yok, gri ton
  } else if (cmax == fr) {
    h = (int)(60 * fmod(((fg - fb) / delta), 6));
  } else if (cmax == fg) {
    h = (int)(60 * (((fb - fr) / delta) + 2));
  } else {
    h = (int)(60 * (((fr - fg) / delta) + 4));
  }
  if (h < 0) h += 360; // Negatif hue değerlerini düzelt

  s = (cmax == 0) ? 0 : delta / cmax; // Saturasyon
  v = cmax; // Value/Brightness (parlaklık)
}


void setGroupColor(int group, int r, int g, int b) {
  ledcWrite(pwmChannels[group][0], r); // Kırmızı
  ledcWrite(pwmChannels[group][1], g); // Yeşil
  ledcWrite(pwmChannels[group][2], b); // Mavi
  currentRGB[group][0] = r; // Mevcut durumu kaydet
  currentRGB[group][1] = g;
  currentRGB[group][2] = b;
}

// Tüm grupları kapatır
void allGroupsOff() {
  for (int g = 0; g < 3; g++) {
    setGroupColor(g, OFF, OFF, OFF);
  }
}

// Bir grubu sönmeye başlatır (bloklamaz)
void startFadeOut(int group) {
  fading[group] = true;
  lastFadeTime[group] = millis(); // Sönmenin başladığı zamanı kaydet
}

// Tüm sönme işlemlerini yöneten, loop içinde çağrılması gereken fonksiyon
void handleFading() {
  unsigned long currentTime = millis();
  for (int group = 0; group < 3; group++) {
    if (fading[group] && currentTime - lastFadeTime[group] >= FADE_INTERVAL) {
      lastFadeTime[group] = currentTime; // Bir sonraki sönme adımının zamanını güncelle

      int r = currentRGB[group][0];
      int g = currentRGB[group][1];
      int b = currentRGB[group][2];

      if (r > MIN_BRIGHTNESS_VAL || g > MIN_BRIGHTNESS_VAL || b > MIN_BRIGHTNESS_VAL) {
        if (r > MIN_BRIGHTNESS_VAL) r = max(r - FADE_STEP, MIN_BRIGHTNESS_VAL);
        if (g > MIN_BRIGHTNESS_VAL) g = max(g - FADE_STEP, MIN_BRIGHTNESS_VAL);
        if (b > MIN_BRIGHTNESS_VAL) b = max(b - FADE_STEP, MIN_BRIGHTNESS_VAL);
        setGroupColor(group, r, g, b); // Yeni, azaltılmış renk değerini uygula
      } else {
        fading[group] = false; // Tamamen söndü, sönmeyi durdur
        setGroupColor(group, MIN_BRIGHTNESS_VAL, MIN_BRIGHTNESS_VAL, MIN_BRIGHTNESS_VAL); // Tamamen kapalı olduğundan emin ol
      }
    }
  }
}

void applyGroupColor(int i) {
  int r, g, b;
  hsvToRgb(hue[i], brightness[i], r, g, b);
  setGroupColor(i, r, g, b); // Artık setGroupColor fonksiyonu currentRGB'yi güncelliyor
}

void applyAllColors() {
  for (int i = 0; i < 3; i++) applyGroupColor(i);
}

void beepGroup(int count) {
  for (int i = 0; i < count; i++) {
    tone(BUZZER_PIN, 2000, 100);
    delay(200);
  }
}

void updateDisplay() {
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Mod başlığı (Tümü, Grup 1, Grup 2, Grup 3) her zaman üst satırda
  display.setCursor(0, 0);
  display.print("Mod:");
  display.setCursor(30, 0); // Mod adını Mod: yazısının sağına alıyoruz
  if (selectedGroup == -1) display.println("Tumu");
  else if (selectedGroup == 0) display.println("Grup 1");
  else if (selectedGroup == 1) display.println("Grup 2");
  else if (selectedGroup == 2) display.println("Grup 3");
  // Animasyon modları için başlıkları bir alt satıra alıyoruz
  else if (selectedGroup == 3) display.println("Rainbow");
  else if (selectedGroup == 4) display.println("Rand. Rainbow");
  else if (selectedGroup == 5) display.println("Fade Loop");
  else if (selectedGroup == 6) display.println("ON/OFF Kayma");
  else if (selectedGroup == 7) display.println("Iz Birakan");
  else if (selectedGroup == 8) display.println("BT Renk"); // Yeni: Bluetooth renk modu
  else if (selectedGroup == -2) display.println("Kapali"); // Bluetooth ile kapatildi

  // Animasyon hızını göster (sadece animasyon modlarında ve alt satırda)
  if (selectedGroup >= 3 && selectedGroup <= 7) {
    display.setCursor(0, 16); // Mod başlığından 2 satır alta in (8+8)
    display.printf("Hiz: %d ms", animationSpeed);
  }

  // LED gruplarının renk ve parlaklık bilgisi (sadece normal modlarda ve BT Renk modunda)
  if (selectedGroup < 3 || selectedGroup == -1 || selectedGroup == 8) { // Normal hue/brightness modları veya BT Renk modu
    for (int i = 0; i < 3; i++) {
      display.setCursor(0, 16 + i * 16); // İlk gruptan itibaren 16 piksel boşluk
     display.printf("G%d H:%3d B:%3d%%", i + 1, hue[i], int(brightness[i] * 100));
    }
  }

  // Şarj durumu varsa sağ üst köşeye ikonu çiz
  if (isChargingPluggedIn()) {
    float batVoltage = readBatteryVoltage();
    drawBatteryMiniIcon(SCREEN_WIDTH - 14, 0, batVoltage, true);
  }else  if (!isChargingPluggedIn()) {
    float batVoltage = readBatteryVoltage();
    drawBatteryMiniIcon(SCREEN_WIDTH - 14, 0, batVoltage, false);
  }

  // Bluetooth bağlantı durumu
  if (isBluetoothConnected) {
    display.setCursor(0, SCREEN_HEIGHT - 8); // En alt satırda
    display.print("BT Bagli");
  }

  display.display();
}

void sarz_oluyor() {
  bool ledlerKapandi = false;
  chargingDisplayActive = true;     // Şarj ekranı aktif hale geldi
  chargingDisplaySuppressed = false; // Önceki bastırma durumunu sıfırla

  while (isChargingPluggedIn()) {
    if (!ledlerKapandi) {
      for (int g = 0; g < 3; g++)
        for (int c = 0; c < 3; c++)
          ledcWrite(pwmChannels[g][c], 0);
      ledlerKapandi = true;
    }

    float voltage = readBatteryVoltage();
    // chargingDisplaySuppressed true ise büyük şarj ikonunu gösterme
    if (!chargingDisplaySuppressed) {
        drawBatteryWithFillBars(voltage, true);
    } else {
        display.clearDisplay();
        updateDisplay(); // Normal arayüze dön ama küçük ikonu göster.
    }

    if (voltage >= 16.4) { // Batarya doluluk eşiği
      digitalWrite(27, LOW);     // Şarjı kes
    } else {
      digitalWrite(27, HIGH); // Şarj devam
    }

    // 🔁 BUTONA BASILIRSA: ŞARJ EKRANINDAN ÇIK (Veya gizle)
    if (digitalRead(ENCODER_BUTTON) == LOW) {
      delay(50); // Buton debounce
      if (digitalRead(ENCODER_BUTTON) == LOW) {
        chargingDisplaySuppressed = true; // 🔥 Şarj ekranını gizle
        break; // sarz_oluyor döngüsünden çık
      }
    }
    delay(200);
  }

  // Şarj soketi çekildiğinde veya kullanıcı iptal ettiğinde
  digitalWrite(27, HIGH); // Şarjı kes (varsayılan olarak off)
  chargingDisplayActive = false; // Şarj ekranı artık aktif değil

  // Normal ekrana geri dön ve renkleri uygula
  display.clearDisplay();
  updateDisplay();
  applyAllColors();
  saveSettingsToEEPROM();
}

void saveSettingsToEEPROM() {
  for (int i = 0; i < 3; i++) {
    EEPROM.write(hueAddr[i], map(hue[i], 0, 360, 0, 255));
    EEPROM.write(brightnessAddr[i], brightness[i] * 255);
  }
  EEPROM.write(GROUP_ADDR, selectedGroup);
  EEPROM.commit();
}

void loadSettingsFromEEPROM() {
  for (int i = 0; i < 3; i++) {
    hue[i] = map(EEPROM.read(hueAddr[i]), 0, 255, 0, 360);
    brightness[i] = EEPROM.read(brightnessAddr[i]) / 255.0;
  }
  selectedGroup = EEPROM.read(GROUP_ADDR);
  if (selectedGroup > 8) selectedGroup = -1; // Geçersiz grup değeri kontrolü (yeni modlar 6, 7 ve 8)
}
int b;
void setup() {
  Serial.begin(115200);
// Wi-Fi ayarları
#if WIFI_ENABLED
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(ssid, password);
  
  // Bağlantı kontrolü
  while (WiFi.status() != WL_CONNECTED) {
     b++;
    delay(1000);
    Serial.println("WiFi'ye bağlanılıyor...");
    if(b>3) break;
  }
  
  Serial.println("WiFi'ye bağlanıldı!");
  Serial.print("IP Adresi: ");
  Serial.println(WiFi.localIP());
  
  // Web sunucusu başlatma
ElegantOTA.begin(&server);
ElegantOTA.onProgress([](size_t current, size_t final) {
  if (current == final) tone(BUZZER_PIN, 2000, 300); // Güncelleme bittiğinde bip sesi
});
ElegantOTA.setAuth("cemyasam", "475234Cem"); // güvenlik şifresi
  server.begin();
  Serial.println("HTTP sunucu başlatıldı");
display.clearDisplay();
display.setTextSize(1);
display.setCursor(0, 0);
display.println("WiFi baglandi!");
display.println(WiFi.localIP());
display.println("OTA aktif...");
display.display();
 #endif


  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  pinMode(ENCODER_BUTTON, INPUT_PULLUP);
  pinMode(27, OUTPUT); // Şarj kontrol pini
  digitalWrite(27, HIGH); // Başlangıçta şarjı kapalı tut (veya devrenize göre ayarla)
  pinMode(BUZZER_PIN, OUTPUT);

  // Bluetooth başlatma
  SerialBT.begin("ESP32_RGB_Kontrol"); // Bluetooth cihazınızın adı
  Serial.println("Bluetooth Aktif. Cihaz adi: ESP32_RGB_Kontrol");
  SerialBT.onData(bluetoothCallback); // Gelen veri için callback fonksiyonu

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), readEncoder, CHANGE);

  for (int g = 0; g < 3; g++)
    for (int c = 0; c < 3; c++)
      setupPWM(pwmPins[g][c], pwmChannels[g][c]);

  EEPROM.begin(EEPROM_SIZE);
  loadSettingsFromEEPROM();
  applyAllColors();

  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  updateDisplay();
}

// Bluetooth'tan veri geldiğinde çağrılacak fonksiyon
void bluetoothCallback(const uint8_t *buffer, size_t len) {
  if (len > 0) {
    String commandStr = "";
    for (int i = 0; i < len; i++) {
      commandStr += (char)buffer[i];
    }
    commandStr.trim(); // Boşlukları temizle
    Serial.printf("Bluetooth Komut: '%s' (Uzunluk: %d)\n", commandStr.c_str(), len);

    // Animasyon komutları (tek karakter)
    if (commandStr.length() == 1) {
      char command = commandStr.charAt(0);
      bluetoothColorMode = false; // Animasyon komutu geldiğinde renk modu değiliz
      allGroupsOff(); // Animasyona geçerken LED'leri kapat

      if (command == 'A') {
        selectedGroup = 3; // Rainbow
      } else if (command == 'B') {
        selectedGroup = 4; // Offset Rainbow
      } else if (command == 'C') {
        selectedGroup = 5; // Fade Loop
      } else if (command == 'D') {
        selectedGroup = 6; // ON/OFF Kayma
      } else if (command == 'E') {
        selectedGroup = 7; // İz Bırakan Kayma
      } else if (command == 'O') { // Tüm LED'leri kapat
        allGroupsOff();
        selectedGroup = -2; // Bluetooth ile kapatıldığını belirtmek için özel mod
      } else if (command == 'N') { // Normal "Tümü" moduna dön
        selectedGroup = -1;
        applyAllColors(); // Renkleri geri uygula
      }
      // Daha fazla animasyon veya özel komut ekleyebilirsiniz
    }
    // RGB renk komutu (örneğin: "255,0,0" veya "0,255,0,1" veya "100,50,200,75" (R,G,B,Parlaklık))
    else if (commandStr.indexOf(',') != -1) {
      selectedGroup = 8; // Bluetooth renk modu
      bluetoothColorMode = true; // Renk ayarı yapıldığını işaretle

      int r_val, g_val, b_val;
      float bt_brightness = 1.0; // Varsayılan parlaklık (eğer komutta verilmezse)
      // Eğer şarj ekranı aktifse, otomatik olarak gizle
      if (isChargingPluggedIn()) { // Sadece şarjdayken bu önemli
          chargingDisplaySuppressed = true;
          Serial.println("BT renk komutu ile şarj ekranı gizlendi.");
      }
      // Virgüle göre ayır
      int firstComma = commandStr.indexOf(',');
      int secondComma = commandStr.indexOf(',', firstComma + 1);
      int thirdComma = commandStr.indexOf(',', secondComma + 1); // Parlaklık için dördüncü değer

      // R,G,B değerlerini ayrıştır
      r_val = commandStr.substring(0, firstComma).toInt();
      g_val = commandStr.substring(firstComma + 1, secondComma).toInt();
      if (thirdComma == -1) { // Eğer dördüncü değer yoksa (yani sadece R,G,B)
        b_val = commandStr.substring(secondComma + 1).toInt();
      } else { // Eğer dördüncü değer varsa (R,G,B,Parlaklık)
        b_val = commandStr.substring(secondComma + 1, thirdComma).toInt();
        // Parlaklık değeri 0-100 arasında gelebilir, 0.0-1.0 aralığına normalize et
        bt_brightness = commandStr.substring(thirdComma + 1).toFloat() / 100.0;
        bt_brightness = constrain(bt_brightness, 0.0, 1.0); // 0-1 aralığında tut
      }

      // Değerleri 0-255 aralığında sınırlayın
      r_val = constrain(r_val, 0, 255);
      g_val = constrain(g_val, 0, 255);
      b_val = constrain(b_val, 0, 255);

      // RGB'den HSV'ye dönüştür ve uygula
      int temp_h;
      float temp_s, temp_v;
      rgbToHsv(r_val, g_val, b_val, temp_h, temp_s, temp_v);

      // Tüm gruplara aynı hue ve Bluetooth'tan gelen parlaklığı uygula
      for (int i = 0; i < 3; i++) {
        hue[i] = temp_h;
        // Gelen RGB renginin kendi Value'su (temp_v) ile Bluetooth'tan gelen parlaklığı (bt_brightness) çarp
        brightness[i] = temp_v * bt_brightness; 
        brightness[i] = constrain(brightness[i], 0.0, 1.0); // 0-1 aralığında tut
      }
      applyAllColors(); // Yeni renkleri uygula
      Serial.printf("BT Renk Ayari: R:%d G:%d B:%d (Parlaklik: %.2f) -> H:%d B:%.2f\n", r_val, g_val, b_val, bt_brightness, hue[0], brightness[0]);
    } // 🔧 Bu parça, Bluetooth üzerinden gelen sadece parlaklık ve animasyon hızı komutlarını da işler hale getirildi.

// Mevcut kod içinde bluetoothCallback fonksiyonu altında bu kısmı BUL:
// 🔧 Bu parça, Bluetooth üzerinden gelen sadece parlaklık ve animasyon hızı komutlarını da işler hale getirildi.

// Mevcut kod içinde bluetoothCallback fonksiyonu altında bu kısmı BUL:
else if (commandStr.length() > 0) {
    // --- ANİMASYON HIZI KOMUTU ---
    char animLetter = commandStr.charAt(commandStr.length() - 1);
    if (animLetter >= 'A' && animLetter <= 'E') {
        String speedStr = commandStr.substring(0, commandStr.length() - 1);
        int speedVal = speedStr.toInt();

        if (speedVal >= 10 && speedVal <= 1000) {
            animationSpeed = speedVal;
            Serial.printf("✅ Animasyon Hizi Ayarlandi: %dms (Mod: %c)\n", animationSpeed, animLetter);

            // En son seçilen animasyon harfiyle eşleşen moda geç
            switch (animLetter) {
              case 'A': selectedGroup = 3; break;
              case 'B': selectedGroup = 4; break;
              case 'C': selectedGroup = 5; break;
              case 'D': selectedGroup = 6; break;
              case 'E': selectedGroup = 7; break;
            }

            // 🔥 Animasyonu tetikleyecek değişkenleri sıfırla
            currentGroup = 0;
            currentColorPhase = 0;
            lastAnimationTime = millis();
            allGroupsOff();
            for (int i = 0; i < 3; i++) {
              fading[i] = false;
              lastFadeTime[i] = 0;
            }

            updateDisplay();
            saveSettingsToEEPROM();
            return; // Diğer işlemleri atla
        } else {
            Serial.printf("⚠️ Gecersiz animasyon hizi: %d\n", speedVal);
            return;
        }
    }

    // --- PARLAKLIK KOMUTU (tek sayı, 0-255 arası) ---
    int numericValue = commandStr.toInt();
    if (numericValue >= 0 && numericValue <= 255) {
        float newBrightness = numericValue / 255.0;
        newBrightness = constrain(newBrightness, 0.0, 1.0);

        selectedGroup = 8;
        bluetoothColorMode = true;

        for (int i = 0; i < 3; i++) {
            int h, r = currentRGB[i][0], g = currentRGB[i][1], b = currentRGB[i][2];
            float s, v;
            rgbToHsv(r, g, b, h, s, v);
            hue[i] = h;
            brightness[i] = newBrightness;
        }

        applyAllColors();
        Serial.printf("✅ BT Parlaklik Ayari: %.2f\n", newBrightness);
        updateDisplay();
        saveSettingsToEEPROM();
        return;
    }
}



    // Ortak animasyon modlarına geçiş sonrası işlemler
    if (selectedGroup >= 3 && selectedGroup <= 7) { // Animasyon modları
      currentGroup = 0;
      currentColorPhase = 0;
      lastAnimationTime = millis(); // Animasyonu hemen başlat
      allGroupsOff(); // Önceki durumdan kalma yanık LED olmasın

      // İz bırakan animasyon için sönme durumlarını sıfırla
      for(int i = 0; i < 3; i++) {
          fading[i] = false;
          lastFadeTime[i] = 0;
      }
    } else if (selectedGroup != -2 && !bluetoothColorMode && (commandStr.length() == 1 && (commandStr.charAt(0) == 'A' || commandStr.charAt(0) == 'B' || commandStr.charAt(0) == 'C' || commandStr.charAt(0) == 'D' || commandStr.charAt(0) == 'E' || commandStr.charAt(0) == 'N'))) {
        // Animasyon seçimi veya "Normal" moda geçiş durumunda renkleri uygulamaya devam et
        // Ancak 'O' (Kapalı) komutu gelmediyse
        applyAllColors();
    } else if (bluetoothColorMode) { // Eğer Bluetooth renk modundaysak, renkleri uygula
      applyAllColors();
    }
    updateDisplay(); // Ekranı güncelle
    saveSettingsToEEPROM(); // Ayarları kaydet
  }
}

unsigned long lastButtonCheck = 0;
unsigned long lastOledUpdateGlobal = 0; // Global OLED güncelleme zamanlayıcısı

void loop() {
  unsigned long currentTime = millis(); // Loop'un başında zamanı bir kez al

  // Bluetooth bağlantı durumunu kontrol et
  static bool prevBluetoothConnected = false;
  // BluetoothSerial kütüphanesinin farklı versiyonları için uyumluluk kontrolü
  #if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 4 // ESP-IDF 4.x ve üzeri
    isBluetoothConnected = SerialBT.hasClient();
  #else // Daha eski ESP-IDF versiyonları veya Arduino ESP32 core'u
    isBluetoothConnected = SerialBT.connected();
  #endif


  if (isBluetoothConnected != prevBluetoothConnected) {
    updateDisplay(); // Bağlantı durumu değişince ekranı güncelle
    prevBluetoothConnected = isBluetoothConnected;
    if (isBluetoothConnected) {
      Serial.println("Bluetooth baglandi!");
    } else {
      Serial.println("Bluetooth baglantisi kesildi.");
    }
  }


  bool charging = isChargingPluggedIn();

  // Şarj takıldıysa VE daha önce kullanıcı tarafından gizlenmediyse şarj ekranını göster
  if (charging && !chargingDisplaySuppressed) {
    sarz_oluyor();
    return; // sarz_oluyor fonksiyonu bloke edici olduğu için burada return yapıyoruz
  }

  // Eğer artık şarjda değilse (kablo çekildiyse), gizleme bayrağını sıfırla
  if (!charging && chargingDisplaySuppressed) {
    chargingDisplaySuppressed = false; // Şarj kablosu çekildi, ekran gizleme sıfırlandı
    Serial.println("Şarj kablosu çekildi, ekran gizleme sıfırlandı.");
  }

  static bool btnPrevState = HIGH;
  bool btnState = digitalRead(ENCODER_BUTTON);

  // Rotary Encoder Okuma ve Ayar (Bluetooth ile kontrol edildiğinde devre dışı kalabilir veya öncelik verilebilir)
  // Şimdilik, Bluetooth bağlı olsa bile encoder çalışmaya devam edecek.
  if (encoderPos != lastEncoderPos) {
    int diff = encoderPos - lastEncoderPos;
    lastEncoderPos = encoderPos;

    if (digitalRead(ENCODER_BUTTON) == LOW) { // Buton basılıyken genel parlaklık ayarı
      if (selectedGroup == -1 || selectedGroup > 2 || selectedGroup == 8) { // BT renk modu için de geçerli
        float newLevel = constrain(brightness[0] + diff * 0.02, 0.0, 1.0);
        for (int i = 0; i < 3; i++) brightness[i] = newLevel;
      } else {
        brightness[selectedGroup] = constrain(brightness[selectedGroup] + diff * 0.02, 0.0, 1.0);
      }
    } else { // Buton basılı değilse:
      if (selectedGroup >= 3 && selectedGroup <= 7) { // Animasyon modları (3, 4, 5, 6, 7) için hız ayarı
        animationSpeed = constrain(animationSpeed - diff * 2, 10, 500); // 10ms-500ms arası
        Serial.printf("Animasyon Hizi: %dms\n", animationSpeed);
      } else { // Diğer modlarda renk ayarı (-1, 0, 1, 2, 8)
        if (selectedGroup == -1 || selectedGroup == 8) { // "Tümü" veya "BT Renk" modu için hue ayarı
          for (int i = 0; i < 3; i++) hue[i] = (hue[i] + diff * 5 + 360) % 360;
        } else if (selectedGroup >= 0 && selectedGroup <= 2) { // Tek grup için hue ayarı
          hue[selectedGroup] = (hue[selectedGroup] + diff * 5 + 360) % 360;
        }
      }
    }
    // Animasyon modları dışındaki durumlarda renkleri hemen uygula
    if (selectedGroup < 3 || selectedGroup == -1 || selectedGroup == 8) { // BT renk modunda da hemen uygula
      applyAllColors();
    }
    updateDisplay(); // Hız değişince OLED'i hemen güncelle
    saveSettingsToEEPROM(); // Ayarları kaydet
  }

  // Butona basılı tutma veya tıklama
  // Normal çalışma modunda buton basımını dinle (şarj ekranı aktif değilse)
  if (!chargingDisplayActive) {
    if (btnPrevState == HIGH && btnState == LOW) {
      buttonPressTime = currentTime;
    }
    if (btnPrevState == LOW && btnState == HIGH) {
      unsigned long duration = currentTime - buttonPressTime;
      if (duration >= 800) {
        // Uzun basış: parlaklık ayarı (aynı kaldı)
        int startEncoder = encoderPos;
        int currentEncoder = encoderPos;
        while (digitalRead(ENCODER_BUTTON) == LOW) { // Buton hala basılıyken
          if (encoderPos != currentEncoder) {
            int diff = encoderPos - currentEncoder;
            currentEncoder = encoderPos;
            if (selectedGroup == -1 || selectedGroup > 2 || selectedGroup == 8) { // BT renk modu için de geçerli
              for (int i = 0; i < 3; i++)
                brightness[i] = constrain(brightness[i] + diff * 0.02, 0.0, 1.0);
            } else {
              brightness[selectedGroup] = constrain(brightness[selectedGroup] + diff * 0.02, 0.0, 1.0);
            }
            applyAllColors();
            updateDisplay();
            saveSettingsToEEPROM();
          }
          delay(50); // Buton basılı tutulurken küçük bir gecikme
        }
      } else {
        // Kısa tıklama: grup/mod geçişi
        selectedGroup++;
        if (selectedGroup > 8) selectedGroup = -1; // Toplam 9 mod var (-1'den 8'e)
        
        bluetoothColorMode = false; // Manuel geçişlerde BT renk modundan çık

        // Yeni animasyon moduna geçildiğinde animasyon değişkenlerini sıfırla ve LED'leri kapat
        if (selectedGroup >= 6 && selectedGroup <= 7) {
            currentGroup = 0;
            currentColorPhase = 0;
            lastAnimationTime = currentTime; // Animasyonu hemen başlat
            allGroupsOff(); // Önceki durumdan kalma yanık LED olmasın

            // İz bırakan animasyon için sönme durumlarını sıfırla
            for(int i = 0; i < 3; i++) {
                fading[i] = false;
                lastFadeTime[i] = 0;
            }
        }

        if (selectedGroup == -1) { // "Tümü" moduna geçildiğinde tüm grupların renklerini ve parlaklıklarını eşitle
            // Başlangıçta EEPROM'dan okunan ilk grubun hue ve parlaklığını referans al
            int referenceHue = map(EEPROM.read(hueAddr[0]), 0, 255, 0, 360);
            float referenceBrightness = (float)EEPROM.read(brightnessAddr[0]) / 255.0;
            for (int i = 0; i < 3; i++) {
                hue[i] = referenceHue;
                brightness[i] = referenceBrightness;
            }
            applyAllColors(); // Renkleri hemen uygula
        } else if (selectedGroup < 6 || selectedGroup == 8) { // Animasyon modlarından çıkıldığında veya BT renk modunda renkleri geri uygula
            applyAllColors();
        }

        beepGroup(1); // Mod değişiminde kısa bip
        updateDisplay();
        saveSettingsToEEPROM();
      }
    }
  }
  btnPrevState = btnState;


  // --- Animasyon Modları ---

  // Rainbow, Offset Rainbow ve Fade Loop modları (mevcut)
  if ((selectedGroup >= 3 && selectedGroup <= 5) && currentTime - lastRainbowUpdate > animationSpeed) {
    if (selectedGroup == 3) {
      static int hueVal = 0;
      hueVal = (hueVal + 3) % 360;
      for (int i = 0; i < 3; i++) hue[i] = hueVal;
      applyAllColors();
    } else if (selectedGroup == 4) {
      static int baseHue = 0;
      int offsets[3] = {0, 120, 240};
      baseHue = (baseHue + 3) % 360;
      for (int i = 0; i < 3; i++) hue[i] = (baseHue + offsets[i]) % 360;
      applyAllColors();
    } else if (selectedGroup == 5) {
      static int groupIndex = 0;
      static bool fadeIn = true;
      static float step = 0.05;
      float maxBrightness = EEPROM.read(brightnessAddr[0]) / 255.0;
      if (maxBrightness < 0.1) maxBrightness = 0.3; // Minimum parlaklık eşiği

      if (fadeIn) {
        brightness[groupIndex] += step;
        if (brightness[groupIndex] >= maxBrightness) {
          brightness[groupIndex] = maxBrightness;
          fadeIn = false;
        }
      } else {
        brightness[groupIndex] -= step;
        if (brightness[groupIndex] <= 0.0) {
          brightness[groupIndex] = 0.0;
          fadeIn = true;
          groupIndex = (groupIndex + 1) % 3;
        }
      }
      applyAllColors();
    }
    lastRainbowUpdate = currentTime;
  }
  
  // 7. Efekt: Kesintisiz On/Off Kayma (selectedGroup == 6)
  if (selectedGroup == 6 && currentTime - lastAnimationTime >= animationSpeed) {
    lastAnimationTime = currentTime; // Zamanı sıfırla

    allGroupsOff(); // Önceki LED'i kapat

    // Sıradaki LED'i yak
    if (currentColorPhase == 0) { // Kırmızı Kayma
      setGroupColor(currentGroup, ON, OFF, OFF);
    } else if (currentColorPhase == 1) { // Mavi Kayma
      setGroupColor(currentGroup, OFF, OFF, ON);
    } else { // Yeşil Kayma
      setGroupColor(currentGroup, OFF, ON, OFF);
    }

    // Bir sonraki adıma geç
    currentGroup++;
    if (currentGroup >= 3) { // Tüm grupları geçtiysek
      currentGroup = 0;       // Başa dön
      currentColorPhase++;    // Bir sonraki renge geç
      if (currentColorPhase >= 3) { // Tüm renkleri geçtiysek
        currentColorPhase = 0;    // Başa dön (Kırmızıdan tekrar başla)
      }
    }
  }

  // 8. Efekt: İz Bırakan Kayma (selectedGroup == 7)
  if (selectedGroup == 7) {
    // Sönmekte olan LED'leri yönet
    handleFading();

    // Yeni LED'i yakma zamanı geldi mi?
    if (currentTime - lastAnimationTime >= animationSpeed) {
      lastAnimationTime = currentTime; // Zamanı sıfırla

      // Önceki LED'i sönmeye başlat
      int prevGroup = (currentGroup - 1 + 3) % 3; // Döngüsel olarak önceki grup
      if (currentGroup != 0 || currentColorPhase != 0) { // İlk yanma anında önceki yok
          startFadeOut(prevGroup);
      } else { // Animasyon ilk başladığında tüm LED'leri kapat
        allGroupsOff();
        // Aynı zamanda tüm sönme bayraklarını sıfırla
        for(int i = 0; i < 3; i++) {
          fading[i] = false;
        }
      }

      // Sıradaki LED'i yak (belirli bir renkte)
      if (currentColorPhase == 0) { // Kırmızı Kayma
        setGroupColor(currentGroup, MAX_BRIGHTNESS_VAL, MIN_BRIGHTNESS_VAL, MIN_BRIGHTNESS_VAL);
      } else if (currentColorPhase == 1) { // Mavi Kayma
        setGroupColor(currentGroup, MIN_BRIGHTNESS_VAL, MIN_BRIGHTNESS_VAL, MAX_BRIGHTNESS_VAL);
      } else { // Yeşil Kayma
        setGroupColor(currentGroup, MIN_BRIGHTNESS_VAL, MAX_BRIGHTNESS_VAL, MIN_BRIGHTNESS_VAL);
      }

      // Bir sonraki adıma geç
      currentGroup++;
      if (currentGroup >= 3) { // Tüm grupları geçtiysek
        currentGroup = 0;       // Başa dön
        currentColorPhase++;    // Bir sonraki renge geç
        if (currentColorPhase >= 3) { // Tüm renkleri geçtiysek
          currentColorPhase = 0;    // Başa dön (Kırmızıdan tekrar başla)
        }
      }
    }
  }


  // OLED Ekran Güncellemesi
  if (currentTime - lastOledUpdateGlobal >= 200) { // 200ms'de bir güncelle
    updateDisplay();
    lastOledUpdateGlobal = currentTime;
  }
}

// bu koda sağ köşeye küçük batarya iconu çizer

void drawBatteryMiniIcon(int x, int y, float voltage, bool charging) {
  const int width = 12;
  const int height = 6;
  // Küçük ikon için max 5 bar
  int fillBars = getFillBarsFromVoltage(voltage);
  if (fillBars == 0 && getNiMHBatteryPercentage(voltage) > 0) fillBars = 1; // 0% değilse en az 1 bar göster

  display.drawRect(x, y, width, height, SSD1306_WHITE);
  display.drawRect(x + width, y + 2, 2, 2, SSD1306_WHITE); // başlık

  for (int i = 0; i < fillBars; i++) {
    int bx = x + 1 + i * 2;
    display.drawFastVLine(bx, y + 1, height - 2, SSD1306_WHITE);
  }

  // Eğer şarj ediliyorsa en son çizgi yanıp sönecek
  if (charging) {
    static unsigned long lastBlink = 0;
    static bool visible = true;
    if (millis() - lastBlink > 500) {
      visible = !visible;
      lastBlink = millis();
    }
    // Maksimum bara ulaştıysa yanıp sönme olmasın
    if (visible && fillBars < (width / 2) - 1) { // Örneğin 5 bar için 4. barı kontrol et
      int bx = x + 1 + fillBars * 2;
      display.drawFastVLine(bx, y + 1, height - 2, SSD1306_WHITE);
    }
  }
}

/*

// --- drawBatteryMiniIcon Fonksiyon Tanımı (Güncellenmiş Boyutlar ve Konum) ---
// --- drawBatteryMiniIcon Fonksiyon Tanımı (Kesin Dikey 15x35px) ---
// Bu fonksiyon dikey bir pil ikonu çizecek.
void drawBatteryMiniIcon(int x, int y, float voltage, bool charging) {
  const int width = 15;  // Pil ikonunun YENİ GENİŞLİĞİ (dikey ikon için kısa kenar)
  const int height = 35; // Pil ikonunun YENİ YÜKSEKLİĞİ (dikey ikon için uzun kenar)

  // Pil gövdesini çiz
  display.drawRect(x, y, width, height, SSD1306_WHITE);

  // Pil başlığını (artı ucu) çiz (üstte olacak)
  int capWidth = 6; // Başlık genişliği
  int capHeight = 4; // Başlık yüksekliği
  int capX = x + (width - capWidth) / 2; // Yatayda ortala
  int capY = y - capHeight; // Gövdenin hemen üstüne
  display.drawRect(capX, capY, capWidth, capHeight, SSD1306_WHITE);

  // Pil seviyesini belirleyen bar sayısını hesapla (maksimum 10 bar)
  // Pil yüksekliği 35px, çerçeve için 2px düşünce 33px kullanılabilir alan kalır.
  // 10 bar için, her bar ortalama 3.3px yüksekliğinde olacak.
  // getFillBarsFromVoltage 0-10 arası değer döndürür, bu yüzden doğrudan kullanabiliriz.
  int fillBars = getFillBarsFromVoltage(voltage); 
  fillBars = constrain(fillBars, 0, 10); // Değeri 0 ile 10 arasına sınırla

  // Eğer NiMH pil kullanıyorsan ve pil neredeyse boşsa (0 bar çıktıysa) ama hala şarj varsa, en az 1 bar göster.
  // Eğer Li-Ion kullanıyorsan veya getNiMHBatteryPercentage tanımlı değilse bu satırı yorum satırı yapabilirsin.
  // if (fillBars == 0 && getNiMHBatteryPercentage(voltage) > 0) fillBars = 1; 

  // Her barın genişliğini ve yüksekliğini hesapla (çerçeve boşluklarını düşerek)
  // Barlar dikey olarak, alttan yukarıya doğru yükselecek.
  int barWidth = width - 2;          // Çerçeve içindeki genişlik
  int barHeight = (height - 2) / 10; // 10 bar için her barın yüksekliği

  // Dolu barları alttan yukarıya doğru çiz
  for (int i = 0; i < fillBars; i++) {
    // Barlar alttan yukarıya doğru yükselmeli
    // y + height - 1: Pilin alt kenarı
    // (i + 1) * barHeight: Çizilen barın yüksekliği kadar yukarı çık
    int barY = y + height - 1 - ((i + 1) * barHeight); 
    display.fillRect(x + 1, barY, barWidth, barHeight - 1, SSD1306_WHITE); // BarHeight-1 çakışmayı önler
  }

  // Şarj durumunda en üstteki bara yanıp sönme efekti ekle
  if (charging) {
    static unsigned long lastBlink = 0;
    static bool visible = true;
    
    // Her 500 milisaniyede bir görünürlük durumunu değiştir
    if (millis() - lastBlink > 500) {
      visible = !visible;
      lastBlink = millis();
    }

    // Yanıp sönecek barın indeksini belirle
    // Eğer pil tam dolu değilse (fillBars < 10), doldurulacak bir sonraki boş çubuk yanıp söner.
    // Eğer pil tam doluysa (fillBars == 10), en üstteki dolu çubuk (indeks 9) yanıp söner.
    int blinkingBarIndex;
    if (fillBars < 10) { 
        blinkingBarIndex = fillBars; // Bir sonraki boş barın indeksi
    } else { 
        blinkingBarIndex = 9; // En üstteki barın indeksi (0-9 arası 10 bar için)
    }

    // Yanıp sönecek barın Y koordinatını hesapla
    int blinkingBarY = y + height - 1 - ((blinkingBarIndex + 1) * barHeight);

    // Sadece geçerli bir bar indeksi ise ve pil çerçevesinin içindeyse çizim yap
    if (blinkingBarIndex >= 0 && blinkingBarIndex < 10 && blinkingBarY >= y + 1) {
        if (visible) {
            display.fillRect(x + 1, blinkingBarY, barWidth, barHeight - 1, SSD1306_WHITE);
        } else {
            // Görünmezse, o barı siyahla doldurarak sil
            display.fillRect(x + 1, blinkingBarY, barWidth, barHeight - 1, SSD1306_BLACK); 
        }
    }
  }
}*/
void drawBatteryWithFillBars(float voltage, bool charging) {
  display.clearDisplay();

  // Pil dış çerçevesi
  const int batteryWidth = SCREEN_WIDTH - 40;    // 88
  const int batteryHeight = SCREEN_HEIGHT - 40; // 24
  const int batteryX = (SCREEN_WIDTH - batteryWidth) / 2;    // 20
  const int batteryY = (SCREEN_HEIGHT - batteryHeight) / 2; // 20

  // Gövde ve başlık çiz
  display.drawRect(batteryX, batteryY, batteryWidth, batteryHeight, SSD1306_WHITE);
  display.drawRect(batteryX + batteryWidth, batteryY + (batteryHeight - 8) / 2, 6, 8, SSD1306_WHITE);

  // Doluluk çizgileri
  int bars = getFillBarsFromVoltage(voltage);    // Kaç çizgi dolacak
  const int barWidth = 8;
  const int barHeight = 20;
  const int spacing = 1;
  const int totalBars = 10;
  const int barAreaWidth = totalBars * (barWidth + spacing) - spacing;

  // Ortalamak için X başlangıç pozisyonu
  int startX = batteryX + (batteryWidth - barAreaWidth) / 2;
  int barY = batteryY + (batteryHeight - barHeight) / 2;

  for (int i = 0; i < bars; i++) {
    int x = startX + i * (barWidth + spacing);

    // Son bar mı ve şarj ediliyor mu?
    if (i == bars - 1 && charging && bars < totalBars) { // Son bara ulaştıysa yanıp sönme olmasın
      static unsigned long lastBlink = 0;
      static bool visible = true;
      if (millis() - lastBlink > 500) {
        visible = !visible;
        lastBlink = millis();
      }
      if (visible) {
        display.fillRect(x, barY, barWidth, barHeight, SSD1306_WHITE);
      }
    } else {
      display.fillRect(x, barY, barWidth, barHeight, SSD1306_WHITE);
    }
  }

  // Voltajı ekranda göster
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(batteryX + 3, batteryY + batteryHeight + 5); // Pilin altına
  display.printf("Voltage: %.2fV", voltage);
  display.display();
}