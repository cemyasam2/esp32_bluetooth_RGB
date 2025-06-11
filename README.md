# 💡 esp32 LED Animasyon Projesi

Bu proje, esp32 kullanarak RGB LED’ler üzerinde animasyonlar oluşturur.  
Rotary encoder ile animasyonlar arasında geçiş yapılabilir. OLED ekran ve EEPROM desteği ile tam bir görsel deneyim sağlar! bluetooth üzerinden android telefonlar ile kontrol edilebnilir.
aşşagıdaki adresteki app ile kontrol edilebilir.

https://play.google.com/store/apps/details?id=easy.tech.rgbledcontrol&hl=tr

## 🎥 YouTube Videosu
👉 [Projeyi buradan izleyebilirsin](https://www.youtube.com/shorts/8XcT7dM-Kzo)

---

## 🔧 Kullanılan Malzemeler
-  ESP32
- 170cm RGB LED
- 1x Rotary Encoder
- 0.96" OLED Ekran (I2C)
- rotary encoder
- EEPROM (dahili)
- 3D baskı kutu (dosya eklidir)
ID	Name	Designator	Footprint	Quantity	Manufacturer Part	Manufacturer	Supplier	Supplier Part	Price
"1"	"10K"	"R16"	"R_AXIAL-0.4"	"1"	""	""	""	""	""
"2"	"1N4007"	"D2"	"DO-41_BD2.4-L4.7-P8.70-D0.9-RD"	"1"	"1N4007"	"SEMTECH"	"LCSC"	"C106903"	"0.019"
"3"	"H2"	"OLEDEKRAN"	"HDR-F-2.54_1X4"	"1"	""	""	"LCSC"	"C225501"	"0.077"
"4"	"H1"	"ROTARYSWITCH"	"HDR-F-2.54_1X5"	"1"	""	""	"LCSC"	"C50950"	"0.064"
"5"	"HDR-F-2.54_1x2"	"CHARGE_IN,POWER"	"HDR-F-2.54_1X2"	"2"	""	""	"LCSC"	"C49661"	"0.038"
"6"	"BC547"	"Q10"	"BC547"	"1"	""	""	""	""	""
"7"	"BUZZER"	"SG1"	"BUZZER-12MM"	"1"	""	""	""	""	""
"8"	"L7805"	"U1"	"L7803"	"1"	""	""	""	""	""
"9"	"ESP32 DEVKIT"	"U2"	"ESP32DEVKITCV4"	"1"	"ESP32 Dev Kit C V4"	"AZ-DELIVERY"	""	""	""
"10"	"200U"	"C1,C2"	"CAP-D6.3×F2.5"	"2"	""	""	""	""	""
"11"	"100NF"	"C3,C5"	"C1206"	"2"	""	""	""	""	""
"12"	"1N4007W"	"D1"	"SOD-123_L2.8-W1.8-LS3.7-RD"	"1"	"1N4007W"	"BLUE ROCKET"	"LCSC"	"C328592"	"0.008"
"13"	"22k"	"R12,R14"	"R1206"	"2"	""	""	""	""	""
"14"	"5k"	"R13,R15"	"R1206"	"2"	""	""	""	""	""
"15"	"10k"	"R18"	"R1206"	"1"	""	""	""	""	""
"16"	"1k"	"R1,R2,R3,R4,R5,R6,R7,R8,R9,R10,R11"	"R1210"	"11"	""	""	""	""	""
"17"	"NJVMJD122T4G"	"Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q12"	"DPAK-3"	"10"	""	""	""	""	""


---

## 🧠 Özellikler
- Renk geçişli animasyonlar
- Encoder ile geçiş kontrolü
- EEPROM’a ayar kaydı
- Şarj durumu takibi
- OLED'de animasyon & pil simgeleri
- bluetooth kontrol

---

## 🖥️ Kod
Ana kod dosyasını `esp32_bluetooth_RGB_.ino` olarak yükleyebilirsin.  


---

## 🖨️ 3D Dosyalar
-[📦 STL dosyasını indir](difizor.stl)
-[📦 STL dosyasını indir](difizorbitis baslangic.stl)
-[📦 STL dosyasını indir](halka ek.stl)
-[📦 STL dosyasını indir](led tutucu.stl)
-[📦 STL dosyasını indir](rgb dikey lamba atma.stl)
-[📦 STL dosyasını indir](rotary tutucua.stl)
-[📦 STL dosyasını indir](taban kapak lamba.stl)
-[📦 STL dosyasını indir](taban.stl)

## 🔌 Devre Şeması
- topprint.pdf
top.pdf
back.pdf
Gerber_rgbcubuk2_PCB_rgbcubuk2_2025-06-11.zip
boardback.jpg
boardfront.jpg
BOM_rgbcubuk2_2025-06-11.csv
PickAndPlace_PCB_rgbcubuk2_2025-06-11.csv
---

## 📄 Lisans
MIT Lisansı altında paylaşılmıştır. Dilediğiniz gibi kullanabilirsiniz.

---

## ☕ Destek Ol
Eğer projeyi beğendiysen, YouTube kanalıma abone olmayı unutma!  
[👉 TinkTime YouTube Kanalı](https://youtube.com/@tinktime)

