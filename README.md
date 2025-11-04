# Allmess Integral MK UltraMaXX (optischer M-Bus) – ESPHome / ESP32-C6

ESPHome External Component zum Auslesen eines **Allmess Integral-MK UltraMaXX** Wärmezählers
über die **optische M-Bus-Schnittstelle** mit korrektem **Wake-Up**, **Parity-Wechsel (8N1 → 8E1)**,
**Init/Request (SND_NKE + REQ_UD2)** und Dekodierung typischer Messwerte.

## Hardware

- **ESP32-C6 WROOM** oder kompatibel
- **Optokopf** nach Standard-Schaltung:
  - IR-LED (z. B. SIR 204 EVL) mit 220 Ω Vorwiderstand
  - Phototransistor (SFH 309 FA) mit 10 kΩ Pull-Up
  - TX = GPIO 20 → IR-LED
  - RX = GPIO 21 ← Phototransistor
  - Gemeinsame Masse (GND)
- Kommunikation: 2400 Bd, 8E1 (nach Wake-Up)

## Funktionsweise

1. Wake-Up: 0x55-Muster ~2,2 s @ 2400 Bd 8N1  
2. Pause: 100 ms  
3. Wechsel auf 2400 Bd 8E1  
4. `SND_NKE` → `E5` (Init-Handshake)  
5. `REQ_UD2` (Anforderung der Daten)  
6. Long-Frame wird dekodiert (VIF 0x06 Energie, 0x13 Volumen, 0x5B/0x5F Temperaturen, 0x2B Leistung, 0x3B Durchfluss)  

## Installation

1. Kopiere den Ordner `waermemessung/` in dein ESPHome-Verzeichnis (z. B. `/config/esphome/` in Home Assistant).  
2. Öffne `waermemessung.yaml` im ESPHome Dashboard.  
3. Passe WLAN-Zugangsdaten an.  
4. Kompiliere und flashe dein ESP32-C6-Board.

Nach erfolgreicher Kommunikation erscheinen in Home Assistant folgende Sensoren:
- Wärmeenergie (kWh)
- Volumen (m³)
- Vorlauf-Temperatur (°C)
- Rücklauf-Temperatur (°C)
- Leistung (W)
- Durchfluss (l/h)

---

## Beispiel-Foto Anschluss
