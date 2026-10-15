# Morsedecoder

CW-(Morse-)Decoder und -Geber für den ESP32 mit farbigem SPI-Display. Der Decoder hört das NF-Signal eines Empfängers oder WebSDR, erkennt den Ton per Goertzel-Filter und zeigt den dekodierten Text groß an, dazu ein Laufband mit Punkten/Strichen, den erkannten Zeichen und einen Frequenz-Bargraphen.

Firmware-Stand: **v44**

## Hardware

- **Board:** ESP32 DevKit V4 (ESP32-WROOM-32, 4 MB Flash)
- **Display:** ST7789, 240 × 320 Pixel, Hochformat (ILI9341 und ST7735 werden ebenfalls unterstützt)
- **Audio-Eingang:** GPIO 34 (ADC1_CH6), Kopplung über 1 µF, Mittenspannung über 2 × 10 kΩ an 3V3/GND
- **Mithör-Ausgang:** GPIO 25 (DAC1) über 10 µF in Reihe und ca. 220 Ω zum Kopfhörer/Verstärker
- **Morsetaste:** GPIO 27 gegen GND (optional)
- **Funktionstaste:** GPIO 33 gegen GND (optional; kurz = Text löschen, lang = Echo)

### Pinbelegung Display (aus `display_config.h`)

| Display | ESP32 |
|---|---|
| SCL/SCK | GPIO 14 |
| SDA/MOSI | GPIO 13 |
| CS | GPIO 26 |
| DC | GPIO 2 |
| RST | GPIO 4 |
| BL | fest an 3V3 (`PIN_TFT_BL -1`) |
| VCC / GND | 3V3 / GND |

MISO wird nicht benötigt.

## Funktionen

- **Goertzel-Tonerkennung** mit 20 kHz Abtastrate, Hochpass gegen Brumm und Knacken
- **Auto-Lock / AFC:** rastet automatisch auf das stärkste CW-Signal im Bereich ca. 600–800 Hz ein und führt nach (`/afc`, `/qsy`, `/afcsw`)
- **Adaptive Rauschsperre und Schwelle:** Schwelle zwischen Rauschboden und Spitzenpegel, einstellbar (`/sq`, `/thrmid`, `/hyst`), Unterdrückung von Breitband-Klicks
- **Handtasten-Timing:** erkennt Handtaste vs. Paddle und arbeitet bei Handtasten mit toleranteren Zeitgrenzen (`/hand`)
- **Pausen-Modell `/stoch`:** unklare Zeichenpausen (1,4–2,2 Punktlängen) werden anhand der Pausenstatistik des Gebers und der Buchstabenhäufigkeit getrennt
- **Laufband mit Zeichenzeile:** Punkte/Striche laufen zeitsynchron durch, darunter die erkannten Zeichen; das Tempo passt sich der WPM an (`/stripauto`)
- **Frequenz-Bargraph** zum Abstimmen
- **Geber:** Text im seriellen Monitor wird als Morse ausgegeben
- **Mithören** über den DAC: getasteter Ton oder gefiltertes Audio
- **`display_config.h`:** alle Display-Einstellungen (Typ, Auflösung, SPI-Takt, Drehung, Spiegelung, Pins, Farben, Layout) in einer eigenen Datei. Bei Updates nur die `.ino` austauschen und die eigene `display_config.h` behalten. Fehlt die Datei, gelten die Standardwerte des Referenzaufbaus.

Alle Befehle: siehe [BEFEHLE.md](BEFEHLE.md).

## Bauen und Flashen (Arduino IDE)

1. ESP32-Boardpaket von Espressif installieren (getestet mit esp32 3.2.0).
2. Board **„ESP32 Dev Module“** wählen.
3. Benötigte Bibliotheken über den Bibliotheksverwalter installieren:
   - Adafruit GFX Library
   - Adafruit ST7735 and ST7789 Library (bzw. Adafruit ILI9341)
   - Adafruit BusIO (wird automatisch mitinstalliert)
4. Den Ordner `morse_esp32` öffnen (enthält `morse_esp32.ino` und `display_config.h`).
5. `display_config.h` an das eigene Display anpassen.
6. Seriellen Port wählen (z. B. COM3 unter Windows) und hochladen.
7. Seriellen Monitor mit 115200 Baud öffnen, `/help` zeigt die Befehle.

Mit arduino-cli:

```
arduino-cli compile --fqbn esp32:esp32:esp32 morse_esp32
arduino-cli upload  --fqbn esp32:esp32:esp32 -p <PORT> morse_esp32
```

## Tipps

- Pegel: Die Kopfzeile meldet „PEGEL ZU NIEDRIG/HOCH“, dann die Lautstärke am Empfänger anpassen.
- Bei falschen Farben, gespiegelter Schrift oder Rand die Abschnitte 1–5 in `display_config.h` prüfen.
