// ==========================================================================
//  display_config.h  –  ALLE Display-Einstellungen des Morse-Decoders
// --------------------------------------------------------------------------
//  Diese Datei gehört DIR. Bei einem Update bitte nur die neue
//  morse_esp32.ino in den Sketch-Ordner kopieren und diese Datei behalten.
//  Fehlt ein Eintrag hier, verwendet die .ino ihren eigenen Standardwert
//  (= Referenzaufbau: ST7789 240x320 Hochformat am ESP32 DevKit V4).
//
//  Verwendete Bibliotheken: Adafruit GFX + Adafruit ST7735 and ST7789
//  (bzw. Adafruit ILI9341). KEIN TFT_eSPI – es gibt also keine
//  User_Setup.h in der Library, alles wird hier eingestellt.
// ==========================================================================
#pragma once

// --------------------------------------------------------------------------
//  1) Display-Typ – genau EINE Zeile aktiv lassen
// --------------------------------------------------------------------------
//#define DISPLAY_ILI9341    // 2,2"/2,4"/2,8" 240x320
//#define DISPLAY_ST7735     // 1,8" 128x160
#define DISPLAY_ST7789       // ST7789 (z. B. 240x320 oder 240x280)

// Nur ST7735: Modulvariante (bei falschen Farben/Rand andere probieren)
//   INITR_BLACKTAB, INITR_GREENTAB, INITR_REDTAB, INITR_144GREENTAB
#define ST7735_TAB  INITR_BLACKTAB

// Nur ST7789: Auflösung des Panels (Breite x Höhe im Hochformat).
//   Die Adafruit-Bibliothek setzt die passenden Speicher-Offsets selbst
//   (z. B. 240x280 -> 20 Zeilen Versatz).
#define ST7789_W  240
#define ST7789_H  320

// --------------------------------------------------------------------------
//  2) SPI-Takt
// --------------------------------------------------------------------------
#define TFT_SPI_HZ  27000000UL   // 27 MHz; bei Bildfehlern z. B. 20000000UL

// --------------------------------------------------------------------------
//  3) Ausrichtung, Spiegelung, Farben des Panels
// --------------------------------------------------------------------------
// Drehung: 0 = Hochformat, 1 = Querformat, 2/3 = jeweils um 180° gedreht
#define DISPLAY_ROTATION 0

// Spiegelung (nur ST7789). Falls Schrift gespiegelt/auf dem Kopf steht:
//   MIRROR_X 1 -> links/rechts spiegeln,  MIRROR_Y 1 -> oben/unten spiegeln,
//   beide 1    -> 180° drehen
#define MIRROR_X  0
#define MIRROR_Y  1   // am Referenzmodul richtig (MADCTL 0x80)

// Farbinversion (nur ST7789): erscheint Schwarz als Weiß bzw. Grün als
// Magenta, hier umschalten (true = Inversion an)
#define TFT_INVERT false

// Rand in Pixeln (z. B. für Displays mit abgerundeten Ecken)
#define MARGIN_X  10
#define MARGIN_Y  8

// Layout unten (v42):
//   BAR_PLOT_H   Höhe der Frequenzbalken in Pixeln (bis v41: 50)
//   CHR_ROW_H    Höhe der Zeichenzeile zwischen Elementen und Frequenzband
//                (0 = Zeile aus); die erkannten Zeichen laufen dort unter
//                ihren Punkten/Strichen mit
//   CHR_TXT_SIZE Schriftfaktor der Zeichenzeile (2 = 12x16 Pixel, 1 = klein)
#define BAR_PLOT_H   32
#define CHR_ROW_H    20
#define CHR_TXT_SIZE 2

// v43: Laufband-Tempo folgt der erkannten WPM (/stripauto on|off)
//   STRIP_DOT_PX  Ziel-Breite eines Punktes in Pixeln (5 = bei 20 WPM 12 ms/Pixel)
//   STRIP_MS_MIN / STRIP_MS_MAX  Grenzen in ms pro Pixel (schnell / langsam)
#define STRIP_DOT_PX 5
#define STRIP_MS_MIN 4
#define STRIP_MS_MAX 30

// Schriftgröße (Faktor des 6x8-Standardfonts). Auskommentiert lassen =
// Standard je Display-Typ (ILI9341/ST7789: 2, ST7735: 1)
//#define TXT_SIZE 2

// --------------------------------------------------------------------------
//  4) Pinbelegung des Displays (ESP32 DevKit V4)
// --------------------------------------------------------------------------
#define PIN_TFT_SCK    14   // SPI SCLK    -> Display SCL/SCK
#define PIN_TFT_MOSI   13   // SPI MOSI    -> Display SDA/MOSI
#define PIN_TFT_CS     26   // Chip Select -> Display CS
#define PIN_TFT_DC      2   // Data/Command-> Display DC (GPIO 2 = Strapping-Pin, geht hier)
#define PIN_TFT_RST     4   // Reset       -> Display RST
#define PIN_TFT_BL     -1   // Hintergrundbeleuchtung: -1 = BL fest an 3V3;
                            //   sonst GPIO eintragen (wird beim Start HIGH)
// MISO wird nicht benötigt.

// --------------------------------------------------------------------------
//  5) Farben (RGB565). Hinweis: am Referenzmodul sind Rot und Blau
//     vertauscht; RB(c) liefert den Code, der richtig erscheint.
//     Ist dein Modul "normal", die Farben hier ggf. anpassen.
// --------------------------------------------------------------------------
#define RB(c) ((uint16_t)((((c) & 0x1F) << 11) | ((c) & 0x07E0) | (((c) >> 11) & 0x1F)))
#define C_BG      0x0000      // schwarz   – Hintergrund
#define C_HEAD    0x001F      // Kopfzeile
#define C_TEXT    0xFFFF      // weiß      – empfangener Text
#define C_SENT    0xFFE0      // gelb      – gesendeter Text
#define C_SYM     0x07E0      // grün      – Punkte/Striche
#define C_INFO    0xC618      // hellgrau  – Infos
#define C_BAR     0x07E0      // grün      – Bargraph-Balken
#define C_BARTGT  0xFFE0      // gelb      – Balken der Zielfrequenz
#define C_HOLD    0xF800      // Peak-Hold-Marke
#define C_DICT    RB(0x07FF)  // cyan      – per Wörterbuch ergänztes Wort
#define C_GRID    0x4208      // dunkelgrau – Raster/Mittenmarke
