/*
 * ==========================================================================
 *  Morse-Decoder / Morse-Geber für ESP32 mit SPI-TFT (ILI9341 / ST7735 / ST7789)
 * ==========================================================================
 *
 *  Board : AZ-Delivery ESP32 DevKit V4 (ESP32-WROOM-32, ESP32-D0WD-V3, 4 MB Flash)
 *  Display: 1,69" ST7789V2, 240x280 (Pins/Einstellungen aus PlatformIO-Projekt "Test ST7789")
 *  Arduino-Board-Einstellung: "ESP32 Dev Module"  (FQBN esp32:esp32:esp32)
 *
 *  Funktionen
 *  ----------
 *  DECODER  : Morsetaste (Taster gegen GND) an GPIO 27.
 *             - Entprellung in Software
 *             - Punkt/Strich-Erkennung über die Tastdauer, adaptive Tempo-
 *               schätzung (WPM passt sich automatisch an)
 *             - Zeichen- und Wortpausen werden erkannt
 *             - A–Z, 0–9 und gängige Satzzeichen
 *             - Ausgabe auf dem Display (aktuelle Punkte/Striche, Text mit
 *               Zeilenumbruch, WPM) und auf Serial (115200 Baud)
 *             - Mithörton über den Summer, solange die Taste gedrückt ist
 *
 *  GEBER    : Text im seriellen Monitor eingeben (Zeilenende "Neue Zeile")
 *             -> wird als Morse über Summer und LED ausgegeben und angezeigt.
 *             Befehle im seriellen Monitor:
 *               /wpm 18     Geber-Tempo setzen (5..40 WPM)
 *               /ton 650    Tonhöhe in Hz setzen (300..1500)
 *               /clear      Display-Text löschen
 *               /hand auto|on|off  Handtaste/Paddle-Erkennung
 *               /qsy on|off|<ms>  AFC-Wechsel nach Durchgang
 *               /hyst <f>      Aus-Schwelle (0.6)
 *               /ratio on|off|<r>  Ton/Breitband-Zusatzkriterium
 *               /thrmid <f>    Schwellenlage (0.5)
 *               /thrdecay <n>  Schwellen-Abfall (Punktlaengen)
 *               /wbrej on|off|<dB>  Breitband-Klick-Unterdrueckung
 *               /scope on|off   Streifen Pegel+Elemente (zeitsynchron)
 *               /ticker on|off  = /scope
 *               /stripauto on|off  Laufband-Tempo folgt der WPM (v43)
 *               /help       Hilfe anzeigen
 *
 *  AUDIO-EINGANG (GPIO 34, ADC1_CH6): Morse-Ton z. B. vom Kopfhörerausgang
 *             eines Tablets (Kopplung 1 µF, Bias 2x10 kOhm an 3V3/GND).
 *             - ADC im Continuous-Mode mit 20 kHz (Minimum des ESP32)
 *             - Goertzel-Filter auf die Zielfrequenz (Standard 700 Hz),
 *               Blöcke von ca. 8 ms
 *             - adaptive Schwelle (Rauschboden + Spitzenwert, Hysterese)
 *             - Ergebnis wirkt wie die Morsetaste (ODER-verknüpft)
 *             Befehle:
 *               /freq 700   Zielfrequenz Audio in Hz (300..1500)
 *               /audio      Pegel-Debug an/aus (Zeilen beginnen mit "#A")
 *               /src both   Quelle: key | audio | both
 *               /stat       einmalige Statuszeile (#S ...)
 *
 *  FUNKTIONSTASTE (GPIO 33, Taster gegen GND):
 *               kurz drücken  (< 1 s) : Text löschen
 *               lang drücken  (>= 1 s): bisher dekodierten Text als Morse
 *                                       wieder ausgeben (Echo)
 *             Während der Geber läuft, bricht ein Druck auf die Morsetaste
 *             oder die Funktionstaste die Ausgabe ab.
 *
 *  Benötigte Bibliotheken (Bibliotheksverwalter):
 *    - Adafruit GFX Library
 *    - Adafruit ILI9341                        (für ILI9341)
 *    - Adafruit ST7735 and ST7789 Library      (für ST7735 / ST7789)
 *    - Adafruit BusIO (wird automatisch mitinstalliert)
 *    - ESP Async WebServer + Async TCP (ESP32Async, v45 WLAN/Webseite)
 *  WLAN: wifi_config.h im Sketch-Ordner (SSID/Passwort, nicht weitergeben)
 */

#include <SPI.h>
#include <Preferences.h>
#include "driver/gptimer.h"
#include "driver/dac_oneshot.h"
#include "hal/dac_ll.h"
#include <algorithm>
#include <Adafruit_GFX.h>

// ==========================================================================
//  v45 WLAN + Webseite (Schritt 1: Live-Anzeige + Befehlszeile)
//  Zugangsdaten in wifi_config.h (Sketch-Ordner, NICHT weitergeben/ins Repo).
//  Fehlt die Datei oder steht dort noch "HIER_PASSWORT", läuft der Decoder
//  ohne WLAN; Zugangsdaten gehen dann auch per /wifi ssid ... / /wifi pass ...
// ==========================================================================
#include <WiFi.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#if __has_include("wifi_config.h")
  #include "wifi_config.h"
#endif
#ifndef WIFI_SSID
  #define WIFI_SSID ""
#endif
#ifndef WIFI_PASS
  #define WIFI_PASS ""
#endif
#ifndef WIFI_HOSTNAME
  #define WIFI_HOSTNAME "morsedecoder"
#endif

// Serielle Ausgabe 1:1 durchreichen und für Web-Befehle zusätzlich mitschreiben
class SerTee : public Stream {
 public:
  HardwareSerial &h; String *cap = nullptr;
  explicit SerTee(HardwareSerial &s) : h(s) {}
  void begin(unsigned long b) { h.begin(b); }
  size_t write(uint8_t c) override { if (cap && cap->length() < 3000) *cap += (char)c; return h.write(c); }
  size_t write(const uint8_t *b, size_t n) override { if (cap) for (size_t i = 0; i < n && cap->length() < 3000; i++) *cap += (char)b[i]; return h.write(b, n); }
  int available() override { return h.available(); }
  int read() override { return h.read(); }
  int peek() override { return h.peek(); }
  void flush() override { h.flush(); }
};
SerTee SerialTee(Serial0);
#undef Serial
#define Serial SerialTee
// v45 Jitter-/Lastmessung Audio-Task (/stat #J, /wifi status)
volatile uint32_t jItMaxUs = 0, jGap20 = 0, jBackMax = 0, jOvf = 0, jIt = 0;

// ==========================================================================
//  1)+2) DISPLAY-EINSTELLUNGEN  -> stehen in display_config.h (Sketch-Ordner)
//  Bei Updates der .ino bleibt die eigene display_config.h erhalten.
//  Fehlende Einträge bekommen hier Standardwerte (Referenzaufbau).
// ==========================================================================
#if __has_include("display_config.h")
  #include "display_config.h"
#endif
#if !defined(DISPLAY_ILI9341) && !defined(DISPLAY_ST7735) && !defined(DISPLAY_ST7789)
  #define DISPLAY_ST7789
#endif
#ifndef ST7735_TAB
  #define ST7735_TAB INITR_BLACKTAB
#endif
#ifndef ST7789_W
  #define ST7789_W 240
#endif
#ifndef ST7789_H
  #define ST7789_H 320
#endif
#ifndef TFT_SPI_HZ
  #define TFT_SPI_HZ 27000000UL
#endif
#ifndef DISPLAY_ROTATION
  #define DISPLAY_ROTATION 0
#endif
#ifndef MIRROR_X
  #define MIRROR_X 0
#endif
#ifndef MIRROR_Y
  #define MIRROR_Y 1
#endif
#ifndef TFT_INVERT
  #define TFT_INVERT false
#endif
#ifndef MARGIN_X
  #define MARGIN_X 10
#endif
#ifndef MARGIN_Y
  #define MARGIN_Y 8
#endif
#ifndef BAR_PLOT_H
  #define BAR_PLOT_H 32      // v42: Höhe der Frequenzbalken in Pixeln (bis v41: 50)
#endif
#ifndef STRIP_DOT_PX
  #define STRIP_DOT_PX 5     // v43: Ziel-Breite eines Punktes im Laufband (Pixel), /stripauto
#endif
#ifndef STRIP_MS_MIN
  #define STRIP_MS_MIN 4     // v43: schnellstes Laufband (ms pro Pixel)
#endif
#ifndef STRIP_MS_MAX
  #define STRIP_MS_MAX 30    // v43: langsamstes Laufband (ms pro Pixel)
#endif
#ifndef CHR_ROW_H
  #define CHR_ROW_H 20       // v42: Zeichenzeile unter den Elementen (0 = aus)
#endif
#ifndef CHR_TXT_SIZE
  #define CHR_TXT_SIZE 2     // v42: Schriftfaktor der Zeichenzeile (2 = 12x16 px)
#endif
#ifndef PIN_TFT_SCK
  #define PIN_TFT_SCK 14
#endif
#ifndef PIN_TFT_MOSI
  #define PIN_TFT_MOSI 13
#endif
#ifndef PIN_TFT_CS
  #define PIN_TFT_CS 26
#endif
#ifndef PIN_TFT_DC
  #define PIN_TFT_DC 2
#endif
#ifndef PIN_TFT_RST
  #define PIN_TFT_RST 4
#endif
#ifndef PIN_TFT_BL
  #define PIN_TFT_BL -1
#endif

// ==========================================================================
//  Weitere Pinbelegung (Taste, Audio, Summer)
// ==========================================================================

#define PIN_KEY        27   // Morsetaste, Taster gegen GND (INPUT_PULLUP)
#define PIN_FUNC       33   // Funktionstaste, Taster gegen GND (INPUT_PULLUP)
                            //   (verlegt von 26, weil 26 jetzt Display-CS ist)
#define PIN_AUDIO      34   // Audio-Eingang (nur Eingang, ADC1 Kanal 6)
                            //   1 µF Koppelkondensator, Bias 2x10 kOhm
#define PIN_BUZZER     25   // Summer (passiv empfohlen), andere Seite GND
#define PIN_LED        -1   // Geber-LED: GPIO 2 ist DC -> LED deaktiviert.
                            //   Optional externe LED + 330 Ohm an GPIO 22
                            //   und hier 22 eintragen.

// Summer-Typ: passiver Summer/Lautsprecher -> Ton per PWM (tone()).
// Bei aktivem Summer (piept selbst bei Gleichspannung) auf 0 setzen.
#define BUZZER_PASSIVE  1

// ==========================================================================
//  3) EINSTELLUNGEN
// ==========================================================================
#define SERIAL_BAUD        115200
#define DEBOUNCE_MS        10      // Entprellzeit Tasten
#define START_WPM          15      // Start-Schätzung Decoder-Tempo
#define MIN_WPM            5
#define MAX_WPM            40
#define WORDGAP_MAX_U 8.0f
#define WORDGAP_RESET_MS 10000
#define WG_HI1 6.5f
#define WG_HI2 5.0f
#define CHAR_GAP_UNITS 2.2f
#ifndef ISO_GAP_U
#define ISO_GAP_U  8.0f   // v24: Einzelpunkt nach > 8 Einheiten Stille ist verdaechtig (vorher 2 s)
#endif
#ifndef ISO_DROP_U
#define ISO_DROP_U 5.0f   // ... und wird verworfen, wenn danach > 5 Einheiten nichts folgt (vorher 7)
#endif
#define WORD_GAP_UNITS     5.0f    // Startwert Wortpause (danach adaptiv, s. wordGapUnits())
#define LONG_PRESS_MS      1000    // Funktionstaste lang

// Audio-Decoder
#define AUDIO_FS_HZ        20000   // ADC-Abtastrate (ESP32: min. 20 kHz)
#define AUDIO_BLOCK_MS     8       // Goertzel-Blocklänge in ms
#define AUDIO_FREQ_HZ      700     // Zielfrequenz (Standard)
#define AUDIO_MIN_MAG      5.0f    // absolute Mindestschwelle (ADC-Counts Amplitude); v24: 12 -> 5 (leise Eingaenge)
#define AUDIO_SNR_ON       4.0f    // Ton erst ab 4x Rauschboden
#define AUDIO_HYST         0.6f    // Aus-Schwelle = 60 % der Ein-Schwelle
#define AUDIO_HP_HZ        200.0f  // Hochpass vor dem Goertzel (gegen Knacken/Brumm)
#define AUDIO_TONAL_ON     0.40f   // Anteil der Zielfrequenz an der Blockenergie (Ein)
#define NF_TONAL           0.90f   // Tonanteil, ab dem der Rauschboden nicht steigt
#define NF_BLOCK_N         60      // Blöcke (~0,5 s) "Ton, aber nicht erkannt" bis zum Absenken
#define AUDIO_TONAL_HOLD   0.20f   // ... zum Halten (Hysterese)
#define AUDIO_SIDETONE     0       // 1 = Summer piept auch bei Audio-Eingang

// Selbsttest ohne Tablet: 1 = ADC liest GPIO 32 statt 34, und GPIO 32 erzeugt
// selbst Morse ("PARIS SOS 0123456789") als 700-Hz-Rechteck. Prüft ADC,
// Goertzel, Schwelle und Decoder ohne externe Verdrahtung. Normal: 0.
#ifndef AUDIO_SELFTEST
#define AUDIO_SELFTEST     0
#endif
#ifndef SELFTEST_ALLCHARS
#define SELFTEST_ALLCHARS  0
#endif
#define SELFTEST_PIN       32
// Selbsttest-Tempo wechselt pro Durchlauf: 12, 18, 25 WPM

// ==========================================================================
//  Display-Objekt anlegen
// ==========================================================================
#if defined(DISPLAY_ILI9341)
  #include <Adafruit_ILI9341.h>
  Adafruit_ILI9341 tft(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
  #ifndef TXT_SIZE
    #define TXT_SIZE 2
  #endif
#elif defined(DISPLAY_ST7735)
  #include <Adafruit_ST7735.h>
  Adafruit_ST7735 tft(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
  #ifndef TXT_SIZE
    #define TXT_SIZE 1
  #endif
#elif defined(DISPLAY_ST7789)
  #include <Adafruit_ST7789.h>
  Adafruit_ST7789 tft(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
  #ifndef TXT_SIZE
    #define TXT_SIZE 2
  #endif
#else
  #error "Bitte genau einen Display-Typ (DISPLAY_ILI9341/ST7735/ST7789) wählen"
#endif

// Farben (RGB565) – Standardwerte, überschreibbar in display_config.h
#ifndef RB
  #define RB(c) ((uint16_t)((((c) & 0x1F) << 11) | ((c) & 0x07E0) | (((c) >> 11) & 0x1F)))
#endif
#ifndef C_BG
  #define C_BG 0x0000
#endif
#ifndef C_HEAD
  #define C_HEAD 0x001F
#endif
#ifndef C_TEXT
  #define C_TEXT 0xFFFF
#endif
#ifndef C_SENT
  #define C_SENT 0xFFE0
#endif
#ifndef C_SYM
  #define C_SYM 0x07E0
#endif
#ifndef C_INFO
  #define C_INFO 0xC618
#endif
#ifndef C_BAR
  #define C_BAR 0x07E0
#endif
#ifndef C_BARTGT
  #define C_BARTGT 0xFFE0
#endif
#ifndef C_HOLD
  #define C_HOLD 0xF800
#endif
#ifndef C_DICT
  #define C_DICT RB(0x07FF)
#endif
#ifndef C_GRID
  #define C_GRID 0x4208
#endif

// ==========================================================================
//  Morse-Tabelle
// ==========================================================================
// txt  = Text für Serial/Eingabe (UTF-8, Prosigns als <XX>)
// disp = Zeichen fürs Display im CP437-Zeichensatz des GFX-Standardfonts (0 = txt verwenden)
struct UiOp { uint8_t type; char c; int16_t k; uint16_t color; char sym[16]; };
struct MorseCode { const char *txt; const char *code; uint8_t disp; };

// Entprellter Taster (hier oben definiert, damit die von der Arduino-IDE
// automatisch erzeugten Funktionsprototypen den Typ schon kennen)
struct Button {
  uint8_t  pin;
  bool     stable;        // entprellter Zustand (true = gedrückt)
  bool     lastRaw;
  uint32_t lastChange;    // Zeitpunkt letzter Rohwert-Änderung
  uint32_t since;         // Zeitpunkt letzter stabiler Änderung
};

// Reihenfolge = Priorität beim Dekodieren (erster Treffer gewinnt). Mehrdeutige
// Codes: .-.-. -> '+' (<AR>), -...- -> '=' (<BT>), -.--. -> '(' (<KN>),
// .-... -> '&' (<AS>), .--.- -> 'À' (Å nur beim Geben).
const MorseCode MORSE[] = {
  {"A", ".-"},    {"B", "-..."},  {"C", "-.-."},  {"D", "-.."},
  {"E", "."},     {"F", "..-."},  {"G", "--."},   {"H", "...."},
  {"I", ".."},    {"J", ".---"},  {"K", "-.-"},   {"L", ".-.."},
  {"M", "--"},    {"N", "-."},    {"O", "---"},   {"P", ".--."},
  {"Q", "--.-"},  {"R", ".-."},   {"S", "..."},   {"T", "-"},
  {"U", "..-"},   {"V", "...-"},  {"W", ".--"},   {"X", "-..-"},
  {"Y", "-.--"},  {"Z", "--.."},
  {"0", "-----"}, {"1", ".----"}, {"2", "..---"}, {"3", "...--"},
  {"4", "....-"}, {"5", "....."}, {"6", "-...."}, {"7", "--..."},
  {"8", "---.."}, {"9", "----."},
  {"\xC3\x84", ".-.-", 0x8E},    {"\xC3\x96", "---.", 0x99},   {"\xC3\x9C", "..--", 0x9A},  // Ä Ö Ü
  {"CH", "----"},                 {"\xC3\x9F", "...--..", 0xE1},                             // CH ß
  {"\xC3\x89", "..-..", 0x90},   {"\xC3\x80", ".--.-", 0x85},  {"\xC3\x85", ".--.-", 0x8F}, // É À Å
  {"\xC3\x91", "--.--", 0xA5},                                                              // Ñ
  {".", ".-.-.-"}, {",", "--..--"}, {"?", "..--.."}, {"'", ".----."},
  {"!", "-.-.--"}, {"/", "-..-."},  {"(", "-.--."},  {")", "-.--.-"},
  {"&", ".-..."},  {":", "---..."}, {";", "-.-.-."}, {"=", "-...-"},
  {"+", ".-.-."},  {"-", "-....-"}, {"_", "..--.-"}, {"\"", ".-..-."},
  {"$", "...-..-"},{"@", ".--.-."},
  {"<SK>", "...-.-"}, {"<SOS>", "...---..."}, {"<HH>", "........"},   // Irrung: nur anzeigen
  {"<AR>", ".-.-."},  {"<BT>", "-...-"}, {"<KN>", "-.--."}, {"<AS>", ".-..."},  // nur Geben
};
const int MORSE_COUNT = sizeof(MORSE) / sizeof(MORSE[0]);
bool txCombineCh = false;            // /ch on: "CH" beim Geben als ---- statt C H

// Morsefolge (".-") -> Tabellenindex, -1 wenn unbekannt
int decodeMorse(const char *code) {
  for (int i = 0; i < MORSE_COUNT; i++)
    if (strcmp(MORSE[i].code, code) == 0) return i;
  return -1;
}

// Text (UTF-8) -> Tabellenindex
int findSym(const char *t) {
  for (int i = 0; i < MORSE_COUNT; i++)
    if (strcasecmp(MORSE[i].txt, t) == 0) return i;
  return -1;
}

// Nächstes Symbol aus Eingabetext lesen (UTF-8, Prosigns <XX>), i wird weitergesetzt.
// Rückgabe: Tabellenindex, -2 = Leerzeichen, -1 = unbekannt (überspringen), -3 = Ende
int nextSymbol(const String &t, unsigned &i) {
  if (i >= t.length()) return -3;
  uint8_t c = (uint8_t)t[i];
  if (c == ' ' || c == '\t') { i++; return -2; }
  if (c == '<') {
    int e = t.indexOf('>', i);
    if (e > (int)i && e - (int)i <= 6) {
      String tok = t.substring(i, e + 1); tok.toUpperCase();
      int k = findSym(tok.c_str());
      if (k >= 0) { i = e + 1; return k; }
    }
    i++; return -1;
  }
  if (c == 0xC3 && i + 1 < t.length()) {              // Ä Ö Ü ß É À Å Ñ (groß/klein)
    uint8_t n = (uint8_t)t[i + 1]; i += 2;
    if (n >= 0xA0) n -= 0x20;                         // klein -> groß (ß 0x9F bleibt)
    char u[3] = {(char)0xC3, (char)n, 0};
    return findSym(u);
  }
  if (c >= 0x80) {                                    // sonstiges UTF-8: ganze Sequenz überspringen
    i++; while (i < t.length() && ((uint8_t)t[i] & 0xC0) == 0x80) i++;
    return -1;
  }
  if (txCombineCh && (c == 'C' || c == 'c') && i + 1 < t.length() && toupper((uint8_t)t[i + 1]) == 'H') {
    i += 2; return findSym("CH");
  }
  char u[2] = {(char)toupper(c), 0}; i++;
  return findSym(u);
}

// Display-Byte (CP437) -> UTF-8-Text (für Echo des Textpuffers)
const char *dispToTxt(uint8_t b) {
  for (int i = 0; i < MORSE_COUNT; i++) if (MORSE[i].disp == b) return MORSE[i].txt;
  return nullptr;
}

// ==========================================================================
//  Ton / LED
// ==========================================================================
int toneHz = 700;          // Tonhöhe, per /ton änderbar

// --------------------------------------------------------------------------
//  Mithören / Mithörton über DAC1 (GPIO 25). Timer-ISR mit 16 kHz schreibt den
//  DAC (nur Ganzzahl-Rechnung in der ISR):
//   /mon tone : Sinus auf Decoder-Frequenz, getastet vom Decoder (nach Squelch)
//   /mon filt : Bandpass (~150 Hz) um die Decoder-Frequenz, Squelch-Gate
//   /mon off  : still
//  Morsetaste/Geber: immer Mithörton (Sidetone) mit /ton-Frequenz.
//  Flanken ~3 ms (Hüllkurve), Lautstärke /monvol 0..100.
// --------------------------------------------------------------------------
#define MON_FS        16000
#define MON_BUF       1024
enum { MON_OFF = 0, MON_TONE = 1, MON_FILT = 2 };
volatile uint8_t  monMode = MON_FILT;
volatile int      monVol  = 60;
int      monBw   = 300;                          // Mithör-Bandpass-Breite Hz (/monbw)                 // 0..100
volatile bool     sideOn  = false;              // Taste/Geber
volatile bool     monGate = false;              // Decoder-Ton an (nach Squelch)
volatile uint32_t monPhInc = 0, sidePhInc = 0;  // Phasenschritt (32 bit)
volatile int8_t   monBuf[MON_BUF];
volatile uint16_t monWr = 0, monRd = 0;
int16_t  sinTab[256];
gptimer_handle_t monTimer = nullptr;
volatile uint32_t monPh = 0; volatile int32_t monEnv = 0;   // Hüllkurve 0..4096
// Biquad-Bandpass (Audio-Task)
float bqB0 = 0, bqB2 = 0, bqA1 = 0, bqA2 = 0, bqZ1 = 0, bqZ2 = 0, bqHz = 0, monGain = 0;

static bool IRAM_ATTR monIsr(gptimer_handle_t, const gptimer_alarm_event_data_t *, void *) {
  int32_t v = 0;
  bool side = sideOn;
  bool want = side || (monMode == MON_TONE && monGate);
  const int32_t step = 4096 / 48;                // ~3 ms Flanke bei 16 kHz
  if (want) { monEnv += step; if (monEnv > 4096) monEnv = 4096; } else { monEnv -= step; if (monEnv < 0) monEnv = 0; }
  if (monEnv > 0) {
    monPh += side ? sidePhInc : monPhInc;
    v = (sinTab[monPh >> 24] * monEnv) >> 12;    // +-127*...
    v = (v * monVol) / 100;
  } else monPh = 0;
  if (!side && monMode == MON_FILT) {
    uint16_t r = monRd, w = monWr;
    uint16_t fill = (w - r) & (MON_BUF - 1);
    if (fill > 0) { v += monBuf[r]; r = (r + 1) & (MON_BUF - 1); if (fill > 600) r = (r + 1) & (MON_BUF - 1); monRd = r; }
  }
  v += 128; if (v < 0) v = 0; if (v > 255) v = 255;
  dac_ll_update_output_value(DAC_CHAN_0, (uint8_t)v);
  return false;
}

void monSetFreqs(float decF) {
  monPhInc  = (uint32_t)(decF   * 4294967296.0 / MON_FS);
  sidePhInc = (uint32_t)(toneHz * 4294967296.0 / MON_FS);
}

void monBegin() {
  for (int i = 0; i < 256; i++) sinTab[i] = (int16_t)(120.0f * sinf(2.0f * PI * i / 256));
  dac_oneshot_handle_t h; dac_oneshot_config_t c = { .chan_id = DAC_CHAN_0 };
  if (dac_oneshot_new_channel(&c, &h) != ESP_OK) { Serial.println("DAC-Init fehlgeschlagen"); return; }
  dac_oneshot_output_voltage(h, 128);
  gptimer_config_t tc = {}; tc.clk_src = GPTIMER_CLK_SRC_DEFAULT; tc.direction = GPTIMER_COUNT_UP; tc.resolution_hz = 1000000;
  if (gptimer_new_timer(&tc, &monTimer) != ESP_OK) return;
  gptimer_event_callbacks_t cb = { .on_alarm = monIsr };
  gptimer_register_event_callbacks(monTimer, &cb, nullptr);
  gptimer_alarm_config_t ac = {}; ac.alarm_count = 1000000 / MON_FS; ac.reload_count = 0; ac.flags.auto_reload_on_alarm = true;
  gptimer_set_alarm_action(monTimer, &ac);
  gptimer_enable(monTimer); gptimer_start(monTimer);
}

void signalOn()  { sideOn = true; }
void signalOff() { sideOn = false; }

void ledSet(bool on) {
#if PIN_LED >= 0
  digitalWrite(PIN_LED, on ? HIGH : LOW);
#endif
}

// ==========================================================================
//  Display-Layout
// ==========================================================================
//  +--------------------------------------+
//  | MORSE ESP32        RX 15 WPM TX 18   |  Kopfzeile
//  | ·  −  ·                              |  aktuelle Punkte/Striche
//  |--------------------------------------|
//  | dekodierter Text mit Zeilenumbruch   |  Textbereich
//  | ...                                  |
//  +--------------------------------------+
int16_t W, H;              // Displaygröße nach Rotation
int16_t charW, charH;      // Zeichengröße in Pixeln
int16_t headH, symY, symH, textY, chrY = 0, chrH = 0;   // v42: Zeichenzeile
int cols, rows;            // Zeichen pro Zeile / Zeilen im Textbereich
int16_t textBot;           // unterer Rand des Textbereichs
bool    barOn = true;      // Bargraph (Abstimmanzeige) an/aus, Befehl /bar on|off
#define LVL_AREA_H  0  // v32: waagerechter Pegelbalken entfernt (war 13)    // Pegelbalken (10 px + Abstand) oben im Bargraph-Bereich
#define BAR_TXT_H   12   // Peak-Textzeile ueber den Balken
#define BAR_AREA_H  (LVL_AREA_H + BAR_TXT_H + 2 + BAR_PLOT_H + 14) // v42: aus Balkenhoehe berechnet (50 -> 78 wie bis v41) // Höhe Bargraph-Bereich unten (Pegel + Text + Balken + Skala)
int16_t barY0;             // Oberkante Bargraph-Bereich

const int MAX_TEXT = 400;  // Obergrenze Textpuffer
char     textBuf[MAX_TEXT];
uint16_t textCol[MAX_TEXT];
int      textLen = 0;

float unitMs = 1200.0f / START_WPM;   // Decoder: geschätzte Punktlänge in ms
int   txWpm  = 18;                    // Geber-Tempo

// --------------------------------------------------------------------------
//  Display-Aufträge: Audio+Decoder laufen als Task auf Core 0, das Display
//  (SPI, langsam) nur in loop() auf Core 1. Aufrufe vom Audio-Core landen in
//  einer Warteschlange und werden in loop() abgearbeitet.
// --------------------------------------------------------------------------
#define AUDIO_CORE 1            // v45: Audio auf Core 1 (Core 0 = WLAN/Netz), Prioritaet ueber loop()
TaskHandle_t audioTaskH = nullptr;
QueueHandle_t uiQ = nullptr;
int mCur = -1;                 // Mehrkanal: aktuell dekodierter Kanal (-1 = Einkanal)
extern bool multiOn; extern int mFreq[];
void mAppendNow(int i, char c); void mHeadNow(int i); void mLayout(); void mClear();
SemaphoreHandle_t audioMtx = nullptr;
void dictReplaceNow(const char *w, int rawLen); void drawHeaderNow(); void drawSymbolsNow(const char *sym); void appendCharNow(char c, uint16_t color); void showSymNow(int k, uint16_t color);
static bool uiDefer() { return uiQ && audioTaskH && xTaskGetCurrentTaskHandle() == audioTaskH; }   // v45: nach Task statt Core
static void uiPush(UiOp &o) { xQueueSend(uiQ, &o, 0); }   // voll -> verwerfen statt blockieren
void drawHeader() { if (mCur >= 0) { UiOp o = {0}; o.type = 5; o.k = mCur; uiPush(o); return; } if (uiDefer()) { UiOp o = {0}; o.type = 0; uiPush(o); } else drawHeaderNow(); }
void drawSymbols(const char *sym) { if (mCur >= 0) return; if (uiDefer()) { UiOp o = {0}; o.type = 1; strncpy(o.sym, sym, 9); uiPush(o); } else drawSymbolsNow(sym); }
void appendChar(char c, uint16_t color) { if (mCur >= 0) { UiOp o = {0}; o.type = 4; o.c = c; o.k = mCur; uiPush(o); return; } if (uiDefer()) { UiOp o = {0}; o.type = 2; o.c = c; o.color = color; uiPush(o); } else appendCharNow(c, color); }
void showSym(int k, uint16_t color) { if (mCur >= 0) { if (MORSE[k].disp) appendChar((char)MORSE[k].disp, color); else for (const char *p = MORSE[k].txt; *p; p++) appendChar(*p, color); return; } if (uiDefer()) { UiOp o = {0}; o.type = 3; o.k = k; o.color = color; uiPush(o); } else showSymNow(k, color); }
// höchstens maxMs Display-Arbeit pro loop()-Durchlauf
void uiDrain(uint32_t maxMs) {
  if (!uiQ) return;
  uint32_t t0 = millis(); UiOp o; bool hdr = false;
  while (millis() - t0 < maxMs && xQueueReceive(uiQ, &o, 0) == pdTRUE) {
    switch (o.type) {
      case 0: hdr = true; break;                       // Kopfzeile nur einmal neu
      case 1: drawSymbolsNow(o.sym); break;
      case 2: appendCharNow(o.c, o.color); break;
      case 3: showSymNow(o.k, o.color); break;
      case 4: mAppendNow(o.k, o.c); break;
      case 5: mHeadNow(o.k); break;
      case 6: dictReplaceNow(o.sym, o.k); break;
    }
  }
  if (hdr) drawHeaderNow();
}

// Letztes Wort (rawLen Zeichen vor dem abschließenden Leerzeichen) ersetzen, Farbe C_DICT
void dictReplaceNow(const char *w, int rawLen) {
  if (multiOn) return;
  int e = textLen; if (e > 0 && textBuf[e - 1] == ' ') e--;
  int s = e - rawLen; if (s < 0) return;
  int nl = strlen(w); if (s + nl + (textLen - e) > MAX_TEXT || s + nl + (textLen - e) > cols * rows) return;
  int tail = textLen - e;
  memmove(textBuf + s + nl, textBuf + e, tail); memmove(textCol + s + nl, textCol + e, tail * sizeof(uint16_t));
  for (int i = 0; i < nl; i++) { textBuf[s + i] = w[i]; textCol[s + i] = C_DICT; }
  textLen = s + nl + tail;
  if (nl == rawLen) for (int i = s; i < textLen; i++) drawTextChar(i); else redrawText();
}
void dictReplace(const char *w, int rawLen) {
  if (uiDefer()) { UiOp o = {0}; o.type = 6; strncpy(o.sym, w, 15); o.k = rawLen; uiPush(o); } else dictReplaceNow(w, rawLen);
}
// v25: Pegelwarnung im Kopfzeilenbereich (0 = keine, 1 = zu niedrig, 2 = zu hoch)
int8_t lvlWarn = 0; bool lvlWarnOn = true;
void drawLvlWarnNow() {
  bool hi = lvlWarn == 2;
  const char *t = hi ? "PEGEL ZU HOCH" : "PEGEL ZU NIEDRIG";
  uint16_t bg = hi ? RB(0xF800) : RB(0xFD20);  // rot / orange (v32: R/B-Tausch des Displays ausgeglichen)
  tft.fillRect(0, 0, W, headH, bg);
  int sz = ((int)strlen(t) * 18 <= W - 4 && headH >= 24) ? 3 : 2;
  tft.setTextSize(sz);
  tft.setTextColor(hi ? 0xFFFF : 0x0000, bg);
  tft.setCursor((W - (int)strlen(t) * 6 * sz) / 2, (headH - 8 * sz) / 2 + (sz == 3 ? 1 : 0));
  tft.print(t);
  tft.setTextSize(TXT_SIZE);
}
extern uint8_t handMode; extern int8_t handIs; bool isHand();
extern uint32_t wifiIpShowUntil;
void drawHeaderNow() {
  if (lvlWarn && !multiOn) { drawLvlWarnNow(); return; }
  tft.fillRect(0, 0, W, headH, C_HEAD);
  tft.setTextSize(1);
  tft.setTextColor(C_TEXT, C_HEAD);
  tft.setCursor(MARGIN_X, MARGIN_Y + (headH - MARGIN_Y - 8) / 2);
  if (wifiIpShowUntil) { tft.print("IP "); tft.print(WiFi.localIP().toString()); }   // v45: IP kurz nach dem Verbinden
  else tft.print("MORSE ESP32");
  char buf[32];
  if (multiOn) snprintf(buf, sizeof(buf), "MULTI 4 K");
  else {
    char hk = (handMode == 2) ? 'P' : (isHand() ? 'H' : (handMode == 0 && handIs == 0 ? 'A' : 'P'));
    snprintf(buf, sizeof(buf), "RX%2d %c TX%2d", (int)(1200.0f / unitMs + 0.5f), hk, txWpm);
  }
  tft.setCursor(W - (int)strlen(buf) * 6 - MARGIN_X, MARGIN_Y + (headH - MARGIN_Y - 8) / 2);
  tft.print(buf);
}

// Aktuelle Punkte/Striche grafisch zeichnen (dot = Kreis, dash = Balken)
// v27 Laufband: Elemente laufen rechts herein und schieben nach links (nur UI-Core)
bool tickerOn = true;
GFXcanvas16 *tkCv = nullptr;
struct TkItem { int32_t x; uint8_t t; };      // t: 0 = Punkt, 1 = Strich
TkItem tkIt[96]; int tkN = 0;
int32_t tkTail = 0, tkOff = 0, tkDrawnOff = -1; char tkPrev[16] = ""; uint32_t tkLast = 0; bool tkDirty = true;
int tkU() { return (TXT_SIZE >= 2) ? 4 : 2; }
void tkAdd(uint8_t t) {
  int u = tkU(); if (tkN >= 96) { memmove(tkIt, tkIt + 1, sizeof(TkItem) * 95); tkN--; }
  tkIt[tkN].x = tkTail; tkIt[tkN].t = t; tkN++;
  tkTail += (t ? 6 * u : 2 * u) + 2 * u; tkDirty = true;
}
void tkGap(int units) {                       // Lücke (Zeichen-/Wortpause) zusätzlich zur Elementpause
  int u = tkU(); if (tkN == 0) return;
  int32_t want = tkIt[tkN - 1].x + (tkIt[tkN - 1].t ? 6 * u : 2 * u) + 2 * u + units * u;
  if (tkTail < want) tkTail = want;
}
void tickerFeed(const char *sym) {
  int n = strlen(sym), p = strlen(tkPrev);
  if (n == 0) { if (p) tkGap(5); }            // Zeichen fertig -> Zeichenlücke
  else if (n > p) for (int i = p; i < n; i++) tkAdd(sym[i] == '-');
  else if (n < p) for (int i = 0; i < n; i++) tkAdd(sym[i] == '-');   // neuer Zeichenbeginn
  strncpy(tkPrev, sym, 15); tkPrev[15] = 0;
}
// v30 Streifen: oben Pegelkurve, unten Ton-Elemente, gemeinsame Zeitachse (ST_MS ms pro Pixelspalte)
// v43: Ring in feinem Raster ST_BASE ms; Anzeige-Maßstab stMs (ms/Pixel) variabel (/stripauto)
#define ST_MS   12        // fester Maßstab bei /stripauto off
#define ST_BASE 4         // Ring-Raster in ms
#define ST_N  2400        // 9,6 s > (W + Vorlauf) * STRIP_MS_MAX
struct StCol { float pk; uint16_t c; uint8_t f; };   // f: 1 = Detektor an, 2 = Ton erkannt (audioOn); c = untere 16 Bit der Spalte
volatile StCol stRing[ST_N];
volatile float stThr = 0, stSq = 0;
int stMs = ST_MS; bool stripAuto = true; float stMsF = ST_MS;
uint32_t stLastC = 0;
// vom Audio-Task je Block aufgerufen (billig); füllt auch die übersprungenen Feinspalten
void stFeed(float mag, bool det, bool on, float thr, float sq, bool wb) {
  uint32_t c = millis() / ST_BASE; uint8_t fl = (det ? 1 : 0) | (on ? 2 : 0) | (wb ? 4 : 0);
  uint32_t a = (c > stLastC && c - stLastC <= 8) ? stLastC + 1 : c;
  for (uint32_t k = a; k <= c; k++) {
    volatile StCol &e = stRing[k % ST_N];
    if (e.c != (uint16_t)k) { e.c = (uint16_t)k; e.pk = mag; e.f = 0; } else if (mag > e.pk) e.pk = mag;
    e.f |= fl;
  }
  stLastC = c; stThr = thr; stSq = sq;
}
// Maßstab sanft an die erkannte Punktlänge anpassen (UI-Core, alle 250 ms)
extern float unitMs;
void stripScale(uint32_t now) {
  static uint32_t tl = 0; static float lastU = 0; static int stab = 0;
  if (!stripAuto) { if (stMs != ST_MS) { stMs = ST_MS; stMsF = ST_MS; tkDirty = true; } return; }
  if (now - tl < 250) return; tl = now;
  float u = unitMs;
  if (lastU > 0 && fabsf(u - lastU) < 0.1f * lastU) { if (stab < 100) stab++; } else stab = 0;
  lastU = u;
  if (stab < 8) return;                                   // erst nach ~2 s stabiler Schätzung
  float tgt = constrain(u / (float)STRIP_DOT_PX, (float)STRIP_MS_MIN, (float)STRIP_MS_MAX);
  stMsF += 0.15f * (tgt - stMsF);                         // geglättet
  int n = stMs;
  if (stMsF > stMs + 0.6f) n = stMs + 1; else if (stMsF < stMs - 0.6f) n = stMs - 1;   // max. 1 Stufe je 250 ms
  n = constrain(n, STRIP_MS_MIN, STRIP_MS_MAX);
  if (n != stMs) { stMs = n; tkDirty = true; }
}
uint32_t stDrawnCol = 0; uint8_t stF[256]; int16_t stH[256]; extern float thrMs;
int16_t stLvlH() { return symH - 22; }
// v32: Skala 14..60 dB (14 dB = 5 Counts liegt knapp über dem Goertzel-Grundrauschen des ADC ohne Signal, ~4 Counts)
#define ST_DB_MIN 14.0f
int16_t stDb(float m, int16_t h) { if (m <= 1) return 0; float v = (20.0f * log10f(m) - ST_DB_MIN) / (60.0f - ST_DB_MIN); return (int16_t)constrain(v * h, 0.0f, (float)h); }
// v42 Zeichenzeile: erkannte Zeichen laufen mittig unter ihren Elementen mit
// (Audio-Core schreibt, UI-Core liest; gleiche Zeitachse wie der Streifen)
#define LAB_N 48
struct LabItem { uint32_t t0, t1; char s[7]; };
volatile LabItem labR[LAB_N]; volatile uint32_t labHead = 0;
GFXcanvas16 *chCv = nullptr;
// v45 Web-Ereignisse (Audio-Task schreibt, loop() liest und verschickt)
struct WebEv { uint8_t type; uint32_t a, b; char s[7]; };   // 1 = Taste an, 2 = Taste aus, 3 = Zeichen
#define WEV_N 128
volatile WebEv wevR[WEV_N]; volatile uint32_t wevHead = 0;
void webEv(uint8_t ty, uint32_t a, uint32_t b, const char *s) {
  uint32_t h = wevHead; volatile WebEv &e = wevR[h % WEV_N];
  e.type = ty; e.a = a; e.b = b; int i = 0; if (s) for (; i < 6 && s[i]; i++) e.s[i] = s[i]; e.s[i] = 0;
  wevHead = h + 1;
}
void labPush(uint32_t t0, uint32_t t1, const char *s) {
  if (!s || !s[0] || s[0] == ' ') return;
  uint32_t h = labHead; volatile LabItem &e = labR[h % LAB_N];
  e.t0 = t0; e.t1 = t1 > t0 ? t1 : t0; int i = 0; for (; i < 6 && s[i]; i++) e.s[i] = s[i]; e.s[i] = 0;
  labHead = h + 1; tkDirty = true;
  webEv(3, e.t0, e.t1, s);
}
void chrRowDraw(uint32_t cur) {
  if (!chCv || chrH <= 0) return;
  chCv->fillScreen(C_BG);
  chCv->setTextSize(CHR_TXT_SIZE); chCv->setTextWrap(false);
  int cw = 6 * CHR_TXT_SIZE, chh = 8 * CHR_TXT_SIZE, y = max(0, (chrH - chh) / 2);
  uint32_t h = labHead; int32_t lastL = 1 << 20;   // v44: linker Rand des zuletzt (rechts) gezeichneten Zeichens
  for (uint32_t n = 0; n < LAB_N && n < h; n++) {
    const volatile LabItem &e = labR[(h - 1 - n) % LAB_N];
    char s[7]; int i = 0; for (; i < 6 && e.s[i]; i++) s[i] = e.s[i]; s[i] = 0;
    int32_t c0 = e.t0 / (uint32_t)stMs, c1 = e.t1 / (uint32_t)stMs;
    int32_t xm = (W - 1) - ((int32_t)cur - (c0 + c1) / 2);
    int tw = i * cw, x = xm - tw / 2;
    if (x + tw > lastL) x = lastL - tw;                // v44: nie ueberlappen (sonst verstuemmelte Glyphen)
    if (x + tw < 0) break;                              // aelter -> alle weiter links
    if (x >= W) continue;
    chCv->setTextColor(C_TEXT); chCv->setCursor(x, y); chCv->print(s);
    lastL = x;
  }
  tft.drawRGBBitmap(0, chrY, chCv->getBuffer(), W, chrH);
}
void tickerDraw(uint32_t now) {
  if (!tickerOn || multiOn || !tkCv) return;
  if (now - tkLast < 30) return; tkLast = now;
  stripScale(now);
  const uint32_t sm = (uint32_t)stMs;
  uint32_t cur = now / sm - 1;                         // letzte vollständige Spalte
  if (cur == stDrawnCol && !tkDirty) return;
  stDrawnCol = cur; tkDirty = false;
  int16_t lh = stLvlH(), ey = lh + 4, eh = 14;
  tkCv->fillScreen(C_BG);
  for (int db = 20; db < 60; db += 20) tkCv->drawFastHLine(0, lh - stDb(powf(10, db / 20.0f), lh), W, C_GRID);
  tkCv->drawFastHLine(0, lh + 1, W, C_GRID);
  int run = 0; uint8_t pf = 0;
  float thrPx = thrMs / sm;
  for (int x = W - 1, n = 0; x >= -40; x--, n++) {    // von rechts (neu) nach links (alt)
    uint32_t c = cur - n;                              // Pixelspalte -> Feinspalten [b0, b1)
    uint32_t b0 = c * sm / ST_BASE, b1 = (c + 1) * sm / ST_BASE; if (b1 <= b0) b1 = b0 + 1;
    bool ok = false; uint8_t f = 0; float pk = 0;
    for (uint32_t b = b0; b < b1; b++) { const volatile StCol &e = stRing[b % ST_N];
      if (e.c == (uint16_t)b) { ok = true; f |= e.f; if (e.pk > pk) pk = e.pk; } }
    if (x >= 0) { stF[x] = f; stH[x] = ok ? stDb(pk, lh) : 0; }
    pf = f;
    // Elemente: Lauf von Ton-Spalten sammeln, beim Ende (links) zeichnen
    if (f & 2) run++;
    else if (run) {
      int x0 = x + 1; bool open = (x0 + run >= W);    // rechts noch laufend
      uint16_t col = open && x0 + run > W - 1 ? C_TEXT : (run > thrPx ? RB(0x07FF) : C_SYM);
      tkCv->fillRect(x0, ey, run, eh, col); run = 0;
    }
  }
  if (run) tkCv->fillRect(-40, ey, run, eh, run > thrPx ? RB(0x07FF) : C_SYM);   // ragt links heraus (wird abgeschnitten)
  // Pegelkurve: hellgrau; farbig nur, wo roher Detektor und Element nach Abschluss sicher abweichen
  //  ORANGE  = Detektor-Lauf ohne jedes Element (verworfen, z.B. zu kurz)
  //  MAGENTA = Detektor-Lücke innerhalb eines Elements (überbrückt), beidseitig Detektor an
  static uint16_t cl[256];
  for (int x = 0; x < W; x++) cl[x] = C_INFO;
  for (int a = 0; a < W;) {
    bool d = stF[a] & 1; int b = a; while (b < W && ((stF[b] & 1) != 0) == d) b++;
    bool closed = a > 0 && b < W;                       // Lauf beidseitig abgeschlossen
    if (closed && d) { bool el = false; for (int i = a; i <= min(b + 2, W - 1); i++) if (stF[i] & 2) el = true;
      if (!el) for (int i = a; i < b; i++) cl[i] = RB(0xFD20); }                     // orange
    if (closed && !d) { bool br = true; for (int i = a; i < b; i++) if (!(stF[i] & 2)) br = false;
      if (br && (stF[b] & 2)) for (int i = a; i < b; i++) cl[i] = RB(0xF81F); }      // magenta
    a = b;
  }
  for (int x = 0; x < W; x++) if (stF[x] & 4) cl[x] = RB(0xFD20);   // v33: Breitband unterdrückt/gehalten -> orange
  for (int x = 0; x < W; x++) if (stH[x]) tkCv->drawFastVLine(x, lh - stH[x], stH[x], cl[x]);
  float t = stThr, q = stSq;
  if (q > 0) tkCv->drawFastHLine(0, lh - stDb(q, lh), W, RB(0xF800));   // Rauschsperre rot
  if (t > 0) tkCv->drawFastHLine(0, lh - stDb(t, lh), W, RB(0xFFE0));   // Schwelle gelb
  tft.drawRGBBitmap(0, symY, tkCv->getBuffer(), W, symH);
  chrRowDraw(cur);
}
void drawSymbolsNow(const char *sym) {
  if (multiOn) return;
  if (tickerOn && tkCv) { tickerFeed(sym); return; }
  tft.fillRect(0, symY, W, symH, C_BG);
  int u  = (TXT_SIZE >= 2) ? 4 : 2;      // Grundgröße
  int x  = MARGIN_X;
  int cy = symY + symH / 2;
  for (const char *p = sym; *p && x < W - 8 * u; p++) {
    if (*p == '.') { tft.fillCircle(x + u, cy, u, C_SYM); x += 2 * u + 3 * u; }
    else           { tft.fillRect(x, cy - u, 6 * u, 2 * u, C_SYM); x += 6 * u + 3 * u; }
  }
}

// Zeichen an Position i im Textbereich zeichnen
void drawTextChar(int i) {
  int line = i / cols, col = i % cols;
  tft.setTextSize(TXT_SIZE);
  tft.setTextColor(textCol[i], C_BG);
  tft.setCursor(MARGIN_X + col * charW, textY + 2 + line * charH);
  tft.print(textBuf[i]);
}

void redrawText() {
  tft.fillRect(0, textY + 1, W, textBot - textY - 1, C_BG);
  for (int i = 0; i < textLen; i++) drawTextChar(i);
}

void clearText() {
  textLen = 0;
  if (multiOn) { mClear(); Serial.println(F("\n[Text gelöscht]")); return; }
  redrawText();
  Serial.println(F("\n[Text gelöscht]"));
}

// Zeichen an Text anhängen; bei vollem Bildschirm eine Zeile nach oben schieben
void appendCharNow(char c, uint16_t color) {
  if (c == ' ' && tickerOn) tkGap(12);         // Wortlücke im Laufband
  if (multiOn) return;                  // Mehrkanal: eigene Anzeige
  if (c == ' ' && (textLen == 0 || textBuf[textLen - 1] == ' ')) return; // keine Doppel-Leerzeichen
  int capacity = cols * rows;
  if (textLen >= capacity) {
    memmove(textBuf, textBuf + cols, textLen - cols);
    memmove(textCol, textCol + cols, (textLen - cols) * sizeof(uint16_t));
    textLen -= cols;
    textBuf[textLen] = c; textCol[textLen] = color; textLen++;
    redrawText();
  } else {
    textBuf[textLen] = c; textCol[textLen] = color; textLen++;
    drawTextChar(textLen - 1);
  }
}

// Tabellen-Symbol anzeigen: CP437-Zeichen bzw. Text (Prosigns, CH)
void showSymNow(int k, uint16_t color) {
  if (MORSE[k].disp) appendChar((char)MORSE[k].disp, color);
  else for (const char *p = MORSE[k].txt; *p; p++) appendChar(*p, color);
}

// --------------------------------------------------------------------------
//  Ausrichtung (MADCTL) passend zu TFT_eSPI setzen – nur ST7789
//  Adafruit_ST7789::setRotation(0) setzt MADCTL = MX|MY (0xC0), TFT_eSPI
//  (ST7789_2) dagegen 0x00. Darum wird MADCTL nach setRotation() hier mit den
//  TFT_eSPI-Werten überschrieben. Die Offsets für 240x280 (je 20 Zeilen oben
//  und unten, 0 Spalten) sind symmetrisch – Spiegeln verschiebt daher nichts,
//  Adafruits setAddrWindow()-Offsets bleiben gültig.
// --------------------------------------------------------------------------
#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_CMD 0x36

void applyOrientation() {
  tft.setRotation(DISPLAY_ROTATION);      // Breite/Höhe + Offsets (Adafruit)
#if defined(DISPLAY_ST7789)
  // MADCTL-Werte wie TFT_eSPI ST7789_2_Rotation.h, Farbreihenfolge RGB (Bit3 = 0)
  static const uint8_t base[4] = {
    0x00,                       // 0: Hochformat
    MADCTL_MX | MADCTL_MV,      // 1: Querformat
    MADCTL_MX | MADCTL_MY,      // 2: Hochformat 180°
    MADCTL_MV | MADCTL_MY       // 3: Querformat 180°
  };
  uint8_t m = base[DISPLAY_ROTATION & 3];
  if (MIRROR_X) m ^= MADCTL_MX;
  if (MIRROR_Y) m ^= MADCTL_MY;
  tft.sendCommand(MADCTL_CMD, &m, 1);
  Serial.printf("Display: ST7789 %dx%d, Rotation %d, MADCTL 0x%02X\n",
                (int)tft.width(), (int)tft.height(), DISPLAY_ROTATION, m);
#endif
}

extern volatile bool tftReq;
void setupDisplay() {
  // v50: harter Reset mit Wartezeit, damit das ST7789 nach Einschalten sicher initialisiert
  pinMode(PIN_TFT_RST, OUTPUT); digitalWrite(PIN_TFT_RST, HIGH); delay(5);
  digitalWrite(PIN_TFT_RST, LOW); delay(20); digitalWrite(PIN_TFT_RST, HIGH); delay(150);
#if PIN_TFT_BL >= 0
  pinMode(PIN_TFT_BL, OUTPUT);
  digitalWrite(PIN_TFT_BL, HIGH);         // Beleuchtung an
#endif
  SPI.begin(PIN_TFT_SCK, -1, PIN_TFT_MOSI, PIN_TFT_CS);
#if defined(DISPLAY_ILI9341)
  tft.begin();
#elif defined(DISPLAY_ST7735)
  tft.initR(ST7735_TAB);
#elif defined(DISPLAY_ST7789)
  tft.init(ST7789_W, ST7789_H, SPI_MODE0);
  tft.invertDisplay(TFT_INVERT);   // Adafruit-Init schaltet INVON ein; Referenzmodul braucht aus
#endif
  tft.setSPISpeed(TFT_SPI_HZ);
  applyOrientation();
  tft.cp437(true);                 // korrekte CP437-Tabelle (Umlaute 0x84, 0x94, ...)
  W = tft.width();
  H = tft.height();
  charW = 6 * TXT_SIZE;
  charH = 8 * TXT_SIZE + 2;
  headH = ((TXT_SIZE >= 2) ? 16 : 12) + MARGIN_Y;
  symH  = tickerOn ? 56 : ((TXT_SIZE >= 2) ? 20 : 12);   // v30: Streifen Pegel+Elemente
  textY = headH + 2;               // Text direkt unter dem Header; Punkt/Strich-Zeile unten (layoutScreen)
  layoutScreen();
}

void barDrawStatic();   // Bargraph-Rahmen (weiter unten definiert)

// Textbereich (und ggf. Bargraph-Bereich) neu einteilen und alles neu zeichnen
void layoutScreen() {
  tkDrawnOff = -1; tkDirty = true;
  barY0   = H - MARGIN_Y - BAR_AREA_H;
  if (multiOn) { mLayout(); return; }
  int16_t bottom = barOn ? barY0 - 2 : H - MARGIN_Y;
  chrH    = (tickerOn && CHR_ROW_H > 0) ? CHR_ROW_H : 0;   // v42: Zeichenzeile nur mit Streifen
  if (chrH && !chCv) chCv = new GFXcanvas16(W, chrH);
  if (!chCv) chrH = 0;
  chrY    = bottom - chrH;
  symY    = chrY - symH;            // Punkt/Strich-Zeile direkt über der Zeichenzeile
  textBot = symY - 2;
  cols  = (W - 2 * MARGIN_X) / charW;
  rows  = (textBot - textY - 2) / charH;
  if (rows < 1) rows = 1;
  if (cols * rows > MAX_TEXT) rows = MAX_TEXT / cols;
  // Text kürzen, falls er nicht mehr passt (neueste Zeilen behalten)
  while (textLen > cols * rows) {
    memmove(textBuf, textBuf + cols, textLen - cols);
    memmove(textCol, textCol + cols, (textLen - cols) * sizeof(uint16_t));
    textLen -= cols;
  }
  tft.fillScreen(C_BG);
  drawHeader();
  drawSymbols("");
  tft.drawFastHLine(0, textY, W, C_INFO);
  tft.drawFastHLine(0, symY - 1, W, C_INFO);
  redrawText();
  if (barOn) barDrawStatic();
}

// ==========================================================================
//  Entprellter Taster
// ==========================================================================

Button keyBtn  = {PIN_KEY,  false, false, 0, 0};   // kombinierter Zustand (Taste ODER Ton)
Button physBtn = {PIN_KEY,  false, false, 0, 0};   // entprellte Morsetaste allein
Button funcBtn = {PIN_FUNC, false, false, 0, 0};

// Liefert true, wenn sich der entprellte Zustand geändert hat.
// raw = true bedeutet "gedrückt" (bzw. Ton erkannt).
bool updateButtonRaw(Button &b, bool raw, uint32_t now) {
  if (raw != b.lastRaw) { b.lastRaw = raw; b.lastChange = now; }
  if (raw != b.stable && (now - b.lastChange) >= DEBOUNCE_MS) {
    b.stable = raw;
    b.since  = now;
    return true;
  }
  return false;
}

bool updateButton(Button &b, uint32_t now) {
  return updateButtonRaw(b, digitalRead(b.pin) == LOW, now);  // gegen GND
}

// ==========================================================================
//  AUDIO-EINGANG: ADC continuous + Goertzel
// ==========================================================================
#include "esp_adc/adc_continuous.h"
#include <math.h>

adc_continuous_handle_t adcHandle = nullptr;
#if AUDIO_SELFTEST
const uint8_t pat_channel = ADC_CHANNEL_4;
#else
const uint8_t pat_channel = ADC_CHANNEL_6;
#endif
bool     audioOk      = false;
int      audioFreq    = AUDIO_FREQ_HZ;
bool     audioDebug   = false;
bool     elemOut      = false;        // v39 /elem: #K1,t / #K0,t,dur je Tastwechsel, #Ct,zeichen (ESP-Zeitstempel ms)
uint32_t charT        = 0;            // v39 Startzeit erstes Element des laufenden Zeichens
bool     magStream    = false;        // /mag: je Goertzel-Block eine Zeile #Mmag,ton%,on (Offline-Analyse)
volatile bool audioOn = false;     // Ton erkannt (nach Schwelle/Hysterese)
// Ton-Flanken mit Zeitstempel aus dem Abtastzähler: Der Decoder bekommt die
// echten Zeitpunkte, auch wenn loop() kurz durch Displayausgaben (Scrollen,
// Bargraph) aufgehalten wurde – die ADC-Daten liegen solange im DMA-Puffer.
struct AudioEdge { uint32_t t; bool on; };
#define AEDGE_N 32
AudioEdge aEdge[AEDGE_N]; volatile int aEdgeHead = 0, aEdgeTail = 0;
uint32_t aProcIdx = 0;               // Abtastwerte im aktuellen audioLoop()-Aufruf
int32_t  aPendIdx[AEDGE_N]; bool aPendOn[AEDGE_N]; int aPendN = 0;
bool audioState = false;             // Ton-Zustand laut Flanken-Warteschlange
#define AUDIO_LAT_MS 50              // Verzögerung ADC-Frame + Goertzel-Block + Mindestdauer: Pausen erst so spät bewerten
// Rauschsperre: Ton nur, wenn Magnitude >= SQ dB über dem mittleren Rauschen (/sq N, 0 = aus);
// ohne AFC-Lock SQ_UNLOCK_DB mehr. Ton zählt erst nach Mindestdauer 0,5 x Punktlänge (20..30 ms).
#define SQ_DB_DEFAULT  4.0f   // v24: 9 -> 4 dB (empfindlicher, Fehl-E per Kontext verworfen)
#define SQ_UNLOCK_DB   3.0f
#define MINON_MIN_MS   20.0f
#define MINON_MAX_MS   30.0f
float    sqDb = SQ_DB_DEFAULT, nfMean = 10.0f;
float    sqBoost = 0;            // adaptive Zusatz-Schwelle (dB) nach erkannten Rausch-Ketten
uint32_t etDropped = 0;         // verworfene E/T-Rauschketten (Zeichen)
int      nfBlocked = 0;              // Zähler Aussperr-Schutz
bool     detOn = false;          // Detektor (vor Mindestdauer-Filter)
int      onCnt = 0;              // Blöcke seit Detektor-Ein (Bestätigung)
uint32_t sqDropped = 0;          // verworfene kurze Töne / isolierte Punkte

// Quelle für den Decoder: Bit0 = Taste, Bit1 = Audio
#define SRC_KEY   1
#define SRC_AUDIO 2
uint8_t  keySource = SRC_KEY | SRC_AUDIO;

// Goertzel-Zustand
float    gCoeff = 0, gS1 = 0, gS2 = 0;
// Schmaler Modus: feste Frequenz (kein AFC-Einrasten), Hann-Fenster 12 ms
// (-3 dB ca. 120 Hz, erste Nullstelle ±167 Hz), Seitenbins ±150 Hz zur
// Unterdrückung von Nachbarstationen. Bargraph läuft weiter als Abstimmhilfe.
#define NARROW_BLOCK_MS 12
#define NARROW_SIDE_HZ  150.0f
// Normalmodus: Nachbar-Bins ±50 Hz zur Detektion (max über 3 Bins), damit ein
// 40..100 Hz versetzter Ton (2. Station im QSO) nicht verloren geht
#define WIDE_SIDE_HZ    50.0f
int      fastAcqN = 0;           // Schnell-Einrasten: Blöcke nach Tonbeginn nach Pause
bool     narrowMode = false;
float    hannW[256]; float hannSum = 1;
float    sCoeffL = 0, sCoeffH = 0, sL1 = 0, sL2 = 0, sH1 = 0, sH2 = 0;
// v33 Breitband-Störunterdrückung (/wbrej): 6 Nachbar-Bins im selben Decoder-Block (±200/300/400 Hz)
float hystF = AUDIO_HYST;   // v35 /hyst: Aus-Schwelle = hystF x Ein-Schwelle (Std. 0.6)
bool ratioOn = false; float ratioHi = 0.7f;   // v34 /ratio (Standard aus: Simulation bringt keinen Gewinn)
float thrMid = 0.5f;     // v33 /thrmid: Lage der Schaltschwelle zwischen Rauschboden (0) und Peak (1)
float thrDecay = 3.0f;   // v33 /thrdecay: Peak-Abfall in Punktlängen (0 = alt, ~4 s)
bool wbOn = true; float wbDb = 6.0f, wbK = 2.0f, wbMed = 0, wbCoefHz = 0;
float wbC[6], wbS1[6], wbS2[6]; bool wbBlk = false; int wbHold = 0; uint32_t wbRej = 0, wbHeld = 0;
const int16_t WB_OFS[6] = {-400, -300, -200, 200, 300, 400};
float    sideRatio = 0, sidePow = 0;          // stärkster Seitenbin / Hauptbin (Diagnose)
Preferences prefs;
int      gN = 160, gCount = 0;
float    dcEst = 2048, blockSum = 0;
float    audioMag = 0, noiseFloor = 5, peakMag = 0, thrOn = AUDIO_MIN_MAG, thrOff = AUDIO_MIN_MAG * AUDIO_HYST;
float    fsMeasured = AUDIO_FS_HZ;
float    hpA = 0.93f, hpY = 0, hpX = 0;   // Hochpass-Zustand
float    blockEnergy = 0, tonality = 0;   // Energie im Block, Tonanteil
uint32_t sampleCount = 0, fsT0 = 0, lastDbg = 0, fsLastChk = 0;
int      rawMin = 4095, rawMax = 0;    // Spitzenwerte für Debug
uint32_t zcCount = 0, zcSamples = 0;   // Nulldurchgänge (Frequenzkontrolle)
bool     zcPos = false;
uint32_t loopMaxUs = 0;               // längste loop()-Laufzeit (ohne serialLoop) seit letzter Statuszeile
uint32_t barMaxUs  = 0;               // längste Bargraph-Zeichenzeit
uint32_t onEdges = 0;                  // Anzahl Ton-Ein-Flanken seit letzter Ausgabe
// /dump: 4096 Rohwerte am Stück aufzeichnen und ausgeben (Signalanalyse)
#define DUMP_LEN 4096
int16_t  dumpBuf[DUMP_LEN];
int      dumpPos = -1;                 // -1 = aus


// ==========================================================================
//  BARGRAPH / ABSTIMMANZEIGE
//  25 Goertzel-Filter 400..1000 Hz (25-Hz-Raster) auf denselben ADC-Daten
//  (nach dem Hochpass), Hann-Fenster über 1024 Abtastwerte (~51 ms bei 20 kHz)
//  -> ca. 20 Spektren/s. Spitzenfrequenz per Parabel-Interpolation zwischen
//  den Bins (Auflösung wenige Hz). Rechenlast ~3 % eines Kerns; gezeichnet
//  wird höchstens 15x/s und nur die geänderten Balkenteile (kein Flimmern,
//  kurze SPI-Zeit -> Decoder-Timing bleibt unberührt, siehe /stat loopMax).
// ==========================================================================
#define BAR_F0      400
#define BAR_STEP    25
#define BAR_N       25            // 400..1000 Hz
#define BAR_WIN     1024          // Fensterlänge in Abtastwerten
#define BAR_DB_MIN  0.0f          // 0 dB  = 1 ADC-Count Amplitude
#define BAR_DB_MAX  60.0f         // 60 dB = 1000 Counts (Vollaussteuerung ~1200)
#define BAR_HOLD_DECAY 0.6f       // Peak-Hold fällt um 0,6 dB pro Spektrum (~12 dB/s)
#define BAR_FPS_MS  66            // max. ~15 Bilder/s

float    barCoef[BAR_N], barS1[BAR_N], barS2[BAR_N];
float    barHann[BAR_WIN];
int      barPos = 0;
float    barDb[BAR_N], barHold[BAR_N];        // geglättet / Peak-Hold (dB)
int16_t  barDrawnH[BAR_N], barDrawnHold[BAR_N];
bool     barNew = false;
float    barPeakHz = 0, barPeakDb = 0, barNoiseDb = 0;
bool     barPeakValid = false;
// AFC: Decoder-Goertzel automatisch auf die Tonfrequenz nachführen (/afc on|off)
#define AFC_MIN_SPEC  3          // Peak muss über >= 3 Spektren (~190 ms) stabil sein
#define AFC_STAB_HZ   12.0f      // ... innerhalb dieser Schwankung
#define AFC_JUMP_HZ   40.0f      // größere Ablage: sofort einrasten
#define AFC_TRACK_HZ  15.0f      // Feinnachführung (Drift) nur bis zu dieser Ablage
uint32_t afcSwMs = 1500;         // /afcsw: Wechsel auf neuen Peak erst nach so langer Stille des alten Signals
#define AFC_HOLD_MS   5000       // ohne Signal länger als das: Frequenz halten ("HOLD")
bool     afcOn = true, afcLocked = false;
float    afcCenter = AUDIO_FREQ_HZ;   // Mitte des AFC-Suchbereichs (= /freq)
#define AFC_RANGE_HZ 100.0f     // AFC rastet nur innerhalb ±100 Hz um /freq ein
uint32_t lastToneMs = 0;
uint32_t fastJumps = 0;           // Zähler Schnell-Einrasten (Diagnose)          // letzter erkannter Ton (Squelch offen)
#define AFC_FREE_MS  30000      // so lange nach dem letzten Ton: Frequenz halten (keine Sprünge)
float    afcHz = AUDIO_FREQ_HZ;  // Sollwert der AFC
float    decHz = AUDIO_FREQ_HZ;  // aktuell im Decoder-Goertzel verwendete Frequenz
float    afcLastM = 0; uint8_t afcCnt = 0; uint32_t afcLastMs = 0;
extern int durLen;
bool afcHold() { return afcOn && afcLocked && millis() - afcLastMs > AFC_HOLD_MS; }
uint32_t barLastDraw = 0;
int      barDrawnTarget = -1;
char     barDrawnTxt[40] = "";
uint32_t barLastTxt = 0;
GFXcanvas16 barCanvas(220, 8);       // Puffer für die Peak-Textzeile


// ==========================================================================
//  PEGELBALKEN (Aussteuerung ADC-Eingang), waagerecht, ohne Schrift
//  Messgröße: max |Rohwert - dc| je Fenster (1024 Werte ~ 63 ms), Skala
//  abschnittsweise (siehe lvlAmpToPx). Zonen: gelb = zu leise (< 100 Counts),
//  grün = gut (Spitzen innerhalb 300..3800), rot = darüber / Clip.
//  Clip (Rohwert <= 8 oder >= 4087): Rahmen 1 s rot. Peak-Hold-Strich weiß,
//  hält 0,5 s und fällt dann mit ~60 px/s.
// ==========================================================================
#define LVL_WIN       1024
#define LVL_QUIET     100.0f       // darunter "zu leise"
#define LVL_GOOD_LO   300          // Rohwert-Grenzen "gut"
#define LVL_GOOD_HI   3800
#define LVL_CLIP_LO   8
#define LVL_CLIP_HI   4087
#define LVL_CLIP_MS   1000
#define C_LVL_Y       0xFFE0       // gelb
#define C_LVL_G       0x07E0       // grün
#define C_LVL_R       0xF800       // rot
#define C_LVL_YD      0x3180       // dunkelgelb (Hintergrund-Zone)
#define C_LVL_GD      0x0180       // dunkelgrün
#define C_LVL_RD      0x3000       // dunkelrot
#define C_LVL_FRAME   C_GRID

int      lvlMin = 4095, lvlMax = 0, lvlCnt = 0;
volatile float lvlAmp = 0;          // letzte Fenster-Amplitude (Counts)
volatile bool  lvlNew = false;
uint32_t lvlClipUntil = 0;          // rote Markierung bis
bool     lvlClipAny = false;        // Clip seit letzter Statuszeile
float    lvlHoldPx = 0; uint32_t lvlHoldT = 0;
int16_t  lvlDrawnFill = 0, lvlDrawnHold = -1; int8_t lvlDrawnClip = -1;
int16_t  lvlXq = 0, lvlXg = 0;      // Zonengrenzen in Pixeln (relativ zum Innenbereich)

void lvlWindowDone() {
  float dc = dcEst;
  float a = max(dc - lvlMin, lvlMax - dc);
  lvlAmp = a < 0 ? 0 : a;
  if (lvlMin <= LVL_CLIP_LO || lvlMax >= LVL_CLIP_HI) { lvlClipUntil = millis() + LVL_CLIP_MS; lvlClipAny = true; }
  lvlMin = 4095; lvlMax = 0; lvlCnt = 0;
  lvlNew = true;
}

// Geometrie: Rahmen über volle Breite (mit Rand), Innenbereich 8 px hoch
int16_t lvlX()  { return MARGIN_X; }
int16_t lvlY()  { return barY0 + 3; }
int16_t lvlW()  { return W - 2 * MARGIN_X; }        // inkl. Rahmen
int16_t lvlIW() { return lvlW() - 2; }              // Innenbreite
float    lvlGoodAmp = 1650;             // Amplitude, ab der die Spitzen 300/3800 überschreiten
// Abschnittsweise Skala, damit jede Zone gut sichtbar ist:
//  10..100 Counts (log)  -> 0..35 %  gelb
//  100..gut (log)        -> 35..85 % grün
//  gut..2048 (linear)    -> 85..100 % rot
int16_t lvlAmpToPx(float a) {
  float v;
  float q = LVL_QUIET, g = lvlGoodAmp;
  if (a <= q) v = a <= 10 ? 0 : 0.35f * log10f(a / 10.0f) / log10f(q / 10.0f);
  else if (a <= g) v = 0.35f + 0.50f * log10f(a / q) / log10f(g / q);
  else v = 0.85f + 0.15f * (a - g) / (2048.0f - g);
  return (int16_t)constrain(v * lvlIW(), 0.0f, (float)lvlIW());
}
// Farbe eines Pixels im Innenbereich je Zone (hell = Balken, dunkel = Hintergrund)
void lvlFill(int16_t a, int16_t b, bool bright) {   // Pixel [a, b) zeichnen
  int16_t x0 = lvlX() + 1, y = lvlY() + 1;
  const int16_t lim[3] = {lvlXq, lvlXg, lvlIW()};
  const uint16_t cb[3] = {C_LVL_Y, C_LVL_G, C_LVL_R}, cd[3] = {C_LVL_YD, C_LVL_GD, C_LVL_RD};
  int16_t start = 0;
  for (int z = 0; z < 3; z++) {
    int16_t lo = max(a, start), hi = min(b, lim[z]);
    if (hi > lo) tft.fillRect(x0 + lo, y, hi - lo, 8, bright ? cb[z] : cd[z]);
    start = lim[z];
  }
}
void lvlDrawStatic() {
  // grün ab "zu leise", rot ab der Grenze, an der die Spitzen 300/3800 überschreiten
  float dc = dcEst > 500 && dcEst < 3600 ? dcEst : 2048;
  lvlGoodAmp = min(dc - LVL_GOOD_LO, LVL_GOOD_HI - dc);
  lvlXq = lvlAmpToPx(LVL_QUIET);
  lvlXg = lvlAmpToPx(lvlGoodAmp);
  return;                         // v32: Balken nicht mehr zeichnen (Grenzwerte bleiben berechnet)
  tft.drawRect(lvlX(), lvlY(), lvlW(), 10, C_LVL_FRAME);
  lvlFill(0, lvlIW(), false);
  lvlDrawnFill = 0; lvlDrawnHold = -1; lvlDrawnClip = -1;
}

void lvlDraw(uint32_t now) {
  if (!lvlNew) return;
  lvlNew = false;
  int16_t px = lvlAmpToPx(lvlAmp);
  // Peak-Hold: 0,5 s halten, dann ~60 px/s abfallen
  if (px >= lvlHoldPx) { lvlHoldPx = px; lvlHoldT = now; }
  else if (now - lvlHoldT > 500) lvlHoldPx = max((float)px, lvlHoldPx - 60.0f * 0.064f);
  int16_t old = lvlDrawnFill;
  if (px > old) lvlFill(old, px, true);
  else if (px < old) lvlFill(px, old, false);
  lvlDrawnFill = px;
  int16_t h = (int16_t)lvlHoldPx;
  if (h >= lvlIW() - 1) h = lvlIW() - 2;
  if (h != lvlDrawnHold || px != old) {
    if (lvlDrawnHold >= 0 && lvlDrawnHold != h) lvlFill(lvlDrawnHold, lvlDrawnHold + 2, lvlDrawnHold + 2 <= px);
    if (h > px + 1) tft.fillRect(lvlX() + 1 + h, lvlY() + 1, 2, 8, C_TEXT);
    lvlDrawnHold = (h > px + 1) ? h : -1;
  }
  int8_t clip = (int32_t)(lvlClipUntil - now) > 0 ? 1 : 0;
  if (clip != lvlDrawnClip) {
    tft.drawRect(lvlX(), lvlY(), lvlW(), 10, clip ? C_LVL_R : C_LVL_FRAME);
    if (clip) tft.drawRect(lvlX() - 1, lvlY() - 1, lvlW() + 2, 12, C_LVL_R);
    else      tft.drawRect(lvlX() - 1, lvlY() - 1, lvlW() + 2, 12, C_BG);
    lvlDrawnClip = clip;
  }
}

void barSetup(float fs) {
  for (int k = 0; k < BAR_N; k++) {
    float f = BAR_F0 + k * BAR_STEP;
    barCoef[k] = 2.0f * cosf(2.0f * PI * f / fs);
    barS1[k] = barS2[k] = 0;
  }
  for (int n = 0; n < BAR_WIN; n++) barHann[n] = 0.5f - 0.5f * cosf(2.0f * PI * n / (BAR_WIN - 1));
  barPos = 0;
}

// Ein Spektrum ist fertig: dB, Glättung, Peak-Hold, Spitzenfrequenz
void barBlockDone() {
  float raw[BAR_N];
  for (int k = 0; k < BAR_N; k++) {
    float p = barS1[k] * barS1[k] + barS2[k] * barS2[k] - barCoef[k] * barS1[k] * barS2[k];
    if (p < 0) p = 0;
    float amp = 2.0f * sqrtf(p) / (BAR_WIN * 0.5f);   // Hann-Verstärkung 0,5 -> Amplitude in Counts
    raw[k] = 20.0f * log10f(amp + 0.1f);
    barS1[k] = barS2[k] = 0;
    barDb[k] = 0.5f * barDb[k] + 0.5f * raw[k];       // Glättung
    barHold[k] -= BAR_HOLD_DECAY;
    if (barDb[k] > barHold[k]) barHold[k] = barDb[k];
  }
  // Spitze im ungeglätteten Spektrum + Parabel-Interpolation
  int km = 0;
  for (int k = 1; k < BAR_N; k++) if (raw[k] > raw[km]) km = k;
  float d = 0;
  if (km > 0 && km < BAR_N - 1) {
    float a = raw[km - 1], b = raw[km], c = raw[km + 1];
    float den = a - 2 * b + c;
    if (den < -1e-3f) d = 0.5f * (a - c) / den;
    d = constrain(d, -0.5f, 0.5f);
  }
  // Rauschboden = Median der Bins
  float tmp[BAR_N];
  memcpy(tmp, raw, sizeof(tmp));
  for (int i = 1; i < BAR_N; i++) { float v = tmp[i]; int j = i - 1; while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; } tmp[j + 1] = v; }
  barNoiseDb = tmp[BAR_N / 2];
  barPeakDb  = raw[km];
  barPeakValid = (barPeakDb > barNoiseDb + 10.0f) && (barPeakDb > 15.0f);
  float hz = BAR_F0 + (km + d) * BAR_STEP;
  barPeakHz = barPeakValid ? (barPeakHz > 0 && fabsf(hz - barPeakHz) < 30 ? 0.6f * hz + 0.4f * barPeakHz : hz) : 0;
  barNew = true;
  afcUpdate(hz, barPeakValid && km > 0 && km < BAR_N - 1);
}

// AFC-Sollwert aus dem interpolierten Spektrums-Peak (nur stabile, deutliche Peaks)
// v36 /qsy: nach Durchgangsende (Pause > max(qsyMinMs, 14 Punktlängen)) sofort auf neuen stabilen Peak im Fangbereich
bool qsyOn = true; uint32_t qsyMinMs = 1000; bool qsyArmed = false; uint32_t qsyJumps = 0;
extern float unitMs;
void afcUpdate(float hz, bool valid) {
  if (!afcOn || narrowMode) return;          // schmal: Bargraph zeigt Peak, kein Einrasten
  if (!valid || fabsf(hz - afcCenter) > AFC_RANGE_HZ) { afcCnt = 0; return; }
  if (afcCnt > 0 && fabsf(hz - afcLastM) > AFC_STAB_HZ) afcCnt = 0;
  afcLastM = hz;
  if (afcCnt < 255) afcCnt++;
  uint32_t now = millis();
  float e = hz - afcHz;
  bool toneRecent = audioOn || now - lastToneMs < 300;
  if (qsyOn && afcLocked && lastToneMs && now - lastToneMs > max((float)qsyMinMs, 14.0f * unitMs)) qsyArmed = true;
  // Feinnachführung: eingerastet, Ton aktiv, Ablage <= 40 Hz -> sofort (ohne
  // Stabilitätszähler) geglättet nachziehen; interpolierte Spitze aus Nachbarbins
  if (afcLocked && toneRecent && fabsf(e) <= AFC_TRACK_HZ) {
    afcHz += 0.12f * e; afcLastMs = now; if (audioOn) lastToneMs = now; qsyArmed = false; return;
  }
  if (afcCnt < AFC_MIN_SPEC) return;
  if (afcLocked && fabsf(e) <= AFC_TRACK_HZ) {
    if (!toneRecent) return;                 // in Pausen nicht nachführen (QRM/Rauschen)
  } else if (afcLocked && lastToneMs && now - lastToneMs < AFC_FREE_MS && !(qsyArmed && barPeakDb >= barNoiseDb + 16.0f)) {
    // Wechsel auf anderes Signal nur, wenn die gehaltene Frequenz seit > 1 s schweigt
    // und der neue Peak deutlich (> 16 dB über Rauschen) und ~1,5 s stabil anliegt
    if (now - lastToneMs < afcSwMs || afcCnt < 8 * AFC_MIN_SPEC || barPeakDb < barNoiseDb + 16.0f) return;
  }
  if (qsyArmed && fabsf(e) > AFC_TRACK_HZ) qsyJumps++;
  qsyArmed = false;
  if (audioOn) lastToneMs = now;
  afcLastMs = now;
  if (fabsf(e) > AFC_JUMP_HZ) { afcHz = hz; afcLocked = false; }   // schnelles Einrasten
  else afcHz += (afcLocked ? 0.05f : 0.3f) * e;                    // danach langsam
  if (fabsf(hz - afcHz) < 5.0f) afcLocked = true;
}

// Sollwert in den Decoder übernehmen: nur zwischen Elementen, große Änderung nur außerhalb eines Zeichens
void afcApply() {
  float d = fabsf(afcHz - decHz);
  if (!afcOn || narrowMode || d < 0.5f) return;
  if (audioOn || durLen > 0) {
    // im Zeichen: in kleinen Schritten nachziehen (max. 1 Hz pro 8-ms-Block = 125 Hz/s)
    if (d > 40.0f) return;                   // große Sprünge nur in Pausen
    decHz += constrain(afcHz - decHz, -1.0f, 1.0f);
  } else decHz = afcHz;
  audioFreq = (int)(decHz + 0.5f);
  gCoeff = 2.0f * cosf(2.0f * PI * decHz / fsMeasured);
  { float sh = narrowMode ? NARROW_SIDE_HZ : WIDE_SIDE_HZ;
    sCoeffL = 2.0f * cosf(2.0f * PI * (decHz - sh) / fsMeasured); sCoeffH = 2.0f * cosf(2.0f * PI * (decHz + sh) / fsMeasured); }
  monSetFreqs(decHz);
  return;
  decHz = afcHz; audioFreq = (int)(decHz + 0.5f);
  gCoeff = 2.0f * cosf(2.0f * PI * decHz / fsMeasured);
}

inline void barSample(float x) {
  float xw = x * barHann[barPos];
  for (int k = 0; k < BAR_N; k++) {
    float s0 = xw + barCoef[k] * barS1[k] - barS2[k];
    barS2[k] = barS1[k]; barS1[k] = s0;
  }
  if (++barPos >= BAR_WIN) { barPos = 0; barBlockDone(); }
}

// Geometrie: Textzeile (Peak), Balken (50 px), Skala
extern bool scopeOn;
#define BAR_PITCH   8
#define BAR_W       6
int16_t barDbToH(float db);
bool scopeOn = true;
int16_t barX0() { return (W - BAR_N * BAR_PITCH) / 2 + 1; }
// v28 Oszillogramm (Hüllkurve) rechts neben den Balken
volatile float scPk = 0, scThr = 0, scSq = 0; volatile bool scDet = false;   // vom Audio-Task billig mitgeschrieben
#define SC_STEP_MS 50
GFXcanvas16 *scCv = nullptr; int16_t scW = 0;
uint8_t scH[240]; uint8_t scD[240]; int scPos = 0; uint32_t scLast = 0; int16_t scTh = -1, scSqH = -1;
int16_t scX() { return MARGIN_X + 1 + BAR_N * BAR_PITCH + 4; }
int16_t scDbToH(float m) { return m <= 1 ? 0 : barDbToH(20.0f * log10f(m)); }
void scopeSetupOld() {
  scW = constrain(W - MARGIN_X - scX(), 0, 240);
  if (!scCv && scW > 8) scCv = new GFXcanvas16(scW, BAR_PLOT_H + 1);
  memset(scH, 0, sizeof(scH)); memset(scD, 0, sizeof(scD));
}
void scopeDraw(uint32_t now) {
  if (!scopeOn || !barOn || multiOn || !scCv || now - scLast < SC_STEP_MS) return;
  scLast = now;
  float pk = scPk; bool dt = scDet; scPk = 0; scDet = false;
  scH[scPos] = scDbToH(pk); scD[scPos] = dt; scPos = (scPos + 1) % scW;
  scTh = scDbToH(scThr); scSqH = scSq > 0 ? scDbToH(scSq) : -1;
  int hb = BAR_PLOT_H;
  scCv->fillScreen(C_BG);
  scCv->drawRect(0, 0, scW, hb + 1, C_GRID);
  for (int db = 20; db < 60; db += 20) scCv->drawFastHLine(1, hb - barDbToH(db), scW - 2, C_GRID);
  for (int i = 0; i < scW - 2; i++) {
    int j = (scPos + 2 + i) % scW;
    int h = scH[j]; if (h > 0) scCv->drawFastVLine(1 + i, hb - h, h, scD[j] ? C_SYM : C_INFO);
  }
  if (scSqH > 0) scCv->drawFastHLine(1, hb - scSqH, scW - 2, C_LVL_R);   // Rauschsperre
  if (scTh > 0) scCv->drawFastHLine(1, hb - scTh, scW - 2, C_BARTGT);     // Schaltschwelle
  tft.drawRGBBitmap(scX(), barPlotTop(), scCv->getBuffer(), scW, hb + 1);
}
int16_t barPlotTop() { return barY0 + LVL_AREA_H + BAR_TXT_H + 2; }
int16_t barPlotBot() { return barPlotTop() + BAR_PLOT_H; }   // erste Zeile unter den Balken
int     barTargetBin() { return constrain((audioFreq - BAR_F0 + BAR_STEP / 2) / BAR_STEP, -1, BAR_N); }

int16_t barDbToH(float db) {
  float v = (db - BAR_DB_MIN) / (BAR_DB_MAX - BAR_DB_MIN);
  return (int16_t)constrain(v * BAR_PLOT_H, 0.0f, (float)BAR_PLOT_H);
}

void barDrawStatic() {
  tft.fillRect(0, barY0, W, BAR_AREA_H, C_BG);
  tft.drawFastHLine(0, barY0, W, C_INFO);
  int16_t yb = barPlotBot(), x0 = barX0();
  tft.drawFastHLine(x0 - 1, yb, BAR_N * BAR_PITCH, C_GRID);
  tft.setTextSize(1);
  tft.setTextColor(C_INFO, C_BG);
  const int lab[3] = {0, 12, 24};                 // 400 / 700 / 1000 Hz
  for (int i = 0; i < 3; i++) {
    int k = lab[i], xc = x0 + k * BAR_PITCH + BAR_W / 2;
    tft.drawFastVLine(xc, yb + 1, (k == 12) ? 5 : 3, (k == 12) ? C_TEXT : C_INFO);  // Mittenmarke 700 Hz
    char b[6]; snprintf(b, sizeof(b), "%d", BAR_F0 + k * BAR_STEP);
    int tw = strlen(b) * 6, tx = constrain(xc - tw / 2, 0, W - tw);
    tft.setCursor(tx, yb + 7); tft.print(b);
  }
  for (int k = 0; k < BAR_N; k++) { barDrawnH[k] = 0; barDrawnHold[k] = -1; }
  barDrawnTarget = -1;
  barDrawnTxt[0] = 0;
  lvlDrawStatic();
}

void barDraw(uint32_t now) {
  if (!barOn || !barNew || now - barLastDraw < BAR_FPS_MS) return;
  barNew = false; barLastDraw = now;
  int16_t yb = barPlotBot(), x0 = barX0();
  int tgt = barTargetBin();
  bool tgtChanged = (tgt != barDrawnTarget);
  for (int k = 0; k < BAR_N; k++) {
    int16_t x = x0 + k * BAR_PITCH;
    uint16_t col = (k == tgt) ? C_BARTGT : C_BAR;
    int16_t h = barDbToH(barDb[k]), oh = barDrawnH[k];
    if (tgtChanged && (k == tgt || k == barDrawnTarget)) oh = 0, tft.fillRect(x, yb - BAR_PLOT_H, BAR_W, BAR_PLOT_H, C_BG), barDrawnHold[k] = -1;
    if (h > oh)      tft.fillRect(x, yb - h, BAR_W, h - oh, col);
    else if (h < oh) tft.fillRect(x, yb - oh, BAR_W, oh - h, C_BG);
    barDrawnH[k] = h;
    // Peak-Hold-Marke (2 px), nur zeichnen, wenn sie über dem Balken liegt
    int16_t hh = barDbToH(barHold[k]);
    if (hh < 2) hh = -1;
    if (hh != barDrawnHold[k]) {
      int16_t oy = barDrawnHold[k];
      if (oy >= 0 && oy > h) tft.fillRect(x, yb - oy, BAR_W, 2, C_BG);   // alte Marke löschen
      if (hh > h + 1) tft.fillRect(x, yb - hh, BAR_W, 2, C_HOLD);
      barDrawnHold[k] = (hh > h + 1) ? hh : -1;
    }
  }
  barDrawnTarget = tgt;
  // Textzeile: Spitzenfrequenz, Ablage zur Zielfrequenz, Pfeil-Hinweis
  char txt[40];
  if (barPeakValid) {
    int pk = (int)(barPeakHz + 0.5f), dv = pk - audioFreq;
    const char *hint = (dv > 15) ? " tiefer v" : (dv < -15) ? " hoeher ^" : " OK";
    snprintf(txt, sizeof(txt), "Peak %4d Hz %+4d%s", pk, dv, hint);
  } else {
    snprintf(txt, sizeof(txt), "Peak ---  Ziel %d Hz", audioFreq);
  }
  if (afcOn) {   // AFC-Hinweis rechts: AFC = eingerastet, AFC? = sucht, HOLD = Signal weg, Frequenz gehalten
    const char *a = afcHold() ? "HOLD" : (afcLocked ? "AFC" : "AFC?");
    int n = strlen(txt); while (n < 31) txt[n++] = ' ';
    snprintf(txt + n, sizeof(txt) - n, "%s", a);
  }
  // Text nur bei Änderung und höchstens 4x/s; in einen RAM-Puffer schreiben
  // und als ein Block übertragen (~1 ms statt Pixel für Pixel)
  if (strcmp(txt, barDrawnTxt) && now - barLastTxt >= 250) {
    barLastTxt = now;
    bool ok = barPeakValid && abs((int)(barPeakHz + 0.5f) - audioFreq) <= 15;
    barCanvas.fillScreen(C_BG);
    barCanvas.setTextSize(1);
    barCanvas.setTextColor(ok ? C_SYM : (barPeakValid ? C_BARTGT : C_INFO));
    barCanvas.setCursor(0, 0);
    barCanvas.print(txt);
    tft.drawRGBBitmap(MARGIN_X, barY0 + LVL_AREA_H + 3, barCanvas.getBuffer(), barCanvas.width(), barCanvas.height());
    strcpy(barDrawnTxt, txt);
  }
}

// Pro Abtastwert im Audio-Task (Core 0): Bandpass + Squelch-Gate in den Puffer
inline void monSample(float x) {
  if (monMode != MON_FILT) return;
  if (fabsf(decHz - bqHz) > 2.0f) {               // Koeffizienten (RBJ-Bandpass, Breite /monbw)
    bqHz = decHz; float w = 2.0f * PI * bqHz / fsMeasured, Q = bqHz / (float)monBw, al = sinf(w) / (2 * Q), a0 = 1 + al;
    bqB0 = al / a0; bqB2 = -al / a0; bqA1 = -2 * cosf(w) / a0; bqA2 = (1 - al) / a0;
  }
  float y = bqB0 * x + bqZ1; bqZ1 = -bqA1 * y + bqZ2; bqZ2 = bqB2 * x - bqA2 * y;
  float tgt = monGate ? 1.0f : 0.0f;              // weiches Gate (~3 ms)
  monGain += (tgt - monGain) * 0.02f;
  // Skalierung: Signal-Amplitude ~ thrOn-Bereich -> ca. 100 DAC-Stufen bei Vollaussteuerung
  float v = y * monGain * monVol * 0.0008f;
  if (v > 127) v = 127; if (v < -127) v = -127;
  uint16_t w = monWr, n = (w + 1) & (MON_BUF - 1);
  if (n != monRd) { monBuf[w] = (int8_t)v; monWr = n; }
}

void mSetup(float fs); void mSample(float x); void mFlushPend(uint32_t tNow, int32_t idxNow, float fs);
void audioSetup() {
  gN = (int)(fsMeasured * (narrowMode ? NARROW_BLOCK_MS : AUDIO_BLOCK_MS) / 1000.0f + 0.5f);
  if (gN > 256) gN = 256;
  float k = gN * decHz / fsMeasured;              // nicht ganzzahliger Bin ist ok
  gCoeff = 2.0f * cosf(2.0f * PI * k / gN);
  monSetFreqs(decHz);
  hannSum = 0;
  for (int i = 0; i < gN; i++) { hannW[i] = narrowMode ? 0.5f - 0.5f * cosf(2.0f * PI * (i + 0.5f) / gN) : 1.0f; hannSum += hannW[i]; }
  float sideHz = narrowMode ? NARROW_SIDE_HZ : WIDE_SIDE_HZ;
  sCoeffL = 2.0f * cosf(2.0f * PI * (decHz - sideHz) / fsMeasured);
  sCoeffH = 2.0f * cosf(2.0f * PI * (decHz + sideHz) / fsMeasured);
  sL1 = sL2 = sH1 = sH2 = 0;
  gS1 = gS2 = 0; gCount = 0; blockSum = 0; blockEnergy = 0;
  float rc = 1.0f / (2.0f * PI * AUDIO_HP_HZ), dt = 1.0f / fsMeasured;
  hpA = rc / (rc + dt);
  barSetup(fsMeasured);
  mSetup(fsMeasured);
}

static bool IRAM_ATTR adcOvfCb(adc_continuous_handle_t, const adc_continuous_evt_data_t *, void *) { jOvf++; return false; }
bool audioBegin() {
  adc_continuous_handle_cfg_t hcfg = {};
  hcfg.max_store_buf_size = 16384;             // ~400 ms Reserve, falls loop() aufgehalten wird
  hcfg.conv_frame_size    = 256;               // 128 Samples pro Frame
  if (adc_continuous_new_handle(&hcfg, &adcHandle) != ESP_OK) return false;

  adc_digi_pattern_config_t pat = {};
  pat.atten     = ADC_ATTEN_DB_12;             // ca. 0,15..2,45 V, Bias 1,65 V
#if AUDIO_SELFTEST
  pat.channel   = ADC_CHANNEL_4;               // GPIO 32 (Selbsttest)
#else
  pat.channel   = ADC_CHANNEL_6;               // GPIO 34
#endif
  pat.unit      = ADC_UNIT_1;
  pat.bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

  adc_continuous_config_t cfg = {};
  cfg.pattern_num    = 1;
  cfg.adc_pattern    = &pat;
  cfg.sample_freq_hz = AUDIO_FS_HZ;
  cfg.conv_mode      = ADC_CONV_SINGLE_UNIT_1;
  cfg.format         = ADC_DIGI_OUTPUT_FORMAT_TYPE1;
  if (adc_continuous_config(adcHandle, &cfg) != ESP_OK) return false;
  { adc_continuous_evt_cbs_t cb = {}; cb.on_pool_ovf = adcOvfCb; adc_continuous_register_event_callbacks(adcHandle, &cb, nullptr); }   // v45
  if (adc_continuous_start(adcHandle) != ESP_OK) return false;
  fsT0 = millis();
  audioSetup();
  return true;
}

// Ein Goertzel-Block ist fertig: Magnitude bewerten, Schwelle nachführen
float bridgeU = 0.2f;   // /bridge: Fading-Überbrückung in Punktlängen (0 = aus, max 0,4)
int bridgeN = 0; uint32_t bridgeIdx = 0;
void audioBlockDone() {
  float power = gS1 * gS1 + gS2 * gS2 - gCoeff * gS1 * gS2;
  if (power < 0) power = 0;
  audioMag = 2.0f * sqrtf(power) / hannSum;     // ~ Amplitude in ADC-Counts (fensterkorrigiert)
  if (narrowMode) {
    float pL = sL1 * sL1 + sL2 * sL2 - sCoeffL * sL1 * sL2, pH = sH1 * sH1 + sH2 * sH2 - sCoeffH * sH1 * sH2;
    float side = sqrtf(max(0.0f, max(pL, pH)));
    sideRatio = power > 1e-6f ? side / sqrtf(power) : 9;
    float aL = 2.0f * sqrtf(max(0.0f, pL)) / hannSum, aH = 2.0f * sqrtf(max(0.0f, pH)) / hannSum;
    sidePow = 0.5f * (aL * aL + aH * aH);     // Leistung der Nachbarn (aus Tonanteil herausrechnen)
    sL1 = sL2 = sH1 = sH2 = 0;
  } else {
    float pL = sL1 * sL1 + sL2 * sL2 - sCoeffL * sL1 * sL2, pH = sH1 * sH1 + sH2 * sH2 - sCoeffH * sH1 * sH2;
    float aL = 2.0f * sqrtf(max(0.0f, pL)) / hannSum, aH = 2.0f * sqrtf(max(0.0f, pH)) / hannSum, a0 = audioMag;
    sL1 = sL2 = sH1 = sH2 = 0;
    audioMag = max(a0, max(aL, aH));            // Detektion: stärkster der 3 Bins
    // Schnell-Einrasten (nur bei offener Rauschsperre, erste Blöcke nach Pause > 1 s):
    // Ablage aus den 3 Bins schätzen und Decoder sofort nachziehen
    if (fastAcqN > 0 && audioOn && afcOn) {
      float d;
      if (a0 >= aL && a0 >= aH) { float den = 2 * a0 - aL - aH; d = den > 1e-3f ? 0.5f * (aH - aL) / den * WIDE_SIDE_HZ : 0; }
      else d = (aH > aL) ? WIDE_SIDE_HZ : -WIDE_SIDE_HZ;
      d = constrain(d, -WIDE_SIDE_HZ, WIDE_SIDE_HZ);
      if (fabsf(d) > 8.0f) {
        float nf = constrain(decHz + d, afcCenter - AFC_RANGE_HZ, afcCenter + AFC_RANGE_HZ);
        decHz = afcHz = nf; audioFreq = (int)(nf + 0.5f); afcLocked = true; afcCnt = 0; fastJumps++;
        float k = gN * decHz / fsMeasured; gCoeff = 2.0f * cosf(2.0f * PI * k / gN);
        sCoeffL = 2.0f * cosf(2.0f * PI * (decHz - WIDE_SIDE_HZ) / fsMeasured);
        sCoeffH = 2.0f * cosf(2.0f * PI * (decHz + WIDE_SIDE_HZ) / fsMeasured);
        monSetFreqs(decHz);
      }
      fastAcqN--;
    }
  }
  dcEst = 0.9f * dcEst + 0.1f * (blockSum / gN); // Gleichanteil (Bias) nachführen
  // Tonanteil: Leistung der Zielfrequenz (A²/2) im Verhältnis zur Blockleistung.
  // Reiner Ton ~1, Knacken/Rauschen/Brumm deutlich kleiner.
  float var = blockEnergy / gN;
  if (narrowMode) var = max(var - sidePow, 0.25f * var);   // Nachbarstationen zählen nicht als "Störung"
  tonality = var > 1.0f ? (audioMag * audioMag * 0.5f) / var : 0;
  // Nachbarstation: Seitenbin (±150 Hz) fast so stark wie die Zielfrequenz -> kein Ton
  // (nur Übersprechen: Hauptbin < 30 % des Seitenbins)
  if (narrowMode && sideRatio > 3.3f) tonality = 0;
  gS1 = gS2 = 0; gCount = 0; blockSum = 0; blockEnergy = 0;
  // v33: Median der 6 Nachbar-Bins (robust gegen einzelne Nachbarstationen)
  if (wbOn) {
    float a[6];
    for (int j = 0; j < 6; j++) { float p = wbS1[j] * wbS1[j] + wbS2[j] * wbS2[j] - wbC[j] * wbS1[j] * wbS2[j]; a[j] = 2.0f * sqrtf(max(0.0f, p)) / hannSum; wbS1[j] = wbS2[j] = 0; }
    std::sort(a, a + 6); wbMed = 0.5f * (a[2] + a[3]);
    if (fabsf(decHz - wbCoefHz) > 2.0f) { wbCoefHz = decHz; for (int j = 0; j < 6; j++) wbC[j] = 2.0f * cosf(2.0f * PI * (decHz + WB_OFS[j]) / fsMeasured); }
    wbBlk = audioMag < wbMed * wbK;                // Ton nicht deutlich über den Nachbarn -> Breitband
  } else wbBlk = false;

  // Spitzenwert: schnell hoch, langsam runter
  if (audioMag > peakMag) peakMag = audioMag;
  else {   // v33: Abfall an WPM gekoppelt (Zeitkonstante thrDecay Punktlängen), damit die Schwelle bei Fading folgt
    float dk = 0.002f;
    if (thrDecay > 0) dk = max(dk, min(0.5f, (gN * 1000.0f / fsMeasured) / (thrDecay * unitMs)));
    peakMag += dk * (noiseFloor - peakMag);
  }

  // Rauschboden nur nachführen, wenn kein Ton anliegt
  if (!detOn) {
    // Nur bei niedrigem Tonanteil (Rauschen) steigen lassen: sonst zieht ein Ton knapp unter der
    // Schwelle den Boden zu sich hoch und sperrt sich selbst aus
    bool tonalBlk = tonality > NF_TONAL;
    if (audioMag < noiseFloor) noiseFloor += 0.10f * (audioMag - noiseFloor);
    else if (!tonalBlk)        noiseFloor += 0.01f * (audioMag - noiseFloor);
    if (!tonalBlk || audioMag < nfMean) nfMean += 0.02f * (audioMag - nfMean);   // Mittelwert für die Rauschsperre
    // Schutz gegen Aussperren: > ~0,5 s Ton deutlich über dem Boden, aber nicht erkannt -> Boden langsam absenken
    nfBlocked = (tonalBlk && audioMag > 2.0f * noiseFloor) ? nfBlocked + 1 : 0;
    if (nfBlocked > NF_BLOCK_N) { noiseFloor *= 0.995f; nfMean *= 0.995f; }
  } else nfBlocked = 0;

  thrOn = noiseFloor * AUDIO_SNR_ON;
  float mid = noiseFloor + thrMid * (peakMag - noiseFloor);   // v33: /thrmid
  if (mid > thrOn) thrOn = mid;
  if (thrOn < AUDIO_MIN_MAG) thrOn = AUDIO_MIN_MAG;
  thrOff = thrOn * hystF;   // v35: /hyst
  if (thrOff < noiseFloor * 2.0f) thrOff = noiseFloor * 2.0f;
  float sqThr = 0;
  if (sqDb > 0) sqThr = nfMean * powf(10.0f, (sqDb + sqBoost + ((afcOn && !afcLocked) ? SQ_UNLOCK_DB : 0)) / 20.0f);

  bool wbMark = false;
  // v34 /ratio: Verhältnis Ton/Breitband (tonality = Ton-Leistung / Block-Varianz, fading-invariant) als Zusatzkriterium
  bool rOn = ratioOn && tonality > ratioHi && audioMag > sqThr && audioMag > 2.0f * noiseFloor;
  bool rHold = ratioOn && tonality > 0.7f * ratioHi && audioMag > sqThr;
  if (!detOn && audioMag > sqThr && ((audioMag > thrOn && tonality > AUDIO_TONAL_ON) || rOn)) {
    if (wbBlk) { wbRej++; wbMark = true; } else { detOn = true; onEdges++; }      // v33: Breitband -> kein Ton-an
  } else if (detOn && ((audioMag < thrOff && !rHold) || tonality < AUDIO_TONAL_HOLD)) {
    // v33: Klick/Knacken im Ton (nur Tonanteil eingebrochen, Pegel noch da) -> max. 2 Blöcke halten
    if (wbBlk && audioMag >= thrOff && wbHold < 2) { wbHold++; wbHeld++; wbMark = true; } else { detOn = false; wbHold = 0; }
  } else wbHold = 0;

  // Mindestdauer: Ein-Flanke erst nach Bestätigung, Zeitstempel = tatsächlicher Beginn
  bool was = audioOn;
  if (detOn && !audioOn) {
    onCnt++;
    float blkMs = gN * 1000.0f / fsMeasured;
    float mo = constrain(0.5f * unitMs, MINON_MIN_MS, MINON_MAX_MS);
    int need = sqDb > 0 ? max(1, (int)ceilf((mo - 4.0f) / blkMs)) : 1;
    if (onCnt >= need) {
      if (millis() - lastToneMs > afcSwMs) fastAcqN = 4;   // neuer Durchgang -> schnell einrasten
      audioOn = true; lastToneMs = millis(); monGate = true;
      if (aPendN < AEDGE_N) { aPendIdx[aPendN] = (int32_t)aProcIdx - (onCnt - 1) * gN; aPendOn[aPendN] = true; aPendN++; }
      onCnt = 0;
    }
  } else if (!detOn) {
    if (onCnt > 0) sqDropped++;
    onCnt = 0;
    if (audioOn) {
      // Fading-Überbrückung: kurze Einbrüche (< bridgeU x Punktlänge) halten den Ton;
      // endet er wirklich, zählt die Aus-Flanke ab dem ersten Einbruch
      if (bridgeN == 0) bridgeIdx = aProcIdx;
      bridgeN++;
      float blkMs = gN * 1000.0f / fsMeasured;
      if (bridgeN * blkMs > bridgeU * unitMs) {
        audioOn = false; monGate = false; bridgeN = 0;
        if (aPendN < AEDGE_N) { aPendIdx[aPendN] = bridgeIdx; aPendOn[aPendN] = false; aPendN++; }
      }
    }
  } else bridgeN = 0;
  stFeed(audioMag, detOn, audioOn, thrOn, sqThr, wbMark);   // v30 Streifen (nur mitschreiben)
  afcApply();
  if (magStream) Serial.printf("#M%d,%d,%d\n", (int)(audioMag + 0.5f), (int)(tonality * 100 + 0.5f), audioOn ? 1 : 0);
  (void)was;
}

void printDecoderStatus();
void printAudioStatus(const char *tag) {
  float zcHz = zcSamples ? (zcCount / 2.0f) * fsMeasured / zcSamples : 0;
  Serial.printf("\n%s ton=%.2f mag=%.1f nf=%.1f peak=%.1f thrOn=%.1f thrOff=%.1f on=%d edges=%u dc=%.0f raw=%d..%d zcHz=%.0f fs=%.0f f=%d N=%d src=%s pk=%.1f dpk=%+.1f pkdb=%.1f nfdb=%.1f loopMax=%uus barMax=%uus lvl=%.0f clip=%d afc=%s afcHz=%.1f lock=%d hold=%d sq=%.0f nfm=%.1f sqDrop=%u\n",
                tag, tonality, audioMag, noiseFloor, peakMag, thrOn, thrOff, audioOn ? 1 : 0, (unsigned)onEdges, dcEst,
                rawMin, rawMax, zcHz, fsMeasured, audioFreq, gN,
                keySource == 3 ? "both" : (keySource == 1 ? "key" : "audio"),
                barPeakValid ? barPeakHz : 0.0f, barPeakValid ? barPeakHz - audioFreq : 0.0f, barPeakDb, barNoiseDb,
                (unsigned)loopMaxUs, (unsigned)barMaxUs, (float)lvlAmp, lvlClipAny ? 1 : 0,
                afcOn ? "on" : "off", decHz, afcLocked ? 1 : 0, afcHold() ? 1 : 0, sqDb, nfMean, (unsigned)sqDropped);
  lvlClipAny = false;
  loopMaxUs = 0; barMaxUs = 0;
  rawMin = 4095; rawMax = 0; zcCount = zcSamples = 0; onEdges = 0;
  printDecoderStatus();
}

// v47: Audio-Uhr aus dem Abtastwert-Zaehler (DMA-Takt). Flanken- und Pausenzeiten haengen
// so nicht mehr davon ab, wann/wie lange der Audio-Task verarbeitet (WLAN/Netz-Unterbrechungen).
// Die Uhr wird nur langsam an millis() nachgefuehrt (Drift der gemessenen Abtastrate) und
// laeuft nie vor millis().
double   aClkMs = -1;          // Zeit des zuletzt verarbeiteten Abtastwerts (ms)
uint32_t aClkEnd = 0;          // dieselbe Zeit als uint32 (Decoder-"now")
float    aClkMinLag = 1e9f; uint32_t aClkChkT = 0;
// Alle vorliegenden ADC-Daten abholen und verarbeiten (aus loop())
void audioLoop(uint32_t now) {
  if (!audioOk) return;
  static uint8_t buf[256];
  uint32_t len = 0;
  aProcIdx = 0; aPendN = 0; uint32_t rdTot = 0;
  while (adc_continuous_read(adcHandle, buf, sizeof(buf), &len, 0) == ESP_OK && len) {
    rdTot += len;
    for (uint32_t i = 0; i + 1 < len; i += SOC_ADC_DIGI_RESULT_BYTES) {
      adc_digi_output_data_t *d = (adc_digi_output_data_t *)&buf[i];
      if (d->type1.channel != pat_channel) continue;
      int raw = d->type1.data;
      if (dumpPos >= 0 && dumpPos < DUMP_LEN) dumpBuf[dumpPos++] = raw;
      if (raw < rawMin) rawMin = raw;
      if (raw > rawMax) rawMax = raw;
      // Hochpass 1. Ordnung entfernt Gleichanteil, Brumm und Knackflanken
      hpY = hpA * (hpY + raw - hpX); hpX = raw;
      float x = hpY;
      monSample(x);
      blockEnergy += x * x;
      aProcIdx++;
      // Nulldurchgänge mit Hysterese zählen -> gemessene Tonfrequenz
      if (zcPos && x < -40) { zcPos = false; zcCount++; }
      else if (!zcPos && x > 40) { zcPos = true; zcCount++; }
      zcSamples++;
      blockSum += raw;
      if (raw < lvlMin) lvlMin = raw;
      if (raw > lvlMax) lvlMax = raw;
      if (++lvlCnt >= LVL_WIN) lvlWindowDone();
      sampleCount++;
      if (multiOn) mSample(x);           // Mehrkanal: 4 Goertzel, eigene Blöcke
      else {
      float xw = x * hannW[gCount];
      float s0 = xw + gCoeff * gS1 - gS2;
      gS2 = gS1; gS1 = s0;
      {
        float a = xw + sCoeffL * sL1 - sL2; sL2 = sL1; sL1 = a;
        float b = xw + sCoeffH * sH1 - sH2; sH2 = sH1; sH1 = b;
      }
      if (wbOn) for (int j = 0; j < 6; j++) { float c = xw + wbC[j] * wbS1[j] - wbS2[j]; wbS2[j] = wbS1[j]; wbS1[j] = c; }
      if (++gCount >= gN) audioBlockDone();
      }
      if (barOn || afcOn) barSample(x);
    }
  }
  if (rdTot > jBackMax) jBackMax = rdTot;   // v45: Rückstau in Bytes (2 Byte je Abtastwert)
  { // v47 Audio-Uhr fortschreiben
    uint32_t ms = millis();
    if (aClkMs < 0) aClkMs = ms;
    aClkMs += aProcIdx * 1000.0 / fsMeasured;
    float lag = (float)((double)ms - aClkMs);
    if (lag < 0) { aClkMs = ms; lag = 0; }                 // nie vor der echten Zeit
    if (lag < aClkMinLag) aClkMinLag = lag;
    if (lag > 1000) { aClkMs = ms; aClkMinLag = 0; }       // Ueberlauf/Langstillstand: neu ansetzen
    if (ms - aClkChkT >= 1000) {                            // 1x/s: Minimal-Verzug langsam auf ~3 ms ziehen
      aClkChkT = ms;
      if (aClkMinLag < 1e8f && aClkMinLag > 4.0f) aClkMs += 0.5 * (aClkMinLag - 3.0f);
      aClkMinLag = 1e9f;
    }
    aClkEnd = (uint32_t)(int64_t)aClkMs;
  }
  // Flanken dieses Aufrufs mit Zeitstempel in die Warteschlange: der letzte
  // gelesene Abtastwert entspricht ungefähr "jetzt"
  for (int i = 0; i < aPendN; i++) {
    uint32_t back = (uint32_t)(((int32_t)aProcIdx - aPendIdx[i]) * 1000.0f / fsMeasured);
    int nh = (aEdgeHead + 1) % AEDGE_N;
    if (nh == aEdgeTail) break;                    // voll (sollte nicht vorkommen)
    aEdge[aEdgeHead].t = aClkEnd - back;   /* v47: Abtast-Uhr */ aEdge[aEdgeHead].on = aPendOn[i]; aEdgeHead = nh;
  }
  aPendN = 0;
  if (multiOn) mFlushPend(aClkEnd, (int32_t)aProcIdx, fsMeasured);   // v47: Abtast-Uhr
  // tatsächliche Abtastrate messen (alle 2 s) und Filter anpassen
  // Abtastrate kumulativ seit Start/letztem Flush messen: Daten, die noch im
  // DMA-Puffer liegen (loop() kurz aufgehalten), verfälschen so nur minimal.
  // Filter (Goertzel, Bargraph) erst bei > 0,3 % Abweichung neu berechnen.
  if (now - fsT0 >= 5000 && now - fsLastChk >= 2000) {
    fsLastChk = now;
    float fs = sampleCount * 1000.0f / (now - fsT0);
    if (fs > 1000 && fabsf(fs - fsMeasured) > fsMeasured * 0.003f) { fsMeasured = fs; audioSetup(); }
  }
  if (audioDebug && now - lastDbg >= 100) { lastDbg = now; printAudioStatus("#A"); }
  if (dumpPos >= DUMP_LEN) {
    Serial.printf("\n#DUMP fs=%.0f n=%d\n", fsMeasured, DUMP_LEN);
    for (int i = 0; i < DUMP_LEN; i += 32) {
      Serial.print("#V");
      for (int j = i; j < i + 32; j++) { Serial.print(' '); Serial.print(dumpBuf[j]); }
      Serial.println();
    }
    Serial.println("#DUMP Ende");
    dumpPos = -1;
  }
}

#if AUDIO_SELFTEST
// Nicht blockierender Morse-Generator auf SELFTEST_PIN
int stoneHz = 0;                      // /stone N: Dauerton N Hz statt Morse (0 = Morse)
int stMorseHz = 700;                  // /smorse N: Tonfrequenz des Selbsttest-Morse
// Rechteck über LEDC direkt (zuverlässiger als tone()/noTone() bei schnellem Wechsel)
void stTone(int hz) {
  static bool attached = false;
  if (!attached) { ledcAttach(SELFTEST_PIN, 1000, 8); attached = true; }
  if (hz > 0) ledcWriteTone(SELFTEST_PIN, hz); else ledcWrite(SELFTEST_PIN, 0);
}
void selftestLoop(uint32_t now) {
  static int playing = -1;
  if (stoneHz > 0) {
    if (playing != stoneHz) { stTone(stoneHz); playing = stoneHz; }
    return;
  }
  if (playing > 0) { stTone(0); playing = -1; }
#ifndef SELFTEST_TEXT
#if SELFTEST_ALLCHARS   // alle Zeichen der Tabelle (UTF-8), CH wird als ---- gegeben
#define SELFTEST_TEXT "ABCDEFGHIJKLM NOPQRSTUVWXYZ 0123456789 \xC3\x84\xC3\x96\xC3\x9C CHA \xC3\x9F" "E \xC3\x89\xC3\x80\xC3\x91 \xC3\x85" "A .,? '!/ ()& :;= +-_ \"$@ <AR>K <SK> <SOS> <HH> <BT>A <KN>A <AS>A OK "
#else
#define SELFTEST_TEXT "PARIS SOS 0123456789 "
#endif
#endif
  static int sym[200]; static int nsym = -1;          // Symbolfolge (Index, -2 = Wortpause)
  if (nsym < 0) {
    String t = SELFTEST_TEXT; nsym = 0; bool chOld = txCombineCh; txCombineCh = SELFTEST_ALLCHARS;
    for (unsigned i = 0; nsym < 200;) { int k = nextSymbol(t, i); if (k == -3) break; if (k != -1) sym[nsym++] = k; }
    while (nsym && sym[nsym - 1] == -2) nsym--;      // Leerzeichen am Ende entfernen
    txCombineCh = chOld;
  }
  static int ci = 0, ei = 0;          // Symbol- / Elementindex
  static bool on = false;
  static uint32_t next = 3000;        // Start nach 3 s
  static const int wpms[3] = {12, 18, 25};
  static int wi = 0;
  const uint32_t u = 1200 / wpms[wi];
  if ((int32_t)(now - next) < 0) return;
  if (on) {                           // Element zu Ende -> Pause
    stTone(0); on = false;
    const char *code = MORSE[sym[ci]].code;
    if (code[ei + 1]) { ei++; next = now + u; return; }
    ei = 0; ci++;                     // nächstes Symbol
    next = now + 3 * u;               // Zeichenpause
    if (ci < nsym && sym[ci] == -2) { ci++; next = now + 7 * u; }   // Wortpause
    if (ci >= nsym) {                                    // Textende: 3 s Pause
      ci = 0; next = now + 3000; wi = (wi + 1) % 3;
      Serial.printf("\n#T Selbsttest naechste Runde: %d WPM\n", wpms[wi]);
    }
    return;
  }
  stTone(stMorseHz); on = true;
  next = now + (MORSE[sym[ci]].code[ei] == '-' ? 3 * u : u);
}
#endif

// Nach blockierenden Aktionen (Geber) alte ADC-Daten verwerfen
void audioFlush() {
  if (!audioOk) return;
  static uint8_t buf[256];
  uint32_t len = 0;
  while (adc_continuous_read(adcHandle, buf, sizeof(buf), &len, 0) == ESP_OK && len) {}
  gS1 = gS2 = 0; gCount = 0; blockSum = 0; audioOn = false; detOn = false; onCnt = 0; aEdgeTail = aEdgeHead; aPendN = 0; audioState = false;
  sampleCount = 0; fsT0 = millis();
}

// ==========================================================================
//  DECODER
// ==========================================================================
// Ausgabe Decoder-Text seriell; im Mehrkanal-Modus mit Kanal-Kennung
int mLastPr = -1;
uint32_t labT0 = 0, labT1 = 0; void labPush(uint32_t t0, uint32_t t1, const char *s);
void dPrint(const char *t) { if (mCur < 0) labPush(labT0, labT1, t); if (elemOut) { Serial.printf("\n#C%u,%s\n", charT, t); return; } if (mCur != mLastPr) { if (mCur >= 0) Serial.printf("\n[K%d %d Hz] ", mCur + 1, mFreq[mCur]); else Serial.print('\n'); mLastPr = mCur; } Serial.print(t); }
void dPrint(char c) { char t[2] = {c, 0}; dPrint(t); }
char     symBuf[10];          // aktuelle Punkte/Striche (Anzeige)
int      symLen = 0;
uint32_t lastRelease = 0;     // Ende des letzten Elements
uint32_t pressStart  = 0;     // Beginn des aktuellen Elements
bool     wordSpacePending = false;   // letztes Zeichen fertig, Wortpause noch offen
bool     cgPending = false;          // Leerzeichen schon per Timeout geschrieben, Pausenlänge noch erfassen
bool     afterPause = false;          // lange Pause -> nächstes Zeichen prüft Tempowechsel
bool     isoCand = false;            // aktuelles Zeichen begann nach > 2 s Stille

// Tempo-Schätzung:
//  - Dauer der Elemente des aktuellen Zeichens (endgültige Punkt/Strich-
//    Entscheidung erst am Zeichenende -> auch das erste Zeichen nach einem
//    Tempowechsel wird richtig erkannt)
//  - Verlauf der letzten 16 Elementdauern: zwei Gruppen (Punkte/Striche)
//  - Pausen innerhalb eines Zeichens (= 1 Einheit) als Referenz, wenn ein
//    Zeichen nur aus Punkten oder nur aus Strichen besteht (S, O, 0, 5 ...)
//  - Pausen innerhalb des aktuellen Zeichens werden mitgespeichert: stellt
//    sich am Zeichenende (mit neuer Tempo-Schätzung) heraus, dass eine davon
//    eine Zeichenpause war, wird das Zeichen nachträglich geteilt
//    (z. B. "TH" bei Tempowechsel nicht mehr als "6")
//  - Pausen zwischen Zeichen werden in Einheiten gespeichert (12 Stück) und in
//    zwei Gruppen geteilt (Zeichen- / Wortpause). Damit funktioniert auch
//    Farnsworth-Timing (gedehnte Pausen). Das Leerzeichen wird deshalb erst
//    beim Beginn des nächsten Zeichens ausgegeben, wenn die Pause bekannt ist.
float    thrMs = 2.0f * 1200.0f / START_WPM;   // Grenze Punkt/Strich
float    durBuf[9];   int durLen  = 0;
float    cgBuf[8];                           // Pausen im aktuellen Zeichen
float    hist[16];    int histLen = 0, histPos = 0;
float    gapIn[8];    int gapLen  = 0, gapPos  = 0;
float    gapEma = 1200.0f / START_WPM;       // schneller Mittelwert der Pausen im Zeichen
float    cgU[12];     int cgLen   = 0, cgPos   = 0;   // Pausen zwischen Zeichen (Einheiten)
// Ton-Verlängerung: Die Tonerkennung (Goertzel-Block, Nachlauf) macht Töne um
// b ms länger und Pausen um b ms kürzer. Punkt = u+b, Pause im Zeichen = u-b.
int      hOutl = 0;                          // Zähler Tempowechsel (handleElement)
float    biasMs = 0;                         // geschätzte Verlängerung b
uint32_t prevPressStart = 0;                 // für Zusammenfügen bei Fading-Lücke

// Mittel der kurzen Pausen im Zeichen (untere Gruppe, = u-b); 0 wenn keine
float gapInShort() {
  if (gapLen == 0) return 0;
  float mn = 1e9; for (int i = 0; i < gapLen; i++) mn = min(mn, gapIn[i]);
  float s = 0; int n = 0;
  for (int i = 0; i < gapLen; i++) if (gapIn[i] < 1.8f * mn) { s += gapIn[i]; n++; }
  return s / n;
}

// v37 Handtaste/Paddle: Statistik der Elementlängen, weichere Timing-Toleranzen bei Hand
#define HAND_N 24
float handDot[HAND_N], handDash[HAND_N]; int handDn = 0, handDd = 0, handDi = 0, handDa = 0;
float handShortW = 0;   // EMA: Anteil kurzer Wortpausen (< 5 Einheiten)
uint8_t handMode = 0;   // 0 = auto, 1 = on (Hand), 2 = off (Paddle)
int8_t  handIs = 0;     // 0 = unbekannt (A), 1 = Hand, -1 = Paddle
float   handScore = 0;  // 0..100
bool isHand() { return handMode == 1 || (handMode == 0 && handIs > 0); }  // handIs: 0 unbekannt, 1 Hand, -1 Paddle
void handFeed(float dur, bool isDash) {
  float *b = isDash ? handDash : handDot; int &n = isDash ? handDd : handDn; int &p = isDash ? handDa : handDi;
  float rel = dur / max(unitMs, 1.0f);   // relativ zur aktuellen Einheit (Tempo-unabhängig)
  b[p] = rel; p = (p + 1) % HAND_N; if (n < HAND_N) n++;
}
float handCd = 0, handCD = 0;              // robuste Streuung Punkte/Striche (v38)
float handOnS = 50, handOffS = 30, handWW = 12;   // Schwellen / Gewicht Wortpausen
int handNmin() { return handDn + handDd; }
void handUpdate() {
  // v38: robust: Median + MAD (1,4826*MAD/Median), Ausreißer (Fading, gespaltene/verschmolzene Elemente) ignoriert
  auto rs = [](float *b, int n, float lo, float hi) -> float {
    float t[HAND_N]; int k = 0;
    for (int i = 0; i < n; i++) if (b[i] > lo && b[i] < hi) t[k++] = b[i];
    if (k < 6) return 0;
    std::sort(t, t + k); float med = t[k / 2]; if (med <= 0) return 0;
    for (int i = 0; i < k; i++) t[i] = fabsf(t[i] - med);
    std::sort(t, t + k); float sp = 1.4826f * t[k / 2];
    float j = 8.0f / max(unitMs, 1.0f);   // Detektor-Raster (8-ms-Block) als Flankenjitter abziehen
    return sqrtf(max(0.0f, sp * sp - j * j)) / med;
  };
  handCd = rs(handDot, handDn, 0.4f, 1.8f); handCD = rs(handDash, handDd, 1.8f, 5.0f);
  float sc = 0;
  sc += constrain((handCd - 0.08f) / 0.17f, 0.0f, 1.0f) * 40;   // Punkt-Unregelmäßigkeit
  sc += constrain((handCD - 0.08f) / 0.17f, 0.0f, 1.0f) * 45;   // Strich-Unregelmäßigkeit
  if (handCd > 0.10f) sc += handShortW * handWW;                 // kurze Wortpausen nur mit Streuung, schwach
  handScore = constrain(sc, 0.0f, 100.0f);
  if (handNmin() < 12) return;                                   // zu wenig Elemente: Anzeige A
  if (handIs > 0) { if (handScore < handOffS) handIs = -1; }
  else            { if (handScore > handOnS) handIs = 1; else if (handScore < handOffS) handIs = -1; }
}

void estimateSpeed() {
  if (histLen == 0) return;
  float mn = 1e9, mx = 0, sum = 0;
  for (int i = 0; i < histLen; i++) { mn = min(mn, hist[i]); mx = max(mx, hist[i]); sum += hist[i]; }
  if (mx >= 2.0f * mn) {
    // zwei Gruppen: Punkte und Striche
    float mid = (mn + mx) / 2, ds = 0, Ds = 0; int dn = 0, Dn = 0;
    for (int i = 0; i < histLen; i++) {
      if (hist[i] < mid) { ds += hist[i]; dn++; } else { Ds += hist[i]; Dn++; }
    }
    if (dn == 0 || Dn == 0) return;
    float da = ds / dn, Da = Ds / Dn;
    thrMs  = (da + Da) / 2;
    unitMs = (da + Da / 3) / 2;
    float gi = gapInShort();
    if (gapLen >= 2 && gi > 0.25f * da && gi < 1.6f * da) {
      // Einheit aus Ton+Pause (verlängerungsfrei): Punkt+Pause = 2u, Strich+Pause = 4u
      unitMs = ((da + gi) / 2 + (Da + gi) / 4) / 2;
      biasMs = constrain((da - gi) / 2, -0.3f * unitMs, 0.45f * unitMs);
    }
  } else {
    // nur eine Gruppe: Punkte oder Striche?
    float avg = sum / histLen;
    if (gapLen > 0) {
      float g = 0; for (int i = 0; i < gapLen; i++) g += gapIn[i]; g /= gapLen;
      unitMs = (avg > 2.0f * g) ? avg / 3 : avg;
    } else if (avg <= 1.6f * unitMs) {
      unitMs = avg;                 // gleich schnell oder schneller -> Punkte
    }                               // sonst mehrdeutig: Schätzung beibehalten
    thrMs = 2.0f * unitMs;
  }
  unitMs = constrain(unitMs, 1200.0f / MAX_WPM, 1200.0f / MIN_WPM);
  // Hand: Punkt/Strich-Grenze nicht unter 1,85 Einheiten (kurzer erster Strich -> sonst K als U)
  if (isHand()) thrMs = min(thrMs, 1.65f * unitMs);  // Hand: kurze Striche (K→U) noch als Strich
}

// Schwelle Zeichenende: 2,5 Einheiten. Werden die Pausen im Zeichen deutlich
// kürzer (Tempo steigt), gilt sofort deren Mittelwert als Einheit.
float charGapMs() {
  float base = (gapEma + biasMs < 0.7f * unitMs) ? gapEma + biasMs : unitMs;
  float cu = CHAR_GAP_UNITS;   // v37: Hand-Anpassung hier getestet, ohne Nutzen
  float g = max(cu * base - biasMs, 1.2f * base);
  return g;
}

// Schwelle Wortpause in Einheiten aus den letzten Pausen zwischen Zeichen
static float medianOf(float *v, int n) {
  float t[12]; for (int i = 0; i < n; i++) t[i] = v[i];
  std::sort(t, t + n); return (n & 1) ? t[n / 2] : (t[n / 2 - 1] + t[n / 2]) / 2;
}
// Werte in cgU sind echte Einheiten (Tonverlängerung herausgerechnet).
// Normal: Zeichen 3, Wort 7 -> Schwelle 5. Am Anfang (< 4 Pausen) konservativ 6,5.
// Farnsworth: zwei Cluster, Mitte der Mediane; ab 4 Pausen.
float wordGapUnitsRaw();
float wordGapUnits() {
  float w = min(wordGapUnitsRaw(), WORDGAP_MAX_U);
  // v37: Wortpausen-Kappung bei Hand getestet und verworfen (Korpus schlechter)
  return w;
}
float wordGapUnitsRaw() {
  // Start: erste Pause konservativ 6,5; danach (1..3 Pausen) 5,0, außer alle
  // bisherigen Pausen sind gedehnt (> 4,5, Farnsworth) -> 1,6 x kleinste
  if (cgLen == 0) return 6.5f;
  if (cgLen < 4) {
    float m0 = 1e9; for (int i = 0; i < cgLen; i++) m0 = min(m0, cgU[i]);
    return (m0 > 4.5f) ? 1.6f * m0 : 5.0f;
  }
  float md = medianOf(cgU, cgLen);
  float mn = 1e9, mx = 0;
  for (int i = 0; i < cgLen; i++) { mn = min(mn, cgU[i]); mx = max(mx, cgU[i]); }
  if (mx >= 1.3f * mn) {
    float mid = (mn + mx) / 2, a[12], b[12]; int na = 0, nb = 0;
    for (int i = 0; i < cgLen; i++) { if (cgU[i] < mid) a[na++] = cgU[i]; else b[nb++] = cgU[i]; }
    float ma = medianOf(a, na), mb = medianOf(b, nb);
    if (cgLen >= 4 && na >= 2 && ma > 4.5f) return (ma + mb) / 2;   // Farnsworth
    if (mb >= 1.3f * ma) return constrain((ma + mb) / 2, 3.7f, WG_HI1); // kurze Wortpausen (Hand/Keyer)
  }
  if (md > 4.5f) return max(5.0f, 1.6f * md);   // nur gedehnte Zeichenpausen
  // eine Gruppe (meist Zeichenpausen ~3): Schwelle knapp darüber
  return constrain(1.45f * md, 3.8f, WG_HI2);
}

void printDecoderStatus() {
  Serial.printf("#D key=%d since=%u lastRel=%u durLen=%d histLen=%d gapLen=%d cgLen=%d unit=%.1f thr=%.1f wpm=%.1f gapEma=%.1f wordU=%.1f bias=%.1f sqBoost=%.1f etDrop=%u\n",
                keyBtn.stable ? 1 : 0, (unsigned)keyBtn.since, (unsigned)lastRelease, durLen, histLen, gapLen, cgLen,
                unitMs, thrMs, 1200.0f / unitMs, gapEma, wordGapUnits(), biasMs, sqBoost, (unsigned)etDropped);
}

// Nach langer Pause: Verlauf vergessen und Tempo auf den Startwert (mittleres
// Tempo) zurücksetzen – so wird das erste Zeichen nach einem Tempowechsel in
// beide Richtungen (schneller/langsamer) am zuverlässigsten erkannt.
void resetSpeedHistory() {
  histLen = histPos = 0; gapLen = gapPos = 0; cgLen = cgPos = 0;
  unitMs = 1200.0f / START_WPM; thrMs = 2.0f * unitMs; gapEma = unitMs; biasMs = 0;
  // Hand-Statistik bewusst behalten (läuft über Sprecherpausen hinweg)
}

// Ein Teilstück (Elemente durBuf[from..to-1], Pausen cgBuf[from..to-2])
// dekodieren und ausgeben. Punkt/Strich-Grenze:
//  - Zeichen enthält deutlich kurze und lange Elemente -> Grenze aus dem
//    Zeichen selbst (robust direkt nach Tempowechseln)
//  - alle Elemente gleich lang -> Vergleich mit den Pausen im Zeichen
//    (Strich = 3 Einheiten, Pause = 1 Einheit)
//  - einzelnes Element ohne Vergleich -> Verhältnis zur Zeichenpause
//  - sonst globale Schwelle aus dem Verlauf
// Rausch-Filter: Folgen kurzer Zeichen (E, T, I, A, N, M) ohne Struktur sind
// typisch für Rauschen/QRM. Solche Zeichen werden zurückgehalten; kommt ein
// längeres Zeichen oder Timeout, wird entschieden: >= 3 E/T in Folge (bzw. >= 4
// kurze Zeichen mit >= 3 E/T) -> verwerfen und Rauschsperre adaptiv anheben.
int  pendK[16]; uint32_t pendT0[16], pendT1[16]; int pendN = 0, pendET = 0, pendChars = 0;
// --------------------------------------------------------------------------
//  Wörterbuch-Ergänzung (/dict on|off): Wort, das genau ein Element (Punkt/
//  Strich) neben einem gängigen CW-Wort liegt, wird ersetzt (Anzeige farbig,
//  seriell "ROH[KORR]"). Wörter im Wörterbuch und rufzeichenartige Wörter
//  (Buchstaben + Ziffern, >= 3 Zeichen) bleiben unverändert.
// --------------------------------------------------------------------------
bool dictOn = true;
uint32_t dictFixes = 0;
static const char *const DICT[] = {
  "CQ","DE","RST","RPRT","RIG","ANT","PWR","WX","NAME","QTH","TNX","TKS","FB","OM","YL","73","88","599","5NN",
  "TU","BK","HW","ES","UR","HR","GM","GA","GE","DR","AGN","PSE","CFM","CPY","SRI","WKD","ABT","BEST","INFO",
  "HPE","CUL","GL","VY","OP","SIG","FER","WID","TEST","DX","QSL","QRZ","QRL","QRM","QRN","QSB","QSO","QRP",
  "QRT","QSY","QRS","QRQ","QRV","QRX","QSK","WATTS","RX","TX","ABT","CONDX","GOOD","HPE","SOLID","COPY",
  "ALL","AND","THE","MY","IS","NR","ANT","DIPOLE","VERT","YAGI","NICE","MNI","CALL","QRU","GUD","TEMP","CLOUDY","SUNNY","RAIN"};
#define DICT_N (int)(sizeof(DICT) / sizeof(DICT[0]))
float softMaxC = 0.6f;   // max. Umkippkosten weiche Entscheidung
extern bool softOn; extern uint32_t softFixes;
char dWord[16]; char dCode[16][8]; int dLen = 0; bool dBad = false;
static const char *codeOf(char c) {
  for (int i = 0; i < MORSE_COUNT; i++) if (MORSE[i].txt[0] == c && !MORSE[i].txt[1]) return MORSE[i].code;
  return nullptr;
}
static bool oneElem(const char *a, const char *b) {   // Elementabstand genau 1 (einfügen/weglassen/vertauschen)
  int la = strlen(a), lb = strlen(b);
  if (la == lb) { int d = 0; for (int i = 0; i < la; i++) d += a[i] != b[i]; return d == 1; }
  if (la + 1 == lb) { const char *t = a; a = b; b = t; la++; lb--; }
  if (la != lb + 1) return false;
  for (int i = 0; i < la; i++) { bool ok = true; for (int j = 0, k = 0; j < la; j++) { if (j == i) continue; if (a[j] != b[k++]) { ok = false; break; } } if (ok) return true; }
  return false;
}
void dictAdd(int k, const char *code) {   // k = Tabellenindex oder -1 (unbekannt '*')
  if (dLen >= 15) { dBad = true; return; }
  if (k >= 0 && MORSE[k].txt[1]) dBad = true;        // Prosign/CH im Wort -> nicht anfassen
  dWord[dLen] = k >= 0 ? MORSE[k].txt[0] : '*'; strncpy(dCode[dLen], code ? code : "", 7); dCode[dLen][7] = 0; dLen++;
}
void dictReplace(const char *w, int rawLen);
void dictEnd() {
  int n = dLen; dLen = 0; dWord[n] = 0;
  if (!dictOn || dBad || n < 3 || mCur >= 0) { dBad = false; return; }
  bool dig = false, let = false;
  for (int i = 0; i < n; i++) { if (isdigit(dWord[i])) dig = true; else if (isalpha(dWord[i])) let = true; }
  if (dig && let && n >= 3) return;                    // Rufzeichen?
  for (int d = 0; d < DICT_N; d++) if (!strcmp(DICT[d], dWord)) return;
  int hit = -1;
  for (int d = 0; d < DICT_N; d++) {
    const char *w = DICT[d]; if ((int)strlen(w) != n) continue;
    int diff = -1, nd = 0;
    for (int i = 0; i < n; i++) if (w[i] != dWord[i]) { diff = i; nd++; }
    if (nd != 1) continue;
    const char *c = codeOf(w[diff]);
    // fehlendes Element (Fading): ab 3 Zeichen; zusätzliches/vertauschtes Element: ab 4 Zeichen
    if (c && oneElem(c, dCode[diff]) && (n >= 4 || strlen(c) > strlen(dCode[diff]))) { if (hit >= 0 && strcmp(DICT[hit], w)) return; hit = d; }   // mehrdeutig -> nichts
  }
  if (hit < 0) return;
  dictFixes++;
  Serial.print("["); Serial.print(DICT[hit]); Serial.print("]");
  dictReplace(DICT[hit], n);
}
// --------------------------------------------------------------------------
//  Wort-Neuauswertung (/reword on|off): Ton- und Pausendauern des Wortes werden
//  gespeichert. Am Wortende werden Punkt/Strich-Grenze und Zeichenpausen-Grenze
//  nur für dieses Wort neu bestimmt und das Wort neu dekodiert. Ist das Ergebnis
//  besser (weniger unbekannte Zeichen, Wörterbuch, Rufzeichen), wird ersetzt
//  (Anzeige türkis, seriell ROH[NEU]).
// --------------------------------------------------------------------------
bool rewordOn = true, softOn = true;
uint32_t rewordFixes = 0, softFixes = 0;
#define WMAX 32
float wOn[WMAX], wGap[WMAX]; int wN = 0; bool wBad = false;
void wPushOn(float d) { if (wN < WMAX) wOn[wN++] = d; else wBad = true; }
void wPushGap(float g) { if (wN > 0 && wN <= WMAX) wGap[wN - 1] = g; }
void wPop() { if (wN > 0) wN--; }
static bool isCall(const char *w) {
  int n = strlen(w), d = 0, l = 0; if (n < 4 || n > 8) return false;
  for (int i = 0; i < n; i++) { if (isdigit(w[i])) d++; else if (isalpha(w[i])) l++; else return false; }
  if (d < 1 || d > 2 || l < 2 || !isalpha(w[n - 1])) return false;
  for (int i = 1; i < n - 1; i++) if (isdigit(w[i]) && isalpha(w[i + 1])) return true;
  return false;
}
static bool inDict(const char *w) { for (int d = 0; d < DICT_N; d++) if (!strcmp(DICT[d], w)) return true; return false; }
static int wordScore(const char *w) {
  int s = 0; for (const char *p = w; *p; p++) if (*p == '*') s -= 10;
  if (inDict(w)) s += 8; else if (isCall(w)) s += 6;
  return s;
}
// Wort mit Schwellen tD (Punkt/Strich) und tG (Pause im Zeichen/Zeichenpause) dekodieren
static bool wDecode(float tD, float tG, char *out, int outMax) {
  char sym[8]; int sl = 0, o = 0;
  for (int i = 0; i < wN; i++) {
    if (sl >= 7) return false;
    sym[sl++] = wOn[i] > tD ? '-' : '.';
    if (i == wN - 1 || wGap[i] > tG) {
      sym[sl] = 0; int k = decodeMorse(sym); sl = 0;
      if (o >= outMax - 1) return false;
      if (k >= 0 && MORSE[k].txt[1]) return false;
      out[o++] = k >= 0 ? MORSE[k].txt[0] : '*';
    }
  }
  out[o] = 0; return true;
}
// beste Zweiteilung (log. Abstand) einer Werteliste; liefert Grenze oder 0
static float bestSplit(float *v, int n, float lo, float hi) {
  float t[WMAX]; for (int i = 0; i < n; i++) t[i] = v[i]; std::sort(t, t + n);
  float best = 0, bs = 1e9;
  for (int i = 0; i < n - 1; i++) {
    float c = (t[i] + t[i + 1]) / 2; if (c < lo || c > hi || t[i + 1] < 1.6f * t[i]) continue;
    float s = 0, ma = 0, mb = 0; for (int j = 0; j <= i; j++) ma += logf(t[j]); ma /= i + 1;
    for (int j = i + 1; j < n; j++) mb += logf(t[j]); mb /= n - i - 1;
    for (int j = 0; j < n; j++) { float d = logf(t[j]) - (j <= i ? ma : mb); s += d * d; }
    if (s < bs) { bs = s; best = c; }
  }
  return best;
}
void rewordEnd() {   // vor dictEnd aufrufen; true -> Wort ersetzt
  int n = dLen;
  if (!rewordOn || wBad || dBad || mCur >= 0 || n < 2 || wN < 3 || n > 14) { wN = 0; wBad = false; return; }
  char raw[16]; for (int i = 0; i < n; i++) raw[i] = dWord[i]; raw[n] = 0;
  int s0 = wordScore(raw);
  if (s0 >= 6) { wN = 0; return; }   // Wörterbuch/Rufzeichen ohne Unbekannte -> lassen
  float tD0 = bestSplit(wOn, wN, 0.5f * unitMs, 6.0f * unitMs); if (tD0 <= 0) tD0 = thrMs;
  float gs[WMAX]; for (int i = 0; i < wN - 1; i++) gs[i] = wGap[i];
  float cands[3] = { tD0, thrMs, 0 }; int nc = 2;
  char best[16] = ""; int bsc = s0; float u = unitMs;
  for (int c = 0; c < nc; c++) {
    float tD = cands[c];
    // Pausen-Grenzen: alle Mitten zwischen sortierten Pausen im plausiblen Bereich
    float t[WMAX]; int m = wN - 1; for (int i = 0; i < m; i++) t[i] = gs[i]; std::sort(t, t + m);
    for (int i = -1; i < m; i++) {
      float tG = (i < 0) ? 1e9f : (i == m - 1 ? t[i] + 1 : (t[i] + t[i + 1]) / 2);
      if (i >= 0 && (tG < 1.5f * u - biasMs || tG > 5.0f * u)) continue;
      char w[16]; if (!wDecode(tD, tG, w, 16)) continue;
      int sc = wordScore(w); if ((int)strlen(w) < 2) continue;
      if (sc > bsc) { bsc = sc; strcpy(best, w); }
    }
  }
  wN = 0;
  if (!best[0] || !strcmp(best, raw)) return;
  rewordFixes++; dLen = 0; dBad = false;
  Serial.print("["); Serial.print(best); Serial.print("]");
  dictReplace(best, n);
}
bool inWordAfterLong = false;   // v26: letztes Zeichen war ein echtes (>=3 Elemente) im selben Wort, mit Vorgaenger-Kontext
uint8_t realChain = 0;          // v26: Zahl echter Zeichen (>=3 Elemente) seit der letzten langen Stille/Rauschkette
void flushPend() {
  const uint32_t sv0 = labT0, sv1 = labT1;   // v44: Zeitstempel des aktuellen Zeichens nicht ueberschreiben
  bool noise = (pendChars >= 3 && pendET == pendChars) || (pendChars >= 4 && pendET >= 3);
  if (noise) { realChain = 0; inWordAfterLong = false; wBad = true; etDropped += pendChars; sqBoost = min(sqBoost + 3.0f, 6.0f); }
  else for (int i = 0; i < pendN; i++) {
    if (pendK[i] == -2) { dPrint(' '); appendChar(' ', C_TEXT); dictEnd(); }
    else { labT0 = pendT0[i]; labT1 = pendT1[i]; dPrint(MORSE[pendK[i]].txt); showSym(pendK[i], C_TEXT); dictAdd(pendK[i], MORSE[pendK[i]].code); }
  }
  pendN = pendET = pendChars = 0;
  labT0 = sv0; labT1 = sv1;
}
void outSpace() {
  inWordAfterLong = false;
  if (pendN > 0) { if (pendN < 16) { pendT0[pendN] = pendT1[pendN] = 0; pendK[pendN++] = -2; } }
  else { dPrint(' '); appendChar(' ', C_TEXT); rewordEnd(); dictEnd(); }
  wN = 0; wBad = false;
}
void outSym(int k) {
  int len = strlen(MORSE[k].code);
  if (len <= 2 && pendN == 0 && inWordAfterLong) {
    // v26: E/T/I/A/N/M direkt nach echtem Zeichen im selben Wort (z. B. D-E) sofort ausgeben,
    // nicht als moegliche Rausch-Kette zurueckhalten/verwerfen
    dPrint(MORSE[k].txt); showSym(k, C_TEXT); dictAdd(k, MORSE[k].code);
    return;
  }
  if (len <= 2) {
    if (pendN >= 16) flushPend();
    pendT0[pendN] = labT0; pendT1[pendN] = labT1; pendK[pendN++] = k; pendChars++; if (len == 1) pendET++;
    return;
  }
  if (pendN > 0) flushPend();
  dPrint(MORSE[k].txt); showSym(k, C_TEXT); dictAdd(k, MORSE[k].code);
  inWordAfterLong = realChain >= 2;   // nur mit Kontext: mind. 2 echte Zeichen vorher (z. B. CQ vor DE)
  if (realChain < 255) realChain++;
  sqBoost = max(0.0f, sqBoost - 0.5f);   // echtes Zeichen -> Zusatz-Schwelle abbauen
}

uint32_t elT[9];   // v42: Startzeit je Element (nur fuer die Zeichenzeile)
void emitSegment(int from, int to, float endGap) {
  int n = to - from;
  if (n > 0 && from >= 0 && to <= 9) { labT0 = elT[from]; labT1 = elT[to - 1] + (uint32_t)durBuf[to - 1]; }
  float thr = thrMs, mn = 1e9, mx = 0, sum = 0;
  for (int i = from; i < to; i++) { mn = min(mn, durBuf[i]); mx = max(mx, durBuf[i]); sum += durBuf[i]; }
  if (n > 1 && mx >= 2.2f * mn) {
    thr = (mn + mx) / 2;
  } else if (n > 1 && mx < 1.5f * mn) {
    float g = 0; int gn = 0;
    for (int i = from; i < to - 1 && i < 8; i++) { g += cgBuf[i]; gn++; }
    if (gn) thr = (sum / n - biasMs > 1.7f * (g / gn + biasMs)) ? 0 : 1e9;
  } else if (n == 1 && endGap > 0 && gapLen == 0 && histLen > 0) {
    float hmn = 1e9, hmx = 0;
    for (int i = 0; i < histLen; i++) { hmn = min(hmn, hist[i]); hmx = max(hmx, hist[i]); }
    if (hmx < 2.0f * hmn) thr = 0.5f * endGap;
  }
  for (int i = 0; i < n; i++) symBuf[i] = durBuf[from + i] > thr ? '-' : '.';
  symBuf[n] = 0;
  int k = decodeMorse(symBuf);
  // Weiche Entscheidung (/soft): unbekannte Folge -> gültiges Zeichen gleicher
  // Länge mit den geringsten "Umkippkosten" |ln(Dauer/Schwelle)| der Elemente
  // Weiche Entscheidung; bei Hand auch gültige Zeichen mit knappen Elementen umdeuten (K↔U)
  float softLim = softMaxC * (isHand() ? 2.2f : 1.0f);
  if (softOn && n >= 2 && n <= 6 && thr > 1 && thr < 1e8 && (k < 0 || isHand())) {
    float bc = 1e9; int bk = -1;
    for (int m = 0; m < MORSE_COUNT; m++) {
      const char *c = MORSE[m].code; if ((int)strlen(c) != n) continue;
      float cost = 0;
      for (int i = 0; i < n && cost < bc; i++) if (c[i] != symBuf[i]) cost += fabsf(logf(max(1.0f, durBuf[from + i]) / thr));
      if (cost < bc) { bc = cost; bk = m; }
    }
    bool better = (k < 0 && bk >= 0 && bc < softLim) || (k >= 0 && bk >= 0 && bk != k && bc < softLim * 0.7f);
    if (better) { k = bk; strcpy(symBuf, MORSE[k].code); softFixes++; }
  }
  if ((k < 0 || n > 6) && n >= 2) {
    // Sicherung: unbekannt oder > 6 Elemente -> an der längsten Pause teilen,
    // wenn sie deutlich länger ist als die kürzeste (verpasste Zeichenpause)
    int best = -1; float gmx = 0, gmn = 1e9;
    for (int i = from; i < to - 1 && i < 8; i++) { gmn = min(gmn, cgBuf[i]); if (cgBuf[i] > gmx) { gmx = cgBuf[i]; best = i; } }
    if (best >= 0 && gmx > 1.5f * gmn + biasMs && (k < 0 || gmx > 1.8f * gmn + biasMs)) {
      emitSegment(from, best + 1, 0); emitSegment(best + 1, to, endGap); return;
    }
  }
  if (k < 0) { dPrint('*'); appendChar('*', C_TEXT); dictAdd(-1, symBuf); }
  else outSym(k);
}


bool stochOn = true;   // v40 /stoch (NVS "stoch")
// v40 /stoch: Zeichenpausen per Wahrscheinlichkeit (Pausenlaenge + Buchstabenhaeufigkeit)
float stIn = 0, stCh = 0;   // adaptive Pausenmittel (ms) innerhalb / zwischen Zeichen
char stPrev = ' ';
static float letterCost(char c) {
  // -ln P, grob nach Haeufigkeit im Amateurfunk-CW
  const char *top = "ETAONSIRDLCQKUH5M9G7WP0BFY1Z2V3X468J"; const char *p = strchr(top, c);
  if (!c || !p) return 7.5f; return 2.2f + 0.12f * (p - top);
}
static float bigramBonus(char a, char b) {
  static const char *bg[] = {"CQ","QC","DE","EQ","NN","5N","TU","73","59","99","RS","ST","UR","BK","FB","GM","GE","HW","OP","ES","QT","TH","K ",0};
  for (int i = 0; bg[i]; i++) if (bg[i][0] == a && bg[i][1] == b) return 1.2f;
  return 0;
}
static float gapCost(float g, float m) { float d = logf(max(g, 1.0f) / max(m, 1.0f)) / 0.28f; return 0.5f * d * d; }
static char segChar(int a, int b) {
  if (b - a > 6) return 0;
  float mn = 1e9, mx = 0; for (int i = a; i < b; i++) { mn = min(mn, durBuf[i]); mx = max(mx, durBuf[i]); }
  float thr = (b - a > 1 && mx >= 2.2f * mn) ? (mn + mx) / 2 : thrMs;
  char s[8]; for (int i = a; i < b; i++) s[i - a] = durBuf[i] > thr ? '-' : '.'; s[b - a] = 0;
  int k = decodeMorse(s); if (k < 0 || MORSE[k].txt[1]) return k < 0 ? 0 : '#'; return MORSE[k].txt[0];
}
// liefert Trennstellen-Maske (Bit i = nach Element i trennen)
uint16_t stochSplit(float base) {
  int n = durLen; if (n < 2) return 0;
  if (stIn <= 0) stIn = base; if (stCh <= 0) stCh = 3.0f * base;
  float best[10]; int from[10]; char lastc[10];
  best[0] = 0; lastc[0] = stPrev;
  for (int j = 1; j <= n; j++) {
    best[j] = 1e9; from[j] = 0;
    for (int i = j - 1; i >= 0; i--) {
      // Pause cgBuf[i-1] vor Segment ist Trennung, Pausen i..j-2 im Segment
      bool ok = true; float c = 0;
      for (int q = i; q < j - 1; q++) { float u = (cgBuf[q] + biasMs) / base; if (u > 3.2f) { ok = false; break; } c += gapCost(cgBuf[q] + biasMs, stIn); }
      if (!ok) break;
      if (i > 0) { float u = (cgBuf[i - 1] + biasMs) / base; if (u < 1.4f) continue; c += gapCost(cgBuf[i - 1] + biasMs, stCh); }
      char ch = segChar(i, j); c += ((float)0.6) * letterCost(ch) - ((float)0.6) * bigramBonus(lastc[i], ch);
      if (best[i] + c < best[j]) { best[j] = best[i] + c; from[j] = i; lastc[j] = ch; }
    }
  }
  uint16_t m = 0; for (int j = n; j > 0; j = from[j]) if (from[j] > 0) m |= 1 << (from[j] - 1);
  stPrev = lastc[n];
  for (int q = 0; q < n - 1; q++) { float g = cgBuf[q] + biasMs; if (m >> q & 1) stCh = 0.8f * stCh + 0.2f * g; else stIn = 0.8f * stIn + 0.2f * g; }
  return m;
}

void finishCharacter(float endGap = 0) {
  if (durLen == 0) return;
  afterPause = false;
  if (afterPause && durLen >= 2 && false) {
    // Pausen im Zeichen = 1 Einheit: passt die kürzeste nicht zur Historie -> Tempowechsel
    afterPause = false;
    float gmin = 1e9; for (int i = 0; i < durLen - 1 && i < 8; i++) gmin = min(gmin, cgBuf[i]);
    float r = (gmin + biasMs) / unitMs;
    if (r > 1.6f || r < 0.6f) {
      histLen = histPos = 0; gapLen = gapPos = 0; biasMs = 0;
      for (int i = 0; i < durLen; i++) { hist[histPos] = durBuf[i]; histPos = (histPos + 1) % 16; histLen++; }
      for (int i = 0; i < durLen - 1 && i < 8; i++) { gapIn[gapPos] = cgBuf[i]; gapPos = (gapPos + 1) % 8; gapLen++; }
      gapEma = gmin;
    }
  }
  estimateSpeed();
  // Nachträglich teilen, wenn eine Pause im Zeichen eine Zeichenpause war
  float mnGap = 1e9;
  for (int i = 0; i < durLen - 1; i++) mnGap = min(mnGap, cgBuf[i]);
  float base = unitMs;
  if (durLen > 1) base = min(unitMs, max(mnGap, 0.7f * unitMs));
  int from = 0;
  if (stochOn) {
    uint16_t sm = stochSplit(base);
    for (int i = 0; i < durLen - 1; i++) {
      if (sm >> i & 1) { emitSegment(from, i + 1, 0); from = i + 1; }
    }
  } else {
    for (int i = 0; i < durLen - 1; i++) {
      if (cgBuf[i] > max(CHAR_GAP_UNITS * base - biasMs, 1.2f * base)) { emitSegment(from, i + 1, 0); from = i + 1; }
    }
  }
  emitSegment(from, durLen, endGap);
  durLen = symLen = 0;
  drawSymbols("");
  drawHeader();               // WPM aktualisieren
  wordSpacePending = true;    // nach einem Zeichen kann ein Wortende kommen
}

// Taste losgelassen / Ton zu Ende: Element speichern, Tempo neu schätzen
void handleElement(uint32_t dur) {
  if (durLen >= 9) finishCharacter();     // Puffer voll -> spätestens jetzt trennen
  // klarer Tempowechsel: 2 Elemente in Folge außerhalb 0,5 .. 4,5 Einheiten -> Historie neu
  float de = dur - biasMs;
  if (histLen >= 4 && (de < 0.5f * unitMs || de > 4.5f * unitMs)) hOutl++; else hOutl = 0;
  if (hOutl >= 2) {
    float keep = durBuf[durLen > 0 ? durLen - 1 : 0]; bool hk = durLen > 0;
    histLen = histPos = 0; gapLen = gapPos = 0; biasMs = 0;
    if (hk) { hist[0] = keep; histLen = 1; histPos = 1; }
    hOutl = 0;
  }
  if (durLen < 9) { elT[durLen] = pressStart; durBuf[durLen++] = dur; }
  hist[histPos] = dur; histPos = (histPos + 1) % 16; if (histLen < 16) histLen++;
  estimateSpeed();
  // vorläufige Anzeige
  symLen = durLen;
  for (int i = 0; i < durLen; i++) symBuf[i] = durBuf[i] > thrMs ? '-' : '.';
  symBuf[symLen] = 0;
  handFeed(dur, dur > thrMs); handUpdate();
  drawSymbols(symBuf);
}


// Übergang des kombinierten Signals (Taste ODER Ton) zum Zeitpunkt t
static uint32_t keyDownT = 0;
void keyTransition(bool down, uint32_t t, bool phys) {
  keyBtn.stable = down; keyBtn.since = t;
  if (elemOut) { if (down) Serial.printf("#K1,%u\n", t); else Serial.printf("#K0,%u,%u\n", t, t - keyDownT); }
  webEv(down ? 1 : 2, t, t - keyDownT, nullptr);
  if (down) keyDownT = t;
  if (down) {                            // gedrückt / Ton erkannt
    if (phys || AUDIO_SIDETONE) signalOn();
    // sehr kurze Lücke (Fading/Rauschen, < 0,4 Pausen-Einheiten) -> mit vorigem Element verbinden
    if (lastRelease && durLen > 0 && (float)(t - lastRelease) < min(0.4f * (unitMs - biasMs), 13.0f)) {
      durLen--; histPos = (histPos + 15) % 16; if (histLen > 0) histLen--; wPop();
      pressStart = prevPressStart; lastRelease = 0;
      return;
    }
    prevPressStart = pressStart = t;
    // zurückgestellter Einzelpunkt bekommt ein Folgeelement nach Zeichenpause -> jetzt ausgeben
    if (lastRelease && durLen > 0 && isoCand && (float)(t - lastRelease) > charGapMs()) finishCharacter(t - lastRelease);
    if (durLen == 0) charT = t;
    if (durLen == 0) isoCand = !phys && (!lastRelease || (float)(t - lastRelease) > min(ISO_GAP_U * unitMs, 2000.0f));   // v24: Stille > ISO_GAP_U Einheiten
    if (lastRelease) {
      float g = (int32_t)(t - lastRelease) > 0 ? t - lastRelease : 0;
      wPushGap(g);
      if (durLen > 0) {
        // Zeichen noch offen -> Pause innerhalb des Zeichens
        if (durLen <= 8) cgBuf[durLen - 1] = g;
        gapIn[gapPos] = g; gapPos = (gapPos + 1) % 8; if (gapLen < 8) gapLen++;
        gapEma = 0.5f * gapEma + 0.5f * g;
      } else if (wordSpacePending || cgPending) {
        // Pause zwischen zwei Zeichen: jetzt ist ihre Länge bekannt
        if (g <= max(15.0f * unitMs, 1500.0f)) {
          cgU[cgPos] = (g + biasMs) / unitMs; cgPos = (cgPos + 1) % 12; if (cgLen < 12) cgLen++;
        }
        if (wordSpacePending && g > wordGapUnits() * unitMs - biasMs) {
          float gu = (g + biasMs) / unitMs; handShortW = 0.85f * handShortW + 0.15f * (gu < 4.5f ? 1.0f : 0.0f);
          outSpace();
        }
        wordSpacePending = false; cgPending = false;
      }
    }
  } else {                               // losgelassen
    signalOff();
    wPushOn(t - pressStart);
    handleElement(t - pressStart);
    lastRelease = t;
  }
}

// Pausen bis zum Zeitpunkt t auswerten (Zeichenende, lange Pause)
void checkGaps(uint32_t t) {
  if (!keyBtn.stable && lastRelease && (int32_t)(t - lastRelease) > 0) {
    uint32_t gap = t - lastRelease;
    if (durLen > 0 && gap > charGapMs()) {
      if (isoCand && durLen == 1 && durBuf[0] < thrMs) {
        // isolierter Punkt nach langer Stille: erst nach einer Wortpause ohne Folgeelement verwerfen
        if (gap > ISO_DROP_U * unitMs) {   // v24: 7 -> ISO_DROP_U
          durLen = symLen = 0; drawSymbols(""); isoCand = false; sqDropped++; wPop();
          histPos = (histPos + 15) % 16; if (histLen > 0) histLen--;
        }
      } else finishCharacter(gap);
    }
    // Leerzeichen per Timeout, nicht erst beim nächsten Zeichen
    if (wordSpacePending && durLen == 0 && gap > wordGapUnits() * unitMs - biasMs) {
      float gu = (gap + biasMs) / unitMs; handShortW = 0.85f * handShortW + 0.15f * (gu < 4.5f ? 1.0f : 0.0f);
      outSpace(); wordSpacePending = false; cgPending = true;
    }
    // Zusatz-Schwelle langsam abbauen (1 dB / 10 s)
    static uint32_t sqDecT = 0;
    if ((int32_t)(t - sqDecT) > 10000) { sqDecT = t; sqBoost = max(0.0f, sqBoost - 1.0f); }
    // kurze Zeichen zurückgehalten: nach ~1,5 s Stille entscheiden
    if (pendN > 0 && durLen == 0 && gap > max(12.0f * unitMs, 1500.0f)) flushPend();
    if (durLen == 0 && gap > max(12.0f * unitMs, 1500.0f)) { realChain = 0; inWordAfterLong = false; }   // v26
    // lange Pause (Sprecherwechsel/Tempowechsel möglich): Verlauf vergessen
    // lange Pause: Tempo neu einlernen, Wortschwelle (cgU) behalten
    // sehr lange Stille (Sprecherwechsel): auch Wortpausen-Statistik verwerfen
    if (gap > WORDGAP_RESET_MS && cgLen > 0) { cgLen = cgPos = 0; }
    if (gap > max(15.0f * unitMs, 1500.0f) && !afterPause) {
      // Tempo auf Startwert (beide Richtungen am sichersten), Wortpausen-Statistik behalten
      float sv[12]; int sl = cgLen, sp = cgPos; memcpy(sv, cgU, sizeof(sv));
      resetSpeedHistory();
      memcpy(cgU, sv, sizeof(sv)); cgLen = sl; cgPos = sp;
      handShortW *= 0.5f;   // Hand: kurze-Wortpausen-EMA bei Sprecherpause dämpfen
      afterPause = true;
    }
  }
}

void decoderLoop(uint32_t now) {
  // 1) Ton-Flanken mit ihren echten Zeitpunkten abarbeiten
  while (aEdgeTail != aEdgeHead) {
    AudioEdge e = aEdge[aEdgeTail]; aEdgeTail = (aEdgeTail + 1) % AEDGE_N;
    audioState = e.on;
    if (!(keySource & SRC_AUDIO)) continue;
    bool comb = ((keySource & SRC_KEY) && physBtn.stable) || audioState;
    checkGaps(e.t);                      // Pause vor dieser Flanke zuerst auswerten
    if (comb != keyBtn.stable) keyTransition(comb, e.t, false);
  }
  // 2) Morsetaste (entprellt)
  bool phys = (digitalRead(PIN_KEY) == LOW);
  if (updateButtonRaw(physBtn, phys, now)) {
    bool comb = ((keySource & SRC_KEY) && physBtn.stable) || ((keySource & SRC_AUDIO) && audioState);
    checkGaps(now);
    if (comb != keyBtn.stable) keyTransition(comb, now, physBtn.stable);
  }
  // 3) Pausen bis jetzt (abzüglich Audio-Latenz, damit eine noch nicht
  //    erkannte Ton-Flanke nicht fälschlich als Zeichenende gilt)
  checkGaps(now - AUDIO_LAT_MS);
}

// ==== MULTI BEGIN ==========================================================
//  MEHRKANAL-MODUS (/multi on|off): 4 Goertzel-Kanäle (Standard 550/650/750/850 Hz,
//  /multif), je eigene Rauschsperre/Rauschboden (v20-Logik), eigenes Timing und
//  eigener Decoder-Zustand. Der Einkanal-Decoder bleibt unverändert: sein Zustand
//  (globale Variablen) wird je Kanal ein-/ausgetauscht.
// ===========================================================================
#define MCH 4
#define M_XTALK 2.0f            // Nachbarkanal > 2x stärker -> nur Übersprechen
bool  multiOn = false;
int   mFreq[MCH] = {550, 650, 750, 850};
struct MDet {
  float c, s1, s2, mag, nf, nfMean, peak, ton; int nfBlocked, onCnt; bool detOn, on;
  int32_t pIdx[AEDGE_N]; bool pOn[AEDGE_N]; int pN;
  AudioEdge e[AEDGE_N]; int eh, et;
  uint32_t edges, drops;
};
MDet  md[MCH];
int   mN = 160, mCnt = 0; float mFs = AUDIO_FS_HZ, mEnergy = 0;
uint32_t mBlocks = 0;

// Decoder-Zustand (alle globalen Variablen des Decoders)
#define DEC_VARS X(symBuf) X(symLen) X(lastRelease) X(pressStart) X(wordSpacePending) X(cgPending) \
  X(afterPause) X(isoCand) X(thrMs) X(durBuf) X(durLen) X(cgBuf) X(hist) X(histLen) X(histPos) X(gapIn) \
  X(gapLen) X(gapPos) X(gapEma) X(cgU) X(cgLen) X(cgPos) X(biasMs) X(prevPressStart) X(pendK) X(pendN) \
  X(pendET) X(pendChars) X(unitMs) X(sqBoost) X(hOutl) X(keyBtn)
struct DecState {
#define X(v) decltype(::v) v;
  DEC_VARS
#undef X
  void save() {
#define X(v) memcpy((void *)&this->v, (const void *)&::v, sizeof(::v));
    DEC_VARS
#undef X
  }
  void load() const {
#define X(v) memcpy((void *)&::v, (const void *)&this->v, sizeof(::v));
    DEC_VARS
#undef X
  }
};
DecState mst[MCH], decFresh;
bool decFreshOk = false;

void mSetup(float fs) {
  mFs = fs; mN = (int)(fs * AUDIO_BLOCK_MS / 1000.0f + 0.5f); if (mN > 256) mN = 256;
  for (int i = 0; i < MCH; i++) { md[i].c = 2.0f * cosf(2.0f * PI * mFreq[i] / fs); md[i].s1 = md[i].s2 = 0; }
  mCnt = 0; mEnergy = 0;
}
void mReset() {
  if (!decFreshOk) { DecState cur; cur.save(); resetSpeedHistory(); durLen = symLen = 0; lastRelease = 0;
    wordSpacePending = cgPending = isoCand = afterPause = false; pendN = pendET = pendChars = 0; sqBoost = 0; hOutl = 0;
    keyBtn.stable = false; decFresh.save(); cur.load(); decFreshOk = true; }
  for (int i = 0; i < MCH; i++) {
    MDet &d = md[i]; d.nf = 5; d.nfMean = 10; d.peak = 0; d.mag = d.ton = 0; d.nfBlocked = d.onCnt = 0;
    d.detOn = d.on = false; d.pN = 0; d.eh = d.et = 0; d.edges = d.drops = 0; mst[i] = decFresh;
  }
}

void mBlockDone() {
  float pw[MCH], var = mEnergy / mN, tot = 0;
  for (int i = 0; i < MCH; i++) {
    MDet &d = md[i];
    float p = d.s1 * d.s1 + d.s2 * d.s2 - d.c * d.s1 * d.s2; if (p < 0) p = 0;
    d.mag = 2.0f * sqrtf(p) / mN; pw[i] = 0.5f * d.mag * d.mag; tot += pw[i];
    d.s1 = d.s2 = 0;
  }
  mEnergy = 0; mCnt = 0; mBlocks++;
  float blkMs = mN * 1000.0f / mFs;
  for (int i = 0; i < MCH; i++) {
    MDet &d = md[i];
    float v = max(var - (tot - pw[i]), 0.25f * var);       // andere Kanäle zählen nicht als Störung
    d.ton = v > 1.0f ? pw[i] / v : 0;
    for (int j = 0; j < MCH; j++) if (j != i && md[j].mag > M_XTALK * d.mag) { d.ton = 0; break; }   // Übersprechen
    if (d.mag > d.peak) d.peak = d.mag; else d.peak += 0.002f * (d.nf - d.peak);
    if (!d.detOn) {                                        // Rauschboden wie v20
      bool tb = d.ton > NF_TONAL;
      if (d.mag < d.nf) d.nf += 0.10f * (d.mag - d.nf); else if (!tb) d.nf += 0.01f * (d.mag - d.nf);
      if (!tb || d.mag < d.nfMean) d.nfMean += 0.02f * (d.mag - d.nfMean);
      d.nfBlocked = (tb && d.mag > 2.0f * d.nf) ? d.nfBlocked + 1 : 0;
      if (d.nfBlocked > NF_BLOCK_N) { d.nf *= 0.995f; d.nfMean *= 0.995f; }
    } else d.nfBlocked = 0;
    float thOn = d.nf * AUDIO_SNR_ON, mid = d.nf + 0.5f * (d.peak - d.nf);
    if (mid > thOn) thOn = mid;
    if (thOn < AUDIO_MIN_MAG) thOn = AUDIO_MIN_MAG;
    float thOff = max(thOn * AUDIO_HYST, d.nf * 2.0f);
    float sqT = sqDb > 0 ? d.nfMean * powf(10.0f, (sqDb + mst[i].sqBoost) / 20.0f) : 0;
    if (!d.detOn && d.mag > thOn && d.mag > sqT && d.ton > AUDIO_TONAL_ON) { d.detOn = true; d.edges++; }
    else if (d.detOn && (d.mag < thOff || d.ton < AUDIO_TONAL_HOLD)) d.detOn = false;
    if (d.detOn && !d.on) {
      d.onCnt++;
      float mo = constrain(0.5f * mst[i].unitMs, MINON_MIN_MS, MINON_MAX_MS);
      int need = sqDb > 0 ? max(1, (int)ceilf((mo - 4.0f) / blkMs)) : 1;
      if (d.onCnt >= need) {
        d.on = true;
        if (d.pN < AEDGE_N) { d.pIdx[d.pN] = (int32_t)aProcIdx - (d.onCnt - 1) * mN; d.pOn[d.pN] = true; d.pN++; }
        d.onCnt = 0;
      }
    } else if (!d.detOn) {
      if (d.onCnt > 0) d.drops++;
      d.onCnt = 0;
      if (d.on) { d.on = false; if (d.pN < AEDGE_N) { d.pIdx[d.pN] = aProcIdx; d.pOn[d.pN] = false; d.pN++; } }
    }
  }
}
// pro Abtastwert (nach Hochpass); aProcIdx ist bereits hochgezählt
inline void mSample(float x) {
  mEnergy += x * x;
  for (int i = 0; i < MCH; i++) { MDet &d = md[i]; float s0 = x + d.c * d.s1 - d.s2; d.s2 = d.s1; d.s1 = s0; }
  if (++mCnt >= mN) mBlockDone();
}
// Flanken (Abtastindex) in Zeitstempel umrechnen: idxNow entspricht tNow
void mFlushPend(uint32_t tNow, int32_t idxNow, float fs) {
  for (int i = 0; i < MCH; i++) {
    MDet &d = md[i];
    for (int k = 0; k < d.pN; k++) {
      uint32_t back = (uint32_t)((idxNow - d.pIdx[k]) * 1000.0f / fs);
      int nh = (d.eh + 1) % AEDGE_N; if (nh == d.et) break;
      d.e[d.eh].t = tNow - back; d.e[d.eh].on = d.pOn[k]; d.eh = nh;
    }
    d.pN = 0;
  }
}
// Decoder aller Kanäle: Zustand je Kanal einladen, Flanken abarbeiten, sichern
void multiDecoderLoop(uint32_t now) {
  DecState g; g.save();
  for (int i = 0; i < MCH; i++) {
    MDet &d = md[i];
    mst[i].load(); mCur = i;
    while (d.et != d.eh) {
      AudioEdge e = d.e[d.et]; d.et = (d.et + 1) % AEDGE_N;
      checkGaps(e.t);
      if (e.on != keyBtn.stable) keyTransition(e.on, e.t, false);
    }
    checkGaps(now - AUDIO_LAT_MS);
    int w0 = (int)(1200.0f / mst[i].unitMs + 0.5f);
    mst[i].save();
    if ((int)(1200.0f / unitMs + 0.5f) != w0) drawHeader();   // WPM-Anzeige des Kanals (auch nach Tempo-Reset)
  }
  mCur = -1; g.load();
}
// ==== MULTI END ============================================================

// ---- Mehrkanal-Anzeige (nur Core 1 / loop) --------------------------------
#define M_COLS 36
#define M_ROWS 3
#define M_LH   9
char    mTxt[MCH][M_COLS * M_ROWS]; int mTL[MCH];
int16_t mRegH = 38;
const uint16_t mCol[MCH] = {0x07FF, 0xFFE0, 0x07E0, 0xF81F};
int16_t mY0(int i) { return textY + 1 + i * mRegH; }
void mHeadNow(int i) {
  int16_t y = mY0(i);
  tft.fillRect(0, y, W, M_LH, C_BG);
  tft.setTextSize(1); tft.setTextColor(mCol[i], C_BG); tft.setCursor(MARGIN_X, y + 1);
  char b[40]; snprintf(b, sizeof(b), "K%d %4d Hz  %2d WPM", i + 1, mFreq[i], (int)(1200.0f / mst[i].unitMs + 0.5f));
  tft.print(b);
}
void mDrawChar(int i, int p) {
  tft.setTextSize(1); tft.setTextColor(C_TEXT, C_BG);
  tft.setCursor(MARGIN_X + (p % M_COLS) * 6, mY0(i) + M_LH + 1 + (p / M_COLS) * M_LH);
  tft.print(mTxt[i][p]);
}
void mRedraw(int i) {
  tft.fillRect(0, mY0(i) + M_LH, W, M_ROWS * M_LH + 1, C_BG);
  for (int p = 0; p < mTL[i]; p++) mDrawChar(i, p);
}
void mAppendNow(int i, char c) {
  if (i < 0 || i >= MCH) return;
  if (c == ' ' && (mTL[i] == 0 || mTxt[i][mTL[i] - 1] == ' ')) return;
  if (mTL[i] >= M_COLS * M_ROWS) {
    memmove(mTxt[i], mTxt[i] + M_COLS, mTL[i] - M_COLS); mTL[i] -= M_COLS;
    mTxt[i][mTL[i]++] = c; mRedraw(i);
  } else { mTxt[i][mTL[i]] = c; mDrawChar(i, mTL[i]++); }
}
void mLayout() {
  int16_t bottom = barOn ? barY0 - 2 : H - MARGIN_Y;
  mRegH = (bottom - textY - 1) / MCH;
  tft.fillScreen(C_BG);
  drawHeaderNow();
  tft.drawFastHLine(0, textY, W, C_INFO);
  for (int i = 0; i < MCH; i++) {
    if (i) tft.drawFastHLine(0, mY0(i) - 1, W, C_GRID);
    mHeadNow(i); mRedraw(i);
  }
  if (barOn) barDrawStatic();
}
void mClear() { for (int i = 0; i < MCH; i++) mTL[i] = 0; mLayout(); }

// Rechenzeit Core 0 (Audio-Task)
uint32_t c0BusyUs = 0, c0WinT0 = 0, c0MaxUs = 0, c0MaxWin = 0; float c0Load = 0;
void c0Account(uint32_t dt, uint32_t nowMs) {
  c0BusyUs += dt; if (dt > c0MaxWin) c0MaxWin = dt;
  if (nowMs - c0WinT0 >= 1000) {
    c0Load = c0BusyUs / ((nowMs - c0WinT0) * 10.0f);   // %
    c0MaxUs = c0MaxWin; c0MaxWin = 0; c0BusyUs = 0; c0WinT0 = nowMs;
  }
}
void printMultiStatus() {
  Serial.printf("#MC multi=%d f=%d,%d,%d,%d c0Load=%.1f%% c0Max=%uus blk=%.1fms\n", multiOn ? 1 : 0,
                mFreq[0], mFreq[1], mFreq[2], mFreq[3], c0Load, (unsigned)c0MaxUs, mN * 1000.0f / mFs);
  if (!multiOn) return;
  for (int i = 0; i < MCH; i++) {
    MDet &d = md[i];
    Serial.printf("#MC%d f=%d mag=%.1f nf=%.1f nfm=%.1f ton=%.2f on=%d edges=%u drops=%u wpm=%.1f sqBoost=%.1f\n", i + 1, mFreq[i],
                  d.mag, d.nf, d.nfMean, d.ton, d.on ? 1 : 0, (unsigned)d.edges, (unsigned)d.drops, 1200.0f / mst[i].unitMs, mst[i].sqBoost);
  }
}

// ==========================================================================
//  GEBER
// ==========================================================================
// Wartet ms Millisekunden; bricht ab (return false), wenn eine Taste gedrückt wird
bool waitAbortable(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    if (digitalRead(PIN_KEY) == LOW || digitalRead(PIN_FUNC) == LOW) return false;
    delay(1);
  }
  return true;
}

// Ein Element (Punkt oder Strich) ausgeben
bool sendElement(uint32_t ms) {
  signalOn(); ledSet(true);
  bool ok = waitAbortable(ms);
  signalOff(); ledSet(false);
  return ok;
}

// Text als Morse ausgeben (Summer + LED + Anzeige)
void sendText(const String &txt) {
  uint32_t unit = 1200 / txWpm;
  Serial.print(F("[TX "));
  Serial.print(txWpm);
  Serial.print(F(" WPM] "));
  appendChar(' ', C_SENT);

  for (unsigned i = 0; i < txt.length();) {
    int k = nextSymbol(txt, i);
    if (k == -3) break;
    if (k == -1) continue;                   // unbekanntes Zeichen überspringen
    if (k == -2) {                           // Wortpause: 7 Einheiten (3 schon vorbei + 4)
      Serial.print(F("/ "));
      appendChar(' ', C_SENT);
      if (!waitAbortable(4 * unit)) goto abort;
      continue;
    }
    const char *code = MORSE[k].code;
    Serial.print(code); Serial.print(' ');
    drawSymbols(code);
    for (const char *p = code; *p; p++) {
      if (!sendElement(*p == '-' ? 3 * unit : unit)) goto abort;
      if (*(p + 1) && !waitAbortable(unit)) goto abort;   // Pause im Zeichen
    }
    showSym(k, C_SENT);
    if (!waitAbortable(3 * unit)) goto abort;             // Pause nach Zeichen
  }
  drawSymbols("");
  Serial.println(F("[TX Ende]"));
  return;

abort:
  signalOff(); ledSet(false);
  drawSymbols("");
  Serial.println(F("\n[TX abgebrochen]"));
  // warten, bis Tasten losgelassen sind, damit der Decoder nichts falsch liest
  while (digitalRead(PIN_KEY) == LOW || digitalRead(PIN_FUNC) == LOW) delay(5);
  delay(50);
}

void printHelp() {
  Serial.println(F("\n=== Morse-Decoder/-Geber ESP32 ==="));
  Serial.println(F("Text eingeben + Enter  -> wird gemorst"));
  Serial.println(F("/wpm N   Geber-Tempo (5..40)"));
  Serial.println(F("/ton N   Tonhöhe Hz (300..1500)"));
  Serial.println(F("/freq N  Audio-Zielfrequenz Hz (300..1500), Startwert der AFC"));
  Serial.println(F("/sq N    Rauschsperre N dB über Rauschen (Standard 4, 0 = aus; ohne AFC-Lock +3 dB)"));
  Serial.println(F("/mon off|tone|filt  Mithoeren an GPIO 25 (DAC): Ton getastet / gefiltertes Audio (Standard filt)"));
  Serial.println(F("/monbw N   Mithoer-Bandpass 100..1000 Hz (Standard 300)"));
  Serial.println(F("/lvlwarn on|off  Pegelwarnung PEGEL ZU NIEDRIG/HOCH im Kopf (Standard an)"));
  Serial.println(F("/monvol N  Mithoer-Lautstaerke 0..100"));
  Serial.println(F("/narrow on|off  Schmal-Modus: fest auf /freq (700 Hz), ~120 Hz Bandbreite, Nachbarn ±150 Hz unterdrückt (Standard aus)"));
  Serial.println(F("/afc on|off|re  Frequenznachführung ±100 Hz um /freq (re = neu einrasten)"));
  Serial.println(F("/audio   Audio-Pegel-Debug an/aus"));
  Serial.println(F("/src X   Quelle: key | audio | both"));
  Serial.println(F("/stat    Audio-Status einmal ausgeben  (/stat reset: Jitter-/Ueberlaufzaehler auf 0)"));
  Serial.println(F("/elem on|off  Element-Ausgabe mit ESP-Zeit: #K1,t / #K0,t,dauer / #Ct,zeichen (Standard aus, fuer Mitschnitt)"));
  Serial.println(F("/ch on|off   CH beim Geben als ---- zusammenfassen (Standard aus: C H)"));
  Serial.println(F("/bar on|off  Bargraph-Abstimmanzeige (Peak-Frequenz auch in /audio: pk=, dpk=)"));
  Serial.println(F("/multi on|off  Mehrkanal-Modus: 4 Toene gleichzeitig dekodieren (Standard aus)"));
  Serial.println(F("/multif A B C D  Mehrkanal-Frequenzen Hz (Standard 550 650 750 850)"));
  Serial.println(F("/dict on|off   Woerterbuch-Ergaenzung (Standard an, korrigierte Woerter tuerkis, seriell ROH[KORR])"));
  Serial.println(F("/reword on|off Wort-Neuauswertung am Wortende (Standard an, ersetzte Woerter tuerkis, seriell ROH[NEU])"));
  Serial.println(F("/soft on|off   Weiche Entscheidung am Zeichenende (Standard an)"));
  Serial.println(F("/stoch on|off  Zeichenpausen per Wahrscheinlichkeit: unklare Pausen 1,4-2,2 Punkte nach Pausenstatistik + Buchstabenhaeufigkeit trennen (Standard an)"));
  Serial.println(F("/bridge x      Fading-Ueberbrueckung x Punktlaengen 0..0.4 (Standard 0.2, 0 = aus)"));
  Serial.println(F("/afcsw ms      AFC wechselt erst nach ms Stille auf neuen Peak (Standard 1500)"));
  Serial.println(F("/stripauto on|off  Laufband-Tempo folgt der WPM, Punkt ~5 Pixel, STRIP_DOT_PX in display_config.h (Standard an; aus = fest 12 ms/Pixel)"));
  Serial.println(F("/clear   Text löschen"));
  Serial.println(F("/hand auto|on|off  Gebeweise Hand/Paddle (Std. auto; bei Hand weichere Timing-Toleranzen)"));
  Serial.println(F("/qsy on|off|<ms>  nach Durchgangspause sofort auf Gegenstation einrasten (Std. an, 1000 ms)"));
  Serial.println(F("/hyst <0.3..0.9>  Aus-Schwelle relativ zur Ein-Schwelle (Std. 0.6; 0.5 = weniger zerrissene Striche)"));
  Serial.println(F("/ratio on|off|<0.3..0.95>  Zusatzkriterium Ton/Breitband-Verhaeltnis (Std. aus, 0.7)"));
  Serial.println(F("/thrmid <0.1..0.9>  Lage der Schaltschwelle Rauschboden..Peak (Std. 0.5)"));
  Serial.println(F("/thrdecay <n>  Schwelle faellt mit n Punktlaengen Zeitkonstante (Std. 3, 0 = alt)"));
  Serial.println(F("/wbrej on|off|<dB>  Breitband-Stoerer (Klicks) unterdruecken, Ton muss <dB> ueber Nachbarn (Std. an, 6)"));
  Serial.println(F("/scope on|off   Streifen Pegel+Elemente (= /ticker)"));
  Serial.println(F("/wifi on|off|status  WLAN + Webseite http://morsedecoder.local (Standard an)"));
  Serial.println(F("/wifi ssid NAME | /wifi pass PASSWORT  WLAN-Zugang speichern (NVS, ueberschreibt wifi_config.h)"));
  Serial.println(F("/help    diese Hilfe"));
  Serial.println(F("Morsetaste GPIO27, Funktionstaste GPIO33 (kurz=löschen, lang=Echo), Audio GPIO34\n"));
}

// Serielle Eingabe zeilenweise lesen und auswerten
void execCmd(String line);   // v45
String inLine;

void serialLoop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == 0 || (uint8_t)c == 0xFF) continue;   // Störbytes ignorieren
    if (c != '\n') { if (inLine.length() < 200) inLine += c; continue; }

    String line = inLine; inLine = "";
    line.trim();
    if (line.length() == 0) continue;
    // Schutz gegen Echo/Störbytes auf der Leitung: gleiche Befehlszeile innerhalb
    // 1 s nur einmal ausführen
    { static String lastCmd; static uint32_t lastCmdT = 0; uint32_t n = millis();
      if (line[0] == '/' && line == lastCmd && n - lastCmdT < 1000) continue;
      lastCmd = line; lastCmdT = n; }
    execCmd(line);
  }
}

// v45: Befehl ausfuehren (seriell und Webseite)
void wifiCmd(String a); void printJitter();
void execCmd(String line) {
  {
    if (line.startsWith("/wifi")) {
      wifiCmd(line.substring(5));
    } else if (line.startsWith("/wpm")) {
      int v = line.substring(4).toInt();
      if (v >= MIN_WPM && v <= MAX_WPM) { txWpm = v; drawHeader(); prefs.putInt("wpm", v); }
      Serial.printf("Geber-Tempo: %d WPM\n", txWpm);
    } else if (line.startsWith("/ton")) {
      int v = line.substring(4).toInt();
      if (v >= 300 && v <= 1500) { toneHz = v; monSetFreqs(decHz); prefs.putInt("ton", v); }
      Serial.printf("Tonhöhe: %d Hz\n", toneHz);
    } else if (line.startsWith("/freq")) {
      int v = line.substring(5).toInt();
      if (v >= 300 && v <= 1500) { audioFreq = v; decHz = afcHz = afcCenter = v; afcLocked = false; afcCnt = 0; audioSetup(); prefs.putInt("freq", v); }
      Serial.printf("Audio-Zielfrequenz: %d Hz\n", audioFreq);
    } else if (line.startsWith("/monbw")) {
      String a = line.substring(6); a.trim();
      if (a.length()) { monBw = constrain(a.toInt(), 100, 1000); prefs.putInt("monbw", monBw); bqHz = 0; }
      Serial.printf("Mithoer-Bandbreite %d Hz\n", monBw);
    } else if (line.startsWith("/monvol")) {
      String a = line.substring(7); a.trim();
      if (a.length()) { monVol = constrain(a.toInt(), 0, 100); prefs.putInt("monvol", monVol); }
      Serial.printf("Mithoer-Lautstaerke %d\n", (int)monVol);
    } else if (line.startsWith("/mon")) {
      String a = line.substring(4); a.trim();
      if (a == "off") monMode = MON_OFF; else if (a == "tone") monMode = MON_TONE; else if (a == "filt") monMode = MON_FILT;
      prefs.putUChar("mon", monMode);
      Serial.printf("Mithoeren: %s (GPIO 25, Lautstaerke %d)\n", monMode == MON_OFF ? "off" : monMode == MON_TONE ? "tone" : "filt", (int)monVol);
    } else if (line.startsWith("/multif")) {
      String a = line.substring(7); a.trim(); int f[MCH], n = 0;
      while (a.length() && n < MCH) { int sp = a.indexOf(' '); String t = sp < 0 ? a : a.substring(0, sp); f[n++] = t.toInt(); a = sp < 0 ? "" : a.substring(sp + 1); a.trim(); }
      bool ok = n == MCH; for (int i = 0; i < n; i++) ok = ok && f[i] >= 300 && f[i] <= 1500;
      if (ok) { for (int i = 0; i < MCH; i++) { mFreq[i] = f[i]; char k[4] = {'m', 'f', (char)('0' + i), 0}; prefs.putInt(k, f[i]); } mSetup(fsMeasured); mReset(); if (multiOn) layoutScreen(); }
      else if (n) Serial.println(F("/multif: 4 Frequenzen 300..1500 Hz angeben"));
      Serial.printf("Mehrkanal-Frequenzen %d %d %d %d Hz\n", mFreq[0], mFreq[1], mFreq[2], mFreq[3]);
    } else if (line.startsWith("/multi")) {
      if (line.endsWith("on") || line.endsWith("off")) {
        bool on = line.endsWith("on");
        if (on != multiOn) {
          if (on) { mSetup(fsMeasured); mReset(); finishCharacter(); durLen = symLen = 0; multiOn = true; }
          else { multiOn = false; audioSetup(); audioOn = detOn = false; onCnt = 0; }
          prefs.putBool("multi", multiOn); layoutScreen();
        }
      }
      Serial.printf("Mehrkanal-Modus %s (%d/%d/%d/%d Hz)\n", multiOn ? "an" : "aus", mFreq[0], mFreq[1], mFreq[2], mFreq[3]);
    } else if (line.startsWith("/narrow")) {
      if (line.endsWith("on")) narrowMode = true; else if (line.endsWith("off")) narrowMode = false;
      prefs.putBool("narrow", narrowMode);
      if (narrowMode) { decHz = afcHz = afcCenter; audioFreq = (int)afcCenter; afcLocked = false; }
      audioSetup(); drawHeader();
      Serial.printf("Schmal-Modus %s, fest %d Hz, Block %d ms\n", narrowMode ? "an" : "aus", audioFreq, narrowMode ? NARROW_BLOCK_MS : AUDIO_BLOCK_MS);
    } else if (line == "/audio") {
      audioDebug = !audioDebug;
      Serial.printf("Audio-Debug %s\n", audioDebug ? "an" : "aus");
    } else if (line.startsWith("/ch")) {
      txCombineCh = line.endsWith("on"); prefs.putBool("ch", txCombineCh);
      Serial.printf("CH-Zusammenfassung %s\n", txCombineCh ? "an" : "aus");
    } else if (line.startsWith("/bar")) {
      String m = line.substring(4); m.trim();
      if (m == "on" || m == "off") {
        bool on = (m == "on");
        if (on != barOn) { barOn = on; if (on) barSetup(fsMeasured); layoutScreen(); prefs.putBool("bar", barOn); }
      }
      Serial.printf("Bargraph %s\n", barOn ? "an" : "aus");
#if AUDIO_SELFTEST
    } else if (line.startsWith("/stone")) {
      stoneHz = line.substring(6).toInt();
      if (stoneHz < 0 || stoneHz > 5000) stoneHz = 0;
      Serial.printf("Selbsttest-Ton: %d Hz\n", stoneHz);
    } else if (line.startsWith("/smorse")) {
      int v = line.substring(7).toInt(); if (v >= 300 && v <= 1500) stMorseHz = v;
      Serial.printf("Selbsttest-Morse: %d Hz\n", stMorseHz);
#endif
    } else if (line.startsWith("/sq")) {
      String a = line.substring(3); a.trim();
      if (a.length()) { sqDb = constrain(a.toFloat(), 0.0f, 30.0f); prefs.putFloat("sq", sqDb); }
      Serial.printf("Rauschsperre %.1f dB%s, verworfen bisher %u\n", sqDb, sqDb > 0 ? "" : " (aus)", (unsigned)sqDropped);
    } else if (line.startsWith("/lvlwarn")) {
      if (line.endsWith("on") || line.endsWith("off")) { lvlWarnOn = line.endsWith("on"); prefs.putBool("lvlwarn", lvlWarnOn); }
      Serial.printf("\n[Pegelwarnung %s, aktuell: %s]\n", lvlWarnOn ? "an" : "aus", lvlWarn == 2 ? "ZU HOCH" : (lvlWarn == 1 ? "ZU NIEDRIG" : "ok"));
    } else if (line.startsWith("/reword")) {
      if (line.endsWith("on") || line.endsWith("off")) { rewordOn = line.endsWith("on"); prefs.putBool("reword", rewordOn); }
      Serial.printf("\n[Wort-Neuauswertung %s, %u Ersetzungen]\n", rewordOn ? "an" : "aus", (unsigned)rewordFixes);
    } else if (line.startsWith("/stoch")) {
      if (line.endsWith("on") || line.endsWith("off")) { stochOn = line.endsWith("on"); prefs.putBool("stoch", stochOn); }
      Serial.printf("\n[Zeichenpausen-Wahrscheinlichkeit (/stoch) %s]\n", stochOn ? "an" : "aus");
    } else if (line.startsWith("/soft")) {
      if (line.endsWith("on") || line.endsWith("off")) { softOn = line.endsWith("on"); prefs.putBool("soft", softOn); }
      Serial.printf("\n[Weiche Entscheidung %s, %u Zeichen]\n", softOn ? "an" : "aus", (unsigned)softFixes);
    } else if (line.startsWith("/stripauto")) {
      if (line.endsWith("on") || line.endsWith("off")) { stripAuto = line.endsWith("on"); prefs.putBool("stripauto", stripAuto); if (stripAuto) stMsF = stMs; tkDirty = true; }
      Serial.printf("\n[Laufband-Tempo nach WPM %s, %d ms/Pixel, Ziel %d Pixel/Punkt]\n", stripAuto ? "an" : "aus", stMs, STRIP_DOT_PX);
    } else if (line.startsWith("/scope") || line.startsWith("/ticker")) {   // beide: Pegel+Elemente-Streifen
      if (line.endsWith("on") || line.endsWith("off")) {
        tickerOn = line.endsWith("on"); prefs.putBool("strip", tickerOn); tkDirty = true; tkPrev[0] = 0;
        symH = tickerOn ? 56 : ((TXT_SIZE >= 2) ? 20 : 12); if (!multiOn) layoutScreen();
      }
      Serial.printf("\n[Pegel/Element-Streifen %s]\n", tickerOn ? "an" : "aus");
    } else if (line.startsWith("/hand") || line.startsWith("/keyer")) {
      String a = line.substring(line.startsWith("/hand") ? 5 : 6); a.trim();
      if (a == "auto" || a == "on" || a == "off") {
        handMode = (a == "on") ? 1 : (a == "off") ? 2 : 0; prefs.putUChar("hand", handMode);
        handIs = (handMode == 1) ? 1 : 0; drawHeader();
      }
      const char *m = handMode == 1 ? "Hand" : (handMode == 2 ? "Paddle" : "auto");
      const char *d = isHand() ? "Hand" : "Paddle";
      Serial.printf("\n[Gebeweise %s, erkannt %s, Score %.0f (Punkt %.2f, Strich %.2f, kurzeWP %.2f, Elem %d)]\n", m, handIs == 0 && handMode == 0 ? "?" : d, handScore, handCd, handCD, handShortW, handNmin());
    } else if (line.startsWith("/qsy")) {
      String a = line.substring(4); a.trim();
      if (a == "on" || a == "off") { qsyOn = a == "on"; prefs.putBool("qsy", qsyOn); qsyArmed = false; }
      else if (a.length()) { qsyMinMs = constrain(a.toInt(), 300, 10000); prefs.putUInt("qsyms", qsyMinMs); }
      Serial.printf("\n[Durchgangswechsel-AFC %s, Pause >= %u ms bzw. 14 Punktlaengen, Wechsel bisher %u]\n", qsyOn ? "an" : "aus", (unsigned)qsyMinMs, (unsigned)qsyJumps);
    } else if (line.startsWith("/hyst")) {
      String a = line.substring(5); a.trim();
      if (a.length()) { hystF = constrain(a.toFloat(), 0.3f, 0.9f); prefs.putFloat("hyst", hystF); }
      Serial.printf("\n[Aus-Schwelle %.2f x Ein-Schwelle]\n", hystF);
    } else if (line.startsWith("/ratio")) {
      String a = line.substring(6); a.trim();
      if (a == "on" || a == "off") { ratioOn = a == "on"; prefs.putBool("ratio", ratioOn); }
      else if (a.length()) { ratioHi = constrain(a.toFloat(), 0.3f, 0.95f); prefs.putFloat("ratiohi", ratioHi); }
      Serial.printf("\n[Ton/Breitband-Kriterium %s, Schwelle %.2f]\n", ratioOn ? "an" : "aus", ratioHi);
    } else if (line.startsWith("/thrmid")) {
      String a = line.substring(7); a.trim();
      if (a.length()) { thrMid = constrain(a.toFloat(), 0.1f, 0.9f); prefs.putFloat("thrmid", thrMid); }
      Serial.printf("\n[Schwellenlage %.2f zwischen Rauschboden und Peak]\n", thrMid);
    } else if (line.startsWith("/thrdecay")) {
      String a = line.substring(9); a.trim();
      if (a.length()) { thrDecay = constrain(a.toFloat(), 0.0f, 50.0f); prefs.putFloat("thrdec", thrDecay); }
      Serial.printf("\n[Schwellen-Abfall %.1f Punktlaengen%s]\n", thrDecay, thrDecay > 0 ? "" : " (aus: alt ~4 s)");
    } else if (line.startsWith("/wbrej")) {
      String a = line.substring(6); a.trim();
      if (a == "on" || a == "off") { wbOn = a == "on"; prefs.putBool("wbrej", wbOn); wbCoefHz = 0; }
      else if (a.length()) { wbDb = constrain(a.toFloat(), 0.0f, 20.0f); wbK = powf(10.0f, wbDb / 20.0f); prefs.putFloat("wbdb", wbDb); }
      Serial.printf("\n[Breitband-Unterdrueckung %s, %.1f dB, unterdrueckt %u, gehalten %u, Nachbarn %.1f]\n", wbOn ? "an" : "aus", wbDb, (unsigned)wbRej, (unsigned)wbHeld, wbMed);
    } else if (line.startsWith("/dict")) {
      if (line.endsWith("on") || line.endsWith("off")) { dictOn = line.endsWith("on"); prefs.putBool("dict", dictOn); }
      Serial.printf("\n[Woerterbuch %s, %u Korrekturen]\n", dictOn ? "an" : "aus", (unsigned)dictFixes);
    } else if (line.startsWith("/bridge")) {
      String a = line.substring(7); a.trim();
      if (a.length()) { bridgeU = constrain(a.toFloat(), 0.0f, 0.4f); prefs.putFloat("bridge", bridgeU); }
      Serial.printf("\n[Fading-Ueberbrueckung %.2f Punktlaengen]\n", bridgeU);
    } else if (line.startsWith("/afcsw")) {
      String a = line.substring(6); a.trim();
      if (a.length()) { afcSwMs = constrain(a.toInt(), 300, 10000); prefs.putUInt("afcsw", afcSwMs); }
      Serial.printf("\n[AFC-Wechsel erst nach %u ms Stille]\n", (unsigned)afcSwMs);
    } else if (line.startsWith("/afc")) {
      if (line.endsWith("re")) { afcOn = true; afcLocked = false; decHz = afcHz = afcCenter; audioFreq = (int)afcCenter; audioSetup(); afcCnt = 0; lastToneMs = 0; Serial.println("AFC neu einrasten"); return; }
      if (line.endsWith("on") || line.endsWith("off")) { afcOn = !line.endsWith("off"); prefs.putBool("afc", afcOn); }
      if (!afcOn) afcLocked = false;
      Serial.printf("AFC %s, %.1f Hz\n", afcOn ? "an" : "aus", decHz);
    } else if (line == "/mag") {
      magStream = !magStream;
      Serial.printf("Mag-Stream %s (fs=%.0f N=%d)\n", magStream ? "an" : "aus", fsMeasured, gN);
    } else if (line == "/dump") {
      dumpPos = 0;
    } else if (line.startsWith("/elem")) {
      String a = line.substring(5); a.trim();
      elemOut = a == "on" ? true : a == "off" ? false : !elemOut;
      Serial.printf("\nElem-Ausgabe %s\n", elemOut ? "an" : "aus");
    } else if (line == "/tft") {   // v50: Display neu initialisieren
      tftReq = true; Serial.println(F("[Display wird neu initialisiert]"));
    } else if (line == "/stat reset") {   // v46
      jItMaxUs = 0; jGap20 = 0; jBackMax = 0; jIt = 0; jOvf = 0; Serial.println(F("\n[Jitter-/Ueberlaufzaehler zurueckgesetzt]"));
    } else if (line == "/stat") {
      printAudioStatus("#S");
      printMultiStatus();
      printJitter();
    } else if (line.startsWith("/src")) {
      String m = line.substring(4); m.trim();
      if (m == "key") keySource = SRC_KEY;
      else if (m == "audio") keySource = SRC_AUDIO;
      else if (m == "both") keySource = SRC_KEY | SRC_AUDIO;
      if (m.length()) prefs.putUChar("src", keySource);
      Serial.printf("Quelle: %s\n", keySource == 3 ? "both" : (keySource == 1 ? "key" : "audio"));
    } else if (line == "/clear") {
      durLen = symLen = 0; drawSymbols("");
      clearText();
    } else if (line == "/help" || line == "?") {
      printHelp();
    } else {
      finishCharacter();                 // evtl. halbes Empfangszeichen abschließen
      sendText(line);
      audioFlush();
      lastRelease = 0;                   // Decoder-Pausen zurücksetzen
      wordSpacePending = false;
    }
  }
}

// Funktionstaste: kurz = löschen, lang = Echo des Textes
uint32_t funcPressedAt = 0;

void funcButtonLoop(uint32_t now) {
  if (updateButton(funcBtn, now)) {
    if (funcBtn.stable) {
      funcPressedAt = now;
    } else {
      uint32_t held = now - funcPressedAt;
      if (held >= LONG_PRESS_MS) {
        // Echo: empfangenen Text erneut senden
        String s;
        for (int i = 0; i < textLen; i++) {
          const char *u = ((uint8_t)textBuf[i] >= 0x80) ? dispToTxt((uint8_t)textBuf[i]) : nullptr;
          if (u) s += u; else s += textBuf[i];
        }
        s.trim();
        if (s.length()) { finishCharacter(); sendText(s); audioFlush(); lastRelease = 0; wordSpacePending = false; }
      } else {
        symLen = durLen = 0; drawSymbols("");
        clearText();
      }
    }
  }
}


// ==========================================================================
//  v45 WLAN + Webserver (Port 80, WebSocket /ws). Netzwerk läuft auf Core 0
//  (WLAN-Treiber, lwIP, AsyncTCP), Audio/Decoder als Task auf Core 1 mit
//  höherer Priorität als loop(). loop() verschickt nur, was Audio-Task und
//  Display ohnehin erzeugen; der Decoder selbst bleibt unverändert.
//  Meldungen an den Browser (Text, eine Zeile je Meldung):
//   S,...  Status ~5/s     B,...  Spektrum ~5/s     L<c0>:<..> Pegel/Element-Spalten (4 ms)
//   K1,t / K0,t,dauer      C,t0,t1,zeichen          T<text>  Textbereich   R<antwort>
// ==========================================================================
bool     wifiOn = true;             // NVS "wifi"
String   wifiSsid, wifiPass;        // aktiv (NVS überschreibt wifi_config.h)
bool     wifiUp = false, wifiSrvUp = false, wifiGotIp = false;
uint32_t wifiTryT = 0, wifiIpShowUntil = 0;
AsyncWebServer webSrv(80);
AsyncWebSocket webWs("/ws");
struct WebCmd { uint32_t client; char line[200]; };
QueueHandle_t webCmdQ = nullptr;
uint32_t wevTail = 0, webTxtSum = 0xFFFFFFFF, webStCol = 0;

#include "web_page.h"   // v50 (Webseite wie v49)

bool wifiCredsOk() { return wifiSsid.length() > 0 && wifiPass.length() > 0 && wifiPass != "HIER_PASSWORT"; }

void webOnWs(AsyncWebSocket *s, AsyncWebSocketClient *c, AwsEventType t, void *arg, uint8_t *data, size_t len) {
  if (t == WS_EVT_CONNECT) { webTxtSum = 0xFFFFFFFF; return; }   // neuer Browser -> Text sofort komplett
  if (t != WS_EVT_DATA) return;
  AwsFrameInfo *fi = (AwsFrameInfo *)arg;
  if (!fi->final || fi->index != 0 || fi->len != len || fi->opcode != WS_TEXT) return;
  WebCmd w; w.client = c->id(); size_t n = len < sizeof(w.line) - 1 ? len : sizeof(w.line) - 1;
  memcpy(w.line, data, n); w.line[n] = 0;
  if (webCmdQ) xQueueSend(webCmdQ, &w, 0);
}

// v46: alle WLAN-Aktionen (begin/disconnect/mode) und WLAN-NVS-Schreibvorgaenge laufen in
// wifiTask auf Core 0. Befehle setzen nur Auftraege (wifiReq), damit loop() die Audio-Sperre
// nie waehrend blockierender WLAN-/Flash-Aufrufe haelt (v45: ~4 s Audio-Stillstand).
volatile uint8_t wifiReq = 0;      // Bit0 start, Bit1 stop, Bit2 NVS ssid, Bit3 NVS pass, Bit4 NVS wifi-flag
volatile uint8_t wifiReason = 0;   // letzter Trennungsgrund (WiFi-Event)
volatile bool wifiEvConn = false;
portMUX_TYPE wifiMux = portMUX_INITIALIZER_UNLOCKED;
String wifiNewSsid, wifiNewPass;
const char *wifiReasonTxt(uint8_t r) {
  switch (r) {
    case 0: return "";
    case WIFI_REASON_NO_AP_FOUND: return "SSID nicht gefunden (NO_AP_FOUND, 2,4 GHz?)";
    case WIFI_REASON_AUTH_FAIL: return "Anmeldung fehlgeschlagen (AUTH_FAIL)";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT: return "Passwort falsch? (4WAY_HANDSHAKE_TIMEOUT)";
    case WIFI_REASON_HANDSHAKE_TIMEOUT: return "Passwort falsch? (HANDSHAKE_TIMEOUT)";
    case WIFI_REASON_AUTH_EXPIRE: return "AUTH_EXPIRE";
    case WIFI_REASON_ASSOC_FAIL: return "ASSOC_FAIL";
    case WIFI_REASON_BEACON_TIMEOUT: return "BEACON_TIMEOUT (Signal schwach)";
    case WIFI_REASON_CONNECTION_FAIL: return "CONNECTION_FAIL";
    default: return "anderer Grund";
  }
}
void wifiEvent(arduino_event_id_t ev, arduino_event_info_t info) {
  if (ev == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) { wifiReason = info.wifi_sta_disconnected.reason; wifiEvConn = false; }
  else if (ev == ARDUINO_EVENT_WIFI_STA_GOT_IP) { wifiReason = 0; wifiEvConn = true; }
}
void wifiStart();
void wifiStop();
void wifiTask(void *) {
  while (millis() < 4000) vTaskDelay(pdMS_TO_TICKS(100));   // v50: WLAN erst nach fertigem Display (Stromspitze)
  for (;;) {
    uint8_t rq; String ss, pw;
    portENTER_CRITICAL(&wifiMux); rq = wifiReq; wifiReq = 0; portEXIT_CRITICAL(&wifiMux);
    if (rq & 0x0C) { ss = wifiNewSsid; pw = wifiNewPass; }
    if (rq & 4) prefs.putString("wssid", ss);
    if (rq & 8) prefs.putString("wpass", pw);
    if (rq & 16) prefs.putBool("wifi", wifiOn);
    if (rq & 4) wifiSsid = ss;
    if (rq & 8) wifiPass = pw;
    if (rq & 2) wifiStop();
    if (rq & 1) { wifiReason = 0; wifiStart(); }
    uint32_t now = millis();
    if (wifiUp && !wifiEvConn && now - wifiTryT > 20000) { wifiTryT = now; WiFi.disconnect(); WiFi.begin(wifiSsid.c_str(), wifiPass.c_str()); }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
void wifiPost(uint8_t bits) { portENTER_CRITICAL(&wifiMux); wifiReq |= bits; portEXIT_CRITICAL(&wifiMux); }

void wifiStart() {
  if (!wifiCredsOk()) {
    Serial.println(F("\n[WLAN: kein Passwort hinterlegt - in wifi_config.h eintragen oder /wifi pass <Passwort>]"));
    return;
  }
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(WIFI_HOSTNAME);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.setTxPower(WIFI_POWER_15dBm);   // v50: weniger Stromspitzen (Display-Versorgung)
  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
  wifiTryT = millis(); wifiUp = true;
  if (!wifiSrvUp) {
    webWs.onEvent(webOnWs);
    webSrv.addHandler(&webWs);
    webSrv.on("/", HTTP_GET, [](AsyncWebServerRequest *r) { AsyncWebServerResponse *rs = r->beginResponse(200, "text/html; charset=utf-8", WEB_PAGE_GZ, WEB_PAGE_GZ_LEN); rs->addHeader("Content-Encoding", "gzip"); rs->addHeader("Cache-Control", "no-cache"); r->send(rs); });
    webSrv.onNotFound([](AsyncWebServerRequest *r) { r->send(404, "text/plain", "nicht gefunden"); });
    webSrv.begin(); wifiSrvUp = true;
  }
  Serial.printf("\n[WLAN: verbinde mit \"%s\" ...]\n", wifiSsid.c_str());
}
void wifiStop() {
  if (!wifiUp) return;
  webWs.closeAll();
  MDNS.end();
  WiFi.disconnect(true); WiFi.mode(WIFI_OFF);
  wifiUp = false; wifiGotIp = false;
}
void wifiSetup() {
  wifiOn = prefs.getBool("wifi", true);
  wifiSsid = prefs.isKey("wssid") ? prefs.getString("wssid", WIFI_SSID) : String(WIFI_SSID);
  wifiPass = prefs.isKey("wpass") ? prefs.getString("wpass", WIFI_PASS) : String(WIFI_PASS);
  if (!wifiPass.length() || wifiPass == "HIER_PASSWORT") wifiPass = WIFI_PASS;   // NVS leer -> Datei
  webCmdQ = xQueueCreate(4, sizeof(WebCmd));
  WiFi.onEvent(wifiEvent);
  xTaskCreatePinnedToCore(wifiTask, "wifi", 6144, nullptr, 1, nullptr, 0);   // v46: Core 0
  if (wifiOn) wifiPost(1);
}
// Jitter-/Lastmessung Audio-Task (für /stat #J und /wifi status)
void printJitter() {
  Serial.printf("#J audioCore=%d itMax=%uus gaps>20ms=%u backlogMax=%.1fms adcOvf=%u iter=%u wifi=%d\n", 1,
                (unsigned)jItMaxUs, (unsigned)jGap20, jBackMax * 1000.0f / (2.0f * fsMeasured), (unsigned)jOvf, (unsigned)jIt, wifiGotIp ? 1 : 0);
  jItMaxUs = 0; jGap20 = 0; jBackMax = 0; jIt = 0;
}
void wifiStatus() {
  const char *st = !wifiOn ? "aus" : !wifiCredsOk() ? "kein Passwort" : wifiGotIp ? "verbunden" : "verbinde ...";
  Serial.printf("\n[WLAN %s, SSID \"%s\", status=%d", st, wifiSsid.c_str(), (int)WiFi.status());
  if (!wifiGotIp && wifiReason) Serial.printf(", Grund %u: %s", (unsigned)wifiReason, wifiReasonTxt(wifiReason));
  if (wifiGotIp) Serial.printf(", IP %s, RSSI %d dBm, Browser %u, http://%s.local/ bzw. http://%s/", WiFi.localIP().toString().c_str(), (int)WiFi.RSSI(), (unsigned)webWs.count(), WIFI_HOSTNAME, WiFi.localIP().toString().c_str());
  Serial.println("]");
  printJitter();
}
// /wifi on|off|status|ssid <Name>|pass <Passwort>
void wifiCmd(String a) {
  a.trim();
  // v46: nur Auftraege an wifiTask (Core 0), nichts Blockierendes hier
  if (a == "on") { wifiOn = true; wifiPost(16 | (wifiUp ? 0 : 1)); }
  else if (a == "off") { wifiOn = false; wifiPost(16 | 2); }
  else if (a.startsWith("ssid ")) { String v = a.substring(5); v.trim(); portENTER_CRITICAL(&wifiMux); portEXIT_CRITICAL(&wifiMux);
    wifiNewSsid = v; if (!(wifiReq & 8)) wifiNewPass = wifiPass; wifiPost(4 | 2 | (wifiOn ? 1 : 0)); Serial.println(F("\n[WLAN-Name wird gespeichert, verbinde neu]")); return; }
  else if (a.startsWith("pass ")) { String v = a.substring(5); v.trim();
    wifiNewPass = v; if (!(wifiReq & 4)) wifiNewSsid = wifiSsid; wifiPost(8 | 2 | (wifiOn ? 1 : 0));
    Serial.printf("\n[WLAN-Passwort wird gespeichert (%u Zeichen), verbinde neu]\n", (unsigned)v.length()); return; }
  wifiStatus();
}

static void webSendStatus(uint32_t now) {
  char b[300];
  float thrDb = stThr > 0 ? 20.0f * log10f(stThr) : 0, sqDbv = stSq > 0 ? 20.0f * log10f(stSq) : 0;
  char hk = (handMode == 2) ? 'P' : (isHand() ? 'H' : (handMode == 0 && handIs == 0 ? 'A' : 'P'));
  snprintf(b, sizeof(b), "S,%u,%d,%d,%.1f,%d,%.0f,%.0f,%.1f,%d,%.0f,%d,%.1f,%.1f,%c,%d,%d,%d,%d,%d,%d,%.0f",
           (unsigned)now, (int)(1200.0f / unitMs + 0.5f), txWpm, unitMs, stMs, decHz, barPeakValid ? barPeakHz : 0.0f, barPeakDb,
           barPeakValid ? 1 : 0, (float)lvlAmp, (int)lvlWarn, thrDb, sqDbv, hk, afcLocked ? 1 : 0, (int)WiFi.RSSI(),
           (int)((int32_t)(lvlClipUntil - now) > 0), multiOn ? 1 : 0, tickerOn ? 1 : 0, STRIP_DOT_PX, thrMs);
  size_t n = strlen(b);   // v48: Einstellungen fuer die Seitenleiste: thrmid, sq, stoch, stripauto, wifi
  snprintf(b + n, sizeof(b) - n, ",%.2f,%.1f,%d,%d,%d", thrMid, sqDb, stochOn ? 1 : 0, stripAuto ? 1 : 0, wifiOn ? 1 : 0);
  webWs.textAll(b);
}
static void webSendBars() {
  String s; s.reserve(200); s = "B,"; s += BAR_F0; s += ','; s += BAR_STEP; s += ','; s += BAR_N;
  for (int i = 0; i < BAR_N; i++) { s += ','; s += (int)(barDb[i] + 0.5f); }
  for (int i = 0; i < BAR_N; i++) { s += ','; s += (int)(barHold[i] + 0.5f); }
  webWs.textAll(s);
}
// Pegel-/Elementspalten (Raster ST_BASE ms) seit der letzten Sendung: je Spalte 2 Zeichen
// ('0'+dB 0..60 bzw. '/' = keine Daten, '0'+Flags wie Streifen)
static void webSendCols(uint32_t now) {
  uint32_t c = now / ST_BASE - 1;
  if (webStCol == 0 || c - webStCol > 400) webStCol = c - 100;
  if (c <= webStCol) return;
  String s; s.reserve(16 + 2 * (c - webStCol)); s = "L"; s += (unsigned)(webStCol + 1); s += ':';
  for (uint32_t k = webStCol + 1; k <= c; k++) {
    const volatile StCol &e = stRing[k % ST_N];
    if (e.c != (uint16_t)k) { s += '/'; s += '0'; continue; }
    float m = e.pk; int db = m > 1 ? (int)constrain(20.0f * log10f(m) + 0.5f, 0.0f, 60.0f) : 0;
    s += (char)('0' + db); s += (char)('0' + (e.f & 7));
  }
  webStCol = c;
  webWs.textAll(s);
}
static void webSendEvents() {
  uint32_t h = wevHead;
  if (h - wevTail > WEV_N) wevTail = h - WEV_N;
  if (h == wevTail) return;
  String s; s.reserve(400); char b[48];
  while (wevTail != h) {
    const volatile WebEv &e = wevR[wevTail % WEV_N]; wevTail++;
    if (e.type == 1) snprintf(b, sizeof(b), "K1,%u\n", (unsigned)e.a);
    else if (e.type == 2) snprintf(b, sizeof(b), "K0,%u,%u\n", (unsigned)e.a, (unsigned)e.b);
    else { char t[7]; int i = 0; for (; i < 6 && e.s[i]; i++) t[i] = e.s[i]; t[i] = 0; snprintf(b, sizeof(b), "C,%u,%u,%s\n", (unsigned)e.a, (unsigned)e.b, t); }
    s += b;
    if (s.length() > 1200) { webWs.textAll(s); s = ""; }
  }
  if (s.length()) webWs.textAll(s);
}
// Textbereich als UTF-8 (Sonderzeichen über die Morsetabelle), nur bei Änderung
static void webSendText(bool force) {
  uint32_t sum = 2166136261u ^ (uint32_t)textLen;
  for (int i = 0; i < textLen; i++) sum = (sum ^ (uint8_t)textBuf[i]) * 16777619u;
  if (!force && sum == webTxtSum) return;
  webTxtSum = sum;
  String s; s.reserve(textLen + 32); s = "T"; s += cols; s += '|';
  for (int i = 0; i < textLen; i++) {
    uint8_t ch = (uint8_t)textBuf[i];
    const char *u = ch >= 0x80 ? dispToTxt(ch) : nullptr;
    if (u) s += u; else s += (char)ch;
  }
  webWs.textAll(s);
}
void execCmd(String line);
void webLoop(uint32_t now) {
  if (!wifiUp) return;
  static uint32_t tSt = 0, tFast = 0, tClean = 0;
  bool conn = WiFi.status() == WL_CONNECTED;
  if (conn && !wifiGotIp) {
    wifiGotIp = true; wifiIpShowUntil = now + 15000; drawHeaderNow();
    MDNS.end(); if (MDNS.begin(WIFI_HOSTNAME)) MDNS.addService("http", "tcp", 80);
    Serial.printf("\n[WLAN verbunden: http://%s.local/  IP %s]\n", WIFI_HOSTNAME, WiFi.localIP().toString().c_str());
  } else if (!conn && wifiGotIp) {
    wifiGotIp = false; wifiTryT = now; Serial.println(F("\n[WLAN getrennt - verbinde im Hintergrund neu]"));
  }
  // v46: Reconnect macht wifiTask (Core 0)
  if (wifiIpShowUntil && (int32_t)(now - wifiIpShowUntil) > 0) { wifiIpShowUntil = 0; drawHeaderNow(); }
  // Befehle vom Browser: wie seriell, Ausgabe zusätzlich an den Browser zurück
  WebCmd w;
  while (webCmdQ && xQueueReceive(webCmdQ, &w, 0) == pdTRUE) {
    String out; String line(w.line); line.trim();
    if (!line.length()) continue;
    xSemaphoreTake(audioMtx, portMAX_DELAY);
    Serial.cap = &out;
    Serial.printf("\n[Web] %s\n", line.startsWith("/wifi pass") ? "/wifi pass ***" : line.c_str());
    execCmd(line);
    Serial.cap = nullptr;
    xSemaphoreGive(audioMtx);
    if (out.length() > 3000) out = out.substring(0, 3000);
    webWs.text(w.client, String("R") + out);
  }
  if (!conn) return;
  if (now - tClean > 1000) { tClean = now; webWs.cleanupClients(); }
  if (!webWs.count()) { wevTail = wevHead; webStCol = 0; return; }
  if (now - tFast >= 50) { tFast = now; webSendEvents(); webSendCols(now); webSendText(false); }
  if (now - tSt >= 200) { tSt = now; webSendStatus(now); if (barOn) webSendBars(); }
}

// v48: Webseite (Design C) gzip-komprimiert in web_page.h


// ==========================================================================
//  SETUP / LOOP
// ==========================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(PIN_KEY,  INPUT_PULLUP);
  pinMode(PIN_FUNC, INPUT_PULLUP);
#if PIN_LED >= 0
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
#endif
  prefs.begin("morse", false);
  narrowMode = prefs.getBool("narrow", false);
  monMode = prefs.getUChar("mon", MON_FILT); monVol = prefs.getInt("monvol", 60); monBw = prefs.getInt("monbw", 300);
  txWpm = prefs.getInt("wpm", txWpm); toneHz = prefs.getInt("ton", toneHz); txCombineCh = prefs.getBool("ch", txCombineCh);
  barOn = prefs.getBool("bar", barOn); sqDb = prefs.getFloat("sq", sqDb); if (!prefs.getBool("v24sq", false)) { if (sqDb == 9.0f) { sqDb = SQ_DB_DEFAULT; prefs.putFloat("sq", sqDb); } prefs.putBool("v24sq", true); } afcOn = prefs.getBool("afc", afcOn);
  keySource = prefs.getUChar("src", keySource);
  audioFreq = prefs.getInt("freq", AUDIO_FREQ_HZ); decHz = afcHz = afcCenter = audioFreq;
  multiOn = prefs.getBool("multi", false);
  tickerOn = prefs.getBool("strip", true); stripAuto = prefs.getBool("stripauto", true);
  dictOn = prefs.getBool("dict", true); rewordOn = prefs.getBool("reword", true); softOn = prefs.getBool("soft", true); stochOn = prefs.getBool("stoch", true); lvlWarnOn = prefs.getBool("lvlwarn", true); thrDecay = prefs.getFloat("thrdec", 3.0f); thrMid = prefs.getFloat("thrmid", 0.5f); hystF = prefs.getFloat("hyst", AUDIO_HYST); handMode = prefs.getUChar("hand", 0); handIs = (handMode == 1) ? 1 : 0; qsyOn = prefs.getBool("qsy", true); qsyMinMs = prefs.getUInt("qsyms", 1000); ratioOn = prefs.getBool("ratio", false); ratioHi = prefs.getFloat("ratiohi", 0.7f); wbOn = prefs.getBool("wbrej", true); wbDb = prefs.getFloat("wbdb", 6.0f); wbK = powf(10.0f, wbDb / 20.0f); bridgeU = prefs.getFloat("bridge", 0.2f); afcSwMs = prefs.getUInt("afcsw", 1500);
  for (int i = 0; i < MCH; i++) { char k[4] = {'m', 'f', (char)('0' + i), 0}; mFreq[i] = prefs.getInt(k, mFreq[i]); }
  mReset();
  setupDisplay();
  if (!tkCv) tkCv = new GFXcanvas16(W, 56);
  audioOk = audioBegin();
  monBegin(); monSetFreqs(decHz);
  uiQ = xQueueCreate(64, sizeof(UiOp));
  audioMtx = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(audioTask, "audio", 8192, nullptr, 20, &audioTaskH, AUDIO_CORE);   // v47: Prio 20 > lwIP tcpip (18, ohne Kernbindung)   // v45: Core 1, Prio 5 > loop (1)
#if AUDIO_SELFTEST
  Serial.println(F("*** AUDIO-SELBSTTEST: ADC an GPIO 32, Generator 700 Hz auf GPIO 32 ***"));
#endif
  Serial.printf("Audio-Eingang GPIO34: %s\n", audioOk ? "aktiv (20 kHz, Goertzel)" : "FEHLER beim ADC-Start");
  delay(200);
  printHelp();
  wifiSetup();   // v45: WLAN im Hintergrund (blockiert nicht)
}

// Audio-Task (Core 0): ADC lesen, Goertzel, Decoder. Kein Display-Zugriff.
void audioTask(void *) {
  uint32_t lastIt = 0;
  for (;;) {
    xSemaphoreTake(audioMtx, portMAX_DELAY);
    uint32_t t0us = micros();
    if (lastIt) { uint32_t g = t0us - lastIt; if (g > jItMaxUs) jItMaxUs = g; if (g > 20000) jGap20++; }   // v45 Jitter
    lastIt = t0us; jIt++;
    uint32_t now = millis();
    audioLoop(now);
#if AUDIO_SELFTEST
    selftestLoop(now);
#endif
    if (multiOn) multiDecoderLoop(aClkEnd); else decoderLoop(aClkEnd);   // v47: Decoder-Zeit = Abtast-Uhr
    uint32_t dt = micros() - t0us;
    if (dt > loopMaxUs) loopMaxUs = dt;
    c0Account(dt, now);               // Laufzeit Audiopfad
    xSemaphoreGive(audioMtx);
    vTaskDelay(1);
  }
}

// v25: Pegelwarnung, läuft komplett auf Core 1 (nur Auswertung vorhandener Messwerte).
// Je Sekunde: aktiv = Bargraph sieht einen Ton nahe der Decoderfrequenz (oder Decoder
// tastet); stark = max. Bargraph-Peak in dB. HIL v24: voll ~51 dB, -18 dB ~33 dB,
// -36 dB ~16 dB, reines Rauschen ohne Ton -> kein gültiger Peak, Stille -> kein Peak.
#define LW_LOW_DB   26.0f   // Sekunde mit Ton, aber max. Peak darunter = "schwach"
#define LW_OK_DB    30.0f   // ab hier zählt die Sekunde als gut (Hysterese)
#define LW_ON_SECS  4       // so viele schwache Sekunden (ohne gute dazwischen) -> Warnung
#define LW_HOLD_MS  4000    // Warnung mindestens so lange stehen lassen
uint32_t lwSecT = 0, lwShownT = 0, lwLastAct = 0;
float lwSecMax = 0; bool lwSecAct = false, lwSecClip = false;
int8_t lwLow = 0, lwClip = 0;
void lvlWarnLoop(uint32_t now) {
  if (barPeakValid && fabsf(barPeakHz - decHz) < 100.0f) { lwSecAct = true; if (barPeakDb > lwSecMax) lwSecMax = barPeakDb; }
  if ((int32_t)(lvlClipUntil - now) > 0) lwSecClip = true;
  if (now - lwSecT < 1000) return;
  lwSecT = now;
  if (lwSecAct) {
    lwLastAct = now;
    if (barOn && lwSecMax < LW_LOW_DB) { if (lwLow < 8) lwLow++; }
    else if (lwSecMax >= LW_OK_DB) lwLow = 0;
  } else if (now - lwLastAct > 10000) lwLow = 0;   // lange kein Ton -> keine Warnung
  if (lwSecClip) { if (lwClip < 5) lwClip++; } else if (lwClip > 0) lwClip--;
  lwSecMax = 0; lwSecAct = lwSecClip = false;
  int8_t w = lvlWarn;
  if (!lvlWarnOn || multiOn) w = 0;
  else if (lwClip >= 3) w = 2;
  else if (w == 2 && lwClip > 0) w = 2;
  else if (lwLow >= LW_ON_SECS) w = 1;
  else if (w == 1 && lwLow > 0) w = 1;
  else w = 0;
  if (w != lvlWarn && lvlWarn && now - lwShownT < LW_HOLD_MS && lvlWarnOn) return;   // Mindestanzeigezeit
  if (w != lvlWarn) {
    lvlWarn = w; lwShownT = now;
    if (w == 1) Serial.println(F("\n[PEGEL ZU NIEDRIG - Ton zu leise, Lautstaerke am WebSDR/Empfaenger erhoehen]"));
    if (w == 2) Serial.println(F("\n[PEGEL ZU HOCH - Eingang uebersteuert, Lautstaerke verringern]"));
    if (w == 0) Serial.println(F("\n[Pegel wieder in Ordnung]"));
    drawHeaderNow();
  }
}


// v50: Display neu initialisieren und komplett neu zeichnen (nur aus loop)
volatile bool tftReq = false; uint32_t tftLast = 0; bool tftIpDone = false;
void tftReinit() { setupDisplay(); tftLast = millis(); }
void loop() {
  uint32_t now = millis();
  if (tftReq || (!tftIpDone && wifiEvConn && now > 6000) || now - tftLast > TFT_REFRESH_MS) {
    if (wifiEvConn) tftIpDone = true;
    tftReq = false; tftReinit(); now = millis();
  }
  uint32_t tb = micros();
  uiDrain(15);
  barDraw(now);
  lvlWarnLoop(now);
  tickerDraw(now);
  uint32_t db = micros() - tb;
  if (db > barMaxUs) barMaxUs = db;                   // Laufzeit Display (Core 1)
  // Taste/Befehle greifen auf Decoder-/Audio-Zustand zu -> mit Sperre
  xSemaphoreTake(audioMtx, portMAX_DELAY);
  funcButtonLoop(now);
  serialLoop();
  xSemaphoreGive(audioMtx);
  webLoop(now);   // v45: WLAN/Webseite (nimmt die Sperre nur fuer Web-Befehle)
  delay(1);
}
