# Befehlsliste (Firmware v44)

Befehle im seriellen Monitor eingeben (115200 Baud, Zeilenende „Neue Zeile“). Text ohne `/` wird als Morse gegeben.

## Geber
| Befehl | Wirkung |
|---|---|
| `/wpm N` | Geber-Tempo (5..40 WPM) |
| `/ton N` | Tonhöhe Hz (300..1500) |
| `/ch on\|off` | CH beim Geben als `----` zusammenfassen (Standard aus: C H) |

## Audio-Eingang und Frequenz
| Befehl | Wirkung |
|---|---|
| `/freq N` | Audio-Zielfrequenz Hz (300..1500), Startwert der AFC |
| `/afc on\|off\|re` | Frequenznachführung ±100 Hz um `/freq` (re = neu einrasten) |
| `/afcsw ms` | AFC wechselt erst nach ms Stille auf einen neuen Peak (Standard 1500) |
| `/qsy on\|off\|<ms>` | nach Durchgangspause sofort auf Gegenstation einrasten (Standard an, 1000 ms) |
| `/narrow on\|off` | Schmal-Modus: fest auf `/freq`, ~120 Hz Bandbreite, Nachbarn ±150 Hz unterdrückt (Standard aus) |
| `/multi on\|off` | Mehrkanal-Modus: 4 Töne gleichzeitig dekodieren (Standard aus) |
| `/multif A B C D` | Mehrkanal-Frequenzen Hz (Standard 550 650 750 850) |
| `/src key\|audio\|both` | Quelle: Taste, Audio oder beides |

## Schwelle und Rauschsperre
| Befehl | Wirkung |
|---|---|
| `/sq N` | Rauschsperre N dB über Rauschen (Standard 4, 0 = aus; ohne AFC-Lock +3 dB) |
| `/thrmid <0.1..0.9>` | Lage der Schaltschwelle zwischen Rauschboden und Peak (Standard 0.5) |
| `/thrdecay <n>` | Schwelle fällt mit n Punktlängen Zeitkonstante (Standard 3, 0 = alt) |
| `/hyst <0.3..0.9>` | Aus-Schwelle relativ zur Ein-Schwelle (Standard 0.6; 0.5 = weniger zerrissene Striche) |
| `/ratio on\|off\|<0.3..0.95>` | Zusatzkriterium Ton/Breitband-Verhältnis (Standard aus, 0.7) |
| `/wbrej on\|off\|<dB>` | Breitband-Störer (Klicks) unterdrücken, Ton muss <dB> über Nachbarn liegen (Standard an, 6) |
| `/bridge x` | Fading-Überbrückung x Punktlängen 0..0.4 (Standard 0.2, 0 = aus) |
| `/lvlwarn on\|off` | Pegelwarnung PEGEL ZU NIEDRIG/HOCH im Kopf (Standard an) |

## Dekodierung
| Befehl | Wirkung |
|---|---|
| `/hand auto\|on\|off` | Gebeweise Hand/Paddle (Standard auto; bei Hand weichere Timing-Toleranzen) |
| `/soft on\|off` | Weiche Entscheidung am Zeichenende (Standard an) |
| `/stoch on\|off` | Zeichenpausen per Wahrscheinlichkeit: unklare Pausen von 1,4–2,2 Punkten nach Pausenstatistik und Buchstabenhäufigkeit trennen (Standard an, wird gespeichert) |
| `/dict on\|off` | Wörterbuch-Ergänzung (Standard an; korrigierte Wörter türkis, seriell `ROH[KORR]`) |
| `/reword on\|off` | Wort-Neuauswertung am Wortende (Standard an; ersetzte Wörter türkis, seriell `ROH[NEU]`) |

## Mithören (GPIO 25, DAC)
| Befehl | Wirkung |
|---|---|
| `/mon off\|tone\|filt` | Ton getastet oder gefiltertes Audio (Standard filt) |
| `/monbw N` | Mithör-Bandpass 100..1000 Hz (Standard 300) |
| `/monvol N` | Mithör-Lautstärke 0..100 |

## Anzeige
| Befehl | Wirkung |
|---|---|
| `/bar on\|off` | Bargraph-Abstimmanzeige (Peak-Frequenz auch in `/audio`: `pk=`, `dpk=`) |
| `/scope on\|off` | Laufband mit Pegel und Elementen (= `/ticker`) |
| `/stripauto on\|off` | Laufband-Tempo folgt der WPM, ein Punkt ≈ 5 Pixel (`STRIP_DOT_PX` in `display_config.h`). Standard an; aus = fest 12 ms/Pixel |
| `/clear` | Text löschen |

## Diagnose
| Befehl | Wirkung |
|---|---|
| `/audio` | Audio-Pegel-Debug an/aus (Zeilen beginnen mit `#A`) |
| `/stat` | Audio-Status einmal ausgeben (`#S ...`) |
| `/elem on\|off` | Element-Ausgabe mit ESP-Zeit: `#K1,t` / `#K0,t,dauer` / `#Ct,zeichen` (Standard aus) |
| `/help` | Hilfe anzeigen |

## Tasten
- Morsetaste an GPIO 27 (gegen GND)
- Funktionstaste an GPIO 33: kurz (< 1 s) = Text löschen, lang (≥ 1 s) = dekodierten Text als Morse wiederholen (Echo)
- Ein Druck auf eine der Tasten bricht eine laufende Geber-Ausgabe ab.
