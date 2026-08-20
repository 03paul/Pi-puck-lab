# Dezentrale marktbasierte Multi-Roboter-Aufgabenzuweisung

Dieses Projekt implementiert die erste Stufe des TU-Darmstadt-Projekts als schnelle,
deterministische Python-Simulation. Die Roboter verteilen PUSH-, SURROUND-, GUARD- und
SURVEY-Aufgaben über eine Peer-to-Peer-Auktion. Es gibt keinen zentralen Allocator.

Die Simulation enthält drei direkt vergleichbare Zuweisungsstrategien:

- `market`: dezentrale Bidding- und Auktionslogik
- `nearest_greedy`: jede Aufgabe geht an den nächstgelegenen geeigneten Roboter
- `random_assignment`: zufällige Zuweisung an einen geeigneten Roboter

Zusätzlich gibt es `nearest_greedy_no_survey` als Ablation ohne SURVEY-Verhalten.

## Bereits berechnetes Ergebnis

Die vollständige Holdout-Auswertung ist bereits gerechnet. Auf 40 zuvor nicht zur
Parametersuche verwendeten, gepaarten Szenario-Seeds erreicht der Marktansatz:

| Strategie | Operationaler Score | Completion Rate | Bearbeitungszeit | Laststreuung |
|---|---:|---:|---:|---:|
| Market | **0,861** | **0,918** | **37,76 s** | **1,42** |
| Nearest Greedy | 0,831 | 0,887 | 47,84 s | 1,84 |
| Greedy ohne SURVEY | 0,813 | 0,861 | 46,98 s | 2,09 |
| Random | 0,813 | 0,788 | 59,60 s | 1,35 |

Gegen Nearest Greedy ist der gepaarte Score-Vorteil `+0,0305` mit 95%-Bootstrap-CI
`[+0,0151; +0,0462]` und Holm-korrigiertem `p=0,0040`. Market ist außerdem im Mittel
`10,08 s` schneller und reduziert die Laststreuung um `0,42` Aufgaben. Das ist ein
empirischer Nachweis für die dokumentierte Szenariofamilie, kein universeller
Dominanzbeweis. Kommunikationskosten und Einzelmetriken werden vollständig offengelegt.

## Technischer Zwischenbericht als PDF

Eine zusammenhängende Dokumentation der gesamten ersten Stufe liegt direkt im
Projektordner als `Technischer_Zwischenbericht_Stufe_1.pdf`. Die bearbeitbare
LaTeX-Quelldatei heißt `Technischer_Zwischenbericht_Stufe_1.tex`.

Der Bericht enthält Zielsetzung, Architektur, dezentrales Protokoll, Bidding-Formeln,
Simulationsparameter, Versuchsdesign, statistische Ergebnisse, Diagramme, Bedienung,
Grenzen des aktuellen Nachweises und den vorgeschlagenen Übergang zu Webots und Pi-Pucks.

## Nächster Schritt: Webots-Kalibrierung vor dem Lab

Bevor der Code auf echte Pi-Pucks geht, sollten `startup_delay` und `effective_speed` in
`parameters/default.json` gegen reale Fahrdynamik in Webots kalibriert werden — sonst beruhen
die Tier-0-Zeitmetriken auf ungeprüften Annahmen. Ablauf, Controller-Vorlage und Fitting-Skript
stehen in [`docs/WEBOTS_CALIBRATION.md`](docs/WEBOTS_CALIBRATION.md). Ein Backend-Adapter für die
Pi-Pucks selbst (`backends/pipuck.py`) existiert noch nicht und ist ein separater Schritt danach.

Zusätzlich lässt sich die unveränderte Marktlogik (`allocation/`) direkt gegen reale Webots-
Fahrdynamik testen, um zu prüfen, ob die in `comparison_summary.md` gemessene Rangfolge
(Market > Nearest Greedy > Random) erhalten bleibt — siehe
[`docs/WEBOTS_MARKET_DEMO.md`](docs/WEBOTS_MARKET_DEMO.md).

## Wichtig zum Ordnerpfad

Das Projekt ist nicht an einen bestimmten Computer oder Benutzernamen gebunden. Ein
lokaler Windows-Pfad ist immer nur der Speicherort auf dem jeweiligen Computer. Der
komplette Ordner `decentralized-mrta` kann auf einen anderen Computer kopiert oder als
ZIP-Datei verschickt und dort an einem beliebigen Ort entpackt werden.

Am einfachsten öffnet man den entpackten Ordner im Windows-Explorer, klickt oben in die
Adresszeile, schreibt `powershell` und drückt Enter. Dann öffnet sich PowerShell direkt
im richtigen Projektordner und ein `cd`-Befehl ist nicht nötig.

Alternativ wechselt man mit dem eigenen tatsächlichen Pfad in den Ordner:

```powershell
cd "PFAD\ZUM\ENTPACKTEN\decentralized-mrta"
```

Alle folgenden Befehle müssen im Hauptordner des Projekts ausgeführt werden. Dort
liegen unter anderem diese `README.md` und die Datei `pyproject.toml`.

## 1. Voraussetzungen prüfen

Benötigt wird Python 3.11 oder neuer:

```powershell
python --version
```

Falls Windows `python` nicht findet, kann man stattdessen bei allen Befehlen `py`
verwenden, zum Beispiel `py --version` oder `py -m pip install -e .`.

## 2. Projekt einmalig installieren

Im Projektordner ausführen:

```powershell
python -m pip install -e .
```

Die Installation ist pro Python-Umgebung nur einmal nötig. Danach können die Simulation,
Auswertung und Replays über die folgenden Befehle gestartet werden.

Optional kann die Installation mit den 18 automatischen Tests geprüft werden:

```powershell
python -m unittest discover -s tests -v
```

Am Ende sollte `OK` stehen.

## 3. Simulation starten und interaktiv ansehen

Der einfachste empfohlene Start erzeugt einen Simulationslauf mit der dezentralen
Marktstrategie und danach einen interaktiven 2D-Player:

```powershell
python -m experiments.run_once --strategy market --seed 42 --output logs\market_42 --interactive-replay
```

Nach dem Lauf liegt der Player hier:

```text
logs\market_42\replay.html
```

Die Datei `replay.html` im Explorer doppelt anklicken. Sie öffnet sich lokal im Browser
und benötigt keine Internetverbindung und keinen laufenden Server.

Im Player kann man:

- mit **Play/Pause** die Animation starten und anhalten
- mit dem Zeitregler an jede Stelle springen
- mit **-1 s** und **+1 s** schrittweise vor- und zurückgehen
- die Geschwindigkeit zwischen `0.25x` und `20x` ändern
- Roboterpfade und SURVEY-Anzeigen ein- oder ausblenden
- mit der Leertaste pausieren beziehungsweise fortsetzen
- mit den Pfeiltasten durch die Zeit gehen

Der Player zeigt die Bewegung der Roboter in der 2D-Welt, Aufgabenpositionen, Status,
Zuweisungen und den zeitlichen Ablauf des Simulationslaufs.

## 4. Andere Strategien mit demselben Szenario vergleichen

Für einen fairen direkten Vergleich bei allen Strategien denselben Seed verwenden. Der
Seed `42` erzeugt jedes Mal dasselbe Szenario; nur die Zuweisungsstrategie ändert sich.

Dezentrale Marktstrategie:

```powershell
python -m experiments.run_once --strategy market --seed 42 --output logs\market_42 --interactive-replay
```

Nearest Greedy:

```powershell
python -m experiments.run_once --strategy nearest_greedy --seed 42 --output logs\greedy_42 --interactive-replay
```

Random Assignment:

```powershell
python -m experiments.run_once --strategy random_assignment --seed 42 --output logs\random_42 --interactive-replay
```

Greedy-Ablation ohne SURVEY:

```powershell
python -m experiments.run_once --strategy nearest_greedy_no_survey --seed 42 --output logs\greedy_no_survey_42 --interactive-replay
```

Danach können die jeweiligen `replay.html`-Dateien nebeneinander im Browser geöffnet
werden.

## 5. Was nach einem Lauf gespeichert wird

Der mit `--output` angegebene Ordner enthält die reproduzierbaren Simulationsdaten:

- `events.csv`: Auktionen, Gebote, Zuweisungen und weitere Ereignisse
- `state.csv`: Position und Zustand der Roboter über die Simulationszeit
- `metadata.json`: Strategie, Seed, Parameter und Metadaten des Laufs
- `replay.html`: interaktiver Player, wenn `--interactive-replay` benutzt wurde
- `replay.gif`: nicht interaktive Animation, wenn `--replay` benutzt wurde

Ein interaktiver Player kann auch später aus einem bereits vorhandenen Logordner erzeugt
werden:

```powershell
python -m experiments.interactive_replay logs\example --output logs\example\replay.html
```

## 6. GIF-Replay erzeugen

Wenn eine einfach verschickbare Animation benötigt wird:

```powershell
python -m experiments.run_once --strategy market --seed 42 --output logs\market_gif --replay --replay-speed 10 --replay-fps 15
```

Das erzeugt `logs\market_gif\replay.gif`. Ein GIF kann nicht pausiert oder mit einer
Zeitleiste gesteuert werden. Dafür ist der interaktive HTML-Player gedacht.

Aus vorhandenen Logs lässt sich ein GIF so nachträglich erzeugen:

```powershell
python -m experiments.replay logs\example --output logs\example\replay.gif
```

## 7. Vollständigen wissenschaftlichen Vergleich neu berechnen

Die vollständige Parametersuche und der anschließende Holdout-Vergleich werden mit
folgendem Befehl reproduziert:

```powershell
python -m experiments.compare
```

Auf der Entwicklungsmaschine dauert das ungefähr 2,5 Minuten; auf anderen Computern
kann die Laufzeit abweichen. Die Parameter werden nur auf den Trainings-Seeds `0-7`
gewählt. Der eigentliche Nachweis verwendet davon getrennte Seeds `1000-1039`.

Die wichtigsten neu geschriebenen Ergebnisdateien liegen danach im Ordner `results`:

- `comparison_summary.md`: lesbarer Ergebnis- und Evidenzbericht
- `paired_effects.csv`: gepaarte Effekte, Konfidenzintervalle, Win-Rates und p-Werte
- `run_metrics.csv`: alle 160 Holdout-Läufe
- `sweep_summary.csv`: Mittelwerte aller geprüften Parameterkonfigurationen
- `pareto_parameters.csv`: nicht dominierte Parameterpunkte
- `benchmark_manifest.json`: exakte Konfiguration und Seed-Trennung
- `comparison.png` und `pareto.png`: berichtsfertige Abbildungen

Die bereits berechneten Dateien sind im ausgelieferten Projekt enthalten. Der große
Vergleich muss daher nicht neu gestartet werden, nur um die vorhandenen Resultate zu
lesen oder zu zeigen.

## Gewählte Bidding-Parameter

Die Standardparameter stehen in `parameters/default.json`. Der geprüfte Suchraum steht
in `parameters/search_space.json`.

```text
alpha      = 1.0    # Gewicht der geschätzten Distanz
beta       = 0.5    # Gewicht der aktuellen Roboterlast
gamma      = 0.0    # Gewicht der Batteriereserve im 300-s-Hauptszenario
delta      = 1.5    # Prioritätsgewinn bei Preemption
theta_hyst = 0.1    # Hysterese gegen unnötiges Wechseln
```

`gamma=0` ist ein Ergebnis der Trainingssuche, kein vergessener Term. In 300 Sekunden
fährt im Holdout kein Roboter leer; ein hoher Batterieterm verursacht dann zusätzliche
Wege ohne messbaren Ausfallvorteil. Für längere Einsätze bleibt `gamma` im Suchraum und
die Batteriedynamik ist vollständig implementiert.

Gebotskosten, wobei das niedrigste Gebot gewinnt:

```text
c_i(task) = alpha*d_hat + beta*w_hat + gamma*(1-e_hat)
```

Preemption wird nur aus `NAVIGATE` erlaubt:

```text
delta*(p_new-p_current) - (c_new-c_current) > theta_hyst
```

Die zeitabhängige Priorität ist:

```text
p(dt) = p0 + (1-p0)*(1-exp(-(lambda*dt)^k))
```

## Warum die Bidding-Logik dezentral ist

Jede `Robot`-Instanz besitzt ihren eigenen lokalen Taskbestand, ihre Gebote, laufenden
Aufträge und selbst gehosteten Auktionen. Der entdeckende Roboter sendet `ANNOUNCE`, alle
erreichbaren Roboter senden `BID`, und ausschließlich der jeweilige Auctioneer sendet
`AWARD`.

Der `World`-Code erzeugt Ereignisse und simuliert Bewegung, Funk und physische
Aufgabenausführung; er wählt keine Gewinner. Das Paket `allocation` importiert weder
`sim` noch `backends` oder `metrics`. Diese Trennung wird durch automatisierte Tests
geprüft.

## Projektstruktur

```text
allocation/     Plattformunabhängige Tasks, Gebote, Nachrichten und Auktionen
backends/       Acht-Fähigkeiten-Schnittstelle und Tier-0-Adapter
sim/            Punktroboter, Funkkanal, Batterie, Raster und Ereignisse
metrics/        Portables CSV-Logging und Log-Auswertung
experiments/    Parametersuche, Holdout-Statistik, Robustheit, Plots und 2D-Replay
parameters/     Laufparameter und Suchraum
tests/          18 Unit- und Invariantentests
results/        Fertige CSVs, Konfidenzintervalle, Bericht und Abbildungen
references/     Die beiden Original-PDFs
docs/           Architektur, Methodik und Anforderungszuordnung
```

Weitere technische Details stehen in `docs/ARCHITECTURE.md` und
`docs/METHODOLOGY.md`.

## Häufige Probleme

### `python` wurde nicht gefunden

Zuerst `py --version` versuchen und danach in den Befehlen `python` durch `py` ersetzen.
Wenn auch `py` nicht gefunden wird, muss Python 3.11 oder neuer installiert werden.

### `No module named experiments`

PowerShell befindet sich wahrscheinlich nicht im Hauptordner `decentralized-mrta` oder
die einmalige Installation fehlt. Prüfen, ob `README.md` und `pyproject.toml` im aktuellen
Ordner sichtbar sind, und danach erneut ausführen:

```powershell
python -m pip install -e .
```

### Der Browser zeigt die Simulation nicht

Prüfen, ob der Lauf ohne Fehlermeldung beendet wurde und ob im angegebenen Logordner
eine Datei `replay.html` liegt. Diese Datei direkt im Explorer doppelt anklicken. Für
Play/Pause muss der HTML-Player und nicht die GIF-Datei geöffnet werden.
