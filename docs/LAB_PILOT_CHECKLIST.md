# Tier-2-Lab-Pilot: Checkliste (MQTT/RCPS-Version)

**Ziel laut Proposal (§12), wörtlich**: "a pilot whose only purpose is risk
elimination: verifying that a complete auction runs between two physical
Pi-Pucks over UDP" — technisch jetzt über **MQTT statt UDP** (siehe unten),
inhaltlich unverändert: eine GUARD-Aufgabe (`n_req=1`), zwei Roboter, ein
vollständiger Auktionszyklus (`ANNOUNCE → BID → AWARD → ACTIVE → DONE`).
**Kein** Strategievergleich — der ist mit statistischer Power schon in
Tier 0 gelaufen, mit einem Einzellauf pro Strategie schon in Tier 1.

**Wichtige Korrektur gegenüber der ersten Version dieser Checkliste**: Das
Lab hat bereits ein eigenes Overhead-Tracking-System (RCPS Dashboard,
`<broker-ip>:3000`), das Posen per **MQTT** auf Topic `robot_pos/all`
publiziert. Es gibt kein selbstgebautes ArUco/OpenCV-Kamera-Skript mehr —
`docs/pipuck_task_simulator.py` abonniert nur noch dieses bestehende Topic.

Neue/geänderte Dateien: `backends/pipuck.py` (54 Tests grün, siehe
`tests/test_pipuck_backend.py`), `docs/pipuck_task_simulator.py`,
`docs/pipuck_market_robot_controller.py`. Nichts davon wurde bisher gegen
den echten Broker oder echte Pi-Pucks getestet — wie bei jedem neuen
Backend in diesem Projekt, mit derselben Erwartung: es wird eine
Debugging-Runde brauchen.

## Deployment: welche Dateien, wie genau

**Kurz**: das **ganze Repo** pushen und auf jedem Pi-Puck klonen/pullen —
nicht einzelne Dateien kopieren. Genau der Workflow aus eurem eigenen
"Getting Started"-Dokument (Schritt 4/7), nur mit diesem Repo statt einem
neuen leeren `YourRepo`. Grund: `docs/pipuck_market_robot_controller.py`
importiert `allocation/`, `sim/`, `backends/`, `metrics/` als Python-Paket
— einzelne Dateien rauskopieren würde nur mit fehlenden Abhängigkeiten
crashen.

1. **Einmalig, von eurem Laptop aus**: dieses Repo (falls noch nicht auf
   GitHub) in ein Repo pushen, auf das ihr von den Pi-Pucks aus Zugriff
   habt (privates Repo reicht).
2. **Auf jedem Pi-Puck** (`ssh pi@pi-puck<id>`, Passwort `raspberry`):
   ```
   git clone <euer-repo-url> decentralized-mrta
   cd decentralized-mrta
   pip3 install paho-mqtt VL53L1X typing_extensions   # falls laut "Getting Started" noch nicht installiert
   ```
   `typing_extensions` ist nötig, weil aktuelle `paho-mqtt`-Versionen intern
   `typing.Literal` (erst ab Python 3.8) mit Fallback auf
   `typing_extensions.Literal` verwenden — auf Python 3.7 (Pi-Puck-Image)
   schlägt sonst schon der `import paho.mqtt.client` in `backends/pipuck.py`
   fehl (`ModuleNotFoundError: No module named 'typing_extensions'`).
   **Kein `pip3 install -e .`** — `pyproject.toml` verlangt `numpy`/`matplotlib`
   (für `experiments/`, hier ungenutzt und auf einem Pi Zero 2 W unnötig
   langsam zu bauen). Die vom Piloten importierten Module (`allocation`,
   `sim.config`, `sim.robot`, `metrics.logger`, `backends.pipuck`) sind
   reine Standardbibliothek; der `sys.path`-Fallback in beiden Skripten
   reicht aus. Ältere `pip`-Versionen auf Raspberry Pi OS scheitern zudem
   an `-e .` ohne `setup.py` ("Directory '.' is not installable") — auch
   deshalb einfach überspringen statt `pip` zu reparieren.
3. **Starten**: `python3 docs/pipuck_market_robot_controller.py r00` (bzw. `r01` auf dem zweiten Pi-Puck) — direkt aus dem geklonten Repo heraus, kein Kopieren einzelner Dateien nötig.
4. **`docs/pipuck_task_simulator.py`** läuft auf einem beliebigen Gerät im selben Netz (z. B. eurem Laptop) — dort genauso: Repo klonen/schon vorhanden, `pip install -e .`, dann direkt starten.
5. **Bei Änderungen** (z. B. `HEADING_OFFSET` nach der ersten Kalibrierung): auf dem Laptop ändern, committen, pushen — dann auf jedem Pi-Puck `git pull` **vor** dem nächsten Start. Nicht direkt auf dem Pi-Puck editieren, sonst gehen Änderungen beim nächsten `git pull` verloren (genau die Warnung aus eurem "Getting Started"-Dokument, Schritt 7).

**`REPO_ROOT` muss NICHT mehr manuell angepasst werden** — beide Skripte
erkennen ihren eigenen Pfad automatisch (`Path(__file__).resolve().parents[1]`),
solange sie aus dem geklonten Repo heraus laufen (nicht kopiert werden, wie
es bei den Webots-Controllern nötig war).

**Roboter-ID**: wird als Kommandozeilenargument übergeben
(`python3 ... r00`), steht **nicht** fest im Skript — dasselbe Skript läuft
unverändert auf beiden Pi-Pucks, nur der Aufrufparameter unterscheidet sich.
Was *einmalig* im Code angepasst werden muss (gilt für beide Pi-Pucks
gleichermaßen, da geteilte Datei): `TRACKING_ID_TO_ROBOT_ID` und
`PILOT_TASK_TRACKING_ID` in `backends/pipuck.py` — siehe Schritt 2 unten.

## 0. Vor dem Lab (ohne Hardware)

- [ ] `pip install -e .` in einer frischen venv, `python -m unittest discover -s tests` — alle Tests grün (54 Stück, inkl. `test_pipuck_backend.py`).
- [ ] `pip install paho-mqtt typing_extensions` auf jedem Gerät, das `backends/pipuck.py` importiert (Pi-Pucks + ggf. Laptop für `docs/pipuck_task_simulator.py`) — `typing_extensions` nur auf Python < 3.8 nötig (Pi-Puck-Image), s. Abschnitt "Deployment" oben.
- [ ] Auf den Pi-Pucks: Setup laut eurem eigenen "Getting Started"-Dokument bereits erledigt (`pi-puck`-Paket, `VL53L1X`, `paho-mqtt` installiert, Epuck2-Firmware geflasht, Selector auf `A`).
- [ ] Repo wie im Abschnitt "Deployment" oben beschrieben auf beide Pi-Pucks (und ggf. den Laptop) geklont — `REPO_ROOT` muss dafür nicht manuell angepasst werden.

## 1. Erste Minuten im Lab: drei Annahmen verifizieren

Alle drei sind im Code klar als `VERIFY` markiert, keine Blackbox:

- [ ] **Broker-IP**: Euer `client.py` nutzt `192.168.178.56`, das Setup-Dokument nennt `192.168.178.43` für Dashboard **und** MQTT. Mit dem Betreuer/der Betreuerin klären, welche IP aktuell läuft, dann `backends/pipuck.py`s `MQTT_BROKER` entsprechend setzen.
- [ ] **`robot_pos/all`-Schema**: Kurzer Empfangstest (z. B. euer `client.py` laufen lassen) — prüfen, ob ein empfangenes Payload wirklich `{"<id>": {"id": ..., "position": [x,y], "angle": <deg>}, ...}` entspricht. Falls die Feldnamen abweichen: nur `_parse_robot_pos_payload()` in `backends/pipuck.py` anpassen, der Rest der Datei bleibt unberührt.
- [ ] **Motor-Wertebereich**: `_MotorDriver` skaliert von `[-MAX_WHEEL_SPEED, MAX_WHEEL_SPEED]` (rad/s) auf die Firmware-Range `[-1000, 1000]` (laut e-puck2-Cheatsheet). Kurztest: `pipuck.epuck.set_motor_speeds(300, 300)` direkt in einer Python-Konsole auf dem Pi-Puck, beobachten wie schnell/langsam er wirklich fährt — falls unplausibel, den Skalierungsfaktor in `_MotorDriver.__init__` anpassen (nicht die restliche Bewegungslogik).

## 2. Identität zuordnen (Roboter **und** Hindernis)

Das RCPS-Tracking kennt keinen Unterschied zwischen Robotern und Hindernissen
— ein Pappzylinder mit Marker sieht für den Tracker aus wie ein ganz normaler
Roboter mit eigener ID. Deshalb zwei getrennte Zuordnungen in `backends/pipuck.py`, beide im Dashboard unter "Robot Info" ablesbar:

- [ ] `TRACKING_ID_TO_ROBOT_ID` auf die numerischen IDs eurer zwei Pi-Pucks setzen (z. B. `{22: "r00", 32: "r01"}`).
- [ ] `PILOT_TASK_TRACKING_ID` auf die numerische ID des Pappzylinders (GUARD-Objekt) setzen — **nicht** die Position von Hand ausmessen, die Position wird jetzt live aus `robot_pos/all` gelesen und erst beim tatsächlichen Erkennen eingefroren (`docs/pipuck_task_simulator.py`).

## 3. Netzwerk

- [ ] Beide Pi-Pucks und ggf. der Laptop (für `docs/pipuck_task_simulator.py`) im selben WiFi wie der MQTT-Broker.
- [ ] Smoke-Test: `docs/pipuck_task_simulator.py` starten, "Connected with result code 0" sollte erscheinen (nicht `5`/`Connection Refused`).

## 4. Physischer Aufbau

- [ ] GUARD-Objekt (Pappzylinder mit Marker, ID = `PILOT_TASK_TRACKING_ID` aus Schritt 2) irgendwo sichtbar in der Arena platzieren — Position muss **nicht** gemessen werden, wird live getrackt.
- [ ] Kurz im RCPS-Dashboard prüfen, dass diese ID auch tatsächlich als "Roboter" auftaucht und eine plausible Position zeigt.

## 5. Software starten (Reihenfolge wichtig)

1. [ ] `python3 docs/pipuck_task_simulator.py` starten. Erwartet: "Connected with result code 0", danach "Task simulator running...".
2. [ ] `python3 docs/pipuck_market_robot_controller.py r00` auf Pi-Puck 1.
3. [ ] `python3 docs/pipuck_market_robot_controller.py r01` auf Pi-Puck 2.
4. [ ] Beide Roboter-Terminals sollten `STRATEGY_NAME=<StrategyName.MARKET: 'market'>` und kurz danach `pose received, starting.` zeigen — **falls nicht**, Abschnitt 1 nochmal prüfen, bevor irgendetwas fährt.

## 6. Smoke-Tests, aufsteigende Komplexität

- [ ] **Nur Posen**: Task-Simulator + einen Roboter-Controller laufen lassen, Objekt noch außerhalb der `DETECTION_RADIUS` (0,3 m). Roboter sollte `EXPLORE` halten und stillstehen. Prüft: MQTT-Kette funktioniert, keine Bewegung ohne Grund.
- [ ] **Ein Roboter, eine Aufgabe**: Objekt in Reichweite bringen (oder Roboter manuell nah heranschieben), nur EINEN Roboter-Controller laufen lassen. Erwartet: `DETECT` in der Task-Simulator-Konsole, `hosts auction` beim Roboter, `AWARD`, Roboter fährt Richtung Objekt, `ACTIVE` nach Ankunft, nach 15s `DONE`.
- [ ] Falls der Roboter **nicht geradeaus** zum Ziel fährt, sondern im Kreis: derselbe `HEADING_OFFSET`-Kalibrierungsfehler wie bei Webots (siehe `backends/pipuck.py`s Kommentar) — mit derselben Methode beheben (echte Fahrtrichtung vs. berechnete Peilung über mehrere Messpunkte vergleichen). **Das ist der wahrscheinlichste Stolperstein.**
- [ ] **Beide Roboter**: vollständiger Pilotlauf wie in Abschnitt 5.

## 7. Der eigentliche Pilotlauf

- [ ] Beide Roboter-Controller + Task-Simulator laufen lassen, bis `DONE` in allen drei Terminals erscheint.
- [ ] Erfolgskriterium ist binär, nicht statistisch: **ein** vollständiger Zyklus `ANNOUNCE → BID → AWARD → ACTIVE → DONE` zwischen zwei echten Pi-Pucks über das reale Netzwerk, mit der unveränderten `allocation/`-Logik.
- [ ] `pipuck_logs/state_r00.csv`, `state_r01.csv` sichern (liegen im jeweiligen Arbeitsverzeichnis der Roboter-Controller).

## 8. Fallback-Plan

Alles hier ist bereits im Proposal (§9) als akzeptabler Rückfall benannt:

- **Broker-IP/Schema falsch** → Abschnitt 1, nur zwei kleine Funktionen betroffen, kein struktureller Umbau.
- **`HEADING_OFFSET` falsch** → mit derselben Methode wie bei Webots neu bestimmen.
- **Roboter zu schnell/unstabil** → `CRUISE_SPEED` in `backends/pipuck.py` weiter senken (aktuell `2.0`, konservativer als Webots' `3.0`).
- **Nichts läuft rechtzeitig zusammen** → ehrliches, berichtbares Ergebnis (Proposal §10: "A null result on any of these is acceptable and will be reported as such"). Für den Bericht zählt: welcher Schritt genau fehlgeschlagen ist und warum.

## 9. Was das für Tier 0/1 "validiert"

Kein neuer Strategievergleich. Der Beitrag zum Bericht: **dieselbe,
unveränderte dezentrale Auktionslogik (`allocation/`)**, die in Tier 0
(statistisch abgesichert) und Tier 1 (Einzellauf, Webots) dieselbe
Rangfolge zeigte, läuft auch auf echter Hardware end-to-end durch — die
dritte und letzte Stufe der im Proposal (§8) versprochenen
Drei-Tier-Methodik. Für den Bericht: eigener, ehrlich gekennzeichneter
Tier-2-Abschnitt (Ergebnis binär: lief / lief nicht, plus genau, was dafür
kalibriert werden musste), nicht als weitere Zeile in der Vergleichstabelle.
