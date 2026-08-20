# Tier 1: Marktlogik in Webots gegentesten

Dieser Schritt testet nicht neue Logik — er lässt den **unveränderten** `allocation/`-Code
(Bieten, Auktion, Nachrichten, Strategien) mit den bereits gitterweit besten Gewichten
(`results/best_parameters.json`) auf echten (simulierten) Pi-Puck-Dynamiken laufen, statt auf
der abstrakten Tier-0-Punktkinematik. Ziel: prüfen, ob die in `comparison_summary.md`
gemessene Rangfolge (Market > Nearest Greedy > Random) unter realer Fahrdynamik erhalten
bleibt — genau die Frage, die das Proposal (§10) als „most valuable result" bezeichnet.

**Umfang dieser Stufe** (siehe Konversation): Bieten/Entscheiden läuft dezentral in `FLEET_SIZE`
unabhängigen Roboter-Prozessen — das ist der eigentliche Claim des Projekts. Die Prüfung, ob
`n_req` Gewinner an einer Aufgabe angekommen sind, sowie RoI-/SURVEY-Erkennung laufen zentral
über einen zusätzlichen Supervisor-Prozess (die „Overhead-Kamera" aus dem Proposal, §2.1) —
das ist in Tier 0 ohnehin schon zentral (`World`), hier nur über Funk statt In-Process-Zugriff.
`allocation/` wurde dafür **nicht** verändert.

## Was neu ist

| Datei | Rolle |
|---|---|
| [`backends/webots.py`](../backends/webots.py) | `WebotsBackend`: Bewegung (Rotate-then-Cruise), Batterie, Funk-Routing. Getestet in `tests/test_webots_backend.py`. |
| [`sim/robot.py`](../sim/robot.py), [`sim/world.py`](../sim/world.py) | Optionaler `backend`-Parameter (Default `None` = Tier 0 exakt unverändert, gegen alle 20 Tests geprüft). |
| [`webots_market_robot_controller.py`](webots_market_robot_controller.py) | Läuft **einmal pro Flotten-Roboter** als eigener Webots-Prozess. |
| [`webots_market_supervisor_controller.py`](webots_market_supervisor_controller.py) | Läuft **einmal**, auf einem zusätzlichen, motorlosen Supervisor-Knoten. |

**Wichtig:** Diese beiden Controller-Skripte sind — wie schon `webots_calibration_controller.py`
— *unteste* Ausgangspunkte, hier liegt kein Webots vor. Die reine Logik (Nachrichten-Routing,
Task-Konstruktion, Zustandsmaschine der Bewegung) ist mit Fake-Devices getestet
(`tests/test_webots_backend.py`), aber das Zusammenspiel in echten Webots-Prozessen nicht.
Rechnet mit einer weiteren Debugging-Runde, genau wie bei der Kalibrierung.

## Welt-Setup

1. **`FLEET_SIZE` Roboter** (Standard 7, muss mit `parameters/default.json`s `robot_count`
   übereinstimmen), jeweils:
   - `name` Feld exakt `"r00"`, `"r01"`, … (Reihenfolge egal, muss nur eindeutig sein). Kein
     `DEF`-Label nötig — der Supervisor findet Roboter über eine rekursive Suche nach diesem
     `name`-Feld (`_find_node_by_name` in `webots_market_supervisor_controller.py`), nicht über
     `getFromDef`. Das Textdatei-Bearbeiten von DEF-Labels hat sich in Tests als riskant
     erwiesen (kann die Welt zum Absturz bringen, vermutlich wegen der EXTERNPROTO-Einbindung)
     und wird deshalb komplett vermieden.
   - `supervisor TRUE` (nur Selbstinspektion der eigenen Pose, siehe `backends/webots.py`)
   - Funkkanal: manche E-puck-PROTOs (z. B. die mit Kamera/Batterie) bringen bereits
     `emitter_channel`/`receiver_channel`-Felder mit — dann nur sicherstellen, dass diese bei
     **allen** Robotern und der Kamera denselben Wert haben (z. B. `1`). Falls nicht vorhanden:
     `Emitter`- und `Receiver`-Kind-Node manuell hinzufügen, beide `name "emitter"`/`"receiver"`,
     gleicher Kanal überall.
   - `controller "webots_market_robot_controller"` (dieselbe Datei für alle Roboter — Webots
     startet dafür automatisch einen eigenen Prozess pro Knoten)
2. **Ein zusätzlicher Supervisor-Knoten** ohne Motoren (z. B. ein einfacher `Robot`-Node mit
   `supervisor TRUE`, `Emitter`/`Receiver` auf demselben Kanal), `controller
   "webots_market_supervisor_controller"`.
3. Arena weiterhin `2×2 m` (`RectangleArena.floorSize = 2 2`), wie bei der Kalibrierung.

## Laufen lassen

1. Alle `FLEET_SIZE + 1` Controller-Dateien aus `docs/` in entsprechend benannte Webots-Controller-
   Verzeichnisse kopieren (`controllers/webots_market_robot_controller/…`,
   `controllers/webots_market_supervisor_controller/…`).
2. Play. Läuft `config.duration` Sekunden (Standard 300 s) — mit Fast-Forward vertretbar.
3. Ergebnis: `webots_logs/<strategy>/events.csv` (vom Supervisor) +
   `webots_logs/<strategy>/state_r00.csv` … `state_r06.csv` (je Roboter), wobei `<strategy>`
   automatisch `STRATEGY_NAME.value` ist (`market`/`nearest_greedy`/`random_assignment`) — siehe
   `LOG_DIR`s Kommentar in beiden Controller-Dateien. **Wichtig**: `LOG_DIR` war früher ein
   flacher `webots_logs`-Ordner ohne Strategie-Unterordner — jeder neue Lauf hat die Dateien des
   vorherigen kommentarlos überschrieben (ein kompletter Market-Lauf ist so verlorengegangen).
   Seit dem Fix bekommt jede Strategie automatisch ihren eigenen Unterordner, nichts mehr von
   Hand umbenennen.

## Logs zusammenführen und auswerten

Pro Strategie (Ordnername = `STRATEGY_NAME.value`, z. B. `market`, `nearest_greedy`,
`random_assignment`):

```powershell
python -c "
import csv, sys
from pathlib import Path
strategy = sys.argv[1]  # z.B. 'market'
log_dir = Path('webots_logs') / strategy
rows = []
for f in sorted(log_dir.glob('state_r*.csv')):
    with f.open(newline='', encoding='utf-8') as stream:
        rows.extend(csv.DictReader(stream))
rows.sort(key=lambda r: float(r['t']))
with (log_dir / 'state.csv').open('w', newline='', encoding='utf-8') as stream:
    writer = csv.DictWriter(stream, fieldnames=rows[0].keys(), lineterminator='\n')
    writer.writeheader()
    writer.writerows(rows)
print(f'merged {len(rows)} state rows')
" market

python -c "
from metrics.analysis import analyse_logs
import json
print(json.dumps(analyse_logs('webots_logs/market/events.csv', 'webots_logs/market/state.csv'), indent=2))
"
```

Dieselbe `metrics/analysis.py`-Funktion, die auch Tier-0-Logs auswertet — direkter Vergleich der
Zahlen mit `results/comparison_summary.md` möglich (gleiches Schema laut `docs/ARCHITECTURE.md`).
Für Greedy/Random dieselben zwei Kommandos mit `nearest_greedy`/`random_assignment` statt `market`.

## Vergleichslauf: Market vs. Baselines

`STRATEGY_NAME` in **beiden** Controller-Dateien (Roboter **und** Supervisor, müssen
übereinstimmen) auf `StrategyName.GREEDY` bzw. `StrategyName.RANDOM` umstellen, Welt neu laden,
denselben `SEED` verwenden (Standard 1000, ein Tier-0-Holdout-Seed) für einen fairen
Vergleich auf demselben Szenario — genau wie in Tier 0 auf `README.md`s Seed-Trennung. Dank des
Strategie-Unterordners (siehe oben) bleiben alle drei Läufe diesmal gleichzeitig auf der Platte.

**Achtung Webots-Editor**: Wenn eine der beiden Controller-Dateien in Webots' eingebautem
Text-Editor offen ist/war, zeigt der Tab möglicherweise noch eine alte Version — ein Speichern
aus diesem Tab überschreibt dann die frisch aktualisierte Datei wieder mit dem alten Stand
(genau das ist einmal passiert: ein `ImportError` für längst entfernten Code). Tab schließen und
neu öffnen (oder Webots neu starten), bevor dort etwas geändert wird.

## Formulierungshilfe für den Bericht

> Stufe 1 (Tier 0) diente dazu, die Kernhypothese — dass dezentrale marktbasierte Allokation
> gegenüber Nearest-Greedy und Random im operationalen Score überlegen ist — an einem schnellen,
> abstrakten Simulator zu prüfen und über eine Gittersuche (24 Parameterkombinationen × 8
> Trainings-Seeds) nahezu optimale Bid-Gewichte zu bestimmen. Die Holdout-Auswertung auf 40
> zuvor ungesehenen Seeds bestätigte die Hypothese mit statistischer Signifikanz
> (`comparison_summary.md`). Als nächste Stufe wird dieselbe, unveränderte Entscheidungslogik
> (`allocation/`) mit den in Tier 0 gefundenen Gewichten gegen reale (in Webots simulierte)
> Fahrdynamik gegengetestet, um zu prüfen, ob die gemessene Rangfolge auch unter Trägheit,
> Beschleunigungsgrenzen und nicht perfekt geradliniger Bewegung erhalten bleibt.

Ehrlich dazusagen (passt zum bestehenden Offenlegungs-Stil des Projekts): Die
Ankunfts-/Erkennungs-Synchronisation läuft in dieser Stufe noch über einen zentralen
Supervisor-Prozess statt vollständig dezentral über Funk — eine bewusste Vereinfachung für
diese Stufe, keine Änderung an der eigentlichen (dezentralen) Bid-/Auktionslogik.

## PUSH: echte Physik versucht, dann zurückgebaut

Ein Zwischenstand hatte PUSH-Objekten echte Kollisionsgeometrie (`boundingObject`/`physics`) und
eine positionsbasierte Zielzonen-Prüfung gegeben, statt des einfachen Duration-Timers. Live
getestet: Die Roboter fuhren zuverlässig zum Objekt, konnten es aber praktisch nicht bewegen —
Webots' Standard-Kontaktreibung übersteigt vermutlich die schwache Schubkraft eines leichten
E-Pucks bei jeder plausiblen Objektmasse. Ein Fix (`WorldInfo.contactProperties` mit niedrigem
`coulombFriction`) wurde nicht verfolgt: PUSH ist laut Proposal ohnehin von der Hardware-Demo
ausgeschlossen ("Push will not be demonstrated on hardware... requires force closure we do not
believe we can achieve reliably"), und GUARD/SURROUND/SURVEY liefern bereits, was Tier 1 zeigen
soll. Bewusst zurückgebaut, nicht nur liegen gelassen — der komplette
Physik-/Zielzonen-/Schub-Code (`push_approach_point()`, `push_target_position()`,
`CTL_PUSH_TARGET`, `spawn_zone_marker()`, das EXECUTE-Override im Roboter-Controller) ist wieder
raus.

PUSH läuft jetzt exakt wie GUARD/SURROUND: rein visuelles Objekt (kein `boundingObject`/`physics`),
Abschluss über denselben Duration-Timer (`n_req` Gewinner halten `arrival_tolerance` für
`duration` Sekunden, dann `DONE`, Objekt verschwindet). Kein Sonderfall mehr im Code.

## Durchsatz-Fix: `assignment_timeout` und `CRUISE_SPEED`

Erster Live-Lauf mit PUSH-Objekten: In 300 s erreichte nur **1 von ~30** angekündigten Aufgaben
je `ACTIVE` (eine GUARD-Aufgabe mit `n_req=1`) — **keine einzige** PUSH-Aufgabe. Ursache: Die
90 s aus `parameters/default.json`s `assignment_timeout` sind auf Tier 0s abstrakte
`effective_speed=0.08 m/s` kalibriert; Webots' reale Fahrgeschwindigkeit (`CRUISE_SPEED=3.0
rad/s ≈ 0.0615 m/s`) war sogar **langsamer** als das, plus Rotations-/Ramp-up-/Realign-Overhead,
den Tier 0 gar nicht kennt. Bei `workload_cap=3` und `assignments[0]`-only-Navigation (unverändertes
Tier-0-Verhalten) hat ein Roboter mit mehreren Aufgaben in der Warteschlange praktisch nie eine
Chance, die 2./3. Aufgabe überhaupt zu erreichen, bevor ihr Timeout abläuft — und PUSH/SURROUND
(`n_req>1`) brauchen zusätzlich *mehrere* Gewinner gleichzeitig innerhalb dieses Fensters.

Eine gezielte Anpassung (**nicht** in `allocation/`, dokumentiert als Kalibrierungswert, nicht
als Logikänderung):

- `webots_market_supervisor_controller.py`: `ASSIGNMENT_TIMEOUT_OVERRIDE_S = 240.0`, per
  `dataclasses.replace()` auf die geladene Config angewendet (sie ist `frozen=True`) — betrifft
  nur diesen Prozess, `parameters/default.json` und damit `comparison_summary.md` bleiben
  unangetastet, da `assignment_timeout` ausschließlich in `sim/world.py`s
  `_update_execution()` gelesen wird (nie in `allocation/`).

**`CRUISE_SPEED` NICHT anfassen** — ein erster Versuch, ihn von 3.0 auf 4.0 rad/s zu erhöhen
(um die Lücke zu Tier 0s `effective_speed=0.08 m/s` zu schließen), wurde nach einem Live-Lauf
wieder zurückgedreht: 4 von 7 Robotern froren am Ende bei exakt der Arenagrenze minus
`arrival_tolerance` fest (z. B. `x=1.963`, mehrfach identisch), null Aufgaben erreichten
`ACTIVE` — schlechter als vorher. 3.0 rad/s bleibt der einzige empirisch als geradeauslauf-stabil
validierte Wert (`docs/WEBOTS_CALIBRATION.md`s Isolationstest).

Auch mit dem `assignment_timeout`-Fix allein: weiterhin 0 `ACTIVE`-Events in 300s, und zwar für
**alle** Aufgabentypen, nicht nur PUSH. Eigentliche Ursache (siehe `sim/robot.py`): `current_task_id
= assignments[0]`, und alle `preemption_cooldown` Sekunden (Default 15.0) kann eine neu auftauchende,
höher priorisierte Aufgabe die aktuelle **komplett verwerfen** (`RELEASE reason="preempted"`,
gesamter bisheriger Reiseweg weg). Bei Tier 0s fast-augenblicklicher Bewegung ist das billig; bei
Webots' realer Fahrzeit (~35–45s pro Diagonalbein) wird ein Roboter im Schnitt 2–3× umgeleitet,
bevor er ankommt — praktisch garantiert, dass nichts fertig wird.

Fix, gleiches Muster wie `ASSIGNMENT_TIMEOUT_OVERRIDE_S` (reine Zeitkalibrierung, nicht in
`allocation/`, `parameters/default.json` bleibt unangetastet): in
`webots_market_robot_controller.py` (nicht im Supervisor — `preemption_cooldown` wird nur in
`sim/robot.py`s `Robot._on_announce()` gelesen, also pro Roboterprozess, nicht zentral) via
`PREEMPTION_COOLDOWN_OVERRIDE_S = 90.0` (6× Default), per `dataclasses.replace()` angewendet.
Preemption bleibt grundsätzlich aktiv (Kernbestandteil der Marktlogik), nur seltener auslösbar.

Falls danach immer noch kaum etwas `ACTIVE` wird: `workload_cap` senken (ändert allerdings die
Bid-Kostenfunktion, also die eigentliche Allokation — mit Bedacht) oder beide Override-Werte
weiter erhöhen — nicht an der Fahrgeschwindigkeit drehen (siehe oben).

## Isolationstest: ein Objekt, drei Roboter (`TEST_SINGLE_OBJECT`)

Selbst mit dem Preemption-Fix: im Isolationstest (`TEST_SINGLE_OBJECT=True`, ein einzelnes
SURROUND-Objekt, keine Konkurrenzaufgaben, keine Preemption möglich) brauchten die 3 Gewinner
trotzdem die vollen 240s bis `COORD_TIMEOUT` — nie `ACTIVE`. Die Positionslogs zeigten: **konstanter
Abstand zum Ziel** (~1.0m) über die gesamte Laufzeit, bei gleichmäßig im Uhrzeigersinn abnehmendem
Winkel — die Roboter umkreisten das Ziel exakt, statt sich ihm zu nähern. Klassische Signatur einer
konstanten ~90°-Verzerrung zwischen berechneter Peilung und tatsächlicher Fahrtrichtung.

Rechnerisch aus den Positionsdaten bestätigt (an 3 unabhängigen Robotern): tatsächliche Fahrtrichtung
≈ Peilung + 90°. Das bedeutet `HEADING_OFFSET` (in `backends/webots.py`) war **falsch** — `-π/2`
(aus der ursprünglichen Einzelroboter-Kalibrierung) auf `0` korrigiert. Warum die alte Kalibrierung
einen anderen Wert ergab, ist ungeklärt (andere Welt-Datei, anderes initiales `rotation`-Feld, oder
ein Test, der nur kurze Strecken prüfte, wo ein 90°-Fehler nicht auffällt) — falls
`WEBOTS_CALIBRATION.md`s Schritt 8 erneut einen anderen Wert liefert, gilt die frische Messung.

Dieser Fix betrifft **jede** Bewegung, nicht nur den Isolationstest — sollte also auch den
allgemeinen Durchsatz (siehe oben) grundlegend verbessern, unabhängig von `assignment_timeout`/
`preemption_cooldown`.

## Ankunftstoleranz: Tier 0s Punkt-Roboter vs. echte Roboterkörper

Mit dem Heading-Fix: die 3 SURROUND-Gewinner fahren jetzt tatsächlich zur Zielmitte statt sie zu
umkreisen — landen aber bei 0.035/0.041/0.060 m Abstand, nie alle gleichzeitig innerhalb
`arrival_tolerance=0.035`. Grund: Ein E-Puck hat ~0.037 m physischen Radius — schon größer als
die Toleranz selbst. Bei `n_req>1` auf demselben `task.position` (SURROUND besonders: Tier 0 kennt
keine Formation/Ring um ein Ziel, nur einen gemeinsamen Punkt) blockieren sich die Gewinner
gegenseitig, bevor alle so nah rankommen. Packungs-Untergrenze für n Kreise mit Radius r um einen
gemeinsamen Mittelpunkt: `r/sin(π/n)` ≈ 0.043 m für n=3 — passt zu den beobachteten Werten.

Fix (wieder reine Tier-1-Kalibrierung, `parameters/default.json` unangetastet):
`backends/webots.py`s `ARRIVAL_TOLERANCE` von 0.035 auf 0.08 m angehoben, per
`dataclasses.replace()` sowohl in `webots_market_robot_controller.py` als auch
`webots_market_supervisor_controller.py` auf `config.arrival_tolerance` angewendet (beide müssen
übereinstimmen, sonst sind sich Robot-Zustand und Supervisor-Ready-Check uneinig).

## Ergebnis: GUARD/SURROUND/SURVEY/PUSH laufen alle über denselben Mechanismus

Mit allen obigen Fixes: alle Aufgabentypen erreichen im vollen 28-Event-Lauf zuverlässig
`ACTIVE`/`DONE` über denselben Mechanismus — `n_req` Gewinner erreichen `arrival_tolerance`,
halten `duration` Sekunden, das Objekt verschwindet, `DONE`. PUSHs Real-Physik-Versuch (siehe
oben) wurde bewusst zurückgebaut statt weiter getunt, seit GUARD/SURROUND/SURVEY zeigen, was
Tier 1 eigentlich zeigen soll, und PUSH laut Proposal ohnehin nicht auf Hardware demonstriert
wird. Completion-Zählung pro Strategie treibt direkt das Scoring aus `metrics/analysis.py` —
nichts Neues nötig, dieselbe Auswertung wie bei Tier 0.

`TEST_SINGLE_OBJECT` ist zurück auf `False` — der Isolationstest hat seinen Zweck erfüllt
(fand `HEADING_OFFSET` und `ARRIVAL_TOLERANCE` als eigentliche Ursachen), jetzt läuft wieder die
volle Tier-0-Kampagne, wie es für den eigentlichen Market-vs-Baseline-Vergleich nötig ist.
