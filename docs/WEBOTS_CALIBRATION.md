# Mini-Webots-Kalibrierung (Tier 0 → Tier 1)

Ziel: `startup_delay` (`a`) und `effective_speed` (`v_eff`) in
[parameters/default.json](../parameters/default.json) gegen echte Fahrdynamik in Webots
kalibrieren, bevor der Code auf die physischen Pi-Pucks geht. Das ist der minimale Teil von
Tier 1 aus dem Proposal (§8) — **keine** vollständige Kollisions-/Sensorrausch-Validierungsstudie,
nur die Kalibrierung, die laut Proposal die Tier-0-Zeiten "defensible rather than invented"
macht.

Nicht Ziel dieses Schritts: Strategien in Webots gegeneinander laufen lassen. Das bleibt offen
als Fortsetzung, siehe [METHODOLOGY.md](METHODOLOGY.md#gültigkeitsbereich).

## 1. Messprotokoll

- **Roboter:** ein Pi-Puck/e-puck2 in Webots, PROTO wie später im Lab verwendet.
- **Läufe:** mindestens 20 Punkt-zu-Punkt-Fahrten, Distanzen über den gesamten Bereich der
  2×2-m-Arena verteilt (0,15 m bis zur Diagonalen ≈ 2,83 m), nicht nur kurze Strecken — sonst
  ist die Extrapolation auf lange Wege ungestützt.
- **Zeitmessung:** vom Fahrbefehl (`drive_to`) bis zum Erreichen von `arrival_tolerance` (0,035 m
  laut `default.json`) um das Ziel — exakt dieselbe Semantik wie
  [`Robot.is_at_target()`](../backends/abstract.py) im Tier-0-Backend, damit der Fit vergleichbar ist.
  Kein Timer für Rotation extra: Wenn das Fahrverhalten erst dreht und dann fährt, gehört das in
  die gemessene Zeit, weil `drive_to` auf Tier 0 auch nur eine Endzeit kennt.
- **Wiederholungen:** falls Zeit reicht, 2–3 Wiederholungen je Distanz mitloggen statt nur 20
  verschiedene Distanzen einmal — verbessert die Bootstrap-CI im Fit spürbar.
- **Output:** eine CSV mit Header `distance_m,elapsed_s`, eine Zeile pro Lauf.

## 2. Webots-Controller

[`webots_calibration_controller.py`](webots_calibration_controller.py) ist ein Startpunkt, kein
fertig getesteter Controller — dieses Environment hat kein Webots, das Skript ist nicht gegen
eine echte Pi-Puck-PROTO gelaufen. Vor Gebrauch prüfen:

- Gerätenamen (`getDevice(...)`) gegen die tatsächliche PROTO-Definition (Standard-e-puck in
  Webots nutzt `"left wheel motor"`/`"right wheel motor"`; die Pi-Puck-Erweiterung kann abweichen)
- `WHEEL_RADIUS`, `AXLE_LENGTH`, `MAX_WHEEL_SPEED` gegen das reale Datenblatt
- Der Robot-Node braucht das Feld `supervisor: TRUE`, damit der Controller per
  `self.getSelf().getPosition()` die Ground-Truth-Pose lesen kann — das steht stellvertretend für
  die Overhead-Kamera aus dem Proposal (§2.1), nicht für ein reales On-Board-GPS.

## 3. Fit ausführen

```powershell
python -m experiments.calibrate_travel_model --runs logs\webots_calibration.csv
```

Gibt `a`, `v_eff`, 95%-Bootstrap-CI (20.000 Resamples, gleiche Methodik wie
`experiments/compare.py`) und R² aus. Bei R² < 0,8 ist das lineare Modell `t = a + d/v_eff` ein
schlechter Fit — dann eher nach Ausreißern oder einem nichtlinearen Anlaufverhalten suchen als
den Wert blind zu übernehmen.

Mit `--apply` direkt in `parameters/default.json` schreiben:

```powershell
python -m experiments.calibrate_travel_model --runs logs\webots_calibration.csv --apply
git diff parameters\default.json
```

Der Patch ändert ausschließlich `startup_delay` und `effective_speed`, alle anderen Werte
bleiben Zeile für Zeile unverändert.

## 4. Danach: erneuter Holdout-Vergleich

```powershell
python -m experiments.compare
```

Prüfen, ob Market weiterhin gegenüber Nearest Greedy und Random gewinnt (siehe
[comparison_summary.md](../results/comparison_summary.md)). Bleibt die Rangfolge stabil, ist das
ein Beleg, dass sie nicht von den ungeprüften Tier-0-Zeitannahmen abhängt. Ändert sie sich, ist
das nach Proposal §10 selbst das interessantere Ergebnis und gehört in den Bericht, nicht unter
den Tisch.

## 5. Report-Update

Im technischen Bericht (`Technischer_Zwischenbericht_Stufe_1.tex`) die alten/neuen Werte für
`startup_delay`/`effective_speed`, die Anzahl und Streuung der Webots-Läufe sowie R² dokumentieren
und explizit vermerken, dass dies eine Kalibrierung, keine vollständige Tier-1-Validierung
(Kollisionen, Sensorrauschen) ist — analog zur bestehenden Offenlegung in
[METHODOLOGY.md](METHODOLOGY.md#gültigkeitsbereich).
