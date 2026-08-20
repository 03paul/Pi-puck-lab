# Architektur und dezentrales Protokoll

## Abhängigkeitsregel

`allocation/` enthält die auf Tier 0, Webots und Pi-Pucks identisch verwendbare Logik.
Sie hängt nur von der Python-Standardbibliothek ab. Die Simulationsschicht darf
`allocation/` importieren; die umgekehrte Richtung ist verboten und wird in
`tests/test_simulation.py` geprüft.

```text
allocation/  <-  sim/  <-  experiments/
     ^             ^
     |             |
 backends/      metrics/
```

Der Pfeil zeigt jeweils auf die verwendete Schicht. Später ersetzen `webots.py` und
`pipuck.py` nur den Backend-Adapter.

## Auktionsablauf

1. Ein Roboter detektiert eine RoI oder ist für das Ausschreiben einer veralteten
   SURVEY-Zelle verantwortlich.
2. Dieser Roboter wird Auctioneer und broadcastet `ANNOUNCE` mit der vollständigen
   Prioritätsfunktion `(p0, lambda, k, t_ref)` und einer Deadline.
3. Jeder erreichbare Roboter wertet seinen lokalen Zustand aus und broadcastet `BID`.
4. Der Auctioneer sortiert `(cost, robot_id)` aufsteigend und broadcastet `AWARD` für die
   `n_req` günstigsten Gebote.
5. Die Gewinner navigieren. Erst wenn genau `n_req` Gewinner am Ziel sind, startet die
   physische Ausführung.
6. Abschluss, Rückgabe oder Ausfall werden als `DONE` beziehungsweise `RELEASE`
   broadcastet. Eine Rückgabe löst beim ursprünglichen Auctioneer eine neue Auktion aus.

Der Funkkanal modelliert unabhängig pro Empfänger Latenz und Paketverlust. Random,
Nearest Greedy und Market verwenden dasselbe Protokoll; nur die lokale Kostenfunktion
unterscheidet sich. Damit ist der Kommunikationsvergleich fair.

## Kosten und Priorität

Die drei Gebotsterme sind auf `[0,1]` normiert. Priorität liegt absichtlich nicht im Gebot,
weil sie innerhalb derselben Auktion alle Gebote nur um dieselbe Konstante verschieben
würde. Sie wirkt auktionsübergreifend über Preemption. Diese ist ausschließlich aus
`NAVIGATE`, nie aus `WAIT_PEERS` oder `EXECUTE`, und höchstens einmal je 15 Sekunden
erlaubt. SURVEY zählt nicht zur Auslastung und wird für eine RoI ohne Hysterese freigegeben.

## Determinismus

Szenario, Funk und Random-Baseline haben getrennte, aus dem Lauf-Seed abgeleitete
`random.Random`-Instanzen. Damit bleibt das physische Szenario zwischen Strategien gleich,
und ein identischer Seed erzeugt byte-identische `events.csv`-Logs.
