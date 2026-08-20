# Vergleichs- und Nachweismethodik

## Fragestellung

Der Vergleich prüft nicht die unhaltbare Behauptung, Market müsse jede Metrik schlagen.
Nearest Greedy ist auf die lokale Entfernung optimiert; Random kann zufällig eine sehr
gleichmäßige Last erzeugen. Geprüft wird, ob die dezentrale Marktstrategie den vorab
dokumentierten operationalen Gesamtnutzen verbessert und auf welchen Einzelmetriken der
Vorteil oder Preis entsteht.

## Trennung von Auswahl und Nachweis

- Trainings-Seeds: `0-7`
- Holdout-Seeds: `1000-1039`
- Parametersuche: Gitter über `beta`, `gamma`, `delta`, `theta_hyst`
- Strategievergleich: dieselben 40 Szenario-Seeds für alle vier Strategien

Die Parameter sehen die Holdout-Seeds nicht. Dadurch wird der Nachweis nicht auf denselben
Zufallsfällen geführt, auf denen die Konfiguration gewählt wurde.

## Operationaler Score

Der Score dient nur dazu, einen Punkt auf der Pareto-Front auszuwählen. Seine Gewichte
stehen maschinenlesbar in `results/benchmark_manifest.json`:

| Komponente | Gewicht |
|---|---:|
| Completion Rate | 0,30 |
| Detection Rate | 0,10 |
| Bearbeitungszeit | 0,10 |
| Lastbalance | 0,15 |
| Coverage | 0,05 |
| mittlere Idleness | 0,08 |
| Detektionslatenz | 0,07 |
| Restakku | 0,05 |
| keine Depletion | 0,10 |

Alle Komponenten werden vor der Gewichtung auf `[0,1]` begrenzt. Der Score ersetzt die
Einzelmetriken nicht; jede Einzelmetrik wird separat berichtet.

## Statistik

Für jede Baseline wird pro Seed die gerichtete Differenz berechnet, sodass ein positiver
Wert immer „Market besser“ bedeutet. Berichtet werden:

- Mittelwert der gepaarten Differenzen
- nichtparametrisches 95%-Bootstrap-Konfidenzintervall mit 20.000 Resamples
- gepaarter Vorzeichen-Randomisierungstest mit 20.000 Ziehungen
- Holm-Korrektur über alle berichteten Metriken je Baseline
- Win-Rate über die 40 Seeds

Ein „Vorteil“ wird nur ausgewiesen, wenn das 95%-Intervall vollständig positiv und der
Holm-korrigierte p-Wert kleiner als 0,05 ist. Analog gilt dies für einen Nachteil.

## Interpretation

Im Holdout ist der Score-Vorteil gegenüber Nearest Greedy und Random signifikant. Gegen
Nearest Greedy sind insbesondere Bearbeitungszeit und Lastbalance besser. Gegen Random
sind Completion Rate, Bearbeitungszeit sowie mittlere und maximale Idleness besser.

Market verursacht gegenüber Nearest Greedy im Mittel etwa zwei zusätzliche Nachrichten
pro abgeschlossener Aufgabe; nach Mehrfachkorrektur ist dieser Unterschied knapp nicht
signifikant. Greedy ohne SURVEY hat erwartungsgemäß viel weniger Nachrichten und in
diesem Boustrophedon-Szenario niedrigere Idleness, verliert aber bei Completion,
Bearbeitungszeit und Lastbalance. Dieser Trade-off wird nicht in den Score hineinerklärt,
sondern offen berichtet.

## Gültigkeitsbereich

Die Evidenz gilt für Tier 0 und die dokumentierte Szenariofamilie. Sie beweist noch nicht,
dass die Rangfolge in Webots oder auf Pi-Pucks erhalten bleibt. Der nächste wissenschaftlich
sinnvolle Schritt ist deshalb Kalibrierung von `effective_speed`, `startup_delay`, Funkverlust
und Batteriemodell gegen Tier 1 sowie ein erneuter gepaarter Vergleich.
