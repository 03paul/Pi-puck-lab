# Holdout-Vergleich: dezentrale Marktauktion vs. Baselines

Auswertung auf 40 zuvor nicht zur Parametersuche verwendeten Seeds (1000-1039), jeweils als gepaarter Vergleich desselben Szenarios.

## Gewählte Parameter

`alpha=1.0`, `beta=0.5`, `gamma=0.0`, `delta=1.5`, `theta_hyst=0.1`. Die Trainingssuche ergab 14 nicht-dominierte Parameterpunkte; der gewählte Punkt maximiert den vorab dokumentierten operationalen Score.

## Mittelwerte auf dem Holdout-Set

| Strategie | Score | Completion | Zeit [s] | Load-SD | Max Idleness [s] | Detektionslatenz [s] | Msg/Task |
|---|---:|---:|---:|---:|---:|---:|---:|
| market | 0.861 | 0.918 | 37.76 | 1.42 | 264.99 | 15.84 | 40.26 |
| nearest_greedy | 0.831 | 0.887 | 47.84 | 1.84 | 267.09 | 14.70 | 38.28 |
| nearest_greedy_no_survey | 0.813 | 0.861 | 46.98 | 2.09 | 238.70 | 17.11 | 10.70 |
| random_assignment | 0.813 | 0.788 | 59.60 | 1.35 | 284.29 | 15.15 | 43.72 |

## Gepaarte Effekte zugunsten des Marktes

Positive Werte bedeuten: Market ist besser. 95%-Bootstrap-CI; p-Werte sind je Baseline mit Holm korrigiert.

| Baseline | Metrik | Verbesserung | 95%-CI | Win-Rate | p(Holm) | Befund |
|---|---|---:|---:|---:|---:|---|
| nearest_greedy | Composite operational score | 0.0305 | [0.0151, 0.0462] | 77.50% | 0.0040 | Vorteil |
| nearest_greedy | Completion rate | 0.0304 | [0.0062, 0.0554] | 65.00% | 0.1482 | unklar/Trade-off |
| nearest_greedy | Mean completion time [s] | 10.0806 | [6.4637, 13.7175] | 87.50% | 0.0005 | Vorteil |
| nearest_greedy | Load imbalance [tasks] | 0.4207 | [0.1722, 0.6606] | 71.25% | 0.0130 | Vorteil |
| nearest_greedy | Coverage fraction | 0.0012 | [-0.0083, 0.0103] | 47.50% | 1.0000 | unklar/Trade-off |
| nearest_greedy | Mean cell idleness [s] | -0.8340 | [-3.1078, 1.4535] | 45.00% | 1.0000 | unklar/Trade-off |
| nearest_greedy | Max cell idleness [s] | 2.1000 | [-13.9753, 17.3125] | 56.25% | 1.0000 | unklar/Trade-off |
| nearest_greedy | Detection latency [s] | -1.1379 | [-3.0838, 0.7332] | 42.50% | 1.0000 | unklar/Trade-off |
| nearest_greedy | Messages/completed task | -1.9793 | [-3.3769, -0.6040] | 30.00% | 0.0549 | unklar/Trade-off |
| nearest_greedy | Mean final battery | -0.0057 | [-0.0090, -0.0023] | 30.00% | 0.0156 | Nachteil |
| nearest_greedy | Depleted robots | 0.0000 | [0.0000, 0.0000] | 50.00% | 1.0000 | unklar/Trade-off |
| nearest_greedy_no_survey | Composite operational score | 0.0484 | [0.0305, 0.0662] | 82.50% | 0.0005 | Vorteil |
| nearest_greedy_no_survey | Completion rate | 0.0571 | [0.0205, 0.0946] | 66.25% | 0.0240 | Vorteil |
| nearest_greedy_no_survey | Mean completion time [s] | 9.2291 | [5.4699, 13.2969] | 75.00% | 0.0005 | Vorteil |
| nearest_greedy_no_survey | Load imbalance [tasks] | 0.6670 | [0.4439, 0.8966] | 83.75% | 0.0005 | Vorteil |
| nearest_greedy_no_survey | Coverage fraction | -0.0153 | [-0.0238, -0.0079] | 23.75% | 0.0007 | Nachteil |
| nearest_greedy_no_survey | Mean cell idleness [s] | -3.7936 | [-6.7087, -1.1137] | 40.00% | 0.0428 | Nachteil |
| nearest_greedy_no_survey | Max cell idleness [s] | -26.2875 | [-43.3884, -9.9747] | 30.00% | 0.0240 | Nachteil |
| nearest_greedy_no_survey | Detection latency [s] | 1.2736 | [-1.6690, 4.2551] | 60.00% | 1.0000 | unklar/Trade-off |
| nearest_greedy_no_survey | Messages/completed task | -29.5555 | [-30.9502, -28.2354] | 0.00% | 0.0005 | Nachteil |
| nearest_greedy_no_survey | Mean final battery | 0.0004 | [-0.0035, 0.0045] | 45.00% | 1.0000 | unklar/Trade-off |
| nearest_greedy_no_survey | Depleted robots | 0.0000 | [0.0000, 0.0000] | 50.00% | 1.0000 | unklar/Trade-off |
| random_assignment | Composite operational score | 0.0479 | [0.0294, 0.0672] | 80.00% | 0.0005 | Vorteil |
| random_assignment | Completion rate | 0.1295 | [0.0813, 0.1795] | 77.50% | 0.0005 | Vorteil |
| random_assignment | Mean completion time [s] | 21.8467 | [17.8752, 25.7692] | 95.00% | 0.0005 | Vorteil |
| random_assignment | Load imbalance [tasks] | -0.0753 | [-0.2918, 0.1273] | 48.75% | 1.0000 | unklar/Trade-off |
| random_assignment | Coverage fraction | 0.0159 | [0.0037, 0.0292] | 65.00% | 0.0885 | unklar/Trade-off |
| random_assignment | Mean cell idleness [s] | 8.6103 | [5.5857, 11.7825] | 80.00% | 0.0005 | Vorteil |
| random_assignment | Max cell idleness [s] | 19.3000 | [5.8625, 33.5375] | 72.50% | 0.0476 | Vorteil |
| random_assignment | Detection latency [s] | -0.6827 | [-3.1898, 1.7124] | 47.50% | 1.0000 | unklar/Trade-off |
| random_assignment | Messages/completed task | 3.4607 | [0.7191, 6.7544] | 60.00% | 0.0987 | unklar/Trade-off |
| random_assignment | Mean final battery | -0.0029 | [-0.0066, 0.0008] | 32.50% | 0.5554 | unklar/Trade-off |
| random_assignment | Depleted robots | 0.0000 | [0.0000, 0.0000] | 50.00% | 1.0000 | unklar/Trade-off |

## Lambda-Robustheit (Poisson-Zellereignisse)

| lambda_model / lambda_true | Detektionslatenz [s] | Detection rate | Completion rate | Max Idleness [s] |
|---:|---:|---:|---:|---:|
| 0.333 | 37.39 | 0.979 | 0.925 | 236.20 |
| 1 | 39.90 | 0.968 | 0.870 | 274.40 |
| 3 | 40.25 | 0.937 | 0.874 | 274.02 |

## Was damit bewiesen ist - und was nicht

Die Tabellen sind ein reproduzierbarer empirischer Nachweis für diese Simulator-, Parameter- und Szenariofamilie. Ein universeller mathematischer Dominanzbeweis ist nicht möglich: Nearest Greedy kann bei reiner lokaler Fahrzeit optimal sein, und die marktbasierte Methode bezahlt Last- und Energiebalance mit zusätzlichen Wegen. Deshalb werden jede Einzelmetrik, Konfidenzintervalle, Trade-offs und der Kommunikationspreis offengelegt.

Die Trennung von Trainings- und Holdout-Seeds verhindert, dass derselbe Zufall sowohl zur Parameterwahl als auch zum Nachweis verwendet wird. Alle Strategien nutzen exakt dasselbe Nachrichtenprotokoll und dieselben Szenario-Seeds.
