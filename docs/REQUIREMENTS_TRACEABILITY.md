# Zuordnung der Briefing-Anforderungen

| Anforderung | Umsetzung |
|---|---|
| Plattformunabhängiges `allocation/` | `allocation/`; Importinvariante im Test |
| Korrigierte Kosten, niedrigstes Gebot | `allocation/bidding.py`, `allocation/auction.py` |
| Dynamische Priorität | `PriorityFn` in `allocation/tasks.py` |
| Fünf kompakte Nachrichtentypen | `allocation/messages.py` |
| Dezentrale Auktion | lokaler Auctioneer in `sim/robot.py` |
| Market, Greedy, Random, fairer Funk | `allocation/strategies.py`, gemeinsamer `MessageBus` |
| Punktbewegung und Startup-Zeit | `sim/robot.py` |
| Batterie und DEPLETED/RELEASE | `sim/robot.py`, Parameter in `default.json` |
| Latenz und Paketverlust | `sim/comms.py` |
| Coverage- und SURVEY-Raster | `sim/world.py` |
| Wahre vs. modellierte Ereignisrate | Poisson-Modus und `experiments/robustness.py` |
| Zwei portable CSV-Logs | `metrics/logger.py` |
| Metriken aus Logs | `metrics/analysis.py` |
| Determinismus und Invarianten | 16 Tests in `tests/` |
| Parameter-Sweep und Pareto-Front | `experiments/sweep.py` |
| Statistischer Holdout-Vergleich | `experiments/compare.py` |
| Animierte 2D-Wiedergabe aus Logs | `experiments/replay.py`, optional über `run_once --replay` |
| Interaktiver Replay-Player | `experiments/interactive_replay.py`, Play/Pause, Timeline und Tempo |
| Tier-0→Tier-1-Kalibrierung von `a`/`v_eff` | `experiments/calibrate_travel_model.py`, Protokoll in `docs/WEBOTS_CALIBRATION.md` |
| Tier-1-Gegentest der Marktlogik in Webots | `backends/webots.py` (getestet in `tests/test_webots_backend.py`), optionaler Backend-Hook in `sim/robot.py`/`sim/world.py`, Controller-Skripte + Anleitung in `docs/WEBOTS_MARKET_DEMO.md` |
