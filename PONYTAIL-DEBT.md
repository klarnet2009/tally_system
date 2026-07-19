# Ponytail debt ledger

Отложенные упрощения. Источник истины — маркеры `ponytail:` в коде; этот файл — снапшот (`grep -rnE '(#|//) ?ponytail:' .`).

- `test_compile.sh:2`, корневые `Arduino.h`/`SPI.h` + `test_compile.sh`/`test_compile_hub.sh` (~60 строк) — стабы для ручной проверки `g++ -fsyntax-only`. ceiling: живут только ради ручного синтакс-чека. upgrade: удалить все четыре файла, когда подтверждено, что вручную никто не запускает (`pio run` — честная проверка).

1 markers, 0 with no trigger.
