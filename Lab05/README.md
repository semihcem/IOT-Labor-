# Lab 05 – Sensor Bay (ESP32-S3-BOX-3)

hello20/     -> Aufgabe 1.2 (20x Hello World, dann Neustart)
sensor_bay/  -> Aufgaben 1.3-1.5 (AHT-Sensor + Radar + Display)

Bauen (ESP-IDF-Terminal, im jeweiligen Ordner):
  $env:IDF_TARGET="esp32s3"      # nur falls die Target-Meldung kommt
  idf.py set-target esp32s3
  idf.py -p COM3 flash monitor

Komponenten fuer sensor_bay (Lab 1.3):
  idf.py add-dependency "espressif/esp-box-3^3.2.1"
  idf.py add-dependency "espressif/aht20^2.0.0"
