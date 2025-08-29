---
# SCD40-ESP

---

## Introduction

**SCD-40-ESP** is a minimal, production-ready C driver for interfacing the **Sensirion SCD4x family** (SCD40/SCD41) with **ESP32** using the native **ESP-IDF** environment. The repository provides a clean and modular structure, including the source files (`scd4x.c/.h`), `CMakeLists.txt`, and `.vscode` folder, ready for integration into any larger embedded project. By following Sensirion’s I²C protocol closely and leveraging ESP-IDF’s command link API, the driver ensures robust and efficient communication with the sensor for CO2, temperature, and relative humidity measurements.

The sensor operates at **I²C address 0x62** and communicates using **16-bit command words** with CRC validation. The driver abstracts this low-level interaction into a well-structured interface, respecting timing, start/stop conditions, and expected response lengths, while providing high-level functions for sensor initialization, periodic data acquisition, and configuration. CRC-8 verification (polynomial 0x31) is implemented manually for full protocol compliance, and all operations adhere to Sensirion’s recommended flowchart logic. Debugging is assisted via consistent `ESP_LOG` tagging throughout the implementation.

This repository is ideal for embedded developers seeking to integrate precise CO₂ sensing into their IoT projects, HVAC systems, or indoor air quality monitors. It is particularly useful when combined with telemetry backends. 

## Documentation

For detailed electrical characteristics, communication protocols, and performance specifications, refer to the official  
[SCD40 Datasheet](https://sensirion.com/media/documents/E0F04247/631EF271/CD_DS_SCD40_SCD41_Datasheet_D1.pdf)

---
