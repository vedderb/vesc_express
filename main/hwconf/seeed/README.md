# Seeed ESP32C3 Hardware Configuration

HW conf for Seeed Studio XAIO-ESP32-C3
Use together with a CAN bus transceiver for example: SN65HVD230

D0 => CAN transceiver RX\
D1 => CAN transceiver TX

build:\
`idf.py build -DHW_NAME="Seeed ESP32-C3"`