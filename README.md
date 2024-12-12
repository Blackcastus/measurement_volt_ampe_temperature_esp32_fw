esp_idf_measurement_volt_ampe_temperature.git

Description:
- Measurement Volt AC.
- Measurement Ampe AC
- Measurement Temperature

1. Block Diagram

|-------------------------------------------|
|			|-------------------------------|
|			|	MODBUS	--->	PZEM-004T	|
|	ESP32	|-------------------------------|
|			|	SPI		--->	MAX31865	|
|			|-------------------------------|
|-------------------------------------------|

2. Flow
ESP		--->	MQTT	--->	EMQX	--->	Store data to data lake (Google cloud)