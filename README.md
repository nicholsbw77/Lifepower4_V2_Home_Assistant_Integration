# Lifepower4_V2_Home_Assistant_Integration
EG4 Lifepower4 V2 Home Assistant integration all cell data is parsed and sent via MQTT to HA Misquitto
Can run adjacent on same device as Solar Assistant.  Just create a virtual python environment in home folder or path you choose.  verify which usbtty to set within the python script.  All Modbus registers are queried and published via mqtt.  

I could never get bms tools to work and never was able to access to see the cell voltages so i figured time to make a nice integration  
In my Battery Monitoring repo you will find some prebuilt dashboard cards for Home assistant as well as a similar python program for JK BMS V19 batteries.
