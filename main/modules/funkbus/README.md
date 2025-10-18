# Funkbus (JUNG/ELTAKO) via OpenMQTTGateway

Send Funkbus button presses (short/long, ON/OFF) over MQTT.  
The gateway builds the 48-bit Funkbus frames and transmits them with the
captured timing (preamble, 1 ms/bit Manchester variant, 7 ms inter-frame gap),
then returns the CC1101 to the listen frequency.

---

## Command topic

Publish commands to:

<base>/commands/MQTTtoFunkbus ```

<base> is your OpenMQTTGateway base topic (e.g. home/OMG_ESP32_Funkbus-CC1101).

Payload schema
Field	Type	Values / Notes
serial	string	5-hex remote ID, e.g. "370F2"
channel	string	"A", "B", "C", or "LS"
button	number	1..8
action	string	"ON" or "OFF"
duration	string	"S" = short press, "L" = long press

The module validates/normalizes the JSON, builds the 48-bit frames, and transmits with the correct Funkbus timing. You’ll see confirmation lines in the serial log.

Examples

Short press ON (A4):

{
  "serial": "370f2",
  "channel": "A",
  "button": 4,
  "action": "ON",
  "duration": "S"
}


Long press OFF (A4):

{
  "serial": "370f2",
  "channel": "A",
  "button": 4,
  "action": "OFF",
  "duration": "L"
}