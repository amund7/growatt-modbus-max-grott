#!/usr/bin/env python3

import json
import random
import time
import datetime
import os
import sys
from pymodbus.exceptions import ModbusIOException
from pymodbus.client import ModbusSerialClient as ModbusClient
from influxdb import InfluxDBClient #, Point
from paho.mqtt import client as mqtt_client
from datetime import datetime

influxhost = "192.168.0.125"
influxport = "8086"
influxdbname = "growatt"
influxuser = "homeassistant"
influxpass = "homeassistant"
influxmeasurement = "inverter"

broker = '192.168.0.111'
port = 1883
topic = "energy/growatt"
client_id = f'python-mqtt-{random.randint(0, 1000)}'
username = 'mqtt'
password = 'mqtt'

interval = 1

numinverters = 1
#inverterusbport1 = "/dev/ttyACM0"
inverterusbport1 = "/dev/ttyUSB0"
#not sure yet if the inverters will allow me to poll them over a single usb connection or not
inverterusbport2 = "/dev/ttyUSB1"
inverterusbport3 = "/dev/ttyUSB2"

verbose = 0
gwverbose = 0
gwinfodump = 0

# Codes
StatusCodes = {
    0: "Standby",
    1: "noUSE",
    2: "Discharge",
    3: "Fault",
    4: "Flash",
    5: "PV Charge",
    6: "AC Charge",
    7: "Combine Charge",
    8: "Combine charge and Bypass",
    9: "PV charge and Bypass",
    10: "AC Charge and Bypass",
    11: "Bypass",
    12: "PV charge and discharge"
}

def merge(*dict_args):
    result = {}
    for dictionary in dict_args:
        result.update(dictionary)
    return result

class Growatt:
    def __init__(self, client, name, unit):
        self.client = client
        self.name = name
        self.unit = unit

    def read(self):
        row = self.client.read_input_registers(0, 116, unit=self.unit)
        #row = self.client.read_input_registers(0, 83, unit=self.unit)
        if gwverbose: print("GWVERBOSE2")
        if gwverbose: print("GWVERBOSE3")
        print(row);

        info = {
            "device": "QMH6CGB003",
            "time": datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
            "buffered": "no",
            "values": {
              "pvserial": "QMH6CGB003",
              "pvstatus": row.registers[0],
              "pvpowerin": row.registers[1] * 65536 + row.registers[2],
              "pv1voltage": row.registers[3],
              "pv1current": row.registers[4],
              "pv1watt":    row.registers[5] * 65536 + row.registers[6],
              "pv2voltage": row.registers[7],
              "pv2current": row.registers[8],
              "pv2watt": row.registers[9] * 65536 + row.registers[10],
              "pv3voltage": row.registers[11],
              "pv3current": row.registers[12],
              "pv3watt": row.registers[13] * 65536 + row.registers[14],
              "pv4voltage": row.registers[15],
              "pv4current": row.registers[16],
              "pv4watt": row.registers[17] * 65536 + row.registers[18],
              "pv5voltage":row.registers[19],
              "pv5current": row.registers[20],
              "pv5watt": row.registers[21] * 65536 + row.registers[22],
              "pv6voltage": row.registers[23],
              "pv6current": row.registers[24],
              "pv6watt": row.registers[25] * 65536 + row.registers[26],
              "pv7voltage": row.registers[27],
              "pv7current": row.registers[28],
              "pv7watt": row.registers[29] * 65536 + row.registers[30],
              "pv8voltage": row.registers[31],
              "pv8current": row.registers[32],
              "pv8watt": row.registers[33] * 65536 + row.registers[34],
              "pvpowerout": row.registers[35] * 65536 + row.registers[36],
              "pvfrequentie": row.registers[37],
              "pvgridvoltage": row.registers[50],
              "pvgridcurrent": row.registers[39],
              "pvgridpower": row.registers[40]*65536+row.registers[41],
              "pvgridvoltage2": row.registers[51],
              "pvgridcurrent2": row.registers[43],
              "pvgridpower2": row.registers[44]*65536+row.registers[45],
              "pvgridvoltage3": row.registers[52],
              "pvgridcurrent3": row.registers[47],
              "pvgridpower3": row.registers[48]*65536+row.registers[49],
              # grid voltages er her, 50-51-52
              "eactoday": row.registers[53]*65536+row.registers[54],
              "eactotal": row.registers[55]*65536+row.registers[56],
              "epvtotal": row.registers[91]*65536+row.registers[92],
              "totworktime": row.registers[57]*65536+row.registers[58],
              "epv1today": row.registers[59]*65536+row.registers[60],
              "epv1total": row.registers[61]*65536+row.registers[62],
              "epv2today": row.registers[63]*65536+row.registers[64],
              "epv2total": row.registers[65]*65536+row.registers[66],
              "epv3today": row.registers[67]*65536+row.registers[68],
              "epv3total": row.registers[69]*65536+row.registers[70],
              "epv4today": row.registers[71]*65536+row.registers[72],
              "epv4total": row.registers[73]*65536+row.registers[74],
              "epv5today": row.registers[75]*65536+row.registers[76],
              "epv5total": row.registers[77]*65536+row.registers[78],
              "epv6today": row.registers[79]*65536+row.registers[80],
              "epv6total": row.registers[81]*65536+row.registers[82],
              "epv7today": row.registers[83]*65536+row.registers[84],
              "epv7total": row.registers[85]*65536+row.registers[86],
              "epv8today": row.registers[87]*65536+row.registers[88],
              "epv8total": row.registers[89]*65536+row.registers[90],
              "pvtemperature": row.registers[93],
              "pvipmtemperature": row.registers[94],
              "pvboosttemp": row.registers[95],
              "temp4": row.registers[96],
              "bat_dsp": row.registers[97],
              "pbusvolt": row.registers[98],
              "nbusvolt": row.registers[99],
              "ipf": row.registers[100],
              "realoppercent": row.registers[101],
              "opfullwatt": row.registers[102]*65536+row.registers[103],
              "deratingmode": row.registers[104],
              "faultcode": row.registers[105],
              #"#remotectrlen": row.registers[],
              #"#remotectrlpwr": row.registers[0],
              #"#warning": row.registers[0],
              "binvwarncode": row.registers[112],
              "realpowerpercent": row.registers[113],
              "invstartdelay": row.registers[114],
              "binvallfaultcode": row.registers[115],
              "eacharge_today": row.registers[112]*65536+row.registers[113],
              "eacharge_total": row.registers[114]*65536+row.registers[115],
              "batterytype": 0,
              #"string1_voltage": row.registers[0],
              #"string1_current": row.registers[0],
              #"string2_voltage": row.registers[0],
              #"string2_current": row.registers[0],
              #"string3_voltage": row.registers[0],
              #"string3_current": row.registers[0],
              #"string4_voltage": row.registers[0],
              #"string4_current": row.registers[0],
              #"string5_voltage": row.registers[0],
              #"string5_current": row.registers[0],
              #"string6_voltage": row.registers[0],
              #"string6_current": row.registers[0],
              #"string7_voltage": row.registers[0],
              #"string7_current": row.registers[0],
              #"string8_voltage": row.registers[0],
              #"string8_current": row.registers[0],
              #"string9_voltage": row.registers[0],
              #"string9_current": row.registers[0],
              #"string10_voltage": row.registers[0],
              #"string10_current": row.registers[0],
              #"string11_voltage": row.registers[0],
              #"string11_current": row.registers[0],
              #"string12_voltage": row.registers[0],
              #"string12_current": row.registers[0],
              #"string13_voltage": row.registers[0],
              #"string13_current": row.registers[0],
              #"string14_voltage": row.registers[0],
              #"string14_current": row.registers[0],
              #"string15_voltage": row.registers[0],
              #"string15_current": row.registers[0],
              #"string16_voltage": row.registers[0],
              #"string16_current": row.registers[0],
              #"pviso": row.registers[200],
              #"r_dci_current": row.registers[201],
              #"s_dci_current": row.registers[202],
              #"t_dci_current": row.registers[203],
              #"pid_bus_voltage": row.registers[204],
              #"gfci_current_205": row.registers[205],
              #"ct_i_r": row.registers[207],
              #"fan_fault_bit": row.registers[229],
              #"out_apparent_pwr": row.registers[230]*65536+row.registers[231],
              #"reactive_pwr_real": row.registers[232]*65536+row.registers[233],
              #"reactive_pwr_nominal": row.registers[234]*65536+row.registers[235],
              #"reactive_pwr_tot": row.registers[236]*65536+row.registers[237]
            }
        }

        if gwinfodump: print(info, sep = "\n")
        return info


    def readDebug(self):
        #row = self.client.read_holding_registers(250, 48, unit=self.unit)
        #row = self.client.read_input_registers(141, 33, unit=self.unit)
        row = self.client.read_holding_registers(88, 1, unit=self.unit) # modbus version
        print(row)
        print(row.registers)



FIRST_RECONNECT_DELAY = 1
RECONNECT_RATE = 2
MAX_RECONNECT_COUNT = 12
MAX_RECONNECT_DELAY = 60
TOPIC_SUBSCRIBE = 'growatt/command'

def on_disconnect(client, userdata, rc):
    print("Disconnected with result code: %s", rc)
    reconnect_count, reconnect_delay = 0, FIRST_RECONNECT_DELAY
    while True: # reconnect_count < MAX_RECONNECT_COUNT:
        print("Reconnecting in %d seconds...", reconnect_delay)
        time.sleep(reconnect_delay)

        try:
            client.reconnect()
            print("Reconnected successfully!")
            return
        except Exception as err:
            print("%s. Reconnect failed. Retrying...", err)

        reconnect_delay *= RECONNECT_RATE
        reconnect_delay = min(reconnect_delay, MAX_RECONNECT_DELAY)
        reconnect_count += 1
    print("Reconnect failed after %s attempts. Exiting...", reconnect_count)

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(TOPIC_SUBSCRIBE)

def on_message(client, userdata, msg):
    print(f"Message received on topic {msg.topic}: {msg.payload.decode()}")
    global message
    message=msg.payload

def connect_mqtt():
     # Set Connecting Client ID
    client = mqtt_client.Client(client_id)
    client.username_pw_set(username, password)

    # Assign event callbacks
    client.on_disconnect = on_disconnect
    client.on_connect = on_connect
    client.on_message = on_message

    # Connect to the broker
    client.connect(broker, port)

    # Start the listening loop in a separate thread
    client.loop_start()

    return client

def publish(client, msg):
    #time.sleep(1)
        #msg = f"messages: {msg_count}"
    result = client.publish(topic, msg)
        # result: [0, 1]
    status = result[0]
    if status == 0:
        if verbose:
            print(f"Send `{msg}` to topic `{topic}`")
        else:
            print(f"Send msg to topic `{topic}`")
    else:
        print(f"Failed to send message to topic {topic}")

mqttclient = connect_mqtt()



print("Connecting to Inverter..", end="")
try:
  client = ModbusClient(method='rtu', port=inverterusbport1, baudrate=38400, stopbits=1, parity='N', bytesize=8, timeout=1)
  client.connect()

except:
  print("Failed")
else:
  print("Done!")



print("Loading inverters.. ", end="")
inverters = []
for i in range(numinverters):

  unit=i+1
  name = "Growatt"+str(unit)
  measurement=influxmeasurement+str(unit)
  print("Name ",name," unit is ",unit," measurement is ",measurement)
  growatt = Growatt(client, name, unit)
  inverters.append({
    'growatt': growatt,
    'measurement': measurement
  })
print("Done!")


while True:
    for inverter in inverters:
        growatt = inverter['growatt']
        #print(growatt.name)
        try:
            now = time.time()
            info = growatt.read()

            if info is None:
                continue

            if verbose: print("CHECK4")
            points = [{
                'time': int(now),
                'measurement': inverter['measurement'],
                "fields": info
            }]
            if verbose: print("CHECK5")

            publish(mqttclient, json.dumps(info))

            if message != None:
                # Parse the JSON string
                data = json.loads(message)
                
                if "activepower" in data:
                    value = data.get("activepower")
                    print("Set power limit ", value)

                    row = growatt.client.write_register(3, value)
                    print(row)

                if "curveanalysis" in data:
                    value = data.get("curveanalysis")
                    print("Curve analysis ", value)

                    
                    row = growatt.client.write_register(250, value)
                    print(row)

                message=None


            #time.sleep(interval)
            #growatt.readDebug()


            #if not influx.write_points(points, time_precision='s'):
            #    print("Failed to write to DB!")
        except Exception as err:
            if verbose: print("ERRORHERE1")
            print(err)

        time.sleep(interval)
