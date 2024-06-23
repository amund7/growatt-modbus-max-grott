# Growatt-XL3 MAX Monitor

Based on https://github.com/sdsolomo/growatt-x000ES

This Python program polls data via modbus from a Growatt MAX inverter and sends it to MQTT in a format recognized by the homeassistant-grott Home Assistant plugin.


At first I used the Growatt WIFI dongle to send directly to Growatt servers. The default update rate was every 5 minutes. I managed to adjust it down to 1 minute, which is also very slow to see much of what's going on when there are clouds, or just any weather really.

I used a Home Assistant plugin that would poll the Growatt server. Usually every morning that would stop working. It could work again after some fiddling like restarting the plugin. The developers of the plugin said this is because of rate limitations set by Growatt, or something of that sort. So I tried GROTT in several configurations, that worked better, but also had issues. But the biggest was I could never get past the 1 minute data rate.

Then I tried the repository mentioned above. It sort of worked, but only for a while, then the inverter would stop sending data, until the next day or a manual power cycle. This is probably because I connected both the Growatt WIFI dongle and my USB-B cable at the same time, I suppose it's sharing either USB controller or Modbus interface, and the two crashes with eachother.

Then I tried the RS485 modbus ports with a RS485 USB interface in the raspberry. With a lot of fiddling around it worked! It's always hard to get the correct A, B, + -, also guessing the correct USB ports of the pi - while there are tricks to find it, dmesg is one.

This solution turned out rock solid! It has been running for I think a year now, pushing all data points every second.


```amund@solarpi:~ $ python3 getconfig.py ```

to keep it running after you log out

```amund@solarpi:~ $ nohup python3 growatt-x000ES-main/getstatus.py > /dev/null &```


Still this solution relies on the HACS add-on https://github.com/muppet3000/homeassistant-grott to scale and recieve the data, set up and update the Home Assistant sensors. I may move away from this in the future, since this solution limits the number and naming of the sensors. I have added a few extra ones directly in Home Assistant, and while my HA config will be messy, it would be straight forward to move all sensors:

```mqtt:
  sensor:
    - name: "QMH6CGB003 Derating mode"
      state_topic: "energy/growatt"
      unique_id: "qmh6cgb003_deratingmode"
      state_class: measurement    
      value_template: "{{ value_json['values']['deratingmode'] }}"

    - name: "QMH6CGB003 Fault code"
      state_topic: "energy/growatt"
      unique_id: "qmh6cgb003_faultcode"
      state_class: measurement    
      value_template: "{{ value_json['values']['faultcode'] }}"

    - name: "QMH6CGB003 Warning code"
      state_topic: "energy/growatt"
      unique_id: "qmh6cgb003_warningcode"
      state_class: measurement    
      value_template: "{{ value_json['values']['binvwarncode'] }}"

    - name: "QMH6CGB003 Real power percent"
      state_topic: "energy/growatt"
      unique_id: "qmh6cgb003_realpowerpercent"
      state_class: measurement
      unit_of_measurement : "%"   
      value_template: "{{ value_json['values']['realpowerpercent'] }}"
```



Also, to get the text for the derating mode I've added:

```template:
  - sensor:
      - unique_id: derating_text
        name: Derating text
        state: "{% set derate = states('sensor.qmh6cgb003_derating_mode')|int %}
        {% if derate == 0 %}{{derate}}: None
        {% elif derate == 1 %}{{derate}}: PV
        {% elif derate == 2 %}{{derate}}: *
        {% elif derate == 3 %}{{derate}}: Vac
        {% elif derate == 4 %}{{derate}}: Fac
        {% elif derate == 5 %}{{derate}}: Tboost
        {% elif derate == 6 %}{{derate}}: Tinv
        {% elif derate == 7 %}{{derate}}: Control
        {% elif derate == 8 %}{{derate}}: *
        {% elif derate == 9 %}{{derate}}: OverBack ByTime
        {%- endif %}"

```



## License

MIT
