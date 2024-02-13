import paho.mqtt.client as mqtt
import time

# Global variable storing whether the robot can take the next disk
canIWork = None
name = ""
# initialy false, so that 0 will be published; becomes True when the robot disconnects
reconnecting = False
# set to True if you don't want to reconnect
doNotReconnect = False


def on_connect(client, userdata, flags, rc):
    global name
    global reconnecting
    if not reconnecting:
        client.publish("robots/progress/{}".format(name), 0, qos=1)

def on_disconnect(client, userdata, rc):
    global name
    global reconnecting
    global canIWork
    global doNotReconnect

    # when the robot is disconnected, it cannot process the next disk
    canIWork = False

    # reconnecting is True, so that when it reconnects it does not publish 0 in progress
    reconnecting = True
    print("Disconnected with result code: {}".format(rc))

    # reconnect attempts
    reconnect_count, reconnect_delay = 0, 2
    while reconnect_count < 12 and not doNotReconnect:
        print("Reconnecting in {} seconds...".format(reconnect_delay))
        time.sleep(reconnect_delay)

        try:
            client.reconnect()
            print("Reconnected successfully!")
            client.publish("robots/progress/{}".format(name), 2, qos=1)
            client.subscribe("robots/allowed/{}".format(name), qos=1)
            return
        except Exception as err:
            print("{}. Reconnect failed. Retrying...".format(err))

        reconnect_delay *= 2
        reconnect_delay = min(reconnect_delay, 60)
        reconnect_count += 1
    print("Reconnect failed after {} attempts. Exiting...".format(reconnect_count))

# either from the allowed topic or from the error topic about a fault
def on_message(client, userdata, message):
    global canIWork
    global doNotReconnect
    global breakTime
    if message.topic == "robots/allowed/{}".format(name):
        canIWork = int(message.payload.decode("utf-8"))
        print("I can take the next disk") if canIWork == 1 else print("I can NOT take the next disk")
    else:
        # error -> disconnect and stop working
        canIWork = False
        print(message.payload.decode("utf-8"))
        doNotReconnect = True
        client.disconnect() 
        time.sleep(2)


name = input("name: ")

# connect to the MQTT Broker and subscribe to the allowed topic
mqttBroker = 'localhost'
client = mqtt.Client(name)

client.on_connect = on_connect
client.on_disconnect = on_disconnect
# set LWT: publish name in the 'disconnected' topic when it disconnects
client.will_set("disconnected", payload=name, qos=1, retain=False)

try:
    client.connect(mqttBroker, keepalive=10)
except Exception as err:
    print(err)
client.subscribe("robots/allowed/{}".format(name), qos=1)
client.subscribe("robots/errors/{}".format(name), qos=1)

client.loop_start()
client.on_message = on_message

# imitate actual work of the robot
while True:
    operation = input()
    client.publish("robots/progress/{}".format(name), 1, qos=1)