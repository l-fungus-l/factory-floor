import paho.mqtt.client as mqtt 

robots = {}
disconnected = {}

def check_robot(robot_name):
    for k,v in robots.items():
        if (robots[k] - robots[robot_name] >= 3):
            return 1
        elif abs(robots[k] - robots[robot_name] - 1) > 2:
            return 0
    return 1


def on_message(client, userdata, message):
    msg = str(message.payload.decode("utf-8"))
    name_robot = message.topic.split('/')[-1]

    # LWT messages
    if message.topic == "disconnected":
        disconnected[msg] = robots[msg]
        del robots[msg]
        print(f"{msg} left")
        print(f"disconnected: {disconnected}")
        print()

    # client processes a disk in the progress topic
    elif int(message.payload.decode("utf-8")) == 1:
        robots[name_robot] +=  1
        print(f"{name_robot} just processed a disk")
        print(robots)
        print("-------------------------------")
        for robot in robots:
            resp = check_robot(robot)
            client.publish(f"robots/allowed/{robot}", resp, qos=1)
            print(f"{robot} can process the next disk.") if resp == 1 else print(f"{robot} can NOT process the next disk.")
        print("-------------------------------")
        print()
        
    # some robot reconnects (message 2 in the progress topic)
    elif int(message.payload.decode("utf-8")) == 2:
        robots[name_robot] = disconnected[name_robot]
        del disconnected[name_robot]
        print(f"{name_robot} reconnected")
        print(robots)
        print()

    # robots publish 0 on connect in the progress topic
    else:
        robots[name_robot] =  0    
        print(f"{name_robot} joined")
        print(robots)
        print("-------------------------------")
        for robot in robots:
            resp = check_robot(robot)
            client.publish(f"robots/allowed/{robot}", resp, qos=1)
            print(f"{robot} can process the next disk.") if resp == 1 else print(f"{robot} can NOT process the next disk.")
        print("-------------------------------")
        print()


mqttBroker = 'localhost'
client = mqtt.Client('superclient')
client.connect(mqttBroker) 

client.subscribe('robots/progress/#', qos=1)
client.subscribe('disconnected', qos=1)

# this is if some robot wants to signal that it is broken. to do later (?)
client.subscribe("robots/faults/#", qos=1)

client.on_message = on_message

client.loop_forever()