import paho.mqtt.client as mqtt

# MQTT Broker information
mqtt_broker = "91.121.93.94"
mqtt_port = 1883
mqtt_topic = "device/temp"


# Callback function for connection
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker")
        client.subscribe("device/temp")
        client.publish("device/led")
    else:
        print("Failed to connect, return code %d" % rc)


# Callback function for receiving messages
def on_message(client, userdata, msg):
    print("Received message: %s" % msg.payload.decode())


def emergency_stop_function(stopvalue):
    # Create an MQTT client
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    # Connect to MQTT Broker
    client.connect(mqtt_broker, mqtt_port, 60)

    # Publish a message
    message = stopvalue  # input("enter the value")
    client.publish("device/led", message)


# Loop continuously to receive messages

# c = emergency_stop_function(1)
