import paho.mqtt.client as mqtt
from flask import Flask, jsonify

app = Flask(__name__)

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


@app.route("/emergency_stop/<pk1>/<pk>", methods=["GET"])
def emergency_stop(pk1, pk):
    # Create an MQTT client
    print("pk", pk1)
    print("pk", pk)
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message

    # Connect to MQTT Broker
    client.connect(mqtt_broker, mqtt_port, 60)

    # Publish a message
    message = pk1 + "|" + pk
    client.publish("device/led", message)
    if pk == "0":
        # Loop continuously to receive messages
        return jsonify({"messages": "successfully power off"})
    else:
        # Loop continuously to receive messages
        return jsonify({"messages": "successfully power on"})


if __name__ == "__main__":
    app.run()
