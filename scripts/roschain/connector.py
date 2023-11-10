
from queue import Queue
import json
import rospy
from paho.mqtt import client as mqtt_client
from collections import OrderedDict
class MQTTCommunicationModule:
    def __init__(self,node_id,endpoint,port,auth=None,DEBUG=False):
        self.node_id = node_id
        self.endpoint = endpoint
        self.port = port
        self.auth = auth
        self.DEBUG = DEBUG
        self.base_topic = "nodes"
        self.log_topic = 'logs'
        self.buffer = Queue()
        self.__init_mqtt()
        self.counter = 0
        self.timeout = 5

    def __init_mqtt(self):
        self.client = mqtt_client.Client(self.node_id)
        if self.auth is not None:
            self.client.username_pw_set(self.auth["username"],self.auth["password"])
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        try:
            self.client.connect(self.endpoint, self.port)
            self.client.subscribe(f"{self.base_topic}/{self.node_id}")
            self.client.subscribe(f"{self.base_topic}")
        except Exception as e:
            rospy.loginfo(f"{self.node_id}: Error connecting to MQTT: {e}")
            return

    def on_message(self, client, userdata, message):
        self.buffer.put({"message":json.loads(message.payload.decode("utf-8")),"type":"incoming"})

    def on_connect(self, client, userdata, flags, rc):
        rospy.loginfo(f"{self.node_id}: Connected with result code " + str(rc))
        self.client.subscribe(f"{self.base_topic}/{self.node_id}")
        self.client.subscribe(f"{self.base_topic}")
        
    def send(self, message):
        if self.DEBUG:
            rospy.loginfo(f'{self.node_id}: Sending message to {message["target"]} with type {message["message"]["type"]}')
        #parse message to string
        if type(message["message"]) == OrderedDict or type(message["message"]) == dict:
          message["message"] = json.dumps(message["message"])
        else:
          message["message"] = str(message["message"])
        try:
            if message["target"] == "all":
                self.client.publish(f"{self.base_topic}", message["message"])
            else:
                self.client.publish(f"{self.base_topic}/{message['target']}", message["message"])
            self.counter += 1
            return True
        except Exception as e:
            rospy.loginfo(f"{self.node_id}: Error sending message: {e}")
            return False
        
    def send_log(self,message):
        self.client.publish(f"{self.log_topic}", f"{self.node_id}|{message}")

    def get(self):
        #self.client.loop()
        if self.is_available():
            return self.buffer.get()
        else:
            return None
        
    def is_available(self):
        self.client.loop_read()
        return not self.buffer.empty()
