import os
import re
import time
import json
import rospy
import logging
from bluepy import btle
from rospy.core import rospydebug, rospyinfo

from std_msgs.msg import Bool
from modules.srv import ContainerFeedback, ContainerFeedbackResponse
from modules.msg import containers_msg, speaker_msg, mqtt_publishers_msg, bt_macs_msg


class ContainerClient:

    AWAKE = "s&5&5"
    SLEEP = "s&120&10"
    VERSION = '1951462423'
    FEEDBACK = '1951431614212021'
    OPEN = "01121608160013161901"
    MODE = {
        '0': 'validacion',
        '1': 'stand-by',
        '2': 'delivery',
        '3': 'idle'
    }
    ERRORS = ['BTLEInternalError', 'BTLEDisconnectError']

    def __init__(self, id):

        self.id = str(id)
        self.mac = ''
        self.opened = False
        self.writing = False
        self.reconnecting = False
        self.vending_machine = False
        self.feedback = {'id': self.id,
                         'connection': False,
                         'battery': 0,
                         'lock': False,
                         'mac': self.mac,
                         'mode': '',
                         'sleep': 0,
                         'awake': 0,
                         'version': ''
                         }

        self.speakerPub = rospy.Publisher(
            '/speaker_topic', speaker_msg, queue_size=10)
        self.feedbackPub = rospy.Publisher(
            '/mqtt_publishers', mqtt_publishers_msg, queue_size=2)

        rospy.Subscriber('/containers_topic', containers_msg,
                         self.processInstruction, queue_size=2)

        srv_name = 'feedback_srv_' + self.id
        s = rospy.Service(srv_name, ContainerFeedback, self.feedback_srv)

    def reconnect(self, data):
        if (data.data and not self.vending_machine and not self.reconnecting):
            self.connectPermamently()

    def connectPermamently(self):
        self.reconnecting = True
        i = 0
        while (i < 4):
            rospy.logdebug(
                "%s activando modo Vending Machine, intento %s", self.mac, str(i))
            self.connect()
            if (self.feedback['connection']):
                self.feedbackPub.publish(
                    mqtt_topic="sensor/", raw_msg=json.dumps(self.feedback), qos=0)
                self.speakerPub.publish(mp3Id="104")
                self.write(self.AWAKE)
                self.getVersion()
                self.reconnecting = False
                self.vending_machine = True
                break
            i += 1
        rospy.logdebug("%s modo Vending Machine: %s",
                       self.mac, str(self.vending_machine))
        self.reconnecting = False

    def processInstruction(self, data):
        if (data.id == self.id or data.id == "0") and not self.reconnecting and self.mac:
            if (data.open):
                if(not self.vending_machine):
                    self.connectPermamently()
                self.openAttempts()
            elif (data.feedback and not self.vending_machine):
                self.feedbackAttempts()

    def updateMac(self, data):
        rospy.logdebug("%s - Current mac: %s", rospy.get_name(), self.mac)
        self.cleanConnection()
        self.mac = data
        rospy.logdebug("%s - New mac: %s", rospy.get_name(), self.mac)
        self.feedback['mac'] = self.mac
        self.connectPermamently()

    def cleanConnection(self):
        if (self.vending_machine):
            rospy.logdebug("Cleaning connection... mac: %s", self.mac)
            self.vending_machine = False
            # Esperar ultima peticion de feedback del contenedor actual.
            rospy.sleep(1.5)
            self.feedback['connection'] = False
            self.feedbackPub.publish(
                mqtt_topic="sensor/", raw_msg=json.dumps(self.feedback), qos=0)
            self.mac = ''
            self.disconnect()
        self.mac = ''

    def openAttempts(self):
        for attempt in range(10):
            try:
                rospy.logdebug("Attempt %s", attempt)
                if (not self.vending_machine):
                    self.connectPermamently()
                self.write(self.OPEN)
                self.opened = True
                rospy.logdebug("Success")
                break
            except:
                rospy.logwarn("Failed")
                rospy.sleep(1.0)
                if (self.reconnecting):
                    break

    def connect(self):
        try:
            rospy.logdebug("%s conectando...", self.mac)
            self.dev = btle.Peripheral(self.mac)
            rospy.logdebug("%s conectado", self.mac)
            self.feedback['connection'] = True
            self.feedbackPub.publish(mqtt_topic="sensor/", raw_msg=json.dumps(self.feedback), qos=0)
            uuidService = btle.UUID("37fc19ab-98ca-4543-a68b-d183da78acdc")
            containerService = self.dev.getServiceByUUID(uuidService)
            self.value = containerService.getCharacteristics()
        except btle.BTLEException as e:
            self.feedback['connection'] = False
            rospy.logerr("Error en conexion: %s", type(e).__name__)
            rospy.logerr(e)

    def write(self, command):
        rospy.logdebug("%s enviando %s", self.mac, command)
        self.writing = True
        self.value[0].write(command)
        self.writing = False
        rospy.logdebug("%s enviado", self.mac)

    def read(self):
        return self.value[0].read()

    def disconnect(self):
        try:
            rospy.logdebug("%s desconectando", self.mac)
            self.dev.disconnect()
            self.resetFeedback()
            rospy.logdebug("%s desconectado", self.mac)
        except btle.BTLEException as e:
            rospy.logerr("Error en desconexion:")
            rospy.logerr(e)
        rospy.sleep(5.0)

    def resetFeedback(self):
        self.feedback = {'id': self.id,
                         'connection': False,
                         'battery': 0,
                         'lock': False,
                         'mac': self.mac,
                         'mode': '',
                         'sleep': 0,
                         'awake': 0,
                         'version': ''
                         }

    def getVendingFeedback(self):
        try:
            # rospy.logdebug("Reading feedback...")
            feedback = self.read().split("&")
            # rospy.logdebug("Read: %s" % feedback)
            self.feedback['battery'] = int(feedback[0])
            self.feedback['lock'] = int(feedback[1])
            if (self.feedback['version']):
                self.feedback['mode'] = self.MODE[feedback[2]]
                self.feedback['sleep'] = int(feedback[3])
                self.feedback['awake'] = int(feedback[4])
            self.feedbackPub.publish(
                mqtt_topic="sensor/", raw_msg=json.dumps(self.feedback), qos=0)
            self.opened = True if self.feedback['lock'] else False
        except btle.BTLEException as e:
            rospy.logerr("Error en feedback: %s", type(e).__name__)
            rospy.logerr(e)
            self.vending_machine = False
            self.opened = False
            self.feedback['battery'] = 0
            self.feedback['lock'] = 0
            self.feedback['connection'] = False
            if (type(e).__name__ in self.ERRORS):
                self.feedbackPub.publish(
                    mqtt_topic="sensor/", raw_msg=json.dumps(self.feedback), qos=0)
                self.reconnecting = True
                self.disconnect()
                self.connectPermamently()

    def feedbackAttempts(self):
        for attempt in range(10):
            try:
                rospy.logdebug("Attempt %s", attempt)
                self.getFeedback()
                rospy.logdebug("Success")
                time.sleep(1)
                break
            except:
                rospy.logwarn("Failed")
                if (self.reconnecting):
                    break
                time.sleep(1)

    def getFeedback(self):
        try:
            rospy.logdebug("%s conectando...", self.mac)
            dev = btle.Peripheral(self.mac)
            rospy.logdebug("%s conectado", self.mac)
            uuidService = btle.UUID("37fc19ab-98ca-4543-a68b-d183da78acdc")
            containerService = dev.getServiceByUUID(uuidService)
            value = containerService.getCharacteristics()
            feedback = value[0].read()
            rospy.logdebug("%s leido", self.mac)
            dev.disconnect()
            rospy.logdebug("%s desconectado", self.mac)

            temp = feedback.split("&")
            self.feedback['battery'] = int(temp[0])
            self.feedback['lock'] = int(temp[1])
            if (self.feedback['version']):
                self.feedback['mode'] = self.MODE[temp[2]]
                self.feedback['sleep'] = int(temp[3])
                self.feedback['awake'] = int(temp[4])
            self.feedbackPub.publish(
                mqtt_topic="sensor/", raw_msg=json.dumps(self.feedback), qos=0)
        except btle.BTLEException as e:
            rospy.logerr("Error en feedback: %s", type(e).__name__)
            rospy.logerr(e)
            self.vending_machine = False
            self.feedback['connection'] = False
            self.feedback['battery'] = 0
            self.feedback['lock'] = 0
            self.feedbackPub.publish(
                mqtt_topic="sensor/", raw_msg=json.dumps(self.feedback), qos=0)
            if (type(e).__name__ in self.ERRORS):
                self.reconnecting = True
                self.disconnect()
                self.connectPermamently()

    def feedback_srv(self, req):
        #if not self.vending_machine:
        #    self.feedbackAttempts()
        return ContainerFeedbackResponse(json.dumps(self.feedback))

    def getVersion(self):
        if(self.feedback['connection']):
            try:
                self.write(self.VERSION)
                ver = self.read()
                if (re.findall('[0-9]\.[0-9]\.[0-9]', ver)):
                    self.feedback['version'] = ver
                rospy.logdebug("%s version: %s", self.mac,
                               self.feedback['version'])
                self.write(self.FEEDBACK)
            except btle.BTLEException as e:
                rospy.logerr("Error en version: %s", type(e).__name__)
                rospy.logerr(e)
                self.vending_machine = False
                self.opened = False
                self.feedback['connection'] = False
                self.feedback['battery'] = 0
                self.feedback['lock'] = 0
                self.feedbackPub.publish(
                    mqtt_topic="sensor/", raw_msg=json.dumps(self.feedback), qos=0)
                if (type(e).__name__ in self.ERRORS):
                    self.reconnecting = True
                    self.disconnect()
                    self.connectPermamently()


def mac_callback(data, master):
    macs = json.loads(data.macs)
    rospy.logdebug("macs: %s", macs)
    if (master.id in macs and macs[master.id]):
        rospy.logdebug("%s - my_mac: %s", rospy.get_name(), macs[master.id])
        if (macs[master.id] != master.mac):
            master.updateMac(macs[master.id])
    else:
        master.cleanConnection()  # unassign container


def set_rospy_log_lvl(log_level_ros):
    # convertion between ros & py logging
    log_level = (log_level_ros * 10) + 10
    logger = logging.getLogger('rosout')
    logger.setLevel(log_level)


def main():
    rospy.init_node('ble_node')
    container_id = rospy.get_param('~container_id')
    # 1 = rospy.INFO same as roscpp
    log_level_verb = rospy.get_param('~log_level', 1)
    set_rospy_log_lvl(log_level_verb)
    rospy.set_param('~pid', os.getpid())
    master = ContainerClient(container_id)
    rospy.Subscriber('/bt_macs_topic', bt_macs_msg,
                     mac_callback, callback_args=master, queue_size=2)

    r = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        if (master.opened or master.vending_machine):
            master.getVendingFeedback()
        elif (not master.feedback['connection'] and not master.reconnecting and master.mac):
            master.connectPermamently()
        r.sleep()

    if (master.vending_machine):
        master.disconnect()
