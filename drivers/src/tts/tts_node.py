import os
import rospy
import rospkg
import logging

from modules.srv import tts_service,tts_serviceResponse
from google.cloud import texttospeech_v1
from rospy.core import rospydebug,rospyinfo

class MoSS:
    def __init__(self):
        self.drivers_folder = rospkg.RosPack().get_path('drivers')
        self.gac = self.drivers_folder + '/config/credentials.json'
        os.environ["GOOGLE_APPLICATION_CREDENTIALS"]=self.gac

        self.client = texttospeech_v1.TextToSpeechClient()
        self.audio_config = texttospeech_v1.types.AudioConfig(audio_encoding='MP3',volume_gain_db=16.0,effects_profile_id=['telephony-class-application'])

        s = rospy.Service('tts_service',tts_service,self.synthesize)
    
    def synthesize(self, req):
        if(req.text and req.path):
            start = rospy.get_time()
            if req.lang == "es":
                self.voice = texttospeech_v1.types.VoiceSelectionParams(language_code='es-419')
            else:
                self.voice = texttospeech_v1.types.VoiceSelectionParams(language_code='en-US',name='en-US-Wavenet-F')

            text =  texttospeech_v1.types.SynthesisInput(text=req.text)
            response = self.client.synthesize_speech(text,self.voice,self.audio_config)
            path = req.path
            with open(path, "wb") as out:
                out.write(response.audio_content)
            end = rospy.get_time()
            duration = end - start
            rospy.logdebug("%s s" % str(duration))
            return tts_serviceResponse(True)
        return tts_serviceResponse(False)

def set_rospy_log_lvl(log_level_ros):
    log_level = (log_level_ros*10) + 10
    logger = logging.getLogger('rosout')
    logger.setLevel(log_level)

def main():
    rospy.init_node('tts_node')
    set_rospy_log_lvl(rospy.get_param('~log_level',0))
    moss = MoSS()

    rospy.spin()