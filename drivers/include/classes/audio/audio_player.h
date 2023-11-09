#include <ros/ros.h>
#include <ros/package.h>

#include <vector>
#include <glob.h>
#include <sstream>
#include <string.h>
#include <stdexcept>
#include <map>

#include <pwd.h>
#include <cstdio>
#include <memory>
#include <string>
#include <fstream>
#include <functional>
#include <sys/stat.h>
#include "libraries/json.hpp"

#include "classes/audio/mpg123_cmd_player.h"
#include "classes/persistence/persistence_reader_writer.h"

#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <modules/speaker_msg.h>
#include <modules/tts_service.h>
#include <modules/imei_service.h>
#include <modules/joystick_msg.h>
#include <drivers/transcript_msg.h>

using namespace std;
using namespace nlohmann;


class AudioPlayer
{
    public:
        AudioPlayer(ros::NodeHandle);
        ~AudioPlayer();

        void PlayBrainOnSound();
        void PlayBrainOffSound();

    private:
        ros::NodeHandle nh;
        ros::ServiceClient clientTTS,clientIMEI;
        ros::Subscriber speaker_set_vol_ros_sub_;
        ros::Subscriber speaker_sub_;
        ros::Subscriber card_tap_event_sub_;
        ros::Subscriber transaction_response_sub_;
        ros::Subscriber speech_transcript_ros_sub_;
        ros::Publisher speaker_vol_ros_pub_;
        ros::ServiceClient heartbeat_ros_srv_client_;
        ros::ServiceServer pairing_srv;
        ros::Timer pairing_timeout, audio_timer;

        const std::string kHeartbeatServiceName = "heartbeat";
        const std::string kRosPackageName = "drivers";
        const std::string kSetVolTopicName = "speaker_set_volume";
        const std::string kSpeakerVolTopicName = "speaker_volume";
        const std::string kVolumePerstKey = "speaker_volume";
        const std::string kJoystickRosPackageName = "joystick";
        const std::string kDefaultTTSLanguage = "en";
        const std::string kTTSTempFolder = "/tmp/";
        const std::string kSpeechTranscriptTopicName =
            "/speech_recognize_node/transcript";
        const std::uint32_t kRosSubQueueSize = 2;
        const std::uint32_t kRosPubQueueSize = 2;
        const int kVolumeDefaultValue = 50;

        Mpg123Player* mpg123_player_;

        json audios;
        hash<string> hasher;
        string mp3_folder, node_name, imei, pairing_script_path{""},
            joystick_pkg_path{""}, custom_mp3_dir_path{""};
        modules::speaker_msg speakerMsg;
        std::map<std::string, ros::Timer> audio_loop_map;

        const char* homedir;

        vector<string> GetLocalFileInfo(string id);
        void UpdateAudios();
        void GetJSONFromURL();
        void InitPairing();
        void AudioLoop(std::string path_id, int period);
        std::string GetAudioFilePath(std::string pattern);
        void Play(std::string mp3_file_path, bool force_loop_stop = false,
                  bool mic_muted = false);
        bool GenerateTTS(string text, string path, string lang = "");
        void ConfigVolControlAndRosCallbacks(void);
        void SpeakerCallback(const modules::speaker_msg msg);
        void CardTapEventMsgCallback(const std_msgs::Empty tap_event);
        void TransactionResponseMsgCallback(
            const std_msgs::Bool trans_response);
        void SpeechTranscriptCallback(const drivers::transcript_msg transcript_msg);
        void SetSpeakerVolCallback(const std_msgs::Int32 volume);
        bool PairingService(std_srvs::SetBool::Request& request,
                            std_srvs::SetBool::Response& response);
};

AudioPlayer::AudioPlayer(ros::NodeHandle nh_priv)
{
    string drivers_path = ros::package::getPath(kRosPackageName);
    joystick_pkg_path = ros::package::getPath(kJoystickRosPackageName);
    node_name = ros::this_node::getName();
    int log_level;
    nh_priv.param("log_level", log_level,0);
    ros::console::levels::Level console_level;
    console_level = (ros::console::levels::Level)log_level;

    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, console_level))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    if ((homedir = getenv("HOME")) == NULL) {
        homedir = getpwuid(getuid())->pw_dir;
    }

    mpg123_player_ = new Mpg123Player(node_name);

    speaker_sub_ = nh.subscribe("speaker_topic", kRosSubQueueSize,
                                &AudioPlayer::SpeakerCallback, this);
    /* TODO several test for thread safe
    card_tap_event_sub_ =
        nh.subscribe("payment/card_tap_event", kRosSubQueueSize,
                     &AudioPlayer::CardTapEventMsgCallback, this);
    */
    transaction_response_sub_ =
        nh.subscribe("payment/transaction_response", kRosSubQueueSize,
                     &AudioPlayer::TransactionResponseMsgCallback, this);
    speech_transcript_ros_sub_ =
        nh.subscribe(kSpeechTranscriptTopicName, kRosSubQueueSize,
                     &AudioPlayer::SpeechTranscriptCallback, this);
    ConfigVolControlAndRosCallbacks();
    pairing_srv = nh.advertiseService("speaker_pairing_service",
                                      &AudioPlayer::PairingService, this);

    mp3_folder = drivers_path + "/mp3/";
    custom_mp3_dir_path = string{homedir} + "/braintemp/custom_audios";

    PlayBrainOnSound();

    clientTTS = nh.serviceClient<modules::tts_service>("tts_service");
    ros::service::waitForService("tts_service");

    /* clientIMEI = nh.serviceClient<modules::imei_service>("imei_service");
    ros::service::waitForService("imei_service");
    modules::imei_service imei_srv;
    if (!clientIMEI.call(imei_srv)){
        ROS_ERROR_STREAM(node_name << "--- IMEI service not working.");
    } else {
        imei = imei_srv.response.imei;
        GetJSONFromURL();
        UpdateAudios();
    } */
}

AudioPlayer::~AudioPlayer()
{
    try
    {
        PersistenceReaderWriter::GetInstance().Dump();
    }
    catch(const std::runtime_error& error)
    {
        /* ROS logs might not be functional at this point */
        printf("[%s/%s] Couldn't dump the persistent data for this node: %s",
               kRosPackageName.c_str(), node_name.c_str(), error.what());
    }

    PlayBrainOffSound();
    mpg123_player_->~Mpg123Player();
}


void
AudioPlayer::PlayBrainOffSound(void)
{
    std::string mp3_file_path = mp3_folder + "106.mp3";
    Play(mp3_file_path);
}


void
AudioPlayer::PlayBrainOnSound(void)
{
    std::string mp3_file_path = mp3_folder + "107.mp3";
    Play(mp3_file_path);
}


void AudioPlayer::GetJSONFromURL(){
    string file = "/home/jetson/braintemp/audios.json";
    string command = ("wget -q https://dev2.teleop.tortops.com/audio/list/"
                      + imei + " -O " + file);

    system(command.c_str());
    ifstream strm(file);
    audios = json::parse(strm);
}

vector<string> FindFileName(const string& pattern){
    glob_t glob_result;
    memset(&glob_result, 0, sizeof(glob_result));
    vector<string> filenames;

    int return_value = glob(pattern.c_str(), GLOB_TILDE, NULL, &glob_result);
    if(return_value != 0) {
        ROS_DEBUG_STREAM("No file matches pattern: " << pattern);
    } else {
        for(size_t i = 0; i < glob_result.gl_pathc; ++i) {
            filenames.push_back(string(glob_result.gl_pathv[i]));
        }
    }
    globfree(&glob_result);
    return filenames;
}

vector<string> Split(string data, char spliter)
{
	vector<string> tokens;
	string temp;
	stringstream check1(data);
	while(getline(check1, temp, spliter))
	{
		tokens.push_back(temp);
	}
	// for(int i = 0; i < tokens.size(); i++)
    //     cout << i << "(" << tokens[i].length() <<"):" << tokens[i] << endl;
	return tokens;
}

vector<string> AudioPlayer::GetLocalFileInfo(string id){
    // Obtener hash de archivo con <id>-*.mp3
    vector<string> result(2);
    string hash, current_file;
    string fpath = mp3_folder + id +"-*.mp3";
    vector<string> paths = FindFileName(fpath);
    if (paths.size()){
        // Si hay un audio con dicho <id>, obtener su hash
        current_file = paths[0];
        vector<string> spl;
        spl = Split(current_file,'/');
        string fname = spl[spl.size()-1];
        spl = Split(fname,'-');
        spl = Split(spl[1],'.');
        hash = spl[0];
    }
    result[0] = hash;
    result[1] = current_file;
    return result;
}

bool AudioPlayer::GenerateTTS(string text, string path, string lang){
    if (lang.empty()) {
        lang = kDefaultTTSLanguage;
    }

    modules::tts_service tts_srv;
    tts_srv.request.text = text;
    tts_srv.request.path = path;
    tts_srv.request.lang = lang;
    if (!clientTTS.call(tts_srv)){
        ROS_ERROR_STREAM(node_name << " --- TTS service not working.");
        return false;
    } else {
        ROS_DEBUG_STREAM(node_name << " --- new TTS audio!");
        return true;
    }
}

bool FileExists(const string name){
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
}

bool DirExists(const string path){
    struct stat buffer;
    if (stat(path.c_str(), &buffer) == -1) {
        return false;
    } else if (S_ISDIR(buffer.st_mode)) {
        return true;
    }
}

string GetExactFilePath(const string pattern) {
    string result{""};
    vector<string> paths = FindFileName(pattern);
    if (!paths.empty()) result = paths[0];
    return result;
}

void DeleteFile(string path){
    string command = "rm " + path;
    system(command.c_str());
}

void AudioPlayer::UpdateAudios(){
    for (auto& x: audios.items()){
        json audio;
        bool is_text_hash;
        vector<string> local_file;
        string audio_id, audio_hash, audio_text, remote_hash, local_hash;
        string audio_collection, url, new_file, current_file, command;

        audio = x.value();
        audio_id = to_string(audio["audio_id"]);
        ROS_DEBUG_STREAM(audio_id);
        audio_hash = audio["hash"];
        audio_text = audio["audio_text"];
        audio_collection = audio["audio_collection"] ;

        if (!audio_hash.empty()){       // Tiene audio hash
            ROS_DEBUG_STREAM("Tiene audio hash.");
            is_text_hash = false;
            remote_hash = audio_hash;
        } else {                        // No tiene audio hash
            ROS_DEBUG_STREAM("No tiene audio hash.");
            is_text_hash = true;
            remote_hash = to_string(hasher(audio["audio_text"]));
        }
        ROS_DEBUG_STREAM("Remote hash: " << remote_hash);
        local_file = GetLocalFileInfo(audio_id);
        local_hash = local_file[0];
        ROS_DEBUG_STREAM("Local hash: " << local_hash);
        current_file = local_file[1];
        ROS_DEBUG_STREAM("Current file: " << current_file);

        if (local_hash.compare(remote_hash)){
            // Los hashes son diferentes
            ROS_DEBUG_STREAM("Hashes diferentes!");
            new_file = mp3_folder + audio_id + "-" + remote_hash + ".mp3";
            if(is_text_hash){
                // Hash remoto es text hash -> generar audio a partir del texto
                ROS_DEBUG_STREAM("Text hash -> Generar mp3...");
                if (GenerateTTS(audio_text,new_file)
                    and FileExists(current_file)){
                    DeleteFile(current_file);
                }
            } else {
                // Hash remoto es audio hash -> descargar audio desde URL
                ROS_DEBUG_STREAM("Audio hash -> Descargando mp3...");
                url = ("https://dev2.teleop.tortops.com/audio/file/"
                       + audio_collection + "/" + audio_id);
                command = "wget -q " + url + " -O " + new_file;
                system(command.c_str());
                if (FileExists(current_file)){
                    DeleteFile(current_file);
                }
            }
        } else {
            ROS_DEBUG_STREAM("Hashes iguales. No hacer nada.");
        }
    }
}

void AudioPlayer::InitPairing() {
  int nRet = system(pairing_script_path.c_str());
  if (nRet) {
    ROS_ERROR("%s -- Error in pairing script", node_name.c_str());
  } else {
    ROS_INFO("%s -- Pairing script succesfully executed", node_name.c_str());
  }
}

bool AudioPlayer::PairingService(std_srvs::SetBool::Request& request,
                             std_srvs::SetBool::Response& response) {
  ROS_DEBUG_STREAM(node_name << " --- Pairing service");
  response.success = true;

  if (request.data) {
    pairing_script_path = joystick_pkg_path + "/scripts/bt_con.py -r -s";
  } else {
    /* Just remove previous paired devices*/
    pairing_script_path = joystick_pkg_path + "/scripts/bt_con.py -r -e -s";
  }

  /* Timeout para llamar funcion de pairing (non blocking service)*/
  if (pairing_timeout) {
    pairing_timeout.stop();
    pairing_timeout.setPeriod(ros::Duration(1));
    pairing_timeout.start();
  } else {
    pairing_timeout = nh.createTimer(
        ros::Duration(1), std::bind(&AudioPlayer::InitPairing, this), true);
  }

  return response.success;
}

std::string AudioPlayer::GetAudioFilePath(std::string audio_id) {
  std::string audio_file;

  /* Custom audio from google bucket */
  if (DirExists(custom_mp3_dir_path)) {
    audio_file =
        GetExactFilePath(custom_mp3_dir_path + "/" + audio_id + "*.mp3");
    if (!audio_file.empty()) {
      ROS_DEBUG_STREAM("Custom MP3 file: " << audio_file);
      return audio_file;
    }
  }

  /* Generic audio from ros package */
  audio_file = GetExactFilePath(mp3_folder + audio_id + "*.mp3");
  return audio_file;
}

void AudioPlayer::Play(string mp3_file_path, bool force_loop_stop,
                       bool mic_muted) {
  if (mp3_file_path.empty()) {
    ROS_WARN("Play: mp3_file_path is empty!");
    return;
  }

  ROS_DEBUG_STREAM("Playing " << mp3_file_path);
  mpg123_player_->Play(mp3_file_path, force_loop_stop, mic_muted);
  ROS_DEBUG_STREAM("Stopped playing " << mp3_file_path);
}

void AudioPlayer::AudioLoop(std::string path_id, int period) {
  ROS_DEBUG_STREAM(node_name << " --- Audio Loop function");
  if (path_id.empty()) {
    ROS_WARN("Audio Loop: Path id is empty!");
    return;
  }

  /* Period equals to 0 is not allowed */
  if (period == 0) {
    ROS_DEBUG("Audio Loop: No time period specified");
    return;
  }

  auto itr = audio_loop_map.find(path_id);
  /* If period is defined enable audio loop */
  if (period > 0) {
    ROS_DEBUG("Enabling audio loop");
    if (itr != audio_loop_map.end()) {
      ROS_DEBUG("%s audio loop found, reconfiguring...", path_id.c_str());
      itr->second.stop();
      itr->second.setPeriod(ros::Duration(period));
      itr->second.start();
    } else {
      ROS_DEBUG("%s audio loop not found, creating...", path_id.c_str());
      audio_timer =
          nh.createTimer(ros::Duration(period),
                         [=](const ros::TimerEvent&) { Play(path_id); });
      audio_loop_map.insert(
          pair<std::string, ros::Timer>(path_id, audio_timer));
    }
  } else {
    /* If period is negative disable audio loop */
    ROS_DEBUG("Disabling audio loop");
    if (itr != audio_loop_map.end()) {
      itr->second.stop();
    }
  }
}

void AudioPlayer::ConfigVolControlAndRosCallbacks(void) {
  std_msgs::Int32 vol_msg;
  std_srvs::Trigger trigger_srv;
  auto& heartbeat = heartbeat_ros_srv_client_;

  try {
    int vol = 0;

    speaker_set_vol_ros_sub_ =
        nh.subscribe(kSetVolTopicName, kRosSubQueueSize,
                     &AudioPlayer::SetSpeakerVolCallback, this);
    speaker_vol_ros_pub_ = nh.advertise<std_msgs::Int32>(
        kSpeakerVolTopicName, kRosPubQueueSize, true);
    heartbeat = nh.serviceClient<std_srvs::Trigger>(kHeartbeatServiceName);

    PersistenceReaderWriter::GetInstance().Initialize(kRosPackageName);
    vol = PersistenceReaderWriter::GetInstance().ReadInt(kVolumePerstKey);
    mpg123_player_->SetPercentageVolume(vol);
  } catch (std::invalid_argument& error) {
    ROS_ERROR(
        "An invalid persistent value of the speaker volume was"
        " retrieved: %s, setting a default value (%d)",
        error.what(), kVolumeDefaultValue);
    mpg123_player_->SetPercentageVolume(kVolumeDefaultValue);
  } catch (std::runtime_error& error) {
    ROS_ERROR(
        "An error occurred while trying to retrieve the persistent"
        " value of the speaker volume: %s, setting a default value"
        " (%d)",
        error.what(), kVolumeDefaultValue);
    mpg123_player_->SetPercentageVolume(kVolumeDefaultValue);
  }

  vol_msg.data = mpg123_player_->GetPercentageVolume();
  speaker_vol_ros_pub_.publish(vol_msg);
  heartbeat.call(trigger_srv);
}

void AudioPlayer::SpeakerCallback(const modules::speaker_msg msg) {
  std::string id = msg.mp3Id;
  std::string text = msg.text;
  std::string mp3_file_path;

  if (!text.empty()) {
    string hashID = to_string(hasher(text));
    mp3_file_path = kTTSTempFolder + "tts-" + hashID + ".mp3";
    if (!FileExists(mp3_file_path)) {
      ROS_INFO_STREAM("File " << mp3_file_path << " not found, generating...");
      if (!GenerateTTS(text, mp3_file_path, msg.lang)) {
        ROS_WARN_STREAM(node_name
                        << " --- mp3 was not successfully generated!");
      }
    }

    Play(mp3_file_path, false, true);
  } else if (!id.empty()) {
    mp3_file_path = GetAudioFilePath(id);

    Play(mp3_file_path, false, true);
    AudioLoop(mp3_file_path, msg.period);
  } else {
    ROS_WARN_STREAM(node_name << " --- Either there was no ID or text in the"
                                 " speaker message");
  }
}

void
AudioPlayer::CardTapEventMsgCallback(const std_msgs::Empty tap_event)
{
    const char* kLoadingAudio = "108";
    std::string file_path;

    file_path = GetAudioFilePath(kLoadingAudio);

    ROS_INFO("Playing loading");
    mpg123_player_->PlayLoop(file_path);
}

void
AudioPlayer::TransactionResponseMsgCallback(
                 const std_msgs::Bool trans_response)
{
    const char* kApprovedAudio = "144";
    const char* kNotApprovedAudio = "132";
    std::string file_path;

    if(trans_response.data == true)
    {
        file_path = GetAudioFilePath(kApprovedAudio);
    }
    else
    {
        file_path = GetAudioFilePath(kNotApprovedAudio);
    }

    ROS_DEBUG_STREAM("Playing " << file_path);
    mpg123_player_->Play(file_path, true, true);
}

void AudioPlayer::SpeechTranscriptCallback(
                const drivers::transcript_msg transcript_msg)
{
  const char* kSentAudio = "103";
  std::string file_path;

  if (transcript_msg.answer.empty()) {
    file_path = GetAudioFilePath(kSentAudio);
    Play(file_path, false, false);
  } else {
    string hashID = to_string(hasher(transcript_msg.answer));
    file_path = kTTSTempFolder + "tts-" + hashID + ".mp3";
    if (!FileExists(file_path)) {
        ROS_INFO_STREAM("File " << file_path << " not found, generating...");
        if (!GenerateTTS(transcript_msg.answer, file_path)) {
        ROS_WARN_STREAM(node_name
                        << " --- mp3 was not successfully generated!");
        }
    }
    Play(file_path, false, true);
  }
}

void AudioPlayer::SetSpeakerVolCallback(const std_msgs::Int32 volume)
{
    try
    {
        std_msgs::Int32 vol_msg;
        std_srvs::Trigger trigger_srv;

        mpg123_player_->SetPercentageVolume(volume.data);
        PersistenceReaderWriter::GetInstance().WriteInt(kVolumePerstKey,
                                                        (int) volume.data);
        vol_msg.data = mpg123_player_->GetPercentageVolume();
        speaker_vol_ros_pub_.publish(vol_msg);
        heartbeat_ros_srv_client_.call(trigger_srv);
    }
    catch(std::invalid_argument& error)
    {
        ROS_ERROR("Couldn't set the speaker volume: %s", error.what());
    }
    catch(std::runtime_error& error)
    {
        ROS_ERROR("Couldn't write the speaker volume for persistent data: %s",
                  error.what());
    }
}
