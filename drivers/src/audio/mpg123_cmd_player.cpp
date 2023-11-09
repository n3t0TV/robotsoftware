#include <cmath>
#include <ros/ros.h>

#include "classes/audio/mpg123_cmd_player.h"
#include "classes/audio/async_cmd_runner_exception.h"


Mpg123Player::Mpg123Player(std::string node_name) : SpeakerJetson(node_name)
{
}

void
Mpg123Player::Play(std::string audio_file_path, bool force_loop_stop,
                        bool mic_muted) {
  std::string cmd = GetCmd(audio_file_path);
  int exit_code = 0;

  if (force_loop_stop) {
    StopLoopAudioIfRunning();
  } else if (loop_audio_cmd_.IsCmdRunning()) {
    return;
  }

  EnableSpeaker();
  MuteMicrophone(mic_muted);
  exit_code = system(cmd.c_str());
  UnmuteMicrophone(mic_muted);
  DisableSpeaker();

  if (exit_code != 0) {
    ROS_ERROR("The command \"%s\" returned error exit code %d", cmd.c_str(),
              exit_code);
  }
}


void
Mpg123Player::PlayLoop(std::string audio_file_path)
{
    std::string cmd = GetLoopCmd(audio_file_path);

    try
    {
        loop_audio_cmd_.StopCmd();
        EnableSpeaker();
        loop_audio_cmd_.StartCmd(cmd.c_str(), kLoopAudioTimeoutInMs);
    }
    catch(const AsyncCmdRunnerException& err)
    {
        DisableSpeaker();
        ROS_ERROR("Couldn't play audio in a loop with command \"%s\"",
                      cmd.c_str());
    }
}


void
Mpg123Player::StopLoopAudioIfRunning(void)
{
    DisableSpeaker();
    loop_audio_cmd_.StopCmd();
}


void
Mpg123Player::MuteMicrophone(bool enable)
{
  if (!enable) {
    return;
  }

  int exit_code = 0;
  std::string exec_cmd(kPulseMuteMicCmd);
  exec_cmd.append(kNullRedirectStdOutputCmd);

  exit_code = system(exec_cmd.c_str());
  if (exit_code != 0) {
    ROS_ERROR("The command \"%s\" returned error exit code %d",
              exec_cmd.c_str(), exit_code);
  } else {
    ROS_INFO("Microphone muted!");
  }
}


void
Mpg123Player::UnmuteMicrophone(bool enable)
{
  if (!enable) {
    return;
  }

  int exit_code = 0;
  std::string exec_cmd(kPulseUnmuteMicCmd);
  exec_cmd.append(kNullRedirectStdOutputCmd);

  exit_code = system(exec_cmd.c_str());
  if (exit_code != 0) {
    ROS_ERROR("The command \"%s\" returned error exit code %d",
              exec_cmd.c_str(), exit_code);
  } else {
    ROS_INFO("Microphone unmuted!");
  }
}


void
Mpg123Player::SetPercentageVolume(int percent_vol)
{
    if((kMinPercentageVol <= percent_vol) && (percent_vol <= kMaxPercentageVol))
    {
        percentage_volume_ = percent_vol;
        SetScaleFactorFromPercentVol(percent_vol);
    }
    else
    {
        std::stringstream err_msg_stream;

        err_msg_stream << "The provided percentage volume (0 to 100) is invalid"
                       << " " << percent_vol;

        throw std::invalid_argument(err_msg_stream.str());
    }
}


int
Mpg123Player::GetPercentageVolume(void)
{
    return percentage_volume_;
}


void
Mpg123Player::SetScaleFactorFromPercentVol(int percent_vol)
{
    const static double kOffset = std::log(kExpOffset);
    const static double kScaleVol = (std::log(kMaxScaleFactor + kExpOffset)
                                     - kOffset);

    double scaled_vol = ((kScaleVol * (double) percent_vol)
                         / ((double) kMaxPercentageVol));

    scale_factor_ = (int) (std::exp(scaled_vol + kOffset) - kExpOffset);
    scale_factor_ = ((scale_factor_ < kMinScaleFactor)
                     ? kMinScaleFactor : scale_factor_);
    scale_factor_ = ((scale_factor_ > kMaxScaleFactor)
                     ? kMaxScaleFactor : scale_factor_);
}


std::string
Mpg123Player::GetCmd(std::string audio_file_path)
{
    std::stringstream cmd_stream;

    cmd_stream << kMpg123Cmd << " " << kMpg123QuietOutput << " "
               << kMpg123ChangeScaleFactor << " "
               << scale_factor_ << " " << audio_file_path;

    return cmd_stream.str();
}


std::string
Mpg123Player::GetLoopCmd(std::string audio_file_path)
{
    std::stringstream cmd_stream;

    cmd_stream << kMpg123Cmd << " " << kMpg123InfiniteLoop << " "
               << kMpg123QuietOutput << " " << kMpg123ChangeScaleFactor << " "
               << scale_factor_ << " " << audio_file_path;

    return cmd_stream.str();
}
