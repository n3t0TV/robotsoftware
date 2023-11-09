#pragma once

#include <string>

#include "classes/audio/async_cmd_runner.h"
#include "classes/audio/speaker_jetson.h"


class Mpg123Player : public SpeakerJetson
{
    public:
        Mpg123Player(std::string node_name);

        void Play(std::string audio_file_path, bool force_loop_stop = false,
                  bool mic_muted = false);

        void PlayLoop(std::string audio_file_path);

        void StopLoopAudioIfRunning(void);

        void MuteMicrophone(bool enable);

        void UnmuteMicrophone(bool enable);

        void SetPercentageVolume(int percent_vol);

        int GetPercentageVolume(void);

    private:
        const int kLoopAudioTimeoutInMs = 15000;

        const char* kMpg123Cmd = "mpg123";

        const char* kMpg123QuietOutput = "-q";

        const char* kMpg123ChangeScaleFactor = "-f";

        const char* kMpg123InfiniteLoop = "--loop -1";

        const char* kPulseMuteMicCmd = "amixer -D pulse set Capture nocap";

        const char* kPulseUnmuteMicCmd = "amixer -D pulse set Capture cap";

        const char* kNullRedirectStdOutputCmd = " > /dev/null";

        const int kMinPercentageVol = 0;

        const int kMaxPercentageVol = 100;

        const int kMinScaleFactor = 0;

        const int kMaxScaleFactor = 32768;

        const double kExpOffset = 2000.0;

        int percentage_volume_ = kMaxPercentageVol;

        int scale_factor_ = kMaxScaleFactor;

        AsyncCmdRunner loop_audio_cmd_;

        void SetScaleFactorFromPercentVol(int percent_vol);

        std::string GetCmd(std::string audio_file_path);

        std::string GetLoopCmd(std::string audio_file_path);
};
