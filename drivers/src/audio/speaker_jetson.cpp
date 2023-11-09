#include <ros/ros.h>

#include "classes/audio/speaker_jetson.h"

SpeakerJetson::SpeakerJetson(std::string node_name):node_name_(node_name)
{
    /* Open GPIO chip */
    chip = gpiod_chip_open_by_name(chipname);
    if(chip == NULL)
        ROS_ERROR_STREAM(node_name_ << " - GPIO error: chip is null");
    /* Open GPIO lines */
    lineSpeakerEn = gpiod_chip_get_line(chip, SPEAK_ENA_PIN);
    if(lineSpeakerEn == NULL)
        ROS_ERROR_STREAM(node_name_ << " - GPIO error: line are null");
    gpiod_line_request_output(lineSpeakerEn, node_name_.c_str(), 1);
}

SpeakerJetson::~SpeakerJetson(void)
{
    /* ROS logs might not be functional at this point */
    printf("[%s] Releasing gpio resources...\n", node_name_.substr(1).c_str());
    gpiod_line_set_value(lineSpeakerEn, 1);
    gpiod_line_release(lineSpeakerEn);
    gpiod_chip_close(chip);
    printf("[%s] GPIO resources released successfully.\n",
           node_name_.substr(1).c_str());
}

void SpeakerJetson::EnableSpeaker(void)
{
    gpiod_line_set_value(lineSpeakerEn, 0);
}

void SpeakerJetson::DisableSpeaker(void)
{
    gpiod_line_set_value(lineSpeakerEn, 1);
}
