#pragma once

#include <string>
#include <gpiod.h>

class SpeakerJetson
{
    public:
        SpeakerJetson(std::string node_name);
        ~SpeakerJetson(void);

    private:
        std::string node_name_;
        /* gpio expander */
        const char *chipname = "gpiochip2";
        /* Pin definitions (Sysfs numbering) from gpio expander */
        const int SPEAK_ENA_PIN = 3;
        struct gpiod_chip *chip;
        struct gpiod_line *lineSpeakerEn;

    protected:
        void EnableSpeaker(void);
        void DisableSpeaker(void);
};
