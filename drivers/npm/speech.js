#!/usr/bin/env node

// Copyright 2019 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * This application demonstrates how to perform infinite streaming using the
 * streamingRecognize operation with the Google Cloud Speech API.
 * Before the streaming time limit is met, the program uses the
 * 'result end time' parameter to calculate the last 'isFinal' transcription.
 * When the time limit is met, the unfinalized audio from the previous session
 * is resent all at once to the API, before continuing the real-time stream
 * and resetting the clock, so the process can repeat.
 * Incoming audio should not be dropped / lost during reset, and context from
 * previous sessions should be maintained as long the utterance returns an
 * isFinal response before 2 * streamingLimit has expired.
 */

'use strict';

const chalk = require('chalk');
const {Writable} = require('stream');
const recorder = require('node-record-lpcm16');
const { NlpManager  } = require('node-nlp');

// Imports the Google Cloud client library
// Currently, only v1p1beta1 contains result-end-time
const speech = require('@google-cloud/speech').v1p1beta1;
const manager = new NlpManager({ languages: ['en']});
const modelProcessThresh = 0.75;


const genericAnswers = [
    'Looking into it',
    'Please wait',
    'Hold on',
    'Just a sec',
    'Just a moment',
    'Hold it right there',
    'Hang on'
];


async function evalPhrase(speech) {
  let response;
  const result = {answer: '', intent: null};

  try {
    response = await manager.process(speech);
    console.log('speech: ', speech);
    console.log('response.classifications: ', response.classifications);
    console.log('response.answer: ', response.answer);
    console.log('response.answers: ', response.answers);

    /**The model couldn't retrieve an answer (or confident answer) */
    if (response.answer === undefined || response.score < modelProcessThresh) {
      result.answer =
          genericAnswers[Math.floor(Math.random() * genericAnswers.length)];
      console.log(result);
      return result;
    }

    if (response.answers.length > 1)  // More than one possible answer
    {
      result.answer =
          response.answers[Math.floor(Math.random() * response.answers.length)]
              .answer;
    } else {
      result.answer = response.answer;
    }
    result.intent = response.intent;
    console.log(result);
    return result;

  } catch (e) {
    console.log('Error generating answer');
    result.answer =
        genericAnswers[Math.floor(Math.random() * genericAnswers.length)];
    console.log(result);
    return result;
  }
}

async function loadModel() {
  try
  {
    /*****CHECK PATH IS CORRECT AND FILE LOADS ****/
    await  manager.load('/home/jetson/setupjetson/ML_MODELS/voicemodel.nlp');
  }
  catch(e)
  {
    console.log('Could not load model file',e);
  }
}

/**
 * Note: Correct microphone settings required: check enclosed link, and make
 * sure the following conditions are met:
 * 1. SoX must be installed and available in your $PATH- it can be found here:
 * http://sox.sourceforge.net/
 * 2. Microphone must be working
 * 3. Encoding, sampleRateHertz, and # of channels must match header of
 * audioInput file you're recording to.
 * 4. Get Node-Record-lpcm16 https://www.npmjs.com/package/node-record-lpcm16
 * More Info: https://cloud.google.com/speech-to-text/docs/streaming-recognize
 * 5. Set streamingLimit in ms. 150000 ms = ~2.5 minutes.
 * Maximum streaming limit should be 1/2 of SpeechAPI Streaming Limit (305 s).
 */
class SpeechRecognize {
  constructor({
    encoding = 'LINEAR16',
    sampleRateHertz = 16000,
    languageCode = 'en-US',
    streamingLimit = 150000,
    GAC = '',
    transcriptCallback = () => void 0,
    logCallback = () => void 0
  } = {}) {
    let context = {
      phrases: ["Toby", "Isaac"],
      boost: 10
    };
    let config = {
      encoding: encoding,
      sampleRateHertz: sampleRateHertz,
      languageCode: languageCode,
      enableAutomaticPunctuation: true,
      speechContexts: [context],
    };
    this.config = config;
    this.request = {
      config,
      interimResults: true,
    };
    this.streamingLimit = streamingLimit;
    this.transcriptCallback = transcriptCallback;
    this.logCallback = logCallback;
    this.restartTimeout = null;

    this.recognizeStream = null;
    this.restartCounter = 0;
    this.audioInput = [];
    this.lastAudioInput = [];
    this.resultEndTime = 0;
    this.isFinalEndTime = 0;
    this.finalRequestEndTime = 0;
    this.newStream = true;
    this.bridgingOffset = 0;
    this.lastTranscriptWasFinal = false;

    process.env.GOOGLE_APPLICATION_CREDENTIALS = GAC;
    this.client = new speech.SpeechClient();
    this.recorder = recorder.record({
      sampleRateHertz: this.sampleRateHertz,
      threshold: 0,  // Silence threshold
      silence: 1000,
      keepSilence: true,
      recordProgram: 'rec',  // Try also "arecord" or "sox"
    });
  }

  startStream() {
    // Clear current audioInput
    this.audioInput = [];
    // Initiate (Reinitiate) a recognize stream
    this.recognizeStream = this.client.streamingRecognize(this.request)
                          .on('error',
                              err => {
                                if (err.code === 11) {
                                  // restartStream();
                                } else {
                                  console.error(
                                      'API request error: ' + err +
                                      ', code: ' + err.code +
                                      ', int code:' + Number(err.code));
                                }
                              })
                          .on('data', this.speechCallback);

    // Restart stream when streamingLimit expires
    this.restartTimeout = setTimeout(this.restartStream.bind(this), this.streamingLimit);
  };

  speechCallback = stream => {
    // Convert API result end time from seconds + nanoseconds to milliseconds
    this.resultEndTime = stream.results[0].resultEndTime.seconds * 1000 +
        Math.round(stream.results[0].resultEndTime.nanos / 1000000);

    // Calculate correct time based on offset from audio sent twice
    const correctedTime =
        this.resultEndTime - this.bridgingOffset + this.streamingLimit * this.restartCounter;

    let stdoutText = '';
    let confidence = '';
    if (stream.results[0] && stream.results[0].alternatives[0]) {
      confidence = stream.results[0].alternatives[0].confidence;
      stdoutText =
          correctedTime + ': ' + stream.results[0].alternatives[0].transcript;
    }

    // this.logCallback(
    //     'isFinal: ' + stream.results[0].isFinal + ', this.lastTranscriptWasFinal: ' +
    //     this.lastTranscriptWasFinal + ', stdoutText: ' + stdoutText);
    if (stream.results[0].isFinal) {
      this.transcriptCallback(
          stream.results[0].alternatives[0].transcript, confidence);

      this.isFinalEndTime = this.resultEndTime;
      this.lastTranscriptWasFinal = true;
    } else {
      this.lastTranscriptWasFinal = false;
    }
  };

  audioInputStreamTransform = new Writable({
    write : (chunk, encoding, next) => {
      if (this.newStream && this.lastAudioInput.length !== 0) {
        // Approximate math to calculate time of chunks
        const chunkTime = this.streamingLimit / this.lastAudioInput.length;
        if (chunkTime !== 0) {
          if (this.bridgingOffset < 0) {
            this.bridgingOffset = 0;
          }
          if (this.bridgingOffset > this.finalRequestEndTime) {
            this.bridgingOffset = this.finalRequestEndTime;
          }
          const chunksFromMS =
              Math.floor((this.finalRequestEndTime - this.bridgingOffset) / chunkTime);
          this.bridgingOffset =
              Math.floor((this.lastAudioInput.length - chunksFromMS) * chunkTime);

          for (let i = chunksFromMS; i < this.lastAudioInput.length; i++) {
            this.recognizeStream.write(this.lastAudioInput[i]);
          }
        }
        this.newStream = false;
      }

      this.audioInput.push(chunk);

      if (this.recognizeStream) {
        this.recognizeStream.write(chunk);
      }

      next();
    },

    final : () => {
      if (this.recognizeStream) {
        this.recognizeStream.end();
      }
    },
  });

  restartStream() {
    this.stopRecording();

    if (this.recognizeStream) {
      this.recognizeStream.end();
      this.recognizeStream.removeListener('data', this.speechCallback);
      this.recognizeStream = null;
    }
    if (this.resultEndTime > 0) {
      this.finalRequestEndTime = this.isFinalEndTime;
    }
    this.resultEndTime = 0;

    this.lastAudioInput = [];
    this.lastAudioInput = this.audioInput;

    this.restartCounter++;

    this.logCallback(
        `${this.streamingLimit * this.restartCounter}: RESTARTING REQUEST`);

    this.newStream = true;

    this.startRecording();
  };

  startRecording() {
    // Start recording and send the microphone input to the Speech API
    if (!this.recorder.process) {
      this.logCallback('Starting new recording process')
      this.recorder.start();
    }
    this.recorder
        .stream()
        .on('error',
            err => {
              console.error('Audio recording error ' + err);
            })
        .pipe(this.audioInputStreamTransform);

    this.startStream();
  };

  stopRecording() {
    this.recorder
        .stream()
        .removeAllListeners('error')
        .unpipe(this.audioInputStreamTransform);
    this.recorder.stop();
    this.recorder.process = null;
    clearTimeout(this.restartTimeout);
    this.logCallback('Recorder stopped!');
  };
}


/**
 * Startting ROS node to publish the trascripted text from the speech API
 */
function createRosNode() {
  const rosnodejs = require('rosnodejs');
  const log = rosnodejs.log;

  log.info('Starting speech ros node ...');
  const rosName = '/speech_recognize_node'
  rosnodejs.initNode(rosName, { onTheFly: true} )
  .then(async (nh) => {
      log.info('Node initialized!');

      await nh.getParam(rosName + '/log_level')
      .then((paramValue) => {
        if (paramValue !== 0) log.getLogger().setLevel(paramValue);
      })
      .catch((error) => {
        log.error("Can't get param: " + rosName + "/log_level");
      });

      const drivers_path = await nh.getParam(rosName + '/package_path')
      .then((paramValue) => {
          return paramValue;
      })
      .catch((error) => {
          log.error("Can't get param: " + rosName + "/package_path");
          return '';
      });

      const pubSpeechCtrl =
          nh.advertise(rosName + '/robot_ctrl', 'std_msgs/Bool');
      const pubTranscript =
          nh.advertise(rosName + '/transcript', 'drivers/transcript_msg');
      const MqttMsg = rosnodejs.require('modules').msg.mqtt_publishers_msg;
      const pubMqttTranscript = nh.advertise('/mqtt_publishers', MqttMsg);
      let mqttTranscriptMsg = new MqttMsg({
        mqtt_topic: 'communication/imei/robot',
        raw_msg: '',
        qos: 0
      });

      const MaxCharPos = 10;
      const MaxSecsToAnswer = 10;
      let answerConditionTimeout;
      let answerCondition = false;
      const rosTranscriptCallback = async (transcript, confidence) => {
        let index = transcript.search(/(\bToby\b|\bIsaac\b)/i);
        if (index != -1 && index < MaxCharPos) {
          answerCondition = true;
          clearTimeout(answerConditionTimeout);
          answerConditionTimeout = setTimeout(() => {
            log.debug("-- Clearing timeout! --")
            answerCondition = false;
          }, (MaxSecsToAnswer * 1000));
        }

        log.debug("transcript: ", transcript, "index: ", index);
        if (answerCondition) {
          //Communication with teleoperator
          mqttTranscriptMsg.raw_msg = JSON.stringify({ text: transcript });
          pubMqttTranscript.publish(mqttTranscriptMsg);

          //Evaluate inmediate answer and response (quick answers/actions)
          let result = await evalPhrase(transcript);
          pubTranscript.publish({
            data: transcript,
            confidence: confidence,
            answer: result.answer
          });

          if (result.intent) {
            switch (result.intent) {
              case "command.lightson":
                //turnLightsOn();
                pubSpeechCtrl.publish({data: true});
                break;

              case "command.lightsoff":
                //turnLightsOff();
                pubSpeechCtrl.publish({data: false});
                break;

              default:
                break;
            }
          }
        }
      }

      const rosLogCallback = (log_msg) => {
        log.debug(log_msg);
      }

      const gac = drivers_path + "/config/credentials.json";
      const speechClass = new SpeechRecognize({
        GAC: gac,
        transcriptCallback: rosTranscriptCallback,
        logCallback: rosLogCallback
      });

      await loadModel();
      speechClass.startRecording();
      log.info('Speech node is listening ...');

      // Test NLP service
      nh.advertiseService(
          rosName + '/test_nlp', 'drivers/string_srv', (req, res) => {
            res.success = true;
            rosTranscriptCallback(req.data, 1);
            return true;
      });

      rosnodejs.on('shutdown', async () => {
        log.info('Stopping speech node ...');
        speechClass.stopRecording();
      });
  });
}

if (require.main === module) {
  createRosNode();
}
