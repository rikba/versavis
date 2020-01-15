////////////////////////////////////////////////////////////////////////////////
//  January 2020
//  Author: Rik Bähnemann <brik@ethz.ch>
////////////////////////////////////////////////////////////////////////////////
//  NmeaParser.h
////////////////////////////////////////////////////////////////////////////////
//
//  Parse NMEA messages received on Serial.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef NmeaParser_h_
#define NmeaParser_h_

#include "versavis_configuration.h"
#include <Arduino.h>
#include <RTClib.h>

struct ZdaMessage {
//  enum class DataFields {kUtcTime, }

};

class NmeaParser {
public:
  enum class SentenceType { kGpzda, kUnknown };
  enum class State { kId, kMsg, kDataField, kCheckSum, kSuccess, kUnknown };
  enum class IdType { kGps, kUnknown };
  enum class MsgType { kZda, kUnknown };

  NmeaParser();
  // Parse an individual character from serial buffer. If sentence is finished
  // return the sentence type.
  SentenceType parseChar(const char c);

private:
  // NMEA description https://resources.winsystems.com/software/nmea.pdf
  // $->ID->MSG->','->Dn->*->CS->[CR][LF]

  // Sentence storage.
  static const uint8_t kIdSize = 2;
  static const uint8_t kMsgSize = 3;
  static const uint8_t kCsSize = 2;
  // Max size minus minimum info.
  static const uint8_t kDataFieldSize = 79 - kIdSize - kMsgSize - kCsSize - 1;
  char id_[kIdSize];
  char msg_[kMsgSize];
  char cs_[kCsSize];
  char data_field_[kDataFieldSize];
  uint8_t calculated_cs_ = 0x00;

  // State and message info.
  State state_ = State::kUnknown;
  IdType id_type_ = IdType::kUnknown;
  MsgType msg_type_ = MsgType::kUnknown;
  SentenceType sentence_type_ = SentenceType::kUnknown;
  uint8_t wrd_idx_ = 0;
  uint8_t df_idx_ = 0;

  void resetSentence();
  void resetWord();
  void transitionState(const State new_state);
  void addCharacter(const char c, char *field, const uint8_t len);

  bool terminateId();
  bool terminateMsg();
  bool terimateDataFieldAndStartNext();
  bool terminateDataFieldAndStartCs();

  bool processIdType();
  bool processMsgType();
  bool processDataField();
  bool processCheckSum();

  bool processZdaMessage();
};

#endif
