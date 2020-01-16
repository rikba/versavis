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
public:
  uint8_t hour = 0;
  uint8_t minute = 0;
  uint8_t second = 0;
  uint32_t hundreth = 0;
  uint8_t day = 0;
  uint8_t month = 0;
  uint16_t year = 0;

  bool update(const char *data, const uint8_t field);
  inline void reset() { *this = ZdaMessage(); }

private:
  bool updateHundredths(const char *data);

  template <class T>
  bool numFromWord(const char *data, const uint8_t start_idx, const uint8_t len,
                   T *result) {
    *result = 0;

    T numeric_limit = ~T(0); // Bitwise NOT of 0. WARNING: only for unsigned int

    if (!result)
      return false;

    uint8_t num_digits = len - start_idx;

    if (sizeof(data) < num_digits)
      return false;

    T power = 1;
    for (auto i = start_idx + len - 1; i >= start_idx; i--) {
      auto digit = data + i;
      if (!isDigit(*digit))
        return false;

      // Savely multiply by power.
      uint8_t num = atoi(digit);
      if (num > numeric_limit / power)
        return false;
      T summand = power * num;

      // Savely add summand.
      if (summand > numeric_limit - *result)
        return false;
      *result += power * atoi(digit);

      // Update power.
      if (i == start_idx) {
        return true;
      } else if (power > (numeric_limit / 10)) {
        return false;
      } else {
        power *= 10;
      }
    }

    return true;
  }
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