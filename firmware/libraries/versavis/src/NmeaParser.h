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

#include "helper.h"
#include "versavis_configuration.h"
#include <Arduino.h>
#include <RTClib.h>

template <class T>
bool numFromWord(const char *data, const uint8_t data_len,
                 const uint8_t start_idx, const uint8_t len, T *result) {
  *result = 0;

  T numeric_limit = ~T(0); // Bitwise NOT of 0. WARNING: only for unsigned int

  if (!result)
    return false;

  uint8_t end_digit = len + start_idx;

  if (data_len < end_digit)
    return false;

  T factor = 1;
  for (auto i = end_digit - 1; i >= start_idx; i--) {
    if (!isDigit(*(data + i)))
      return false;
    uint8_t digit = *(data + i) - 48; // ASCII to int

    // Savely multiply by factor.
    if (digit > numeric_limit / factor)
      return false;
    T summand = factor * digit;

    // Savely add summand.
    if (summand > numeric_limit - *result)
      return false;
    *result += summand;

    // Update factor.
    if (i == start_idx) {
      return true; // Finished parsing number.
    } else if (factor > (numeric_limit / 10)) {
      return false; // Cannot store number in variable.
    } else {
      factor *= 10;
    }
  }

  return false; // Loop finished early. Should not happen.
}

struct ZdaMessage {
  // A GPZDA sentence: $GPZDA,173538.00,14,01,2020,,*69[...]\n
public:
  uint8_t hour = 0;
  uint8_t minute = 0;
  uint8_t second = 0;
  uint32_t hundreth = 0;
  uint8_t day = 0;
  uint8_t month = 0;
  uint16_t year = 0;

  bool update(const char *data, const uint8_t len, const uint8_t field);
  inline void reset() { *this = ZdaMessage(); }

private:
  bool updateHundredths(const char *data, const uint8_t data_len);
};

class NmeaParser {
public:
  enum class SentenceType { kGpZda, kUnknown };

  NmeaParser();
  // Parse an individual character from serial buffer. If sentence is finished
  // return the sentence type.
  SentenceType parseChar(const char c);

  inline ZdaMessage getGpZdaMessage() { return gp_zda_message_; }

private:
  // NMEA description https://resources.winsystems.com/software/nmea.pdf
  // $->ID->MSG->','->Dn->*->CS->[CR][LF]
  enum class State { kUnknown, kId, kMsg, kDataField, kCheckSum, kSuccess };
  enum class IdType { kGps, kUnknown };
  enum class MsgType { kZda, kUnknown };

  // Sentence storage.
  static const uint8_t kIdSize = 2;
  static const uint8_t kMsgSize = 3;
  static const uint8_t kCsSize = 2;
  // Max size minus minimum info.
  static const uint8_t kDataFieldSize = 79 - kIdSize - kMsgSize - kCsSize - 1;
  // +1 for null termination.
  char id_[kIdSize + 1];
  char msg_[kMsgSize + 1];
  char cs_[kCsSize + 1];
  char data_field_[kDataFieldSize + 1];
  uint8_t cs_calculated_ = 0x00;

  // State and message info.
  State state_ = State::kUnknown;
  IdType id_type_ = IdType::kUnknown;
  MsgType msg_type_ = MsgType::kUnknown;
  SentenceType sentence_type_ = SentenceType::kUnknown;
  uint8_t wrd_idx_ = 0; // The index of the letter in the current word.
  uint8_t df_idx_ = 0;  // The index of the word in the current sentence.

  void resetSentence();
  void resetWord();
  void transitionState(const State new_state);
  void addCharacter(const char c, char *field, const uint8_t len);
  void addToCheckSum(const char c);

  bool terminateId();
  bool terminateMsg();
  bool terimateDataFieldAndStartNext();
  bool terminateDataFieldAndStartCs();

  bool processIdType();
  bool processMsgType();
  bool processDataField();
  bool processCheckSum();
  bool processSentenceType();

  bool processZdaMessage();

  // Received messages.
  ZdaMessage gp_zda_message_;
};

#endif
