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
#include <Uart.h>

template <class T>
bool numFromWord(const char *data, const uint8_t start_idx, const uint8_t len,
                 T *result) {
  bool success = ((start_idx + len - 1) < strlen(data));

  // Create copy of data range.
  char cpy[len + 1];
  memset(cpy, '\0', len + 1);
  memcpy(cpy, data + start_idx, len);

  // Check if all digits.
  for (auto i = 0; i < len; ++i) {
    success &= isDigit(cpy[i]);
  }

  // Convert to unsigned long integer.
  auto conversion = strtoul(cpy, NULL, 10);

  // Check within data range.
  T numeric_limit = ~T(0); // Bitwise NOT of 0. WARNING: only for unsigned int
  success &= (conversion < numeric_limit); // Inside target object range.

  if (result)
    *result = conversion;

  return success;
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
  char str[23]; // TODO(rikba): Add this member only for debugging.

  bool update(const char *data, const uint8_t field);
  inline void reset() { *this = ZdaMessage(); }

private:
  void toString();
  bool updateHundredths(const char *data);
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
  uint8_t df_idx_ = 0;  // The index of the data field in the current sentence.

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
