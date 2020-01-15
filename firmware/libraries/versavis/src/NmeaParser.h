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

enum class SentenceType { kGpzda, kUnknown };
enum class WordType { kHeader, kData, kCheckSum };

class NmeaParser {
public:
  NmeaParser();
  // Parse an individual character from serial buffer. If sentence is finished
  // return the sentence type.
  SentenceType parseChar(const char c);
  // NmeaParser() {}
  bool parseBuffer(char *buf, uint8_t len);

private:
  void clearBuffer();
  SentenceType parseSentence();
  bool parseGpzda();
  char *nextWord(const WordType word_type);
  void addToCheckSum(const char *word);
  bool checkSum(const char* word);

  SentenceType getSentenceType(char *header);

  static const uint8_t kMaxSentenceLength = 85;
  char buffer_[kMaxSentenceLength];
  uint8_t idx = 0;
  char check_sum_ = 0x00;

  DateTime date_time_;
};

#endif
