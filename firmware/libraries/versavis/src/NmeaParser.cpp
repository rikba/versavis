#include "NmeaParser.h"

#include "helper.h"

const char kSentenceStart = '$';
const char kCheckSumDelim = '*';
const char kDataFieldDelim = ',';
const char kSentenceEnd1 = '\r';
const char kSentenceEnd2 = '\n';

NmeaParser::NmeaParser() {
  resetSentence();
  resetWord();
}

NmeaParser::SentenceType NmeaParser::parseChar(const char c) {
  DEBUG_PRINT("New char: ");
  DEBUG_PRINT(c);
  DEBUG_PRINTLN(" state: ");

  // Control state transitions.
  if (c == kSentenceStart) {
    transitionState(State::kId);
  } else if (state_ == State::kId) {
    // Fill ID field.
    addCharacter(c, id_, kIdSize);
    if (wrd_idx_ == kIdSize) {
      transitionState(State::kMsg);
    }
  } else if (state_ == State::kMsg) {
    // Fill MSG field.
    addCharacter(c, msg_, kMsgSize);
    if (c == kDataFieldDelim) {
      transitionState(State::kDataField);
    }
  } else if (state_ == State::kDataField) {
    // Fill data field.
    addCharacter(c, data_field_, kDataFieldSize);
    if (c == kDataFieldDelim) {
      transitionState(State::kDataField);
    } else if (c == kCheckSumDelim) {
      transitionState(State::kCheckSum);
    }
  } else if (state_ == State::kCheckSum) {
    // Fill check sum.
    addCharacter(c, cs_, kCsSize);
    if ((c == kSentenceEnd1) || (c == kSentenceEnd2)) {
      transitionState(State::kSuccess);
    }
  } else if (state_ == State::kSuccess) {
    transitionState(State::kUnknown);
  } else {
    DEBUG_PRINTLN("Failure state.");
  }

  return sentence_type_;
}

void NmeaParser::resetSentence() {
  memset(id_, '\0', kIdSize);
  memset(msg_, '\0', kMsgSize);
  memset(cs_, '\0', kCsSize);
  memset(data_field_, '\0', kDataFieldSize);
  calculated_cs_ = 0x00;

  id_type_ = IdType::kUnknown;
  msg_type_ = MsgType::kUnknown;
  sentence_type_ = SentenceType::kUnknown;
  df_idx_ = 0;
}

void NmeaParser::resetWord() { wrd_idx_ = 0; }

void NmeaParser::transitionState(const State new_state) {
  // Execute state transitions.
  bool success = false;
  switch (new_state) {
  case State::kId:
    // Initial state, can always be reached.
    resetSentence();
    success = true;
    break;
  case State::kMsg:
    // Transition from ID
    success = processIdType();
    break;
  case State::kDataField:
    // Transition from kMsg to first kDataField.
    if (state_ == State::kMsg) {
      success = processMsgType();
    }
    // Transition from one data field to next data field.
    else {
      success = processDataField();
    }
    break;
  case State::kCheckSum:
    // Transition from data field.
    success = processDataField();
    break;
  case State::kSuccess:
    // Transition from check sum.
    success = processCheckSum();
  default:
    state_ = State::kUnknown;
    break;
  }

  if (success) {
    resetWord();
    state_ = new_state;
  } else {
    resetSentence();
    state_ = State::kUnknown;
  }
}

void NmeaParser::addCharacter(const char c, char *field, const uint8_t len) {
  if ((wrd_idx_ < len) && (c != kDataFieldDelim) && (c != kCheckSumDelim) &&
      (c != kSentenceEnd1) && (c != kSentenceEnd2)) {
    *(field + wrd_idx_++) = c;
  }
}

bool NmeaParser::processIdType() {
  const char kGps[3] = "GP";

  if (state_ != State::kId) {
    id_type_ = IdType::kUnknown;
  } else if (strcmp(id_, kGps) == 0) {
    id_type_ = IdType::kGps;
    DEBUG_PRINTLN("Detected ID type: GPS");
  } else {
    id_type_ = IdType::kUnknown;
  }

  return id_type_ != IdType::kUnknown;
}

bool NmeaParser::processMsgType() {
  const char kZda[4] = "ZDA";

  if (state_ != State::kMsg) {
    msg_type_ = MsgType::kUnknown;
  } else if (strcmp(msg_, kZda) == 0) {
    msg_type_ = MsgType::kZda;
    DEBUG_PRINTLN("Detected MSG type: ZDA");
  } else {
    msg_type_ = MsgType::kUnknown;
  }

  return msg_type_ != MsgType::kUnknown;
}

bool NmeaParser::processDataField() {
  bool success = false;

  switch (msg_type_) {
  }

  df_idx_++;
  return success;
}

// bool NmeaParser::processZdaMessage() {
// switch (data_field_) {
//  case 0:
//}
//}

bool NmeaParser::processCheckSum() { return false; }

// void NmeaParser::clearBuffer() {
//   memset(buffer_, '\0', kMaxSentenceLength);
//   idx = 0;
//   calculated_cs_ = 0x00;
// }
//
// SentenceType NmeaParser::parseSentence() {
//   auto sentence_type = SentenceType::kUnknown;
//
//   if (parseGpzda())
//     sentence_type == SentenceType::kGpzda;
//
//   return sentence_type;
// }
//
// char *NmeaParser::nextWord(const WordType word_type) {
//   const char kWordDelim[2] = ",";
//   const char kCheckSumDelim[2] = "\r";
//
//   char *word = NULL;
//   switch (word_type) {
//   case WordType::kHeader:
//     word = strtok(buffer_, kWordDelim); // First word, ende with ','
//     break;
//   case WordType::kData:
//     word = strtok(NULL, kWordDelim); // Next word, ends with ','
//     break;
//   case WordType::kCheckSum:
//     word = strtok(NULL, kCheckSumDelim); // Last word, ends with '\r'
//     break;
//   default:
//     break;
//   }
//
//   addToCheckSum(word);
//
//   return word;
// }
//
// void NmeaParser::addToCheckSum(const char *word) {
//   if (word == NULL)
//     return;
//
//   // Do not add checksum word.
//   const char kChecksumStart = '*';
//   if (*word == kChecksumStart)
//     return;
//
//   const char kTokenDelimiter = '\0';
//   while (*word != kTokenDelimiter)
//     calculated_cs_ ^= *word++; // XOR all bytes.
// }
//
// bool NmeaParser::checkSum(const char *word) {
//   if (word == NULL)
//     return false;
//
//   int cs = atoi(word);
//   DEBUG_PRINTLN("Checksums");
//   DEBUG_PRINTLN(word);
//   DEBUG_PRINTLN(cs);
//   DEBUG_PRINTLN(calculated_cs_);
//
//   return cs == calculated_cs_;
// }
//
// bool NmeaParser::parseGpzda() {
//   // A GPZDA sentence: $GPZDA,173538.00,14,01,2020,,*69[...]\n
//
//   // Get Header
//   auto header = nextWord(WordType::kHeader);
//   if (header == NULL)
//     return false;
//   // Check header.
//   const char kHeaderGpzda[6] = "GPZDA";
//   if (strcmp(header, kHeaderGpzda) != 0)
//     return false;
//
//   // Get time.
//   auto time = nextWord(WordType::kData);
//   if (time == NULL)
//     return false;
//   DEBUG_PRINTLN(time);
//
//   // Get day.
//   auto day = nextWord(WordType::kData);
//   if (day == NULL)
//     return false;
//   DEBUG_PRINTLN(day);
//
//   // Get month.
//   auto month = nextWord(WordType::kData);
//   if (month == NULL)
//     return false;
//   DEBUG_PRINTLN(month);
//
//   //  // Get year.
//   //  auto year = nextWord(WordType::kData);
//   //  if (year == NULL)
//   //    return false;
//   //  DEBUG_PRINTLN(year);
//   //
//   //  // Get zone.
//   //  auto zone = nextWord(WordType::kData);
//   //  if (zone == NULL)
//   //    return false;
//   //  DEBUG_PRINTLN(zone);
//   //
//   //  // Get zone minutes.
//   //  auto zone_minutes = nextWord(WordType::kData);
//   //  if (zone_minutes == NULL)
//   //    return false;
//   //  DEBUG_PRINTLN(zone_minutes);
//
//   // Get check sum.
//   auto cs = nextWord(WordType::kCheckSum);
//   if (cs == NULL)
//     return false;
//   DEBUG_PRINTLN(cs);
//
//   if (!checkSum(cs)) {
//     DEBUG_PRINTLN("WRONG CS!!");
//     return false;
//   }
//
//   return true;
// }

// bool NmeaParser::parseBuffer(char *buf, uint8_t len) {
//   if (!buf || len == 0)
//     return false; // Invalid buffer.
//
//   // Copy buffer.
//   char buffer_copy[len];
//   memcpy(buffer_copy, buf, len);
//
//   buffer_ = static_cast<char *>(buf);
//   len_ = len;
//
//   auto sentence = getNextSentence(buffer_); // First sentence.
//   while (sentence) {
//     // auto header = getNextWord(sentence); // Header.
//     // auto sentence_type = getSentenceType(header);
//     // switch (sentence_type) {
//     // case SentenceType::kGpzda:
//     //  DEBUG_PRINTLN(header);
//     //  break;
//     // default:
//     //  DEBUG_PRINT("Not implemented.");
//     //  DEBUG_PRINTLN(header);
//     //}
//     // if (word)
//     DEBUG_PRINTLN(sentence);
//     // DEBUG_PRINTLN(sentence);
//     sentence = getNextSentence();
//   }
//
//   // if (findHeader("$GPZDA")) {
//   //   DEBUG_PRINTLN("Found header.");
//   // }
//   //
//   // DEBUG_PRINTLN("AGAIN");
//   // if (findHeader("$GPZDA")) {
//   //   DEBUG_PRINTLN("Found header.");
//   // }
//
//   return false;
// }
//
// bool parseSentence(char *token) {
//   if (token == NULL)
//     return false;
// }
//
// char *NmeaParser::getNextSentence(char *token /* = NULL */) {
//   auto sentence = strtok(token, kSentenceDelim);
//   return sentence;
// }
//
// char *NmeaParser::getNextWord(char *token /* = NULL */) {
//   auto word = strtok(token, kWordDelim);
//   return word;
// }
//
// SentenceType NmeaParser::getSentenceType(char *header) {
//   if (header == NULL)
//     return SentenceType::kUnknown;
//
//   if (strcmp(header, "$GPZDA") == 0) {
//     return SentenceType::kGpzda;
//   } else {
//     return SentenceType::kUnknown;
//   }
// }
