#include "NmeaParser.h"

#include "helper.h"

const char kSentenceStart = '$';
const char kCheckSumDelim = '*';
const char kDataFieldDelim = ',';
const char kSentenceEnd1 = '\r';
const char kSentenceEnd2 = '\n';

NmeaParser::NmeaParser() { resetSentence(); }

NmeaParser::SentenceType NmeaParser::parseChar(const char c) {
  DEBUG_PRINT("Received char: ");
  DEBUG_PRINTLN(c);

  // Control state transitions.
  switch (state_) {
  case State::kUnknown:
    if (c == kSentenceStart) { // Start sentence.
      transitionState(State::kId);
    }
    break;
  case State::kId:
    addToCheckSum(c);
    addCharacter(c, id_, kIdSize);  // Fill ID field.
    if (wrd_idx_ == kIdSize) {      // ID complete.
      transitionState(State::kMsg); // Transition to MSG field.
    }
    break;
  case State::kMsg:
    addToCheckSum(c);
    addCharacter(c, msg_, kMsgSize);      // Fill MSG field.
    if (c == kDataFieldDelim) {           // MSG type complete
      transitionState(State::kDataField); // Transition to first data field.
    }
    break;
  case State::kDataField:
    addToCheckSum(c);
    addCharacter(c, data_field_, kDataFieldSize); // Fill data field.
    if (c == kDataFieldDelim) { // Data field complete and next data field.
      transitionState(State::kDataField);
    } else if (c == kCheckSumDelim) { // Data field complete followed by cs.
      transitionState(State::kCheckSum);
    }
    break;
  case State::kCheckSum:
    addCharacter(c, cs_, kCsSize);                      // Fill check sum.
    if ((c == kSentenceEnd1) || (c == kSentenceEnd2)) { // Done!
      transitionState(State::kSuccess);
    }
    break;
  default:
    transitionState(State::kUnknown);
  }

  return sentence_type_;
}

void NmeaParser::resetSentence() {
  memset(id_, '\0', kIdSize + 1);
  memset(msg_, '\0', kMsgSize + 1);
  memset(cs_, '\0', kCsSize + 1);
  memset(data_field_, '\0', kDataFieldSize + 1);
  cs_calculated_ = 0x00;

  id_type_ = IdType::kUnknown;
  msg_type_ = MsgType::kUnknown;
  sentence_type_ = SentenceType::kUnknown;
  df_idx_ = 0;
  resetWord();
}

void NmeaParser::resetWord() { wrd_idx_ = 0; }

void NmeaParser::transitionState(const State new_state) {
  // Execute state transitions. If the transition fails the state machine goes
  // into state Unknown.
  bool success = true;

  switch (new_state) {
  case State::kUnknown: // Can be reached from any state. Starting point.
    resetSentence();
    success &= true;
    break;
  case State::kId:
    if (state_ == State::kUnknown) { // Transition from kUnknown.
      success &= true;               // Do nothing.
    }
    break;
  case State::kMsg:
    if (state_ == State::kId) {   // Transition from kId.
      success &= processIdType(); // Check valid ID.
    }
    break;
  case State::kDataField:
    if (state_ == State::kMsg) {              // Transition from kMsg.
      success &= processMsgType();            // Check valid message.
    } else if (state_ == State::kDataField) { // Transition from kDataField.
      success &= processDataField();          // Check valid data.
    }
    break;
  case State::kCheckSum:
    if (state_ == State::kDataField) { // Transition from kDataField.
      success &= processDataField();   // Check valid data.
    }
    break;
  case State::kSuccess:
    if (state_ == State::kCheckSum) { // Transition from kCheckSum.
      success &= processCheckSum();   // Check valid check sum.
    }
  default:
    break;
  }

  if (success) {
    resetWord();
    state_ = new_state;
  } else {
    DEBUG_PRINTLN("Failed transition.");
    resetSentence();
    state_ = State::kUnknown;
  }
}

void NmeaParser::addCharacter(const char c, char *field, const uint8_t len) {

  if ((wrd_idx_ < len) && (c != kDataFieldDelim) && (c != kCheckSumDelim) &&
      (c != kSentenceEnd1) && (c != kSentenceEnd2)) {
    DEBUG_PRINT("Adding letter: ");
    DEBUG_PRINTLN(c);
    *(field + wrd_idx_++) = c;
  }
}

void NmeaParser::addToCheckSum(const char c) {
  if (c != kCheckSumDelim)
    cs_calculated_ ^= c;
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
  case MsgType::kZda:
    success = zda_message_.update(data_field_, wrd_idx_, df_idx_);
    break;
  default:
    break;
  }

  df_idx_++; // Increment data field number.
  return success;
}

bool NmeaParser::processCheckSum() {
  uint8_t cs_received = strtol(cs_, NULL, 16);
  return cs_received == cs_calculated_;
}

bool ZdaMessage::update(const char *data, const uint8_t len,
                        const uint8_t field) {
  bool success = false;

  switch (field) {
  case 0:
    success = numFromWord<uint8_t>(data, len, 0, 2, &hour);
    DEBUG_PRINT(hour);
    DEBUG_PRINT("(");
    DEBUG_PRINT(success);
    DEBUG_PRINT(")");
    DEBUG_PRINT(":");
    success &= numFromWord<uint8_t>(data, len, 2, 2, &minute);
    DEBUG_PRINT(minute);
    DEBUG_PRINT("(");
    DEBUG_PRINT(success);
    DEBUG_PRINT(")");
    DEBUG_PRINT(":");
    success &= numFromWord<uint8_t>(data, len, 4, 2, &second);
    DEBUG_PRINT(second);
    DEBUG_PRINT("(");
    DEBUG_PRINT(success);
    DEBUG_PRINT(")");
    DEBUG_PRINT(".");
    success &= updateHundredths(data, len);
    DEBUG_PRINT(hundreth);
    DEBUG_PRINT("(");
    DEBUG_PRINT(success);
    DEBUG_PRINTLN(")");
    break;
  case 1:
    success = numFromWord<uint8_t>(data, len, 0, 2, &day);
    DEBUG_PRINTLN(day);
    break;
  case 2:
    success = numFromWord<uint8_t>(data, len, 0, 2, &month);
    DEBUG_PRINTLN(month);
    break;
  case 3:
    success = numFromWord<uint16_t>(data, len, 0, 4, &year);
    DEBUG_PRINTLN(year);
    break;
  case 4:
    success = true; // Ignore time zone field.
    break;
  case 5:
    success = true; // Ignore time zone offset field.
    break;
  default:
    break;
  }

  if (!success)
    reset();

  return success;
}

bool ZdaMessage::updateHundredths(const char *data, const uint8_t data_len) {
  hundreth = 0;

  if (data_len < 7)
    return true; // No decimal seconds.
  else if (*(data + 6) != '.')
    return false; // Missing decimal point.
  else if (data_len < 8)
    return true; // No digits.

  uint8_t len = data_len - 7; // Get tail length.
  return numFromWord<uint32_t>(data, data_len, 7, len, &hundreth);
}

// void NmeaParser::clearBuffer() {
//   memset(buffer_, '\0', kMaxSentenceLength);
//   idx = 0;
//   cs_calculated_ = 0x00;
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
//     cs_calculated_ ^= *word++; // XOR all bytes.
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
//   DEBUG_PRINTLN(cs_calculated_);
//
//   return cs == cs_calculated_;
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
