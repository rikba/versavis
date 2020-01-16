#include "NmeaParser.h"

#include "helper.h"

const char kSentenceStart = '$';
const char kCheckSumDelim = '*';
const char kDataFieldDelim = ',';
const char kSentenceEnd1 = '\r';
const char kSentenceEnd2 = '\n';

NmeaParser::NmeaParser() { resetSentence(); }

NmeaParser::SentenceType NmeaParser::parseChar(const char c) {
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
  case State::kSuccess:
    transitionState(State::kUnknown);
    break;
  default:
    transitionState(State::kUnknown);
  }

  return sentence_type_;
}

void NmeaParser::resetSentence() {
  resetWord();

  memset(id_, '\0', kIdSize + 1);
  memset(msg_, '\0', kMsgSize + 1);
  memset(cs_, '\0', kCsSize + 1);
  cs_calculated_ = 0x00;

  id_type_ = IdType::kUnknown;
  msg_type_ = MsgType::kUnknown;
  sentence_type_ = SentenceType::kUnknown;
  df_idx_ = 0;
}

void NmeaParser::resetWord() {
  memset(data_field_, '\0', kDataFieldSize + 1);
  wrd_idx_ = 0;
}

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
    if (state_ == State::kCheckSum) {   // Transition from kCheckSum.
      success &= processCheckSum();     // Check valid check sum.
      success &= processSentenceType(); // Update sentence type.
    }
  default:
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

bool NmeaParser::processSentenceType() {
  if ((id_type_ == IdType::kGps) && (msg_type_ == MsgType::kZda)) {
    sentence_type_ = SentenceType::kGpZda;
  }

  return sentence_type_ != SentenceType::kUnknown;
}

void NmeaParser::addCharacter(const char c, char *field, const uint8_t len) {

  if ((wrd_idx_ < len) && (c != kDataFieldDelim) && (c != kCheckSumDelim) &&
      (c != kSentenceEnd1) && (c != kSentenceEnd2)) {
    *(field + wrd_idx_++) = c;
  }
}

void NmeaParser::addToCheckSum(const char c) {
  if (c != kCheckSumDelim)
    cs_calculated_ ^= c;
}

bool NmeaParser::processIdType() {
  const char kGps[3] = "GP";

  if (strcmp(id_, kGps) == 0) {
    id_type_ = IdType::kGps;
  } else {
    id_type_ = IdType::kUnknown;
  }

  return id_type_ != IdType::kUnknown;
}

bool NmeaParser::processMsgType() {
  const char kZda[4] = "ZDA";

  if (strcmp(msg_, kZda) == 0) {
    msg_type_ = MsgType::kZda;
  } else {
    msg_type_ = MsgType::kUnknown;
  }

  return msg_type_ != MsgType::kUnknown;
}

bool NmeaParser::processDataField() {
  bool success = false;

  switch (msg_type_) {
  case MsgType::kZda:
    success = gp_zda_message_.update(data_field_, df_idx_);
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

bool ZdaMessage::update(const char *data, const uint8_t field) {
  bool success = true;

  switch (field) {
  case 0:
    success &= numFromWord<uint8_t>(data, 0, 2, &hour);
    success &= numFromWord<uint8_t>(data, 2, 2, &minute);
    success &= numFromWord<uint8_t>(data, 4, 2, &second);
    success &= updateHundredths(data);
    break;
  case 1:
    success &= numFromWord<uint8_t>(data, 0, 2, &day);
    break;
  case 2:
    success &= numFromWord<uint8_t>(data, 0, 2, &month);
    break;
  case 3:
    success &= numFromWord<uint16_t>(data, 0, 4, &year);
    break;
  case 4:
    success &= true; // Ignore time zone field.
    break;
  case 5:
    success &= true; // Ignore time zone offset field.
    break;
  default:
    success &= false; // This field is not handled.
    break;
  }

  if (!success) {
    reset();
  } else {
    toString();
  }

  return success;
}

bool ZdaMessage::updateHundredths(const char *data) {
  hundreth = 0;

  auto data_len = strlen(data);
  if (data_len < 7)
    return true; // No decimal seconds.
  else if (*(data + 6) != '.')
    return false; // Missing decimal point.
  else if (data_len < 8)
    return true; // No digits.

  uint8_t len = data_len - 7; // Get tail length.
  return numFromWord<uint32_t>(data, 7, len, &hundreth);
}

void ZdaMessage::toString() {
  sprintf(str, "%02d:%02d:%04d:%02d:%02d:%02d.%02d", day, month, year, hour,
          minute, second, hundreth);
}
