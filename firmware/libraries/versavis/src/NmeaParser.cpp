#include "NmeaParser.h"

#include "helper.h"

NmeaParser::NmeaParser() { clearBuffer(); }

SentenceType NmeaParser::parseChar(const char c) {
  const char kSentenceStart = '$';
  const char kSentenceEnd = '\n';

  auto sentence_type = SentenceType::kUnknown;
  if (c == kSentenceStart) {
    DEBUG_PRINTLN("New sentence.");
    clearBuffer(); // New sentence.
  } else if (c == kSentenceEnd) {
    sentence_type = parseSentence(); // Sentence done.
  } else if (idx < kMaxSentenceLength) {
    buffer_[idx++] = c; // Save character to buffer.
  }

  return sentence_type;
}

void NmeaParser::clearBuffer() {
  memset(buffer_, '\0', kMaxSentenceLength);
  idx = 0;
  check_sum_ = 0x00;
}

SentenceType NmeaParser::parseSentence() {
  auto sentence_type = SentenceType::kUnknown;

  if (parseGpzda())
    sentence_type == SentenceType::kGpzda;

  return sentence_type;
}

char *NmeaParser::nextWord(const WordType word_type) {
  const char kWordDelim[2] = ",";
  const char kCheckSumDelim[2] = "\r";

  char *word = NULL;
  switch (word_type) {
  case WordType::kHeader:
    word = strtok(buffer_, kWordDelim); // First word, ende with ','
    break;
  case WordType::kData:
    word = strtok(NULL, kWordDelim); // Next word, ends with ','
    break;
  case WordType::kCheckSum:
    word = strtok(NULL, kCheckSumDelim); // Last word, ends with '\r'
    break;
  default:
    break;
  }

  addToCheckSum(word);

  return word;
}

void NmeaParser::addToCheckSum(const char *word) {
  if (word == NULL)
    return;

  // Do not add checksum word.
  const char kChecksumStart = '*';
  if (*word == kChecksumStart)
    return;

  const char kTokenDelimiter = '\0';
  while (*word != kTokenDelimiter)
    check_sum_ ^= *word++; // XOR all bytes.
}

bool NmeaParser::checkSum(const char *word) {
  if (word == NULL)
    return false;

  int cs = atoi(word);
  DEBUG_PRINTLN("Checksums");
  DEBUG_PRINTLN(word);
  DEBUG_PRINTLN(cs);
  DEBUG_PRINTLN(check_sum_);

  return cs == check_sum_;
}

bool NmeaParser::parseGpzda() {
  // A GPZDA sentence: $GPZDA,173538.00,14,01,2020,,*69[...]\n

  // Get Header
  auto header = nextWord(WordType::kHeader);
  if (header == NULL)
    return false;
  // Check header.
  const char kHeaderGpzda[6] = "GPZDA";
  if (strcmp(header, kHeaderGpzda) != 0)
    return false;

  // Get time.
  auto time = nextWord(WordType::kData);
  if (time == NULL)
    return false;
  DEBUG_PRINTLN(time);

  // Get day.
  auto day = nextWord(WordType::kData);
  if (day == NULL)
    return false;
  DEBUG_PRINTLN(day);

  // Get month.
  auto month = nextWord(WordType::kData);
  if (month == NULL)
    return false;
  DEBUG_PRINTLN(month);

  //  // Get year.
  //  auto year = nextWord(WordType::kData);
  //  if (year == NULL)
  //    return false;
  //  DEBUG_PRINTLN(year);
  //
  //  // Get zone.
  //  auto zone = nextWord(WordType::kData);
  //  if (zone == NULL)
  //    return false;
  //  DEBUG_PRINTLN(zone);
  //
  //  // Get zone minutes.
  //  auto zone_minutes = nextWord(WordType::kData);
  //  if (zone_minutes == NULL)
  //    return false;
  //  DEBUG_PRINTLN(zone_minutes);

  // Get check sum.
  auto cs = nextWord(WordType::kCheckSum);
  if (cs == NULL)
    return false;
  DEBUG_PRINTLN(cs);

  if (!checkSum(cs)) {
    DEBUG_PRINTLN("WRONG CS!!");
    return false;
  }

  return true;
}

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
