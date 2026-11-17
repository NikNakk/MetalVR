// A JSON document as values, for the configuration headsets keep in their own firmware (a Vive's or an Index's
// lens and IMU calibration, a Rift S's IMU calibration). Small and strict: what does not parse is rejected whole.
#pragma once

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace metalvr {

class JsonValue {
public:
  enum class Type { Null, Bool, Number, String, Array, Object };

  // the document, or nothing if `text` is not one JSON value (with only white space around it)
  static std::optional<JsonValue> parse(std::string_view text);

  Type type() const { return type_; }
  bool isNumber() const { return type_ == Type::Number; }
  bool isString() const { return type_ == Type::String; }
  bool isArray() const { return type_ == Type::Array; }
  bool isObject() const { return type_ == Type::Object; }

  double number(double otherwise = 0.0) const { return type_ == Type::Number ? number_ : otherwise; }
  const std::string &string() const { return string_; }
  bool boolean(bool otherwise = false) const { return type_ == Type::Bool ? boolean_ : otherwise; }

  // an object's member, or null if there is none (or this is not an object)
  const JsonValue &operator[](std::string_view key) const;
  // an array's element, or null
  const JsonValue &operator[](size_t index) const;
  size_t size() const { return type_ == Type::Array ? array_.size() : type_ == Type::Object ? object_.size() : 0; }
  const std::vector<JsonValue> &elements() const { return array_; }
  const std::vector<std::pair<std::string, JsonValue>> &members() const { return object_; }

  // the first `n` numbers of an array into `out`; false unless it is an array of at least `n` numbers
  bool numbers(float *out, size_t n) const;
  bool numbers(double *out, size_t n) const;

private:
  friend class JsonParser;
  Type type_ = Type::Null;
  bool boolean_ = false;
  double number_ = 0.0;
  std::string string_;
  std::vector<JsonValue> array_;
  std::vector<std::pair<std::string, JsonValue>> object_;
};

} // namespace metalvr
