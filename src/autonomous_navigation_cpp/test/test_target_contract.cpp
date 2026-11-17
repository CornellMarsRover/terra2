// Port of src/autonomous_navigation/test/test_target_contract.py.
#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

#include "autonomous_navigation_cpp/target_contract.hpp"

using autonomous_navigation_cpp::target_contract::decode_target_request;
using autonomous_navigation_cpp::target_contract::encode_target_request;
using autonomous_navigation_cpp::target_contract::TargetRequest;

TEST(TargetContract, RoundTrip)
{
  struct Case {long long mission; const char * name; long long marker;};   // marker -1: none
  const Case cases[] = {
    {1, "coordinate", -1}, {3, "ar1", 1}, {4, "ar2", 2}, {5, "ar3", 3}, {7, "ar0", 0},
    {9, "ar42", 42}, {2, "mallet", -1}, {6, "bottle", -1}};
  for (const Case & c : cases) {
    const std::string encoded = encode_target_request(c.mission, c.name);
    const TargetRequest decoded = decode_target_request(encoded);
    EXPECT_EQ(decoded.mission_index, c.mission);
    EXPECT_EQ(decoded.target_name, c.name);
    if (c.marker < 0) {
      EXPECT_FALSE(decoded.marker_id.has_value()) << c.name;
    } else {
      ASSERT_TRUE(decoded.marker_id.has_value()) << c.name;
      EXPECT_EQ(*decoded.marker_id, c.marker);
    }
  }
}

TEST(TargetContract, RejectsMalformedValues)
{
  for (const char * value : {"", "ar1", "x:ar1", "0:ar1", "1:", "1:arx", "1:ar-2", "1:a:b"}) {
    EXPECT_THROW(decode_target_request(value), std::invalid_argument) << value;
  }
}

TEST(TargetContract, EncoderStripsNamesAndRejectsBadFields)
{
  EXPECT_EQ(encode_target_request(3, " ar1 "), "3:ar1");
  EXPECT_THROW(encode_target_request(0, "ar1"), std::invalid_argument);
  EXPECT_THROW(encode_target_request(1, "ar:1"), std::invalid_argument);
}

TEST(TargetContract, PythonIntParsing)
{
  // int() accepts surrounding whitespace, a sign and digit-group underscores.
  EXPECT_EQ(decode_target_request(" 12 :ar+7").mission_index, 12);
  EXPECT_EQ(*decode_target_request("1_0:ar0_3").marker_id, 3);
  EXPECT_THROW(decode_target_request("1__0:ar1"), std::invalid_argument);
  EXPECT_THROW(decode_target_request("1.0:ar1"), std::invalid_argument);
  EXPECT_THROW(decode_target_request("1:ar"), std::invalid_argument);
}
