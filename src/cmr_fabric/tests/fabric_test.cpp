#include "gtest/gtest.h"

class FabricTest : public ::testing::Test
{
  protected:
    FabricTest() = default;

    ~FabricTest() override = default;
};

TEST_F(FabricTest, NodeDoesEnable) { ASSERT_FALSE(true); }
