#include <gtest/gtest.h>

using std::cout;
using std::endl;

class MyEnv : public ::testing::Environment {
  public:
    void SetUp() override {}
    void TearDown() override {}
};

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::AddGlobalTestEnvironment(new MyEnv);
    return RUN_ALL_TESTS();
}