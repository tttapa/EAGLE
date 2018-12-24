#include <gtest/gtest.h>

#include <ArgParser.hpp>

using namespace std::string_literals;

TEST(ArgParser, ArgParser) {
    std::string result1 = "";
    std::string result2 = "";
    ArgParser parser;
    auto onMatchTest1 = [&result1](const char *argv[]) { result1 = argv[1]; };
    auto onMatchTest2 = [&result2](const char *argv[]) { result2 = argv[1]; };
    auto notInArgv    = [](const char *[]) { FAIL(); };
    ArgMatcherArguments<1> matcher1 = {"--test1", onMatchTest1};
    ArgMatcherArguments<1> matcher2 = {"--test2", "-t", onMatchTest2};
    ArgMatcherArguments<1> matcher3 = {"--not-in-argv", notInArgv};
    parser.add(matcher1);
    parser.add(matcher2);
    parser.add(matcher3);
    const char *argv[] = {"cmd", "--test1", "success", "-t",
                          "success as well!"};
    parser.parse(argv);
    ASSERT_EQ(result1, "success");
    ASSERT_EQ(result2, "success as well!");
}

TEST(ArgParser, notEnoughArguments) {
    class TestArgMatcher : public ArgMatcher {
      public:
        TestArgMatcher(const char *flag, bool &success)
            : ArgMatcher{flag}, success{success} {}
        void onMatch(const char *[]) const override { FAIL(); }
        void onNotEnoughArguments(int argc) const override {
            ASSERT_EQ(argc, 0);
            success = true;
        }
        // TestArgMatcher requires 1 argument after the flag
        size_t getNumberOfArguments() const override { return 1; }

      private:
        bool &success;
    };
    ArgParser parser;
    bool success = false;
    parser.add(TestArgMatcher{"--test", success});
    const char *argv[] = {"cmd", "--test"};
    parser.parse(2, argv);
    ASSERT_TRUE(success);
}