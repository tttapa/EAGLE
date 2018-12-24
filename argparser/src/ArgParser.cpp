#include "ArgParser.hpp"
#include <ANSIColors.hpp>

void ArgParser::parse(int argc, const char *argv[]) const {
    --argc;  // ignore the first arg
    ++argv;  // which is the command
    for (int i = 0; i < argc; ++i) {
        bool matched = false;
        for (auto &argMatcher : argMatchers) {
            if (argMatcher->match(argv[i])) {
                size_t argLeft = argc - i - 1;
                if (argLeft < argMatcher->getNumberOfArguments())
                    argMatcher->onNotEnoughArguments(argLeft);
                else
                    argMatcher->onMatch(&argv[i]);
                i += argMatcher->getNumberOfArguments();
                matched = true;
                break;
            }
        }
        if (!matched)
            std::cerr << ANSIColors::red << "Unknown option `" << argv[i]
                      << "`, ignoring." << std::endl
                      << ANSIColors::reset;
    }
}
