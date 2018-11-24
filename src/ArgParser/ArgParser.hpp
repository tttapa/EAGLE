#pragma once

#include <algorithm>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>
#include <Util/ANSIColors.hpp>

class ArgMatcher {
  public:
    ArgMatcher(const char *flag, const char *abbr = "")
        : flag{flag}, abbr{abbr} {}

    virtual bool match(const char *s) const {
        bool match = strcmp(s, flag) == 0 || strcmp(s, abbr) == 0;
        return match;
    }
    virtual void onMatch(const char *argv[]) const = 0;
    virtual void onNotEnoughArguments(int argc) const {
        std::cerr << "Error: not enough arguments for " << flag << ": "
                  << getNumberOfArguments() << " expected, only " << argc
                  << " provided." << std::endl;
        printUsage();
    }
    virtual void printUsage() const {}
    virtual size_t getNumberOfArguments() const { return 0; }
    const char *getName() const { return flag; }

  private:
    const char *const flag;
    const char *const abbr;
};

template <size_t N>
class ArgMatcherArguments : public ArgMatcher {

  public:
    using Args_t = std::array<const char *, N + 1>;
    ArgMatcherArguments(
        const char *flag, const char *abbr,
        const std::function<void(const char *argv[])> &onMatchFn)
        : ArgMatcher{flag, abbr}, onMatchFn{onMatchFn} {}
    ArgMatcherArguments(
        const char *flag,
        const std::function<void(const char *argv[])> &onMatchFn)
        : ArgMatcher{flag}, onMatchFn{onMatchFn} {}
    ArgMatcherArguments(const ArgMatcherArguments &) = default;
    void onMatch(const char *argv[]) const override { onMatchFn(argv); }
    size_t getNumberOfArguments() const override { return N; }

  private:
    const std::function<void(const char *[])> onMatchFn;
};

class ArgParser {
  public:
    template <class T>
    void add(const T &argMatcher) {
        argMatchers.push_back(std::make_unique<T>(argMatcher));
    }
    template <size_t N = 1>
    void add(const char *flag, const char *abbr,
             const std::function<void(const char *argv[])> &onMatchFn) {
        add(ArgMatcherArguments<N>{flag, abbr, onMatchFn});
    }
    template <size_t N = 1>
    void add(const char *flag,
             const std::function<void(const char *argv[])> &onMatchFn) {
        add(ArgMatcherArguments<N>{flag, onMatchFn});
    }
    template <size_t N>
    void parse(const char *(&argv)[N]) {
        parse(N, argv);
    }
    void parse(int argc, const char *argv[]) const {
        --argc; // ignore the first arg
        ++argv; // which is the command
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

  private:
    std::vector<std::unique_ptr<ArgMatcher>> argMatchers;
};