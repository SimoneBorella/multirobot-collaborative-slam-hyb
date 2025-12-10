#include <iostream>
#include <map>
#include <sstream>
#include <string>

class ArgumentParser {
public:
    void parse(int argc, char* argv[]) {
        for (int i = 1; i < argc; i += 2) {
            if (argv[i][0] == '-' && argv[i][1] == '-') {
                std::string key = argv[i] + 2;
                if (i + 1 < argc) {
                    args[key] = argv[i + 1];
                }
            }
        }
    }

    template <typename T>
    T get(const std::string& key) const {
        auto it = args.find(key);
        if (it != args.end()) {
            T value;
            std::istringstream ss(it->second);
            ss >> value;
            if constexpr (std::is_same_v<T, std::string>) {
                return it->second;
            }
            if (ss.fail()) {
                throw std::runtime_error("Invalid conversion for key: " + key);
            }
            return value;
        }
        throw std::runtime_error("Argument not found: " + key);
    }

    template <typename T>
    T get(const std::string& key, T defaultValue) const {
        auto it = args.find(key);
        if (it != args.end()) {
            T value;
            std::istringstream ss(it->second);
            ss >> value;
            if constexpr (std::is_same_v<T, std::string>) {
                return it->second;
            }
            if (ss.fail()) {
                throw std::runtime_error("Invalid conversion for key: " + key);
            }
            return value;
        }
        return defaultValue;
    }

private:
    std::map<std::string, std::string> args;
};
