#ifndef STR_UTILS_HPP
#define STR_UTILS_HPP

#include <string>
#include <array>
#include <stdexcept>

template <size_t N>
std::array<char, N> str2arr(const std::string& str) {
    if (str.size() > N - 1) {
        throw std::invalid_argument("Error: Input string is too large!");
    }
    std::array<char, N> arr {};
    for (size_t i = 0; i < str.size(); ++i) {
        arr[i] = str[i];
    }
    arr[str.size()] = '\0';
    return arr;
}

template <size_t N>
std::string arr2str(const std::array<char, N>& arr) {
    std::string str;
    for (auto c : arr) {
        if (c == '\0') break;
        str += c;
    }
    return str;
}

#endif //STR_UTILS_HPP