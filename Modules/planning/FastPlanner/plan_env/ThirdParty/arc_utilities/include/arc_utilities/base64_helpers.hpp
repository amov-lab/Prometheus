#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <zlib.h>

#ifndef BASE64_HELPERS_HPP
#define BASE64_HELPERS_HPP

namespace Base64Helpers
{
    std::vector<uint8_t> Decode(const std::string& encoded);

    std::string Encode(const std::vector<uint8_t>& binary);
}

#endif // BASE64_HELPERS_HPP
