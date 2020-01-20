#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <zlib.h>

#ifndef ZLIB_HELPERS_HPP
#define ZLIB_HELPERS_HPP

namespace ZlibHelpers
{
    std::vector<uint8_t> DecompressBytes(const std::vector<uint8_t>& compressed);

    std::vector<uint8_t> CompressBytes(const std::vector<uint8_t>& uncompressed);

    std::vector<uint8_t> LoadFromFileAndDecompress(const std::string& path);

    void CompressAndWriteToFile(const std::vector<uint8_t>& uncompressed, const std::string& path);
}

#endif // ZLIB_HELPERS_HPP
