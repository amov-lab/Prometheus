#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <zlib.h>
#include "arc_utilities/arc_exceptions.hpp"
#include "arc_utilities/zlib_helpers.hpp"

namespace ZlibHelpers
{
    // MAKE SURE THE INPUT BUFFER IS LESS THAN 4GB IN SIZE
    std::vector<uint8_t> DecompressBytes(const std::vector<uint8_t>& compressed)
    {
        z_stream strm;
        std::vector<uint8_t> buffer;
        const size_t BUFSIZE = 1024 * 1024;
        uint8_t temp_buffer[BUFSIZE];
        strm.zalloc = Z_NULL;
        strm.zfree = Z_NULL;
        strm.opaque = Z_NULL;
        int ret = inflateInit(&strm);
        if (ret != Z_OK)
        {
            (void)inflateEnd(&strm);
            std::cerr << "ZLIB unable to init inflate stream" << std::endl;
            throw std::invalid_argument("ZLIB unable to init inflate stream");
        }
        strm.avail_in = (uint32_t)compressed.size();
        strm.next_in = const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(compressed.data()));
        do
        {
            strm.next_out = temp_buffer;
            strm.avail_out = BUFSIZE;
            ret = inflate(&strm, Z_NO_FLUSH);
            if (buffer.size() < strm.total_out)
            {
                buffer.insert(buffer.end(), temp_buffer, temp_buffer + BUFSIZE - strm.avail_out);
            }
        }
        while (ret == Z_OK);
        if (ret != Z_STREAM_END)
        {
            (void)inflateEnd(&strm);
            std::cerr << "ZLIB unable to inflate stream with ret=" << ret << std::endl;
            throw std::invalid_argument("ZLIB unable to inflate stream");
        }
        (void)inflateEnd(&strm);
        std::vector<uint8_t> decompressed(buffer);
        return decompressed;
    }

    // MAKE SURE THE INPUT BUFFER IS LESS THAN 4GB IN SIZE
    std::vector<uint8_t> CompressBytes(const std::vector<uint8_t>& uncompressed)
    {
        z_stream strm;
        std::vector<uint8_t> buffer;
        const size_t BUFSIZE = 1024 * 1024;
        uint8_t temp_buffer[BUFSIZE];
        strm.zalloc = Z_NULL;
        strm.zfree = Z_NULL;
        strm.opaque = Z_NULL;
        strm.avail_in = (uint32_t)uncompressed.size();
        strm.next_in = const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(uncompressed.data()));
        strm.next_out = temp_buffer;
        strm.avail_out = BUFSIZE;
        int ret = deflateInit(&strm, Z_BEST_SPEED);
        if (ret != Z_OK)
        {
            (void)deflateEnd(&strm);
            std::cerr << "ZLIB unable to init deflate stream" << std::endl;
            throw std::invalid_argument("ZLIB unable to init deflate stream");
        }
        while (strm.avail_in != 0)
        {
            ret = deflate(&strm, Z_NO_FLUSH);
            if (ret != Z_OK)
            {
                (void)deflateEnd(&strm);
                std::cerr << "ZLIB unable to deflate stream" << std::endl;
                throw std::invalid_argument("ZLIB unable to deflate stream");
            }
            if (strm.avail_out == 0)
            {
                buffer.insert(buffer.end(), temp_buffer, temp_buffer + BUFSIZE);
                strm.next_out = temp_buffer;
                strm.avail_out = BUFSIZE;
            }
        }
        int deflate_ret = Z_OK;
        while (deflate_ret == Z_OK)
        {
            if (strm.avail_out == 0)
            {
                buffer.insert(buffer.end(), temp_buffer, temp_buffer + BUFSIZE);
                strm.next_out = temp_buffer;
                strm.avail_out = BUFSIZE;
            }
            deflate_ret = deflate(&strm, Z_FINISH);
        }
        if (deflate_ret != Z_STREAM_END)
        {
            (void)deflateEnd(&strm);
            std::cerr << "ZLIB unable to deflate stream" << std::endl;
            throw std::invalid_argument("ZLIB unable to deflate stream");
        }
        buffer.insert(buffer.end(), temp_buffer, temp_buffer + BUFSIZE - strm.avail_out);
        (void)deflateEnd(&strm);
        std::vector<uint8_t> compressed(buffer);
        return compressed;
    }

    std::vector<uint8_t> LoadFromFileAndDecompress(const std::string& path)
    {
        std::ifstream input_file(path, std::ios::binary | std::ios::in | std::ios::ate);
        if (!input_file.is_open())
        {
            throw_arc_exception(std::runtime_error, "Couldn't open file " + path);
        }
        std::streamsize size = input_file.tellg();
        input_file.seekg(0, std::ios::beg);
        std::vector<uint8_t> file_buffer((size_t)size);
        if (!(input_file.read(reinterpret_cast<char*>(file_buffer.data()), size)))
        {
            throw_arc_exception(std::runtime_error, "Unable to read entire contents of file");
        }
        const std::vector<uint8_t> decompressed = ZlibHelpers::DecompressBytes(file_buffer);
        return decompressed;
    }

    void CompressAndWriteToFile(const std::vector<uint8_t>& uncompressed, const std::string& path)
    {
        const auto compressed = CompressBytes(uncompressed);
        std::ofstream output_file(path, std::ios::out | std::ios::binary);
        output_file.write(reinterpret_cast<const char*>(compressed.data()), (std::streamsize)compressed.size());
        output_file.close();
    }
}
