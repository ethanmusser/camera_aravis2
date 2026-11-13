// Copyright (c) 2026 Ethan J. Musser
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "camera_aravis2/conversion_utils.h"

#if __has_include(<catch2/catch_test_macros.hpp>)
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#else
#include <catch2/catch.hpp>
#endif

#include <array>
#include <limits>

namespace
{

const std::array<uint32_t, 8> COUNTS{0, 1, 255, 256, 65535, 65536, 0x123456, 0xffffff};

sensor_msgs::msg::Image::SharedPtr makePackedImage(bool big_endian)
{
    auto input              = std::make_shared<sensor_msgs::msg::Image>();
    input->width            = 4;
    input->height           = 2;
    input->step             = 14;
    input->is_bigendian     = big_endian ? 1 : 0;
    input->encoding         = "BayerRG24";
    input->header.frame_id  = "rggb";
    input->header.stamp.sec = 123;
    input->data.assign(26, 0xee);
    for (size_t pixel = 0; pixel < COUNTS.size(); ++pixel)
    {
        const size_t offset = (pixel / 4) * 14 + (pixel % 4) * 3;
        for (size_t byte = 0; byte < 3; ++byte)
        {
            const auto shift = 8 * (big_endian ? 2 - byte : byte);
            input->data.at(offset + byte) =
              static_cast<uint8_t>((COUNTS.at(pixel) >> shift) & 0xff);
        }
    }
    return input;
}

}  // namespace

TEST_CASE("Bayer24 conversion preserves counts, byte order, metadata, and padded rows",
          "[conversion]")
{
    const bool big_endian = GENERATE(false, true);
    auto input            = makePackedImage(big_endian);
    auto output           = std::make_shared<sensor_msgs::msg::Image>();
    output->data.assign(32, 0xff);
    REQUIRE(camera_aravis2::CONVERSIONS_DICTIONARY.at("BayerRG24")(input, output));
    REQUIRE(output->header == input->header);
    REQUIRE(output->encoding == "32SC1");
    REQUIRE(output->width == 4);
    REQUIRE(output->height == 2);
    REQUIRE(output->step == 16);
    REQUIRE(output->data.size() == 32);
    REQUIRE(output->is_bigendian == 0);
    for (size_t pixel = 0; pixel < COUNTS.size(); ++pixel)
    {
        uint32_t actual = 0;
        for (size_t byte = 0; byte < 4; ++byte)
            actual |= static_cast<uint32_t>(output->data.at(pixel * 4 + byte)) << (8 * byte);
        REQUIRE(actual == COUNTS.at(pixel));
    }
}

TEST_CASE("Malformed Bayer24 input is rejected before changing the output", "[conversion]")
{
    sensor_msgs::msg::Image::SharedPtr output;
    auto input = makePackedImage(false);
    SECTION("missing input")
    {
        input.reset();
    }
    SECTION("empty width")
    {
        input->width = 0;
    }
    SECTION("empty height")
    {
        input->height = 0;
    }
    SECTION("short row")
    {
        input->step = 11;
    }
    SECTION("truncated payload")
    {
        input->data.resize(25);
    }
    SECTION("overflowing output stride")
    {
        input->width = std::numeric_limits<uint32_t>::max();
    }
    REQUIRE_FALSE(camera_aravis2::unpack24MonoImg(input, output));
    REQUIRE(output == nullptr);
}

TEST_CASE("Bayer24 conversion allocates output and rejects an in-place expansion", "[conversion]")
{
    auto input = makePackedImage(false);
    sensor_msgs::msg::Image::SharedPtr output;
    REQUIRE(camera_aravis2::unpack24MonoImg(input, output));
    REQUIRE(output != input);
    REQUIRE_FALSE(camera_aravis2::unpack24MonoImg(input, input));
}
