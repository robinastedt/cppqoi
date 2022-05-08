#pragma once

#ifndef CPPQOI_DEBUG_ENABLE
#define CPPQOI_DEBUG_ENABLE 0
#endif

#include <cstddef>
#include <cstdint>
#include <iostream>

namespace cppqoi {
    void* debug_print_malloc(size_t size) {
        std::cout << "malloc(" << size << ")" << std::endl;
        return malloc(size);
    }
} // namespace cppqoi

#if CPPQOI_DEBUG_ENABLE
    #define CPPQOI_DEBUG(expr) expr
    #define STBI_MALLOC(sz)           cppqoi::debug_print_malloc(sz)
    #define STBI_REALLOC(p,newsz)     realloc(p,newsz)
    #define STBI_FREE(p)              free(p)
#else
    #define CPPQOI_DEBUG(expr)
#endif

#include <stb_image.h>
#include <stb_image_write.h>

#include <array>
#include <vector>
#include <stdexcept>
#include <filesystem>
#include <fstream>
#include <bit>

namespace cppqoi {
    #define CPPQOI_INTERNAL_ERROR(msg) \
        do { \
            throw std::logic_error{ \
                std::string{"Internal error! "} + (msg) + " at " + __FILE__ + ":" + std::to_string(__LINE__) \
            }; \
        } while (false)
    #define CPPQOI_ASSERT(expr) \
        if (!(expr)) { \
            CPPQOI_INTERNAL_ERROR("Assertion failed: \"" #expr "\""); \
        }

    constexpr size_t lookupTableSize{64};
    constexpr size_t runLengthMax{62};  // Due to 1111'1110 and 1111'1111 being reserved
    constexpr std::array<uint8_t, 8> encodingEndMarker{0x00u, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};
    namespace tags {
        constexpr uint8_t index {0b0000'0000};
        constexpr uint8_t diff  {0b0100'0000};
        constexpr uint8_t luma  {0b1000'0000};
        constexpr uint8_t run   {0b1100'0000};
        constexpr uint8_t rgb   {0b1111'1110};
        constexpr uint8_t rgba  {0b1111'1111};
    }

    using Encoded = std::vector<uint8_t>;

    #pragma pack(push, 1)
    struct Pixel {
        uint8_t r, g, b, a;

        size_t hash() const {
            return (r * 3 + g * 5 + b * 7 + a * 11) % lookupTableSize;
        }

        constexpr inline auto operator<=>(const Pixel& rhs) const = default;
    };
    #pragma pack(pop)
    static_assert(sizeof(Pixel) == 4);

    class PixelView {
        const Pixel* m_previous;
        const Pixel* m_current;
    public:
        PixelView(const Pixel* previous, const Pixel* current)
        : m_previous(previous)
        , m_current(current) {}

        const Pixel& getPrevious() const { return *m_previous; }
        const Pixel& getCurrent() const { return *m_current; }

        constexpr inline auto operator<=>(const PixelView& rhs) const {
            return m_current <=> rhs.m_current;
        }
        constexpr inline bool operator==(const PixelView& rhs) const {
            return operator<=>(rhs) == 0;
        };
        constexpr inline bool operator!=(const PixelView& rhs) const {
            return operator<=>(rhs) != 0;
        }

        friend class PixelIterator;
    };

    class PixelIterator {
        PixelView m_pixelView;
    public:
        PixelIterator(PixelView pixelView) : m_pixelView(std::move(pixelView)) {}
        const PixelView* operator->() const { return &m_pixelView; }
        const PixelView& operator*() const { return m_pixelView; }
        PixelIterator& operator++() {
            m_pixelView = {m_pixelView.m_current, m_pixelView.m_current + 1};
            return *this;
        }

        PixelIterator operator+(size_t i) {
            if (i == 0) return *this;
            return {{m_pixelView.m_current + (i - 1), m_pixelView.m_current + i}};
        }

        constexpr inline auto operator<=>( const PixelIterator& rhs) const = default;
    };

    enum ColorSpace : uint8_t {
        SRGB_LINEAR_ALPHA = 0,
        ALL_LINEAR = 1
    };

    class BitMap {
        uint32_t m_width;
        uint32_t m_height;
        uint8_t m_channels;
        ColorSpace m_colorspace;
        stbi_uc* m_stbdata;
        std::vector<Pixel> m_extdata;

        static_assert(sizeof(*m_stbdata) == 1);

        static stbi_uc*
        load(const std::filesystem::path& filePath, uint32_t& widthOut, uint32_t& heightOut, uint8_t& channelsOut, ColorSpace& colorspaceOut) {
            int width{0};
            int height{0};
            int channels{0};
            stbi_uc* const data = stbi_load(filePath.c_str(), &width, &height, &channels, STBI_rgb_alpha);
            if (!data) {
                throw std::runtime_error{"Failed to load image: " + filePath.string()};
            }
            try {
                if (width < 0) {
                    throw std::runtime_error("Negative width: " + std::to_string(width));
                }
                if (height < 0) {
                    throw std::runtime_error("Negative height: " + std::to_string(height));
                }
                if (channels != 3 && channels != 4) {
                    throw std::runtime_error{"Unsupported number of channels: " + std::to_string(channels)};
                }
            } catch (...) {
                stbi_image_free(data);
                throw;
            }
            
            widthOut = width;
            heightOut = height;
            channelsOut = channels;
            colorspaceOut = ColorSpace::SRGB_LINEAR_ALPHA;  // TODO: Support other colorspaces?
            return data;
        }

    public:
        BitMap(const std::filesystem::path& filePath)
        : m_width()
        , m_height()
        , m_channels()
        , m_colorspace()
        , m_stbdata(load(filePath, m_width, m_height, m_channels, m_colorspace))
        , m_extdata() {
            CPPQOI_DEBUG(std::cout << "BitMap(width=" << m_width << ", height=" << m_height << ", channels=" << (int)m_channels << ")" << std::endl);
        }

        BitMap(uint32_t width, uint32_t height, uint8_t channels, ColorSpace colorspace, std::vector<Pixel> extdata)
        : m_width(width)
        , m_height(height)
        , m_channels(channels)
        , m_colorspace(colorspace)
        , m_stbdata(nullptr)
        , m_extdata(std::move(extdata)) {
            CPPQOI_ASSERT(m_extdata.size() == m_width * m_height) ;
        }

        BitMap(const BitMap&) = delete;
        BitMap(BitMap&&) = default;

        BitMap& operator=(const BitMap&) = delete;
        BitMap& operator=(BitMap&&) = default;

        ~BitMap() {
            if (m_stbdata) {
                stbi_image_free(m_stbdata);
            }
        }

        uint32_t getWidth() const { return m_width; }
        uint32_t getHeight() const { return m_height; }
        uint8_t getChannels() const { return m_channels; }
        ColorSpace getColorSpace() const { return m_colorspace; }

        const uint8_t* getData() const {
            return m_stbdata ? reinterpret_cast<const uint8_t*>(m_stbdata) : reinterpret_cast<const uint8_t*>(m_extdata.data());
        }

        const Pixel* getPixels() const {
            return m_stbdata ? reinterpret_cast<const Pixel*>(m_stbdata) : m_extdata.data();
        }

        PixelIterator begin() const {
            static constexpr Pixel beforeFirst{0, 0, 0, 255};
            return {{&beforeFirst, getPixels()}};
        }

        PixelIterator end() const {
            const Pixel*const afterLast = getPixels() + (m_width * m_height);
            return {{afterLast - 1, afterLast}};
        }

        bool operator==(const BitMap& rhs) const {
            if (m_width != rhs.m_width || m_height != rhs.m_height || m_channels != rhs.m_channels || m_colorspace != rhs.m_colorspace) {
                return false;
            }

            for (size_t index = 0; index < m_width * m_height * m_channels; index += m_channels) {
                const Pixel p1 = m_stbdata ?
                    Pixel{
                        m_stbdata[index],
                        m_stbdata[index + 1],
                        m_stbdata[index + 2],
                        (m_channels == 3) ? (uint8_t)255u : m_stbdata[index + 3]
                    } :
                    m_extdata[index / m_channels];
                const Pixel p2 = rhs.m_stbdata ?
                    Pixel{
                        rhs.m_stbdata[index],
                        rhs.m_stbdata[index + 1],
                        rhs.m_stbdata[index + 2],
                        (rhs.m_channels == 3) ? (uint8_t)255u : rhs.m_stbdata[index + 3]
                    } :
                    rhs.m_extdata[index / m_channels];
                if (!m_stbdata && !rhs.m_stbdata && p1 != p2) {
                    return false;
                }
            }
            return true;
        }
    };

    #pragma pack(push, 1)
    template<typename T>
    class BigEndian {
        std::array<uint8_t, sizeof(T)> m_data;

        static std::array<uint8_t, sizeof(T)>
        encode(T integer) {
            std::array<uint8_t, sizeof(T)> bytes;
            const uint8_t* data = reinterpret_cast<const uint8_t*>(&integer);
            for (size_t i = 0; i < sizeof(T); ++i) {
                if constexpr (std::endian::native == std::endian::big) {
                    bytes[i] = data[i];
                } else {
                    bytes[i] = data[sizeof(T) - 1 - i];
                }
            }
            return bytes;
        }

        static T
        decode(const std::array<uint8_t, sizeof(T)>& bytes) {
            T integer = 0;
            uint8_t* data = reinterpret_cast<uint8_t*>(&integer);
            for (size_t i = 0; i < sizeof(T); ++i) {
                if constexpr (std::endian::native == std::endian::big) {
                    data[i] = bytes[i];
                } else {
                    data[i] = bytes[sizeof(T) - 1 - i];
                }
            }
            return integer;
        }

    public:
        BigEndian(T x)
        : m_data(encode(x)) {
            static_assert(sizeof(BigEndian<T>) == sizeof(T));
        }

        operator T() const {
            return decode(m_data);
        }
    };
    #pragma pack(pop)


    #pragma pack(push, 1)
    struct QoiHeader {
        static constexpr std::array<uint8_t, 4> MAGIC{'q', 'o', 'i', 'f'};
        std::array<uint8_t, 4> magic;
        BigEndian<uint32_t> width;
        BigEndian<uint32_t> height;
        uint8_t channels;
        ColorSpace colorspace;

        QoiHeader() = default;
        QoiHeader(const BitMap& bitmap)
        : magic(MAGIC)
        , width(bitmap.getWidth())
        , height(bitmap.getHeight())
        , channels(bitmap.getChannels())
        , colorspace(bitmap.getColorSpace()) {}

        uint32_t getWidth() const {
            return width;
        }

        uint32_t getHeight() const {
            return height;
        }
    };
    #pragma pack(pop)
    static_assert(sizeof(QoiHeader) == 14);

    template<typename T>
    void write(Encoded& output, const T& obj) {
        const Encoded::value_type* const data = reinterpret_cast<const Encoded::value_type*>(&obj);
        for (const Encoded::value_type* ptr = data; ptr < data + sizeof(T); ++ptr) {
            output.emplace_back(*ptr);
        }
    }

    Encoded
    encode(const BitMap& bitmap) {
        Encoded output;

        QoiHeader header{bitmap};
        write(output, header);

        std::array<Pixel, lookupTableSize> lookupTable{};
        for (PixelIterator pixelIt = bitmap.begin(), end = bitmap.end(); pixelIt != end; ++pixelIt) {
            const size_t lookupIndex = pixelIt->getCurrent().hash();
            const Pixel& previous = pixelIt->getPrevious();
            const Pixel& current = pixelIt->getCurrent();

            // QOI_OP_RUN
            if (previous == current) {
                size_t runLength = 1;
                PixelIterator oneAhead = pixelIt + 1;
                while (oneAhead != end && runLength < runLengthMax && oneAhead->getPrevious() == oneAhead->getCurrent()) {
                    ++runLength;
                    ++pixelIt;
                    ++oneAhead;
                }
                const uint8_t chunk = tags::run | (runLength - 1 /*bias -1*/);
                write(output, chunk);
                lookupTable[lookupIndex] = current;
                continue;
            }

            // QOI_OP_INDEX
            if (lookupTable[lookupIndex] == current) {
                const uint8_t chunk = tags::index | lookupIndex;
                write(output, chunk);
                lookupTable[lookupIndex] = current;
                continue;
            }

            const uint8_t dr = current.r - previous.r;
            const uint8_t dg = current.g - previous.g;
            const uint8_t db = current.b - previous.b;
            const uint8_t da = current.a - previous.a;

            // QOI_OP_DIFF
            {
                static constexpr uint8_t bias = 2;

                const uint8_t drBiased = dr + bias;
                const uint8_t dgBiased = dg + bias;
                const uint8_t dbBiased = db + bias;

                if (da == 0u && drBiased < 4u && dgBiased < 4u && dbBiased < 4u) {
                    const uint8_t chunk = tags::diff | drBiased << 4 | dgBiased << 2 | dbBiased;
                    write(output, chunk);
                    lookupTable[lookupIndex] = current;
                    continue;
                }
            }
            
            // QOI_OP_LUMA
            {
                const uint8_t drdg = dr - dg;
                const uint8_t dbdg = db - dg;

                static constexpr uint8_t longBias = 32;
                static constexpr uint8_t shortBias = 8;

                const uint8_t dgBiased = dg + longBias;
                const uint8_t drdgBiased = drdg + shortBias;
                const uint8_t dbdgBiased = dbdg + shortBias;

                if (da == 0u && dgBiased < 64u && drdgBiased < 16u && dbdgBiased < 16u) {
                    const std::array<uint8_t, 2> chunks {
                        (uint8_t)(tags::luma | dgBiased),
                        (uint8_t)(drdgBiased << 4 | dbdgBiased)
                    };
                    write(output, chunks);
                    lookupTable[lookupIndex] = current;
                    continue;
                }
            }

            // QOI_OP_RGB
            if (bitmap.getChannels() == 3) {
                std::array<uint8_t, 4> chunks {
                    tags::rgb,
                    current.r,
                    current.g,
                    current.b
                };
                write(output, chunks);
                lookupTable[lookupIndex] = current;
                continue;
            }

            
            // QOI_OP_RGBA
            {
                CPPQOI_ASSERT(bitmap.getChannels() == 4);
                std::array<uint8_t, 5> chunks {
                    tags::rgba,
                    current.r,
                    current.g,
                    current.b,
                    current.a
                };
                write(output, chunks);
                lookupTable[lookupIndex] = current;
                continue;
            }
        }

        write(output, encodingEndMarker);

        CPPQOI_DEBUG(std::cout << "Output size: " << std::fixed << std::setprecision(1) << (output.size() / 1024.0) << "kib" << std::endl);

        return output;
    }

    BitMap
    decode(const Encoded& encoded) {
        if (encoded.size() < sizeof(QoiHeader)) {
            throw std::runtime_error("File too small");
        }
        const QoiHeader *const header = reinterpret_cast<const QoiHeader*>(encoded.data());
        if (header->magic != QoiHeader::MAGIC) {
            throw std::runtime_error("File is not of QOI file format");
        }
        const uint32_t width = header->getWidth();
        const uint32_t height = header->getHeight();
        const uint8_t channels = header->channels;
        CPPQOI_DEBUG(std::cout << "width=" << width << ", height=" << height << ", channels=" << (int)channels << std::endl);
        const ColorSpace colorspace = header->colorspace;
        const size_t bitmapSize = width * height * channels;

        switch (channels) {
            case 3:
            case 4:
                break;
            default:
                throw std::runtime_error("Unsupported number of channels: " + std::to_string(channels));
        }
        
        switch (colorspace) {
            case SRGB_LINEAR_ALPHA:
            case ALL_LINEAR:
                break;
            default:
                throw std::runtime_error("Invalid colorspace flag: " + std::to_string(colorspace));
        }

        const uint8_t *const data = reinterpret_cast<const uint8_t *>(encoded.data() + sizeof(QoiHeader));
        const uint8_t *const dataEnd = encoded.data() + encoded.size() - sizeof(encodingEndMarker);
        std::vector<Pixel> output{width * height};
        std::array<Pixel, lookupTableSize> lookupTable{};
        auto outputIt = output.begin();
        const auto outputEnd = output.end();

        Pixel previous{0, 0, 0, 255};
        for (const uint8_t* chunk = data; chunk != dataEnd; ++chunk, ++outputIt) {
            if (outputIt == outputEnd) {
                throw std::runtime_error{
                    "Specified bitmap size does not conform to decoded size. "
                    "Expected " + std::to_string(bitmapSize) + " bytes but there is more."
                };
            }
            switch (*chunk) {
                case tags::rgb: {
                    if (chunk + 3 >= dataEnd) {
                        throw std::runtime_error("Reached EOF while parsing QOI_OP_RGB");
                    }
                    const Pixel pixel{*++chunk, *++chunk, *++chunk, 255};
                    previous = lookupTable[pixel.hash()] = *outputIt = pixel;
                } break;
                case tags::rgba: {
                    if (chunk + 4 >= dataEnd) {
                        throw std::runtime_error("Reached EOF while parsing QOI_OP_RGBA");
                    }
                    const Pixel pixel{*++chunk, *++chunk, *++chunk, *++chunk};
                    previous = lookupTable[pixel.hash()] = *outputIt = pixel;
                } break;
                default: {
                    switch (*chunk & 0b1100'0000) {
                        case tags::index: {
                            previous = *outputIt = lookupTable[*chunk & 0b0011'1111];
                        } break;
                        case tags::diff: {
                            static constexpr uint8_t bias = 2;
                            const uint8_t dr = ((*chunk & 0b0011'0000) >> 4) - bias;
                            const uint8_t dg = ((*chunk & 0b0000'1100) >> 2) - bias;
                            const uint8_t db = ((*chunk & 0b0000'0011) - bias);
                            const Pixel pixel{
                                .r = (uint8_t)(previous.r + dr),
                                .g = (uint8_t)(previous.g + dg),
                                .b = (uint8_t)(previous.b + db),
                                .a = (uint8_t)(previous.a),
                            };
                            previous = lookupTable[pixel.hash()] = *outputIt = pixel;
                        } break;
                        case tags::luma: {
                            if (chunk + 1 >= dataEnd) {
                                throw std::runtime_error("Reached EOF while parsing QOI_OP_LUMA");
                            }
                            static constexpr uint8_t longBias = 32;
                            static constexpr uint8_t shortBias = 8;
                            const uint8_t dg = (*chunk++ & 0b0011'1111) - longBias;
                            const uint8_t drdg = ((*chunk & 0b1111'0000) >> 4) - shortBias;
                            const uint8_t dbdg = (*chunk & 0b0000'1111) - shortBias;
                            const uint8_t dr = drdg + dg;
                            const uint8_t db = dbdg + dg;
                            const Pixel pixel{
                                .r = (uint8_t)(previous.r + dr),
                                .g = (uint8_t)(previous.g + dg),
                                .b = (uint8_t)(previous.b + db),
                                .a = (uint8_t)(previous.a),
                            };
                            previous = lookupTable[pixel.hash()] = *outputIt = pixel;
                        } break;
                        case tags::run: {
                            static constexpr uint8_t bias = -1;
                            const uint8_t runLength = (*chunk & 0b0011'1111) - bias;
                            CPPQOI_ASSERT(runLength >= 1);
                            CPPQOI_ASSERT(runLength <= runLengthMax);
                            lookupTable[previous.hash()] = *outputIt = previous;
                            for (size_t run = 1; run < runLength; ++run) {
                                *++outputIt = previous;
                            }
                        } break;
                        default:
                            __builtin_unreachable();
                    }
                }
            }
        }

        if (outputIt != outputEnd) {
            throw std::runtime_error{
                "Specified bitmap size does not conform to decoded size. "
                "Expected " + std::to_string(bitmapSize) + " bytes "
                "but managed to read " + std::to_string((outputIt - output.begin()) * sizeof(Pixel))
            };
        }

        const auto& endMarker = *reinterpret_cast<const decltype(encodingEndMarker)*>(dataEnd);
        if (endMarker != encodingEndMarker) {
            std::stringstream errMsg{"Unexpected end marker: "};
            for (uint8_t b : endMarker) {
                errMsg << std::hex << (int)b;
            }
            throw std::runtime_error{errMsg.str()};
        }

        return BitMap{width, height, channels, colorspace, std::move(output)};
    }

} // namespace cppqoi

