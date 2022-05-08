#include "cppqoi.hh"

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

namespace cppqoi
{
    void
    mainImpl(int argc, char** argv) {
        if (argc != 3) {
            throw std::runtime_error("Wrong number of arguments");
        }

        const std::filesystem::path inputFile{argv[1]};
        const std::filesystem::path outputFile{argv[2]};

        CPPQOI_DEBUG(std::cout << "Input: " << inputFile.c_str() << std::endl);
        CPPQOI_DEBUG(std::cout << "Output: " << outputFile.c_str() << std::endl);

        if (inputFile.extension() == ".qoi") {
            std::ifstream inStream{inputFile.c_str(), std::ios::in | std::ios::binary};

            const Encoded encoded{std::istreambuf_iterator<char>(inStream),
                                  std::istreambuf_iterator<char>()};
            const BitMap bitmap{decode(encoded)};
            const std::string outExtension{outputFile.extension()};
            if (outExtension == ".bmp") {
                if (!stbi_write_bmp(outputFile.c_str(), bitmap.getWidth(), bitmap.getHeight(), STBI_rgb_alpha, bitmap.getData())) {
                    throw std::runtime_error("Failed to write output: " + outputFile.string());
                }
            } else if (outExtension == ".png") {
                stbi_write_png_compression_level = 16;
                if (!stbi_write_png(outputFile.c_str(), bitmap.getWidth(), bitmap.getHeight(), STBI_rgb_alpha, bitmap.getData(), bitmap.getWidth() * sizeof(Pixel))) {
                    throw std::runtime_error("Failed to write output: " + outputFile.string());
                }
            } else {
                throw std::runtime_error("Unsupported output filetype: " + outputFile.string());
            }
        } else {
            BitMap bitmap{inputFile};
            Encoded output{encode(bitmap)};
            std::ofstream outStream{outputFile, std::ios::out | std::ios::binary};
            outStream.write(reinterpret_cast<const char*>(output.data()), output.size());
        }
        
    }

    int
    main(int argc, char** argv) noexcept {
        static constexpr uint8_t bias = 2;
        constexpr uint8_t chunk = 3;
        constexpr uint8_t db = ((chunk & 0b0000'0011) - bias);
        try {
            mainImpl(argc, argv);
        } catch (const std::exception& ex) {
            std::cerr << ex.what() << std::endl;
            return 1;
        } catch (...) {
            std::cerr << "Unknown exception in main" << std::endl;
            return 1;
        }
        return 0;
    }
} // namespace cppqoi

int main(int argc, char** argv) noexcept {
    return cppqoi::main(argc, argv);
}