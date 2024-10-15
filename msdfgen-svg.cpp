
#include <string>

// Module API

std::string uploadGeneratedDistanceField(const std::string &svg, const std::string &settingsJson);
std::string encodeGeneratedDistanceField(const std::string &svg, const std::string &settingsJson);


// IMPLEMENTATION

#include <cstdio>
#include <cstring>
#include <stack>
#include <vector>
#include <algorithm>
#include <gl/GL.h>
#include <png.h>
#ifdef __EMSCRIPTEN__
#include <emscripten/bind.h>
#endif

#define MSDFGEN_NO_FREETYPE
#define MSDFGEN_ENABLE_SVG
#define MSDFGEN_USE_SKIA
#define MSDFGEN_USE_DROPXML
#include "skia-simplify.cpp"
#include "dropXML.hpp"
#define fopen(...) nullptr // Not used but prevent warnings
#include "msdfgen.cpp"
#include "Settings.h"
#include "SettingsParser.cpp"

#ifdef DEBUG_PRINTOUTS
#define dPrintf(...) fprintf(stderr, __VA_ARGS__)
#else
#define dPrintf(...)
#endif

using msdfgen::byte;

namespace {

struct Metrics {
    double scale;
    double pixelRange;
    double zeroDistanceValue;
};

Settings prevSettings = { };
Metrics prevMetrics = { };
std::string prevSvg;
bool prevResult = false;
std::vector<float> floatSdfBuffer;
std::vector<byte> byteSdfBuffer;

bool parseSettings(Settings &settings, const std::string &settingsJson) {
    settings.type = Settings::Type::msdf;
    settings.width = 256;
    settings.height = 256;
    settings.baseRange = 4;
    settings.outerRange = 0;
    settings.innerRange = 0;
    settings.rangeUnits = Settings::RangeUnits::pixels;
    settings.scaleMode = Settings::ScaleMode::explicitScale;
    settings.explicitScale = 1;
    settings.placementMode = Settings::PlacementMode::noTranslate;
    settings.miterLimit = 0;
    settings.format = Settings::Format::png;
    SettingsParser::Error error = SettingsParser::parse(settings, settingsJson.c_str());
    dPrintf("SettingsParser::parse result: %s @ %d\n", error.typeString(), error.position);
    return error == SettingsParser::Error::OK;
}

std::string serializeMetrics(const Metrics &metrics) {
    char buffer[256];
    sprintf(buffer, "{\"scale\":%.17g,\"pixelRange\":%.17g,\"zeroDistanceValue\":%.17g}", metrics.scale, metrics.pixelRange, metrics.zeroDistanceValue);
    return buffer;
}

bool generateDistanceField(Metrics &metrics, Settings &settings, const std::string &svg, const std::string &settingsJson) {
    dPrintf("generateDistanceField(svg[%d], %s)\n", int(svg.size()), settingsJson.c_str());
    /*dPrintf(
        "generateDistanceField("
        "    svg: %s\n"
        "    settings: %s\n"
        ")\n",
        svg.c_str(), settingsJson.c_str()
    );*/
    if (!parseSettings(settings, settingsJson)) {
        dPrintf("parseSettings failed\n");
        return false;
    }
    if (svg == prevSvg && !memcmp(&settings, &prevSettings, sizeof(Settings))) {
        dPrintf("Reusing previous result (%s)\n", prevResult ? "success" : "failure");
        if (prevResult)
            metrics = prevMetrics;
        return prevResult;
    }
    prevSvg = svg;
    prevSettings = settings;
    prevResult = false;

    double outerUnitRange = 0;
    double innerUnitRange = 0;
    double outerPxRange = 0;
    double innerPxRange = 0;

    switch (settings.rangeUnits) {
        case Settings::RangeUnits::svgUnits:
            outerUnitRange = .5*settings.baseRange + settings.outerRange;
            innerUnitRange = .5*settings.baseRange + settings.innerRange;
            break;
        case Settings::RangeUnits::pixels:
            outerPxRange = .5*settings.baseRange + settings.outerRange;
            innerPxRange = .5*settings.baseRange + settings.innerRange;
            break;
        /*case Settings::RangeUnits::proportionalToDimensions:
            outerPxRange = innerPxRange = std::max(settings.width, settings.height);
            outerPxRange *= .5*settings.baseRange + settings.outerRange;
            innerPxRange *= .5*settings.baseRange + settings.outerRange;
            break;*/
    }

    double miterLimit = settings.type != Settings::Type::sdf ? settings.miterLimit : 0;

    if (!(
        outerUnitRange + innerUnitRange >= 0 &&
        outerPxRange + innerPxRange >= 0 &&
        (outerUnitRange + innerUnitRange > 0 || outerPxRange + innerPxRange > 0) &&
        settings.width > 0 &&
        settings.height > 0 &&
        settings.miterLimit >= 0
    )) {
        dPrintf(
            "Settings check failed:\n"
            "    outerUnitRange = %.17g\n"
            "    innerUnitRange = %.17g\n"
            "    outerPxRange = %.17g\n"
            "    innerPxRange = %.17g\n"
            "    width = %11d\n"
            "    height =%11d\n"
            "    miterLimit = %.17g\n",
            outerUnitRange, innerUnitRange, outerPxRange, innerPxRange, settings.width, settings.height, settings.miterLimit
        );
        return false;
    }

    msdfgen::Shape shape;
    msdfgen::Shape::Bounds svgViewBox = { };
    int parseSvgResult = msdfgen::parseSvgShape(shape, svgViewBox, svg.c_str(), svg.size());
    if (!(parseSvgResult&msdfgen::SVG_IMPORT_SUCCESS_FLAG)) {
        dPrintf("msdfgen::parseSvgShape failed with %8x\n", parseSvgResult);
        return false;
    }

    msdfgen::Shape::Bounds bounds = { };
    if (settings.scaleMode == Settings::ScaleMode::fitBoundingBox || settings.placementMode == Settings::PlacementMode::centerBoundingBox)
        bounds = shape.getBounds(outerUnitRange, outerUnitRange > 0 ? settings.miterLimit : 0);

    double scale = 1;
    msdfgen::Vector2 translate;
    msdfgen::MSDFGeneratorConfig msdfgenConfig;
    msdfgenConfig.overlapSupport = false;
    switch (settings.scaleMode) {
        case Settings::ScaleMode::explicitScale:
            scale = settings.explicitScale;
            break;
        case Settings::ScaleMode::fitCanvas:
            scale = std::min(
                (double) settings.width / (double) (svgViewBox.r - svgViewBox.l),
                (double) settings.height / (double) (svgViewBox.t - svgViewBox.b)
            );
            break;
        case Settings::ScaleMode::fitPaddedCanvas:
            scale = std::min(
                ((double) settings.width - 2*outerPxRange) / ((double) (svgViewBox.r-svgViewBox.l) + 2*outerUnitRange),
                ((double) settings.height - 2*outerPxRange) / ((double) (svgViewBox.t-svgViewBox.b) + 2*outerUnitRange)
            );
            break;
        case Settings::ScaleMode::fitBoundingBox:
            scale = std::min(
                ((double) settings.width - 2*outerPxRange) / (double) (bounds.r - bounds.l),
                ((double) settings.height - 2*outerPxRange) / (double) (bounds.t - bounds.b)
            );
            break;
    }

    if (scale <= 0 || std::isinf(scale)) {
        dPrintf("Scale check failed, scale = %.17g\n", scale);
        return false;
    }

    double totalOuterPxRange = outerPxRange + scale*outerUnitRange;
    double totalInnerPxRange = innerPxRange + scale*innerUnitRange;
    double totalOuterUnitRange = outerUnitRange + outerPxRange/scale;
    double totalInnerUnitRange = innerUnitRange + innerPxRange/scale;
    metrics.scale = scale;
    metrics.pixelRange = totalOuterPxRange + totalInnerPxRange;
    metrics.zeroDistanceValue = totalOuterPxRange/metrics.pixelRange;

    switch (settings.placementMode) {
        case Settings::PlacementMode::noTranslate:
            break;
        case Settings::PlacementMode::paddedTranslate:
            translate = msdfgen::Vector2(totalOuterUnitRange, totalOuterUnitRange);
            break;
        case Settings::PlacementMode::centerCanvas:
            translate = .5*(msdfgen::Vector2(settings.width, settings.height)/scale - msdfgen::Vector2(svgViewBox.r-svgViewBox.l, svgViewBox.t-svgViewBox.b));
            break;
        case Settings::PlacementMode::centerBoundingBox:
            translate = .5*(msdfgen::Vector2(settings.width, settings.height)/scale - msdfgen::Vector2(bounds.r-bounds.l, bounds.t-bounds.b)) - msdfgen::Vector2(bounds.l, bounds.b);
            break;
    }
    msdfgen::SDFTransformation transformation(msdfgen::Projection(scale, translate), msdfgen::Range(-totalOuterUnitRange, totalInnerUnitRange));

    bool byteConversion = false;
    switch (settings.format) {
        case Settings::Format::png:
            byteConversion = true;
            break;
        case Settings::Format::tiff:
            byteConversion = false;
            break;
    }

    size_t pixelCount = (size_t) settings.width*settings.height;

    switch (settings.type) {
        case Settings::Type::sdf:
        {
            floatSdfBuffer.resize(1*pixelCount);
            msdfgen::BitmapRef<float, 1> floatBitmap(floatSdfBuffer.data(), settings.width, settings.height);
            msdfgen::generateSDF(floatBitmap, shape, transformation, msdfgenConfig);
            break;
        }
        case Settings::Type::psdf:
        {
            floatSdfBuffer.resize(1*pixelCount);
            msdfgen::BitmapRef<float, 1> floatBitmap(floatSdfBuffer.data(), settings.width, settings.height);
            msdfgen::generatePSDF(floatBitmap, shape, transformation, msdfgenConfig);
            break;
        }
        case Settings::Type::msdf:
        {
            floatSdfBuffer.resize(3*pixelCount);
            byteSdfBuffer.resize((byteConversion ? 3 : 1)*pixelCount);
            msdfgenConfig.errorCorrection.buffer = byteSdfBuffer.data();
            msdfgen::BitmapRef<float, 3> floatBitmap(floatSdfBuffer.data(), settings.width, settings.height);
            msdfgen::edgeColoringSimple(shape, 3);
            msdfgen::generateMSDF(floatBitmap, shape, transformation, msdfgenConfig);
            break;
        }
        case Settings::Type::mtsdf:
        {
            floatSdfBuffer.resize(4*pixelCount);
            byteSdfBuffer.resize((byteConversion ? 4 : 1)*pixelCount);
            msdfgenConfig.errorCorrection.buffer = byteSdfBuffer.data();
            msdfgen::BitmapRef<float, 4> floatBitmap(floatSdfBuffer.data(), settings.width, settings.height);
            msdfgen::edgeColoringSimple(shape, 3);
            msdfgen::generateMTSDF(floatBitmap, shape, transformation, msdfgenConfig);
            break;
        }
    }

    if (byteConversion) {
        byteSdfBuffer.resize(floatSdfBuffer.size());
        const float *src = floatSdfBuffer.data();
        for (byte *dst = byteSdfBuffer.data(), *dstEnd = dst+byteSdfBuffer.size(); dst < dstEnd; ++dst, ++src)
            *dst = msdfgen::pixelFloatToByte(*src);
    }

    dPrintf("generateDistanceField succeeded\n");
    prevMetrics = metrics;
    return prevResult = true;
}

// PNG

class PngGuard {
    png_structp png;
    png_infop info;

public:
    inline PngGuard(png_structp png, png_infop info) : png(png), info(info) { }
    inline ~PngGuard() {
        png_destroy_write_struct(&png, &info);
    }

};

void pngIgnoreError(png_structp, png_const_charp) { }

void pngWrite(png_structp png, png_bytep data, png_size_t length) {
    std::vector<byte> &output = *reinterpret_cast<std::vector<byte> *>(png_get_io_ptr(png));
    output.insert(output.end(), data, data+length);
}

void pngFlush(png_structp) { }

bool pngEncode(std::vector<byte> &output, const byte *pixels, int width, int height, int channels, int colorType) {
    if (!(pixels && width && height))
        return false;
    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, &pngIgnoreError, &pngIgnoreError);
    if (!png)
        return false;
    png_infop info = png_create_info_struct(png);
    PngGuard guard(png, info);
    if (!info)
        return false;
    std::vector<const byte *> rows(height);
    for (int y = 0; y < height; ++y)
        rows[y] = pixels+channels*width*(height-y-1);
    if (setjmp(png_jmpbuf(png)))
        return false;
    png_set_write_fn(png, &output, &pngWrite, &pngFlush);
    png_set_IHDR(png, info, width, height, 8, colorType, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
    png_set_compression_level(png, 9);
    png_set_rows(png, info, const_cast<png_bytepp>(&rows[0]));
    png_write_png(png, info, PNG_TRANSFORM_IDENTITY, NULL);
    return true;
}

// BASE64

#define CEILDIV(x, d) (((x)+((d)-1))/(d))

const char BASE64_DIGITS[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

size_t base64Length(size_t dataLen) {
    return 4*CEILDIV(dataLen, 3);
}

std::string base64Encode(const byte *data, size_t length) {
    std::string result;
    result.resize(base64Length(length));
    char *dst = result.data();
    const byte *src = data, *end = data+length-2;
    for (; src < end; src += 3, dst += 4) {
        dst[0] = BASE64_DIGITS[src[0]>>2];
        dst[1] = BASE64_DIGITS[(src[0]<<4&0x3f)|(src[1]>>4&0x0f)];
        dst[2] = BASE64_DIGITS[(src[1]<<2&0x3f)|(src[2]>>6&0x03)];
        dst[3] = BASE64_DIGITS[src[2]&0x3f];
    }
    if (src < end+2) {
        dst[0] = BASE64_DIGITS[src[0]>>2];
        if (src == end) {
            dst[1] = BASE64_DIGITS[(src[0]<<4&0x3f)|(src[1]>>4&0x0f)];
            dst[2] = BASE64_DIGITS[src[1]<<2&0x3f];
        } else {
            dst[1] = BASE64_DIGITS[src[0]<<4&0x3f];
            dst[2] = '=';
        }
        dst[3] = '=';
    }
    return (std::string &&) result;
}

}

std::string uploadGeneratedDistanceField(const std::string &svg, const std::string &settingsJson) {
    Metrics metrics;
    Settings settings;
    if (!generateDistanceField(metrics, settings, svg, settingsJson)) {
        dPrintf("generateDistanceField failed\n");
        return std::string();
    }

    switch (settings.format) {
        case Settings::Format::png:
            switch (prevSettings.type) {
                case Settings::Type::sdf:
                case Settings::Type::psdf:
                    glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, settings.width, settings.height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, byteSdfBuffer.data());
                    break;
                case Settings::Type::msdf:
                    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, settings.width, settings.height, 0, GL_RGB, GL_UNSIGNED_BYTE, byteSdfBuffer.data());
                    break;
                case Settings::Type::mtsdf:
                    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, settings.width, settings.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, byteSdfBuffer.data());
                    break;
            }
            break;
        case Settings::Format::tiff:
            // TODO
            dPrintf("TODO tiff\n");
            return std::string();
    }

    dPrintf("GL error status: %d\n", int(glGetError()));
    dPrintf("uploadGeneratedDistanceField succeeded\n");
    return serializeMetrics(metrics);
}

std::string encodeGeneratedDistanceField(const std::string &svg, const std::string &settingsJson) {
    Metrics metrics;
    Settings settings;
    if (!generateDistanceField(metrics, settings, svg, settingsJson)) {
        dPrintf("encodeGeneratedDistanceField failed\n");
        return std::string();
    }

    bool success = false;
    std::vector<byte> encodedImage;
    switch (settings.format) {
        case Settings::Format::png:
            switch (prevSettings.type) {
                case Settings::Type::sdf:
                case Settings::Type::psdf:
                    success = pngEncode(encodedImage, byteSdfBuffer.data(), settings.width, settings.height, 1, PNG_COLOR_TYPE_GRAY);
                    break;
                case Settings::Type::msdf:
                    success = pngEncode(encodedImage, byteSdfBuffer.data(), settings.width, settings.height, 3, PNG_COLOR_TYPE_RGB);
                    break;
                case Settings::Type::mtsdf:
                    success = pngEncode(encodedImage, byteSdfBuffer.data(), settings.width, settings.height, 4, PNG_COLOR_TYPE_RGB_ALPHA);
                    break;
            }
            break;
        case Settings::Format::tiff:
            // TODO
            dPrintf("TODO tiff\n");
            break;
    }
    if (success)
        return base64Encode(encodedImage.data(), encodedImage.size());
    return std::string();
}

#ifdef __EMSCRIPTEN__

using namespace emscripten;

EMSCRIPTEN_BINDINGS(my_module) {
    function("uploadGeneratedDistanceField", &uploadGeneratedDistanceField);
    function("encodeGeneratedDistanceField", &encodeGeneratedDistanceField);
}

#endif
