
#pragma once

struct Settings {

    enum class Type {
        sdf,
        psdf,
        msdf,
        mtsdf
    } type;

    int width, height;

    double baseRange;
    double outerRange, innerRange;

    enum class RangeUnits {
        pixels,
        svgUnits
    } rangeUnits;

    enum class ScaleMode {
        explicitScale,
        fitCanvas,
        fitPaddedCanvas,
        fitBoundingBox
    } scaleMode;

    double explicitScale;

    enum class PlacementMode {
        noTranslate,
        paddedTranslate,
        centerCanvas,
        centerBoundingBox
    } placementMode;

    double miterLimit;

    enum class Format {
        png,
        tiff
    } format;

};
