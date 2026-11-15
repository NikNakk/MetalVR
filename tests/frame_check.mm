// Looks at a frame metalvrd composed from xr_smoke's rendering (--snapshot) and checks that it is the picture
// xr_smoke drew: the left eye red, the right eye blue, and its checkerboard where it was put.
//
//   frame_check FRAME.png [--pattern top|bottom|none]
//
// xr_smoke stamps the checkerboard in the upper half of each eye. `--pattern top` requires it there, `bottom`
// requires it in the lower half (a vertically flipped submission), `none` does not look for it. The pattern is
// only looked for in a frame without the quad layer, which carries a checkerboard of its own.
#import <CoreGraphics/CoreGraphics.h>
#import <Foundation/Foundation.h>
#import <ImageIO/ImageIO.h>

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

int main(int argc, char **argv) {
  @autoreleasepool {
    if (argc < 2) {
      std::printf("usage: frame_check FRAME.png [--pattern top|bottom|none]\n");
      return 2;
    }
    std::string pattern = "none";
    for (int i = 2; i < argc; ++i)
      if (!std::strcmp(argv[i], "--pattern") && i + 1 < argc)
        pattern = argv[++i];

    NSURL *url = [NSURL fileURLWithPath:[NSString stringWithUTF8String:argv[1]]];
    CGImageSourceRef source = CGImageSourceCreateWithURL((__bridge CFURLRef)url, nullptr);
    CGImageRef image = source ? CGImageSourceCreateImageAtIndex(source, 0, nullptr) : nullptr;
    if (!image) {
      std::printf("FAILED: cannot read %s\n", argv[1]);
      return 1;
    }
    const size_t width = CGImageGetWidth(image), height = CGImageGetHeight(image);
    std::vector<uint8_t> pixels(width * height * 4);
    CGColorSpaceRef space = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
    CGContextRef context = CGBitmapContextCreate(pixels.data(), width, height, 8, width * 4, space,
                                                 static_cast<uint32_t>(kCGImageAlphaNoneSkipLast));
    CGContextDrawImage(context, CGRectMake(0, 0, width, height), image);
    CGContextRelease(context);
    CGColorSpaceRelease(space);
    CGImageRelease(image);
    CFRelease(source);

    int failures = 0;
    const size_t eye_width = width / 2;
    double white_y[2] = {0, 0};
    size_t white[2] = {0, 0};
    for (int eye = 0; eye < 2; ++eye) {
      double sum[3] = {0, 0, 0};
      for (size_t y = 0; y < height; ++y)
        for (size_t x = 0; x < eye_width; ++x) {
          const uint8_t *p = &pixels[(y * width + eye * eye_width + x) * 4];
          sum[0] += p[0];
          sum[1] += p[1];
          sum[2] += p[2];
          if (p[0] > 235 && p[1] > 235 && p[2] > 235) {
            ++white[eye];
            white_y[eye] += static_cast<double>(y);
          }
        }
      const double count = static_cast<double>(eye_width * height);
      const double r = sum[0] / count, g = sum[1] / count, b = sum[2] / count;
      std::printf("%s eye: mean color (%.0f %.0f %.0f)", eye == 0 ? "left" : "right", r, g, b);
      const bool ok = eye == 0 ? r > b + 40 : b > r + 40;
      std::printf(ok ? "\n" : " -- expected %s\n", eye == 0 ? "red" : "blue");
      failures += ok ? 0 : 1;
    }
    if (pattern != "none") {
      for (int eye = 0; eye < 2; ++eye) {
        // the checkerboard is 256 pixels square, half of it white
        if (white[eye] < 256 * 256 / 4) {
          std::printf("%s eye: no checkerboard\n", eye == 0 ? "left" : "right");
          ++failures;
          continue;
        }
        const double where = white_y[eye] / static_cast<double>(white[eye]) / static_cast<double>(height);
        const bool top = where < 0.45, bottom = where > 0.55;
        const bool ok = pattern == "top" ? top : bottom;
        std::printf("%s eye: checkerboard at %.2f of the height%s\n", eye == 0 ? "left" : "right", where,
                    ok ? "" : (pattern == "top" ? " -- expected in the upper half" : " -- expected in the lower half"));
        failures += ok ? 0 : 1;
      }
    }
    std::printf(failures ? "FAILED: the composed frame is not what was drawn\n" : "the composed frame is what was drawn\n");
    return failures ? 1 : 0;
  }
}
