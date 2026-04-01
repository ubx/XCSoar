// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "Export.hpp"
#include "Buffer.hpp"

#ifdef DITHER
#include "Dither.hpp"
#endif

#include <cassert>

#ifdef SOFTWARE_ROTATE_DISPLAY
#include <algorithm>

template<typename D, typename S, typename C>
static void
RotateConvertCopy(D *dest, unsigned dest_pitch,
                  const S *src, unsigned src_pitch,
                  unsigned src_width, unsigned src_height,
                  DisplayOrientation orientation,
                  C convert)
{
  switch (TranslateDefaultDisplayOrientation(orientation)) {
  case DisplayOrientation::DEFAULT:
  case DisplayOrientation::LANDSCAPE:
    for (unsigned y = 0; y < src_height; ++y) {
      D *d = dest + y * dest_pitch;
      const S *s = src + y * src_pitch;
      for (unsigned x = 0; x < src_width; ++x)
        d[x] = convert(s[x]);
    }
    break;

  case DisplayOrientation::REVERSE_LANDSCAPE:
    for (unsigned y = 0; y < src_height; ++y) {
      D *d = dest + (src_height - 1 - y) * dest_pitch + (src_width - 1);
      const S *s = src + y * src_pitch;
      for (unsigned x = 0; x < src_width; ++x)
        *d-- = convert(*s++);
    }
    break;

  case DisplayOrientation::PORTRAIT:
    for (unsigned y = 0; y < src_height; ++y) {
      const S *s = src + y * src_pitch;
      for (unsigned x = 0; x < src_width; ++x)
        dest[y + (src_width - 1 - x) * dest_pitch] = convert(*s++);
    }
    break;

  case DisplayOrientation::REVERSE_PORTRAIT:
    for (unsigned y = 0; y < src_height; ++y) {
      const S *s = src + y * src_pitch;
      for (unsigned x = 0; x < src_width; ++x)
        dest[(src_height - 1 - y) + x * dest_pitch] = convert(*s++);
    }
    break;
  }
}

template<typename T>
static void
RotateCopy(T *dest, unsigned dest_pitch,
           const T *src, unsigned src_pitch,
           unsigned src_width, unsigned src_height,
           DisplayOrientation orientation)
{
  if (orientation == DisplayOrientation::DEFAULT ||
      orientation == DisplayOrientation::LANDSCAPE) {
    for (unsigned y = 0; y < src_height; ++y)
      std::copy_n(src + y * src_pitch, src_width, dest + y * dest_pitch);
    return;
  }

  RotateConvertCopy(dest, dest_pitch, src, src_pitch,
                    src_width, src_height, orientation,
                    [](T x) { return x; });
}
#endif

#ifdef GREYSCALE

#ifdef KOBO

static void
CopyGreyscale(uint8_t *dest_pixels, unsigned dest_pitch,
              const uint8_t *src_pixels, unsigned src_pitch,
              unsigned width, unsigned height)
{
  for (unsigned y = 0; y < height;
       ++y, dest_pixels += dest_pitch, src_pixels += src_pitch)
    std::copy_n(src_pixels, width, dest_pixels);
}

#endif /* KOBO */

void
CopyFromGreyscale(
#ifdef DITHER
                  Dither &dither,
#endif
#ifdef KOBO
                  bool enable_dither,
#endif
                  void *dest_pixels, unsigned dest_pitch, [[maybe_unused]] unsigned dest_bpp,
                  ConstImageBuffer<GreyscalePixelTraits> src
#ifdef SOFTWARE_ROTATE_DISPLAY
                  ,DisplayOrientation orientation)

  if (orientation != DisplayOrientation::DEFAULT &&
      orientation != DisplayOrientation::LANDSCAPE) {
    /* software rotate */
    if (dest_bpp == 2) {
      RotateConvertCopy((RGB565Color *)dest_pixels, dest_pitch / 2,
                        (const Luminosity8 *)src.data, src.pitch / sizeof(Luminosity8),
                        src.size.width, src.size.height, orientation,
                        [](const Luminosity8 &c) { return ToRGB565(c); });
    } else {
      RotateConvertCopy((uint32_t *)dest_pixels, dest_pitch / 4,
                        (const Luminosity8 *)src.data, src.pitch / sizeof(Luminosity8),
                        src.size.width, src.size.height, orientation,
                        [](const Luminosity8 &c) { return ToRGB8(c); });
    }
    return;
  }
#else
)
#endif
{
  const uint8_t *src_pixels = reinterpret_cast<const uint8_t *>(src.data);

#ifdef KOBO
  if (!enable_dither) {
    CopyGreyscale((uint8_t *)dest_pixels, dest_pitch,
                  src_pixels, src.pitch,
                  src.size.width, src.size.height);
    return;
  }
#endif

#ifdef DITHER

  dither.DitherGreyscale(src_pixels, src.pitch,
                         (uint8_t *)dest_pixels,
                         dest_pitch,
                         src.size.width, src.size.height);

#ifndef KOBO
  if (dest_bpp == 4) {
    const unsigned n_pixels = (dest_pitch / dest_bpp)
      * src.height;
    int32_t *d = (int32_t *)dest_pixels + n_pixels;
    const int8_t *end = (int8_t *)dest_pixels;
    const int8_t *s = end + n_pixels;

    while (s != end)
      *--d = *--s;
  }
#endif

#else

  const unsigned src_pitch = src.pitch;

  if (dest_bpp == 2) {
    for (unsigned row = src.size.height; row > 0;
         --row, src_pixels += src_pitch, dest_pixels += dest_pitch)
      CopyGreyscaleToRGB565((RGB565Color *)dest_pixels,
                            (const Luminosity8 *)src_pixels, src.size.width);
  } else {
    for (unsigned row = src.size.height; row > 0;
         --row, src_pixels += src_pitch, dest_pixels += dest_pitch)
      CopyGreyscaleToRGB8((uint32_t *)dest_pixels,
                           (const Luminosity8 *)src_pixels, src.size.width);
  }

#endif
}

#else /* GREYSCALE */

void
CopyFromBGRA(void *_dest_pixels, unsigned _dest_pitch, unsigned dest_bpp,
             ConstImageBuffer<BGRAPixelTraits> src
#ifdef SOFTWARE_ROTATE_DISPLAY
             ,DisplayOrientation orientation)
#else
)
#endif
{
  assert(dest_bpp == 4 || dest_bpp == 2);

  const uint32_t dest_pitch = _dest_pitch / dest_bpp;
  const uint32_t src_pitch = src.pitch / sizeof(*src.data);

  if (dest_bpp == 2) {
    /* convert to RGB565 */

    RGB565Color *dest_pixels = reinterpret_cast<RGB565Color *>(_dest_pixels);
    const BGRA8Color *src_pixels = src.data;

#ifdef SOFTWARE_ROTATE_DISPLAY
    if (orientation == DisplayOrientation::DEFAULT ||
        orientation == DisplayOrientation::LANDSCAPE) {
      for (unsigned row = src.size.height; row > 0;
           --row, src_pixels += src_pitch, dest_pixels += dest_pitch)
        BGRAToRGB565((RGB565Color *)dest_pixels,
                     (const BGRA8Color *)src_pixels,
                     src.size.width);
        } else {
          /* software rotate AND convert to RGB565 */
          RotateConvertCopy(dest_pixels, dest_pitch, src_pixels, src_pitch,
                            src.size.width, src.size.height, orientation,
                            [](const BGRA8Color &c) { return ToRGB565(c); });
        }
  } else {
    BGRA8Color *dest_pixels = reinterpret_cast<BGRA8Color *>(_dest_pixels);
    const BGRA8Color *src_pixels = src.data;
    RotateCopy(dest_pixels, dest_pitch, src_pixels, src_pitch,
               src.size.width, src.size.height, orientation);
  }
#else

    for (unsigned row = src.size.height; row > 0;
        --row, src_pixels += src_pitch, dest_pixels += dest_pitch)
      BGRAToRGB565((RGB565Color *)dest_pixels,
                   (const BGRA8Color *)src_pixels,
                   src.size.width);
  } else {
    BGRA8Color *dest_pixels = reinterpret_cast<BGRA8Color *>(_dest_pixels);
    const BGRA8Color *src_pixels = src.data;

    for (unsigned row = src.size.height; row > 0;
         --row, src_pixels += src_pitch, dest_pixels += dest_pitch)
      std::copy_n(src_pixels, src.size.width, dest_pixels);
  }
#endif
}

#endif
