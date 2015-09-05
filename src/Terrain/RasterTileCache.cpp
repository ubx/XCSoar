/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2015 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#include "Terrain/RasterTileCache.hpp"
#include "Terrain/RasterLocation.hpp"
#include "WorldFile.hpp"
#include "ZzipStream.hpp"
#include "Math/Angle.hpp"
#include "IO/ZipLineReader.hpp"
#include "Operation/Operation.hpp"
#include "Math/FastMath.h"

extern "C" {
#include "jasper/jp2/jp2_cod.h"
#include "jasper/jpc/jpc_dec.h"
#include "jasper/jpc/jpc_t1cod.h"
}

#include <string.h>
#include <algorithm>

static void
CopyOverviewRow(short *gcc_restrict dest, const jas_seqent_t *gcc_restrict src,
                unsigned width, unsigned skip)
{
  for (unsigned x = 0; x < width; x += skip)
    *dest++ = src[x];
}

void
RasterTileCache::PutTileData(unsigned index,
                             unsigned start_x, unsigned start_y,
                             unsigned end_x, unsigned end_y,
                             const struct jas_matrix &m)
{
  if (scan_overview) {
    tiles.GetLinear(index).Set(start_x, start_y, end_x, end_y);

    const unsigned dest_pitch = overview.GetWidth();

    start_x >>= OVERVIEW_BITS;
    start_y >>= OVERVIEW_BITS;

    if (start_x >= overview.GetWidth() || start_y >= overview.GetHeight())
      return;

    unsigned width = m.numcols_, height = m.numrows_;
    if (start_x + (width >> OVERVIEW_BITS) > overview.GetWidth())
      width = (overview.GetWidth() - start_x) << OVERVIEW_BITS;
    if (start_y + (height >> OVERVIEW_BITS) > overview.GetHeight())
      height = (overview.GetHeight() - start_y) << OVERVIEW_BITS;

    const unsigned skip = 1 << OVERVIEW_BITS;

    short *gcc_restrict dest = overview.GetData()
      + start_y * dest_pitch + start_x;

    for (unsigned y = 0; y < height; y += skip, dest += dest_pitch)
      CopyOverviewRow(dest, m.rows_[y], width, skip);
  } else {
    auto &tile = tiles.GetLinear(index);
    if (!tile.IsRequested())
      return;

    tile.CopyFrom(m);
  }
}

struct RTDistanceSort {
  const RasterTileCache &rtc;

  RTDistanceSort(RasterTileCache &_rtc):rtc(_rtc) {}

  bool operator()(unsigned short ai, unsigned short bi) const {
    const RasterTile &a = rtc.tiles.GetLinear(ai);
    const RasterTile &b = rtc.tiles.GetLinear(bi);

    return a.GetDistance() < b.GetDistance();
  }
};

inline bool
RasterTileCache::PollTiles(int x, int y, unsigned radius)
{
  if (scan_overview)
    return false;

  /* tiles are usually 256 pixels wide; with a radius smaller than
     that, the (optimized) tile distance calculations may fail;
     additionally, this ensures that tiles which are slightly out of
     the screen will be loaded in advance */
  radius += 256;

  /**
   * Maximum number of tiles loaded at a time, to reduce system load
   * peaks.
   */
  constexpr unsigned MAX_ACTIVATE = MAX_ACTIVE_TILES > 32
    ? 16
    : MAX_ACTIVE_TILES / 2;

  /* query all tiles; all tiles which are either in range or already
     loaded are added to RequestTiles */

  request_tiles.clear();
  for (int i = tiles.GetSize() - 1; i >= 0 && !request_tiles.full(); --i)
    if (tiles.GetLinear(i).VisibilityChanged(x, y, radius))
      request_tiles.append(i);

  /* reduce if there are too many */

  if (request_tiles.size() > MAX_ACTIVE_TILES) {
    /* sort by distance */
    const RTDistanceSort sort(*this);
    std::sort(request_tiles.begin(), request_tiles.end(), sort);

    /* dispose all tiles which are out of range */
    for (unsigned i = MAX_ACTIVE_TILES; i < request_tiles.size(); ++i) {
      RasterTile &tile = tiles.GetLinear(request_tiles[i]);
      tile.Disable();
    }

    request_tiles.shrink(MAX_ACTIVE_TILES);
  }

  /* fill ActiveTiles and request new tiles */

  dirty = false;

  unsigned num_activate = 0;
  for (unsigned i = 0; i < request_tiles.size(); ++i) {
    RasterTile &tile = tiles.GetLinear(request_tiles[i]);
    if (tile.IsEnabled())
      continue;

    if (++num_activate <= MAX_ACTIVATE)
      /* request the tile in the current iteration */
      tile.SetRequest();
    else
      /* this tile will be loaded in the next iteration */
      dirty = true;
  }

  return num_activate > 0;
}

short
RasterTileCache::GetHeight(unsigned px, unsigned py) const
{
  if (px >= width || py >= height)
    // outside overall bounds
    return RasterBuffer::TERRAIN_INVALID;

  const RasterTile &tile = tiles.Get(px / tile_width, py / tile_height);
  if (tile.IsEnabled())
    return tile.GetHeight(px, py);

  // still not found, so go to overview
  return overview.GetInterpolated(px << (SUBPIXEL_BITS - OVERVIEW_BITS),
                                   py << (SUBPIXEL_BITS - OVERVIEW_BITS));
}

short
RasterTileCache::GetInterpolatedHeight(unsigned int lx, unsigned int ly) const
{
  if ((lx >= overview_width_fine) || (ly >= overview_height_fine))
    // outside overall bounds
    return RasterBuffer::TERRAIN_INVALID;

  unsigned px = lx, py = ly;
  const unsigned int ix = CombinedDivAndMod(px);
  const unsigned int iy = CombinedDivAndMod(py);

  const RasterTile &tile = tiles.Get(px / tile_width, py / tile_height);
  if (tile.IsEnabled())
    return tile.GetInterpolatedHeight(px, py, ix, iy);

  // still not found, so go to overview
  return overview.GetInterpolated(lx >> OVERVIEW_BITS,
                                   ly >> OVERVIEW_BITS);
}

void
RasterTileCache::SetSize(unsigned _width, unsigned _height,
                         unsigned _tile_width, unsigned _tile_height,
                         unsigned tile_columns, unsigned tile_rows)
{
  width = _width;
  height = _height;
  tile_width = _tile_width;
  tile_height = _tile_height;

  overview.Resize(width >> OVERVIEW_BITS, height >> OVERVIEW_BITS);
  overview_width_fine = width << SUBPIXEL_BITS;
  overview_height_fine = height << SUBPIXEL_BITS;

  tiles.GrowDiscard(tile_columns, tile_rows);
}

void
RasterTileCache::SetLatLonBounds(double _lon_min, double _lon_max,
                                 double _lat_min, double _lat_max)
{
  const Angle lon_min(Angle::Degrees(_lon_min));
  const Angle lon_max(Angle::Degrees(_lon_max));
  const Angle lat_min(Angle::Degrees(_lat_min));
  const Angle lat_max(Angle::Degrees(_lat_max));

  bounds = GeoBounds(GeoPoint(std::min(lon_min, lon_max),
                              std::max(lat_min, lat_max)),
                     GeoPoint(std::max(lon_min, lon_max),
                              std::min(lat_min, lat_max)));
}

inline void
RasterTileCache::ParseBounds(const char *data)
{
  /* this code is obsolete, since new map files include a "world
     file", but we keep it for compatibility */

  data = strstr(data, "XCSoar");
  if (data == nullptr)
    return;

  float lon_min, lon_max, lat_min, lat_max;
  if (sscanf(data + 6, "%f %f %f %f",
             &lon_min, &lon_max, &lat_min, &lat_max) == 4)
    SetLatLonBounds(lon_min, lon_max, lat_min, lat_max);
}

void
RasterTileCache::ProcessComment(const char *data, unsigned size)
{
  if (scan_overview) {
    char buffer[128];
    if (size < sizeof(buffer)) {
      memcpy(buffer, data, size);
      buffer[size] = 0;

      ParseBounds(buffer);
    }
  }
}

void
RasterTileCache::Reset()
{
  width = 0;
  height = 0;
  bounds.SetInvalid();
  segments.clear();
  scan_overview = true;

  overview.Reset();

  for (auto it = tiles.begin(), end = tiles.end(); it != end; ++it)
    it->Disable();
}

gcc_pure
inline const RasterTileCache::MarkerSegmentInfo *
RasterTileCache::FindMarkerSegment(uint32_t file_offset) const
{
  for (const auto &s : segments)
    if (s.file_offset >= file_offset)
      return &s;

  return nullptr;
}

long
RasterTileCache::SkipMarkerSegment(long file_offset) const
{
  if (scan_overview)
    /* use all segments when loading the overview */
    return 0;

  if (remaining_segments > 0) {
    /* enable the follow-up segment */
    --remaining_segments;
    return 0;
  }

  const MarkerSegmentInfo *segment = FindMarkerSegment(file_offset);
  if (segment == nullptr)
    /* past the end of the recorded segment list; shouldn't happen */
    return 0;

  long skip_to = segment->file_offset;
  while (segment->IsTileSegment() &&
         !tiles.GetLinear(segment->tile).IsRequested()) {
    ++segment;
    if (segment >= segments.end())
      /* last segment is hidden; shouldn't happen either, because we
         expect EOC there */
      break;

    skip_to = segment->file_offset;
  }

  remaining_segments = segment->count;
  return skip_to - file_offset;
}

/**
 * Does this segment belong to the preceding tile?  If yes, then it
 * inherits the tile number.
 */
static constexpr bool
IsTileSegment(unsigned id)
{
  return id == 0xff93 /* SOD */ ||
    id == 0xff52 /* COD */ ||
    id == 0xff53 /* COC */ ||
    id == 0xff5c /* QCD */ ||
    id == 0xff5d /* QCC */ ||
    id == 0xff5e /* RGN */ ||
    id == 0xff5f /* POC */ ||
    id == 0xff61 /* PPT */ ||
    id == 0xff58 /* PLT */ ||
    id == 0xff64 /* COM */;
}

void
RasterTileCache::MarkerSegment(long file_offset, unsigned id)
{
  if (!scan_overview || segments.full())
    return;

  if (operation != nullptr)
    operation->SetProgressPosition(file_offset / 65536);

  if (IsTileSegment(id) && !segments.empty() &&
      segments.back().IsTileSegment()) {
    /* this segment belongs to the same tile as the preceding SOT
       segment */
    ++segments.back().count;
    return;
  }

  if (segments.size() >= 2 && !segments.back().IsTileSegment() &&
      !segments[segments.size() - 2].IsTileSegment()) {
    /* the last two segments are both "generic" segments and can be merged*/
    assert(segments.back().count == 0);

    ++segments[segments.size() - 2].count;

    /* reuse the second segment */
    segments.back().file_offset = file_offset;
  } else
    segments.append(MarkerSegmentInfo(file_offset,
                                      MarkerSegmentInfo::NO_TILE));
}

extern thread_local RasterTileCache *raster_tile_current;

static bool
LoadJPG2000(jas_stream_t *in, bool scan_overview)
{
  /* Get the first box.  This should be a JP box. */
  auto box = jp2_box_get(in);
  if (box == nullptr)
    return false;

  if (box->type != JP2_BOX_JP ||
      box->data.jp.magic != JP2_JP_MAGIC) {
    jp2_box_destroy(box);
    return false;
  }

  jp2_box_destroy(box);

  /* Get the second box.  This should be a FTYP box. */
  box = jp2_box_get(in);
  if (box == nullptr)
    return false;

  auto type = box->type;
  jp2_box_destroy(box);
  if (type != JP2_BOX_FTYP)
    return false;

  /* find the JP2C box */
  do {
    box = jp2_box_get(in);
    if (box == nullptr)
      /* not found */
      return false;

    type = box->type;
    jp2_box_destroy(box);
  } while (type != JP2_BOX_JP2C);

  jpc_dec_importopts_t opts;
  opts.debug = 0;
  opts.maxlyrs = JPC_MAXLYRS;
  opts.maxpkts = -1;

  jpc_initluts();

  const auto dec = jpc_dec_create(&opts, in);
  if (dec == nullptr)
    return false;

  bool success = jpc_dec_decode(dec) == 0;
  jpc_dec_destroy(dec);
  return success;
}

bool
RasterTileCache::LoadJPG2000(const char *jp2_filename)
{
  raster_tile_current = this;

  const auto in = OpenJasperZzipStream(jp2_filename);
  if (!in) {
    Reset();
    return false;
  }

  if (operation != nullptr)
    operation->SetProgressRange(jas_stream_length(in) / 65536);

  bool success = ::LoadJPG2000(in, scan_overview);
  jas_stream_close(in);
  return success;
}

bool
RasterTileCache::LoadWorldFile(const TCHAR *path)
{
  const auto new_bounds = ::LoadWorldFile(path, GetWidth(), GetHeight());
  bool success = new_bounds.IsValid();
  if (success)
    bounds = new_bounds;
  return success;
}

bool
RasterTileCache::LoadOverview(const char *path, const TCHAR *world_file,
                              OperationEnvironment &_operation)
{
  assert(operation == nullptr);
  operation = &_operation;

  Reset();

  bool initialised = LoadJPG2000(path);
  scan_overview = false;

  if (initialised && world_file != nullptr)
    LoadWorldFile(world_file);

  if (initialised && !bounds.IsValid())
    initialised = false;

  if (!initialised)
    Reset();

  operation = nullptr;
  return initialised;
}

void
RasterTileCache::UpdateTiles(const char *path, int x, int y, unsigned radius)
{
  if (!PollTiles(x, y, radius))
    return;

  remaining_segments = 0;

  LoadJPG2000(path);

  /* permanently disable the requested tiles which are still not
     loaded, to prevent trying to reload them over and over in a busy
     loop */
  for (auto it = request_tiles.begin(), end = request_tiles.end();
      it != end; ++it) {
    RasterTile &tile = tiles.GetLinear(*it);
    if (tile.IsRequested() && !tile.IsEnabled())
      tile.Clear();
  }

  ++serial;
}

bool
RasterTileCache::SaveCache(FILE *file) const
{
  if (!IsValid())
    return false;

  assert(bounds.IsValid());

  /* save metadata */
  CacheHeader header;

  /* zero-fill all implicit padding bytes (to make valgrind happy) */
  memset(&header, 0, sizeof(header));

  header.version = CacheHeader::VERSION;
  header.width = width;
  header.height = height;
  header.tile_width = tile_width;
  header.tile_height = tile_height;
  header.tile_columns = tiles.GetWidth();
  header.tile_rows = tiles.GetHeight();
  header.num_marker_segments = segments.size();
  header.bounds = bounds;

  if (fwrite(&header, sizeof(header), 1, file) != 1 ||
      /* .. and segments */
      fwrite(segments.begin(), sizeof(*segments.begin()), segments.size(), file) != segments.size())
    return false;

  /* save tiles */
  unsigned i;
  for (i = 0; i < tiles.GetSize(); ++i)
    if (tiles.GetLinear(i).IsDefined() &&
        (fwrite(&i, sizeof(i), 1, file) != 1 ||
         !tiles.GetLinear(i).SaveCache(file)))
      return false;

  i = -1;
  if (fwrite(&i, sizeof(i), 1, file) != 1)
    return false;

  /* save overview */
  size_t overview_size = overview.GetWidth() * overview.GetHeight();
  if (fwrite(overview.GetData(), sizeof(*overview.GetData()),
             overview_size, file) != overview_size)
    return false;

  /* done */
  return true;
}

bool
RasterTileCache::LoadCache(FILE *file)
{
  Reset();

  /* load metadata */
  CacheHeader header;
  if (fread(&header, sizeof(header), 1, file) != 1 ||
      header.version != CacheHeader::VERSION ||
      header.width < 1024 || header.width > 1024 * 1024 ||
      header.height < 1024 || header.height > 1024 * 1024 ||
      header.num_marker_segments < 4 ||
      header.num_marker_segments > segments.capacity() ||
      header.bounds.IsEmpty())
    return false;

  SetSize(header.width, header.height,
          header.tile_width, header.tile_height,
          header.tile_columns, header.tile_rows);
  bounds = header.bounds;
  if (!bounds.IsValid())
    return false;

  /* load segments */
  for (unsigned i = 0; i < header.num_marker_segments; ++i) {
    MarkerSegmentInfo &segment = segments.append();
    if (fread(&segment, sizeof(segment), 1, file) != 1)
      return false;
  }

  /* load tiles */
  unsigned i;
  while (true) {
    if (fread(&i, sizeof(i), 1, file) != 1)
      return false;

    if (i == (unsigned)-1)
      break;

    if (i >= tiles.GetSize())
      return false;

    if (!tiles.GetLinear(i).LoadCache(file))
      return false;
  }

  /* load overview */
  size_t overview_size = overview.GetWidth() * overview.GetHeight();
  if (fread(overview.GetData(), sizeof(*overview.GetData()),
            overview_size, file) != overview_size)
    return false;

  scan_overview = false;
  return true;
}
