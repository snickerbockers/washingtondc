/*******************************************************************************
 *
 *
 *    WashingtonDC Dreamcast Emulator
 *    Copyright (C) 2017, 2019 snickerbockers
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 ******************************************************************************/

#ifndef GDI_FILE_H_
#define GDI_FILE_H_

#include <stdbool.h>

#include "washdc/stringlib.h"

#define GDI_DATA_TRACK 3

#define GDI_SECONDARY_DATA_TRACK 5

struct gdi_track {
    unsigned fad_start;    // block address offset
    unsigned ctrl;         // ???
    unsigned sector_size;  // sector size, typically (but not always) 2352

    /*
     * store both the relative and absolute paths.
     * relative is used for UI/error-reporting
     * absolute is how we actually access the file.
     */
    struct string rel_path, abs_path;

    unsigned offset;

    /* this is used in the loaded code, it should always be true */
    bool valid;
};

struct gdi_info {
    unsigned n_tracks;

    struct gdi_track *tracks;
};

void mount_gdi(char const *path);

#endif
