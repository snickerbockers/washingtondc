/*******************************************************************************
 *
 *
 *    WashingtonDC Dreamcast Emulator
 *    Copyright (C) 2017-2020 snickerbockers
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

#ifndef OPENGL_TARGET_H_
#define OPENGL_TARGET_H_

#include "washdc/gfx/gfx_il.h"

/* code for configuring opengl's rendering target (which is a texture+FBO) */

void opengl_target_init(void);

void opengl_target_bind_obj(struct gfx_il_inst *cmd);
void opengl_target_unbind_obj(struct gfx_il_inst *cmd);

// call this before rendering to the target
void opengl_target_begin(unsigned width, unsigned height, int tgt_handle);

// call this when done rendering to the target
void opengl_target_end(int tgt_handle);

#endif
