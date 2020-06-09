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

#ifdef _WIN32
#include "i_hate_windows.h"
#endif

#ifndef OPENGL_RENDERER_H_
#define OPENGL_RENDERER_H_

#include <GL/gl.h>

#include "washdc/gfx/gfx_all.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * functions the renderer calls to interact with stuff like the windowing
 * system and overlay.
 */
struct opengl_renderer_callbacks {
    // tells the window to check for events.  This is optional and can be NULL
    void (*win_update)(void);

    // tells the overlay to draw using OpenGL.  This is optional and can be NULL
    void (*overlay_draw)(void);
};
void
opengl_renderer_set_callbacks(struct opengl_renderer_callbacks const
                              *callbacks);

extern struct rend_if const opengl_rend_if;

GLuint opengl_renderer_tex(unsigned obj_no);

unsigned opengl_renderer_tex_get_width(unsigned obj_no);
unsigned opengl_renderer_tex_get_height(unsigned obj_no);

void opengl_renderer_tex_set_dims(unsigned obj_no,
                                  unsigned width, unsigned height);
void opengl_renderer_tex_set_format(unsigned obj_no, GLenum fmt);
void opengl_renderer_tex_set_dat_type(unsigned obj_no, GLenum dat_tp);
void opengl_renderer_tex_set_dirty(unsigned obj_no, bool dirty);
GLenum opengl_renderer_tex_get_format(unsigned obj_no);
GLenum opengl_renderer_tex_get_dat_type(unsigned obj_no);
bool opengl_renderer_tex_get_dirty(unsigned obj_no);

void opengl_renderer_update_tex(unsigned tex_obj);
void opengl_renderer_release_tex(unsigned tex_obj);

#ifdef __cplusplus
}
#endif

#endif
