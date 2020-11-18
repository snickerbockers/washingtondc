/*******************************************************************************
 *
 *
 *    WashingtonDC Dreamcast Emulator
 *    Copyright (C) 2020 snickerbockers
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

#ifndef CONFIG_FILE_H_
#define CONFIG_FILE_H_

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * text file containing configuration settings.
 *
 * This is completely unrelated to the bullshit in config.h/config.c; that only
 * pertains to runtime settings and not everything in there even maps to the
 * config file.
 */

void cfg_init(FILE *cfg_file);
void cfg_cleanup(void);

void cfg_create_default_config(FILE *cfg_file);

void cfg_put_char(char ch);

char const *cfg_get_node(char const *key);

int cfg_get_bool(char const *key, bool *outp);

int cfg_get_rgb(char const *key, int *red, int *green, int *blue);

int cfg_get_int(char const *key, int *val);

#ifdef __cplusplus
}
#endif

#endif
