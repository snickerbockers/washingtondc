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

#ifndef WASHDC_COMPILER_BULLSHIT_H_
#define WASHDC_COMPILER_BULLSHIT_H_

#if defined(__GNUC__) || defined(__GNUG__)

#define WASHDC_NORETURN __attribute__((__noreturn__))
#define WASHDC_UNUSED __attribute__((unused))

#elif defined(_MSC_VER)

#define WASHDC_NORETURN __declspec(noreturn)
#define WASHDC_UNUSED

#else
#error unknown compiler
#endif

#endif
