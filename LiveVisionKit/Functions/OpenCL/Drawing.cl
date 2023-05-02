R"(
//     *************************** LiveVisionKit ****************************
//     Copyright (C) 2022  Sebastian Di Marco (crowsinc.dev@gmail.com)
//
//     This program is free software: you can redistribute it and/or modify
//     it under the terms of the GNU General Public License as published by
//     the Free Software Foundation, either version 3 of the License, or
//     (at your option) any later version.
//
//     This program is distributed in the hope that it will be useful,
//     but WITHOUT ANY WARRANTY; without even the implied warranty of
//     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//     GNU General Public License for more details.
//
//     You should have received a copy of the GNU General Public License
//     along with this program.  If not, see <https://www.gnu.org/licenses/>.
//     **********************************************************************


//----------------------------------------------------------------------------------------------------------------------

__kernel void grid(
    __global uchar* dst, int dst_step, int dst_offset, int dst_rows, int dst_cols,
    float cell_width, float cell_height, int line_thickness, uchar4 line_colour 
)
{
    int2 coord = (int2)(get_global_id(0), get_global_id(1));
    int index = coord.y * dst_step + (3 * coord.x) + dst_offset; 

    if(coord.x < dst_cols && coord.y < dst_rows && (fmod(coord.x, cell_width) < line_thickness || fmod(coord.y, cell_height) < line_thickness))
        vstore3(line_colour.xyz, 0, dst + index); 
}

//----------------------------------------------------------------------------------------------------------------------

__kernel void points(
    __global uchar* pts, int pts_step, int pts_offset, int pts_rows, int pts_cols,
    __global uchar* dst, int dst_step, int dst_offset, int dst_rows, int dst_cols,
    int point_width, uchar4 point_colour
)
{
    // Exit early if out of bounds (for uneven sizes)
    if(get_global_id(0) >= pts_rows) return;
 
    int pts_index = 8 * get_global_id(0) + pts_offset;
    int2 coord = as_int2(vload8(0, pts + pts_index));

    int min_x = max(coord.x - point_width, 0);
    int min_y = max(coord.y - point_width, 0);
    int max_x = min(coord.x + point_width, dst_cols);
    int max_y = min(coord.y + point_width, dst_rows);

    // Draw square point
    for(int y = min_y; y < max_y; y++)
    {
        for(int x = min_x; x < max_x; x++)
        {
            int index = y * dst_step + (3 * x) + dst_offset;
            vstore3(point_colour.xyz, 0, dst + index);
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------

// )"
