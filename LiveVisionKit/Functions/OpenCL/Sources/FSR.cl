R"(
// =====================================================================================================================
// AMD LICENSE
// =====================================================================================================================
// Copyright (c) 2021 Advanced Micro Devices, Inc. All rights reserved.
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// =====================================================================================================================
// MIT LICENSE
// =====================================================================================================================
// Copyright (c) 2014 Michal Drobot (for concepts used in "FLOAT APPROXIMATIONS").
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and
// to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
// OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// =====================================================================================================================
//                                              FLOAT APPROXIMATIONS
// =====================================================================================================================
// Michal Drobot has an excellent presentation on these: "Low Level Optimizations For GCN",
//  - Idea dates back to SGI, then to Quake 3, etc.
//  - https://michaldrobot.files.wordpress.com/2014/05/gcn_alu_opt_digitaldragons2014.pdf
//     - sqrt(x)=rsqrt(x)*x
//     - rcp(x)=rsqrt(x)*rsqrt(x) for positive x
//  - https://github.com/michaldrobot/ShaderFastLibs/blob/master/ShaderFastMathLib.h
//----------------------------------------------------------------------------------------------------------------------
// These below are from perhaps less complete searching for optimal.
// Used FP16 normal range for testing with +4096 32-bit step size for sampling error.
// So these match up well with the half approximations.
//======================================================================================================================

float APrxLoSqrtF1(float a){return as_float((as_uint(a) >> 1) + (uint)(0x1fbc4639));} 
float2 APrxLoSqrtF2(float2 a){return as_float2((as_uint2(a) >> 1) + (uint2)(0x1fbc4639));} 
float3 APrxLoSqrtF3(float3 a){return as_float3((as_uint3(a) >> 1) + (uint3)(0x1fbc4639));} 
float4 APrxLoSqrtF4(float4 a){return as_float4((as_uint4(a) >> 1) + (uint4)(0x1fbc4639));} 

float APrxLoRsqF1(float a){return as_float((uint)(0x5f347d74) - (as_uint(a) >> 1));}
float2 APrxLoRsqF2(float2 a){return as_float2((uint2)(0x5f347d74) - (as_uint2(a) >> 1));}
float3 APrxLoRsqF3(float3 a){return as_float3((uint3)(0x5f347d74) - (as_uint3(a) >> 1));}
float4 APrxLoRsqF4(float4 a){return as_float4((uint4)(0x5f347d74) - (as_uint4(a) >> 1));}

float APrxLoRcpF1(float a){return as_float((uint)(0x7ef07ebb) - as_uint(a));}
float2 APrxLoRcpF2(float2 a){return as_float2((uint2)(0x7ef07ebb) - as_uint2(a));}
float3 APrxLoRcpF3(float3 a){return as_float3((uint3)(0x7ef07ebb) - as_uint3(a));}
float4 APrxLoRcpF4(float4 a){return as_float4((uint4)(0x7ef07ebb) - as_uint4(a));}

float APrxMedRcpF1(float a) {float b = as_float((uint)(0x7ef19fff) - as_uint(a));return b*(-b*a+2.0f);}
float2 APrxMedRcpF2(float2 a){float2 b = as_float2((uint2)(0x7ef19fff) - as_uint2(a)); return b*(-b*a+(float2)(2.0f));}
float3 APrxMedRcpF3(float3 a){float3 b = as_float3((uint3)(0x7ef19fff) - as_uint3(a)); return b*(-b*a+(float3)(2.0f));}
float4 APrxMedRcpF4(float4 a){float4 b = as_float4((uint4)(0x7ef19fff) - as_uint4(a)); return b*(-b*a+(float4)(2.0f));}

//======================================================================================================================
//                                                    HELPER FUNCTIONS
//======================================================================================================================

float saturate(float x){return fmax(0.0f, fmin(1.0f, x));}

float3 max4(float3 a, float3 b, float3 c, float3 d){return max(a, max(b, max(c, d)));}
float3 min4(float3 a, float3 b, float3 c, float3 d){return min(a, min(b, min(c, d)));}

float max4f(float a, float b, float c, float d){return max(a, max(b, max(c, d)));}
float min4f(float a, float b, float c, float d){return min(a, min(b, min(c, d)));}

uint ABfe(uint src, uint off, uint bits){uint mask = (1u << bits) - 1; return (src >> off) & mask;}
uint ABfiM(uint src, uint ins, uint bits){uint mask = (1u << bits) - 1; return (ins & mask) | (src & (~mask));}

int2 remap8x8(uint id){return convert_int2((uint2)(ABfe(id, 1u, 3u), ABfiM(ABfe(id, 3u, 3u), id, 1u)));}
int2 remapRed8x8(uint id){return convert_int2((uint2)(ABfiM(ABfe(id,2u,3u),id,1u),ABfiM(ABfe(id,3u,3u),ABfe(id,1u,2u),2u)));}

//======================================================================================================================
//                                    FSR - [EASU] EDGE ADAPTIVE SPATIAL UPSAMPLING
//======================================================================================================================

// Filtering for a given tap for the scalar.
void easu_tap(float3* aC, float* aW, float2 off, float2 dir, float2 len, float lob, float clp, float3 c)
{ 
    // Rotate offset by direction.
    float2 v;
    v.x = (off.x * ( dir.x)) + (off.y * dir.y);
    v.y = (off.x * (-dir.y)) + (off.y * dir.x);
    v *= len; // Anisotropy

    // Compute distance^2.
    // Limit to the window as at corner, 2 taps can easily be outside.
    float d2 = min(v.x * v.x + v.y * v.y, clp);

    // Approximation of lancos2 without sin() or rcp(), or sqrt() to get x.
    //  (25/16 * (2/5 * x^2 - 1)^2 - (25/16 - 1)) * (1/4 * x^2 - 1)^2
    //  |_______________________________________|   |_______________|
    //                   base                             window
    // The general form of the 'base' is, (a*(b*x^2-1)^2-(a-1))
    // Where 'a=1/(2*b-b^2)' and 'b' moves around the negative lobe.
    float wA = lob * d2 - 1.0f;
    float wB = (2.0f/5.0f) * d2 - 1.0f;

    wA *= wA;
    wB = (25.0f/16.0f) * (wB * wB) - (25.0f/16.0f - 1.0f);

    // Do weighted average.
    float w = wB * wA;
    *aC += c * w;
    *aW += w;
}

//----------------------------------------------------------------------------------------------------------------------

// Accumulate direction and length.
void easu_accumulate(
    float2* dir, float* len, float2 pp,
    bool biS, bool biT, bool biU, bool biV,
    float lA, float lB, float lC, float lD, float lE
)
{
    // Compute bilinear weight, branches factor out as predicates are compiler time immediates.
    //  s t
    //  u v

    float w = 0.0f;
    if(biV) w = pp.x  * pp.y;
    if(biU) w = (1.0f - pp.x) * pp.y;
    if(biT) w = pp.x  * (1.0f - pp.y);
    if(biS) w = (1.0f - pp.x) * (1.0f - pp.y);

    // Direction is the '+' diff.
    //    a
    //  b c d
    //    e
    // Then takes magnitude from abs average of both sides of 'c'.
    // Length converts gradient reversal to 0, smoothly to non-reversal at 1, shaped, then adding horz and vert terms.

    float dc = lD - lC;
    float cb = lC - lB;
    float lenX = APrxLoRcpF1(max(fabs(dc), fabs(cb)));

    float dirX = lD - lB;
    dir->x += dirX * w;

    lenX = saturate(fabs(dirX) * lenX);
    lenX *= lenX;
    *len += lenX * w;

    // Repeat for the y axis.
    float ec = lE - lC;
    float ca = lC - lA;
    float lenY = APrxLoRcpF1(max(fabs(ec), fabs(ca)));
    
    float dirY = lE - lA;
    dir->y += dirY * w; 

    lenY = saturate(fabs(dirY) * lenY);
    lenY *= lenY;
    *len += lenY * w;
}

// )" R"(
//----------------------------------------------------------------------------------------------------------------------

void easu(__global uchar* src, int src_step, int src_offset, int2 src_coord, float2 sub_pixel, uchar3* dst_pixel)
{

    // Required pixel load ops, given that we are processing the current point 'f'.
    //      +---+---+
    //      | b | c |       | R0 (6  bytes)
    //  +---+---+---+---+   |
    //  | e | f | g | h |   | R1 (12 bytes)
    //  +---+---+---+---+   |
    //  | i | j | k | l |   | R2 (12 bytes)
    //  +---+---+---+---+   |
    //      | n | o |       | R3 (6  bytes)
    //      +---+---+   
    //

    int r0_index = (src_coord.y - 1) * src_step + (3 * src_coord.x) + src_offset;
    int r1_index = r0_index + src_step - 3;

    uchar8 r0_data  =  vload8(0, src + r0_index); 
    uchar16 r1_data = vload16(0, src + r1_index); 
    uchar16 r2_data = vload16(0, src + r1_index + src_step); 
    uchar8 r3_data  =  vload8(0, src + r0_index + 3 * src_step); 

    // Convert from uchars to float representation
    const float norm_factor = 0.00392156862f;

    float8 r0_px  =  convert_float8(r0_data) * norm_factor;
    float16 r1_px = convert_float16(r1_data) * norm_factor;
    float16 r2_px = convert_float16(r2_data) * norm_factor;
    float8 r3_px  =  convert_float8(r3_data) * norm_factor;

    // NOTE: BGR format is used instead of RGB
    float4 bczzR = (float4)(r0_px.s0, r0_px.s3, 0, 0);
    float4 bczzG = (float4)(r0_px.s1, r0_px.s4, 0, 0);
    float4 bczzB = (float4)(r0_px.s2, r0_px.s5, 0, 0);

    float4 ijfeR = (float4)(r2_px.s0, r2_px.s3, r1_px.s3, r1_px.s0);
    float4 ijfeG = (float4)(r2_px.s1, r2_px.s4, r1_px.s4, r1_px.s1);
    float4 ijfeB = (float4)(r2_px.s2, r2_px.s5, r1_px.s5, r1_px.s2);

    float4 klhgR = (float4)(r2_px.s6, r2_px.s9, r1_px.s9, r1_px.s6);
    float4 klhgG = (float4)(r2_px.s7, r2_px.sA, r1_px.sA, r1_px.s7);
    float4 klhgB = (float4)(r2_px.s8, r2_px.sB, r1_px.sB, r1_px.s8);

    float4 zzonR = (float4)(0, 0, r3_px.s3, r3_px.s0);
    float4 zzonG = (float4)(0, 0, r3_px.s4, r3_px.s1);
    float4 zzonB = (float4)(0, 0, r3_px.s5, r3_px.s2);

#ifndef YUV_INPUT
    // In YUV the first channel already represents luma
    float4 bczzL = bczzR;
    float4 ijfeL = ijfeR;
    float4 klhgL = klhgR;
    float4 zzonL = zzonR;
#else
    // Simplest multi-channel approximate luma possible (luma times 2, in 2 FMA/MAD).
    float4 bczzL = bczzB * (float4)(0.5) + (bczzR * (float4)(0.5) + bczzG);
    float4 ijfeL = ijfeB * (float4)(0.5) + (ijfeR * (float4)(0.5) + ijfeG);
    float4 klhgL = klhgB * (float4)(0.5) + (klhgR * (float4)(0.5) + klhgG);
    float4 zzonL = zzonB * (float4)(0.5) + (zzonR * (float4)(0.5) + zzonG);
#endif

    // Accumulate for bilinear interpolation.
    float len = 0.0f;
    float2 dir = (float2)(0.0f);
    easu_accumulate(&dir, &len, sub_pixel, true , false, false, false, bczzL.x, ijfeL.w, ijfeL.z, klhgL.w, ijfeL.y);
    easu_accumulate(&dir, &len, sub_pixel, false, true , false, false, bczzL.y, ijfeL.z, klhgL.w, klhgL.z, klhgL.x);
    easu_accumulate(&dir, &len, sub_pixel, false, false, true , false, ijfeL.z, ijfeL.x, ijfeL.y, klhgL.x, zzonL.w);
    easu_accumulate(&dir, &len, sub_pixel, false, false, false, true , klhgL.w, ijfeL.y, klhgL.x, klhgL.y, zzonL.z);

    // Normalize with approximation, and cleanup close to zero.
    float2 dir2 = dir * dir;
    float dirR = dir2.x + dir2.y;
    bool zro = dirR < (1.0f/32768.0f);
    dirR = APrxLoRsqF1(dirR);
    dirR = zro ? 1.0f : dirR;
    dir.x = zro ? 1.0f : dir.x;
    dir *= (float2)(dirR);

    // Transform from {0 to 2} to {0 to 1} range, and shape with square.
    len = len * 0.5f;
    len *= len;

    // Stretch kernel {1.0 vert|horz, to sqrt(2.0) on diagonal}.
    float stretch = (dir.x * dir.x + dir.y * dir.y) * APrxLoRcpF1(max(fabs(dir.x),fabs(dir.y)));

    // Anisotropic length after rotation,
    //  x := 1.0 lerp to 'stretch' on edges
    //  y := 1.0 lerp to 2x on edges
    float2 len2 = (float2)(1.0f + (stretch - 1.0f) * len, 1.0f + -0.5f * len);

    // Based on the amount of 'edge',
    // the window shifts from +/-{sqrt(2.0) to slightly beyond 2.0}.
    float lob = 0.5f + ((1.0f / 4.0f - 0.04f) - 0.5f) * len;
    
    // Set distance^2 clipping point to the end of the adjustable window.
    float clp = APrxLoRcpF1(lob);

    // Accumulation mixed with min/max of 4 nearest.
    //    b c
    //  e f g h
    //  i j k l
    //    n o
    float3 mi4 = min4(
        (float3)(ijfeR.z,ijfeG.z,ijfeB.z), 
        (float3)(klhgR.w,klhgG.w,klhgB.w), 
        (float3)(ijfeR.y,ijfeG.y,ijfeB.y),
        (float3)(klhgR.x,klhgG.x,klhgB.x)
    );

    float3 ma4 = max4(
        (float3)(ijfeR.z,ijfeG.z,ijfeB.z), 
        (float3)(klhgR.w,klhgG.w,klhgB.w), 
        (float3)(ijfeR.y,ijfeG.y,ijfeB.y),
        (float3)(klhgR.x,klhgG.x,klhgB.x)
    );

    // Accumulation.
    float3 aC = (float3)(0.0f);
    float aW = 0.0f;

    easu_tap(&aC, &aW, (float2)( 0.0,-1.0) - sub_pixel, dir, len2, lob, clp, (float3)(bczzR.x,bczzG.x,bczzB.x)); // b
    easu_tap(&aC, &aW, (float2)( 1.0,-1.0) - sub_pixel, dir, len2, lob, clp, (float3)(bczzR.y,bczzG.y,bczzB.y)); // c
    easu_tap(&aC, &aW, (float2)(-1.0, 1.0) - sub_pixel, dir, len2, lob, clp, (float3)(ijfeR.x,ijfeG.x,ijfeB.x)); // i
    easu_tap(&aC, &aW, (float2)( 0.0, 1.0) - sub_pixel, dir, len2, lob, clp, (float3)(ijfeR.y,ijfeG.y,ijfeB.y)); // j
    easu_tap(&aC, &aW, (float2)( 0.0, 0.0) - sub_pixel, dir, len2, lob, clp, (float3)(ijfeR.z,ijfeG.z,ijfeB.z)); // f
    easu_tap(&aC, &aW, (float2)(-1.0, 0.0) - sub_pixel, dir, len2, lob, clp, (float3)(ijfeR.w,ijfeG.w,ijfeB.w)); // e
    easu_tap(&aC, &aW, (float2)( 1.0, 1.0) - sub_pixel, dir, len2, lob, clp, (float3)(klhgR.x,klhgG.x,klhgB.x)); // k
    easu_tap(&aC, &aW, (float2)( 2.0, 1.0) - sub_pixel, dir, len2, lob, clp, (float3)(klhgR.y,klhgG.y,klhgB.y)); // l
    easu_tap(&aC, &aW, (float2)( 2.0, 0.0) - sub_pixel, dir, len2, lob, clp, (float3)(klhgR.z,klhgG.z,klhgB.z)); // h
    easu_tap(&aC, &aW, (float2)( 1.0, 0.0) - sub_pixel, dir, len2, lob, clp, (float3)(klhgR.w,klhgG.w,klhgB.w)); // g
    easu_tap(&aC, &aW, (float2)( 0.0, 2.0) - sub_pixel, dir, len2, lob, clp, (float3)(zzonR.w,zzonG.w,zzonB.w)); // n
    easu_tap(&aC, &aW, (float2)( 1.0, 2.0) - sub_pixel, dir, len2, lob, clp, (float3)(zzonR.z,zzonG.z,zzonB.z)); // o

    // Normalize and dering.
    float3 fpx = min(ma4, max(mi4, aC * (float3)(native_recip(aW))));
    *dst_pixel = convert_uchar3(fpx * 255.0f);
}

// )" R"(
//----------------------------------------------------------------------------------------------------------------------


__kernel void easu_scale(
    __global uchar* src, int src_step, int src_offset, int src_rows, int src_cols,
    __global uchar* dst, int dst_step, int dst_offset, int dst_rows, int dst_cols,
    float2 rscale // Inverse scaling (from the point of view of the dst)
)
{
    // Swizzle the threads for potentially better cache use.
    int id = get_local_id(1) * 8 + get_local_id(0);
    
    int2 dst_coord = remapRed8x8(id) + (int2)(get_group_id(0) << 3, get_group_id(1) << 3); 
    float2 sub_pixel = convert_float2(dst_coord) * rscale;
    int2 src_coord = convert_int2_rtz(sub_pixel);
    sub_pixel -= floor(sub_pixel);

    // Nest the border conditions together to help load balance and minimize branches.
    if(src_coord.x == 0 || src_coord.y == 0 || src_coord.x >= src_cols - 4 || src_coord.y >= src_rows - 4 || dst_coord.x >= dst_cols || dst_coord.y >= dst_rows)
    {
        // If we are out of the src bounds (but inside dst bounds), scale by nearest neighbour. 
        if(dst_coord.x < dst_cols && dst_coord.y < dst_rows)
        {
            int src_index = src_coord.y * src_step + (3 * src_coord.x) + src_offset;
            int dst_index = dst_coord.y * dst_step + (3 * dst_coord.x) + dst_offset;
            vstore3(vload3(0, src + src_index), 0, dst + dst_index);
        }
        return;
    }

    // Run EASU
    uchar3 dst_pixel;
    easu(src, src_step, src_offset, src_coord, sub_pixel, &dst_pixel);

    // Write pixel.
    int dst_index = dst_coord.y * dst_step + (3 * dst_coord.x) + dst_offset;
    vstore3(dst_pixel, 0, dst + dst_index);
}

//----------------------------------------------------------------------------------------------------------------------

__kernel void easu_remap(
    __global uchar* src, int src_step, int src_offset, int src_rows, int src_cols,
    __global uchar* dst, int dst_step, int dst_offset, int4 dst_bounds,
    __global uchar* map, int map_step, int map_offset, uchar4 background_colour
)
{
    // Swizzle the threads for potentially better cache use.
    int id = get_local_id(1) * 8 + get_local_id(0);
    int2 dst_coord = remapRed8x8(id) + (int2)(get_group_id(0) << 3, get_group_id(1) << 3); 

    // Exit early if out of bounds (for uneven output sizes)
    if(dst_coord.x >= dst_bounds.z || dst_coord.y >= dst_bounds.w)
        return;

    // Load remapping offset
    int map_index = dst_coord.y * map_step + (8 * dst_coord.x) +  map_offset;
    float2 offset = as_float2(vload8(0, map + map_index));

    // Remap the src coord
    float2 sub_pixel = convert_float2(dst_coord + dst_bounds.xy) + offset;
    int2 src_coord = convert_int2_rtz(sub_pixel);
    sub_pixel -= floor(sub_pixel);

    // Nest the border conditions on the src to help load balance and minimize branches.
    uchar3 dst_pixel = background_colour.xyz;
    if(src_coord.x < 1 || src_coord.y < 1 || src_coord.x >= src_cols - 4 || src_coord.y >= src_rows - 4)
    {
        // If we are still within the overall src bounds use nearest neighbour. 
        if(src_coord.x >= 0 && src_coord.x < src_cols && src_coord.y >= 0 && src_coord.y < src_rows)
        {
            int src_index = src_coord.y * src_step + (3 * src_coord.x) + src_offset;
            int dst_index = dst_coord.y * dst_step + (3 * dst_coord.x) + dst_offset;
            vstore3(vload3(0, src + src_index), 0, dst + dst_index);
            return;
        }
    }
    else easu(src, src_step, src_offset, src_coord, sub_pixel, &dst_pixel);

    // Write pixel.
    int dst_index = dst_coord.y * dst_step + (3 * dst_coord.x) + dst_offset;
    vstore3(dst_pixel, 0, dst + dst_index);
}

//----------------------------------------------------------------------------------------------------------------------

__kernel void easu_remap_homography(
    __global uchar* src, int src_step, int src_offset, int src_rows, int src_cols,
    __global uchar* dst, int dst_step, int dst_offset, int4 dst_bounds,
    float4 r1, float4 r2, float4 r3, uchar4 background_colour
)
{
    // Swizzle the threads for potentially better cache use.
    int id = get_local_id(1) * 8 + get_local_id(0);
    int2 dst_coord = remapRed8x8(id) + (int2)(get_group_id(0) << 3, get_group_id(1) << 3); 

    // Exit early if out of bounds (for uneven output sizes)
    if(dst_coord.x >= dst_bounds.z || dst_coord.y >= dst_bounds.w)
        return;

    // Calculate remapping offset
    float2 fcoord = convert_float2(dst_coord);
    float dz = 1.0f / (r3.x * fcoord.x + r3.y * fcoord.y + r3.z);
    float2 offset = (float2)(
        (r1.x * fcoord.x + r1.y * fcoord.y + r1.z) * dz,
        (r2.x * fcoord.x + r2.y * fcoord.y + r2.z) * dz
    ) - fcoord;

    // Remap the src coord
    float2 sub_pixel = convert_float2(dst_coord + dst_bounds.xy) + offset;
    int2 src_coord = convert_int2_rtz(sub_pixel);
    sub_pixel -= floor(sub_pixel);

    // Nest the border conditions on the src to help load balance and minimize branches.
    uchar3 dst_pixel = background_colour.xyz;
    if(src_coord.x < 1 || src_coord.y < 1 || src_coord.x >= src_cols - 4 || src_coord.y >= src_rows - 4)
    {
        // If we are still within the overall src bounds use nearest neighbour. 
        if(src_coord.x >= 0 && src_coord.x < src_cols && src_coord.y >= 0 && src_coord.y < src_rows)
        {
            int src_index = src_coord.y * src_step + (3 * src_coord.x) + src_offset;
            int dst_index = dst_coord.y * dst_step + (3 * dst_coord.x) + dst_offset;
            vstore3(vload3(0, src + src_index), 0, dst + dst_index);
            return;
        }
    }
    else easu(src, src_step, src_offset, src_coord, sub_pixel, &dst_pixel);

    // Write pixel.
    int dst_index = dst_coord.y * dst_step + (3 * dst_coord.x) + dst_offset;
    vstore3(dst_pixel, 0, dst + dst_index);
}


// )" R"(
//==============================================================================================================================
//                                      FSR - [RCAS] ROBUST CONTRAST ADAPTIVE SHARPENING
//==============================================================================================================================

__kernel void rcas(
    __global uchar* src, int src_step, int src_offset, int src_rows, int src_cols,
    __global uchar* dst, int dst_step, int dst_offset, float sharpness
)
{ 
    // Algorithm uses minimal 3x3 pixel neighborhood.
    //    b 
    //  d e f
    //    h

    // Swizzle the threads for potentially better cache use.
    int id = get_local_id(1) * 8 + get_local_id(0);
    int2 coord = remapRed8x8(id) + (int2)(get_group_id(0) << 3, get_group_id(1) << 3); 

    int src_index = coord.y * src_step + (3 * coord.x) + src_offset;
    int dst_index = coord.y * dst_step + (3 * coord.x) + dst_offset;

    // Nest the border conditions together to help load balance and minimize branches.
    if(coord.x == 0 || coord.x >= src_cols - 1 || coord.y == 0 || coord.y >= src_rows - 1)
    {
        // Perform direct copy if we are on the border of the image. 
        if(coord.x <= src_cols || coord.y <= src_rows)
            vstore3(vload3(0, src + src_index), 0, dst + dst_index);
        return;
    }

    // Do not run sharpening on edges to avoid going out of bounds
    const float norm_factor = 0.00392156862f;

    uchar3 r0 = vload3(0, src + src_index - src_step);
    uchar8 r1 = vload8(0, src + src_index - 3);
    uchar3 r2 = vload3(0, src + src_index + src_step);

    float3 b = convert_float3(r0) * norm_factor;
    float3 h = convert_float3(r2) * norm_factor;
    float3 d = convert_float3((uchar3)(r1.s0, r1.s1, r1.s2)) * norm_factor;
    float3 e = convert_float3((uchar3)(r1.s3, r1.s4, r1.s5)) * norm_factor;
    float3 f = convert_float3((uchar3)(r1.s6, r1.s7, src[src_index + 5])) * norm_factor;
    
    // Rename 
    float bR=b.z; float bG=b.y; float bB=b.x;
    float dR=d.z; float dG=d.y; float dB=d.x;
    float eR=e.z; float eG=e.y; float eB=e.x;
    float fR=f.z; float fG=f.y; float fB=f.x;
    float hR=h.z; float hG=h.y; float hB=h.x;

    // Min and max of ring.
    float mn4R = min4f(bR,dR,fR,hR);
    float mn4G = min4f(bG,dG,fG,hG);
    float mn4B = min4f(bB,dB,fB,hB);
    float mx4R = max4f(bR,dR,fR,hR);
    float mx4G = max4f(bG,dG,fG,hG);
    float mx4B = max4f(bB,dB,fB,hB);

    // Immediate constants for peak range.
    float2 peakC = (float2)(1.0, -4.0);

    // Limiters, these need to be high precision RCPs.
    float hitMinR = min(mn4R, eR) * native_recip(4.0f * mx4R);
    float hitMinG = min(mn4G, eG) * native_recip(4.0f * mx4G);
    float hitMinB = min(mn4B, eB) * native_recip(4.0f * mx4B);
    float hitMaxR = (peakC.x - max(mx4R,eR)) * native_recip(4.0f * mn4R + peakC.y);
    float hitMaxG = (peakC.x - max(mx4G,eG)) * native_recip(4.0f * mn4G + peakC.y);
    float hitMaxB = (peakC.x - max(mx4B,eB)) * native_recip(4.0f * mn4B + peakC.y);
    float lobeR = max(-hitMinR, hitMaxR);
    float lobeG = max(-hitMinG, hitMaxG);
    float lobeB = max(-hitMinB, hitMaxB);
    float lobe = clamp(max(lobeR,max(lobeG,lobeB)), -0.1875f, 0.0f) * sharpness;

    // Resolve, which needs the medium precision rcp approximation to avoid visible tonality changes.
    float rcpL = APrxMedRcpF1(4.0f * lobe + 1.0f);
    float3 fpx = (((b + d + h + f) * lobe) + e) * rcpL;

    uchar3 dst_pixel = convert_uchar3(fpx * 255.0f);
    vstore3(dst_pixel, 0, dst + dst_index);
} 

// )"
