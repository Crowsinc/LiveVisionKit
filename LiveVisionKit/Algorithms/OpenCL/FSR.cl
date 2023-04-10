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

void easu(__global uchar* src, int src_step, int src_offset, float2 src_coord, uchar3* dst_pixel)
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

    float2 pp = src_coord - floor(src_coord);

    int2 pf = convert_int2_rtz(src_coord);
    int r0_index = (pf.y - 1) * src_step + (3 * pf.x) + src_offset;
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
    easu_accumulate(&dir, &len, pp, true , false, false, false, bczzL.x, ijfeL.w, ijfeL.z, klhgL.w, ijfeL.y);
    easu_accumulate(&dir, &len, pp, false, true , false, false, bczzL.y, ijfeL.z, klhgL.w, klhgL.z, klhgL.x);
    easu_accumulate(&dir, &len, pp, false, false, true , false, ijfeL.z, ijfeL.x, ijfeL.y, klhgL.x, zzonL.w);
    easu_accumulate(&dir, &len, pp, false, false, false, true , klhgL.w, ijfeL.y, klhgL.x, klhgL.y, zzonL.z);

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

    easu_tap(&aC, &aW, (float2)( 0.0,-1.0) - pp, dir, len2, lob, clp, (float3)(bczzR.x,bczzG.x,bczzB.x)); // b
    easu_tap(&aC, &aW, (float2)( 1.0,-1.0) - pp, dir, len2, lob, clp, (float3)(bczzR.y,bczzG.y,bczzB.y)); // c
    easu_tap(&aC, &aW, (float2)(-1.0, 1.0) - pp, dir, len2, lob, clp, (float3)(ijfeR.x,ijfeG.x,ijfeB.x)); // i
    easu_tap(&aC, &aW, (float2)( 0.0, 1.0) - pp, dir, len2, lob, clp, (float3)(ijfeR.y,ijfeG.y,ijfeB.y)); // j
    easu_tap(&aC, &aW, (float2)( 0.0, 0.0) - pp, dir, len2, lob, clp, (float3)(ijfeR.z,ijfeG.z,ijfeB.z)); // f
    easu_tap(&aC, &aW, (float2)(-1.0, 0.0) - pp, dir, len2, lob, clp, (float3)(ijfeR.w,ijfeG.w,ijfeB.w)); // e
    easu_tap(&aC, &aW, (float2)( 1.0, 1.0) - pp, dir, len2, lob, clp, (float3)(klhgR.x,klhgG.x,klhgB.x)); // k
    easu_tap(&aC, &aW, (float2)( 2.0, 1.0) - pp, dir, len2, lob, clp, (float3)(klhgR.y,klhgG.y,klhgB.y)); // l
    easu_tap(&aC, &aW, (float2)( 2.0, 0.0) - pp, dir, len2, lob, clp, (float3)(klhgR.z,klhgG.z,klhgB.z)); // h
    easu_tap(&aC, &aW, (float2)( 1.0, 0.0) - pp, dir, len2, lob, clp, (float3)(klhgR.w,klhgG.w,klhgB.w)); // g
    easu_tap(&aC, &aW, (float2)( 0.0, 2.0) - pp, dir, len2, lob, clp, (float3)(zzonR.w,zzonG.w,zzonB.w)); // n
    easu_tap(&aC, &aW, (float2)( 1.0, 2.0) - pp, dir, len2, lob, clp, (float3)(zzonR.z,zzonG.z,zzonB.z)); // o

    // Normalize and dering.
    float3 fpx = min(ma4, max(mi4, aC * (float3)(native_recip(aW))));
    *dst_pixel = convert_uchar3(fpx * 255.0f);
}

// )" R"(
//----------------------------------------------------------------------------------------------------------------------


__kernel void easu_scale(
    __global uchar* src, int src_step, int src_offset, float2 rscale,
    __global uchar* dst, int dst_step, int dst_offset, int2 bounds
)
{
    // Swizzle the threads for potentially better cache use.
    int id = get_local_id(1) * 8 + get_local_id(0);
    int2 dst_coord = remapRed8x8(id) + (int2)(get_group_id(0) << 3, get_group_id(1) << 3); 

    // Exit early if out of bounds (for uneven output sizes)
    if(dst_coord.x >= bounds.x || dst_coord.y >= bounds.y)
        return;

    float2 src_coord = convert_float2(dst_coord) * rscale;

    // Run EASU
    uchar3 dst_pixel;
    easu(src, src_step, src_offset, src_coord, &dst_pixel);

    // Write pixel.
    int dst_index = dst_coord.y * dst_step + (3 * dst_coord.x) + dst_offset;
    vstore3(dst_pixel, 0, dst + dst_index);
}

__kernel void easu_remap(
    __global uchar* src, int src_step, int src_offset, int4 src_bounds, float2 rscale,
    __global uchar* dst, int dst_step, int dst_offset, int2 bounds,
    __global uchar* map, int map_step, int map_offset
)
{
    // Swizzle the threads for potentially better cache use.
    int id = get_local_id(1) * 8 + get_local_id(0);
    int2 dst_coord = remapRed8x8(id) + (int2)(get_group_id(0) << 3, get_group_id(1) << 3); 

    // Exit early if out of bounds (for uneven output sizes)
    if(dst_coord.x >= bounds.x || dst_coord.y >= bounds.y)
        return;

    // Load remapping offset
    int map_index = dst_coord.y * map_step + (8 * dst_coord.x) +  map_offset;
    float2 offset = as_float2(vload8(0, map + map_index));

    float2 src_coord = (convert_float2(dst_coord) + offset) * rscale;

    uchar3 dst_pixel = (uchar)(0,0,0);
    if(src_coord.x >= src_bounds.x && src_coord.y >= src_bounds.y && 
       src_coord.x <  src_bounds.z && src_coord.y <  src_bounds.w )
        easu(src, src_step, src_offset, src_coord, &dst_pixel);
        
    // Write pixel.
    int dst_index = dst_coord.y * dst_step + (3 * dst_coord.x) + dst_offset;
    vstore3(dst_pixel, 0, dst + dst_index);
}

// )" R"(
//==============================================================================================================================
//                                      FSR - [RCAS] ROBUST CONTRAST ADAPTIVE SHARPENING
//==============================================================================================================================

// The original version '#define FSR_RCAS_LIMIT (0.25-(1.0/16.0))'
// is not supported, so we manually compute the calculation. 
#define FSR_RCAS_LIMIT 0.1875 

// Input callback prototypes that need to be implemented by calling shader
//AF4 FsrRcasLoadF(ASU2 p);
//void FsrRcasInputF(inout AF1 r, inout AF1 g, inout AF1 b);

//------------------------------------------------------------------------------------------------------------------------------
// void FsrRcasF(
//   out AF1 pixR, // Output values, non-vector so port between RcasFilter() and RcasFilterH() is easy.
//   out AF1 pixG,
//   out AF1 pixB,
//   #ifdef FSR_RCAS_PASSTHROUGH_ALPHA
//   out AF1 pixA,
//   #endif
//   AF2 ip, // Integer pixel position in output.
//   AF4 con) // Constant generated by RcasSetup().
// { 
//   // Algorithm uses minimal 3x3 pixel neighborhood.
//   //    b 
//   //  d e f
//   //    h
//   ASU2 sp=ASU2(ip);
//   AF3 b=FsrRcasLoadF(sp+ASU2( 0,-1)).rgb;
//   AF3 d=FsrRcasLoadF(sp+ASU2(-1, 0)).rgb;
//   #ifdef FSR_RCAS_PASSTHROUGH_ALPHA
//     AF4 ee=FsrRcasLoadF(sp);
//     AF3 e=ee.rgb;pixA=ee.a;
//   #else
//     AF3 e=FsrRcasLoadF(sp).rgb;
//   #endif
//   AF3 f=FsrRcasLoadF(sp+ASU2( 1, 0)).rgb;
//   AF3 h=FsrRcasLoadF(sp+ASU2( 0, 1)).rgb;
//   // Rename (32-bit) or regroup (16-bit).
//   AF1 bR=b.r;
//   AF1 bG=b.g;
//   AF1 bB=b.b;
//   AF1 dR=d.r;
//   AF1 dG=d.g;
//   AF1 dB=d.b;
//   AF1 eR=e.r;
//   AF1 eG=e.g;
//   AF1 eB=e.b;
//   AF1 fR=f.r;
//   AF1 fG=f.g;
//   AF1 fB=f.b;
//   AF1 hR=h.r;
//   AF1 hG=h.g;
//   AF1 hB=h.b;
//   // Run optional input transform.
//   FsrRcasInputF(bR,bG,bB);
//   FsrRcasInputF(dR,dG,dB);
//   FsrRcasInputF(eR,eG,eB);
//   FsrRcasInputF(fR,fG,fB);
//   FsrRcasInputF(hR,hG,hB);
//   // Luma times 2.
//   AF1 bL=bB*AF1_(0.5)+(bR*AF1_(0.5)+bG);
//   AF1 dL=dB*AF1_(0.5)+(dR*AF1_(0.5)+dG);
//   AF1 eL=eB*AF1_(0.5)+(eR*AF1_(0.5)+eG);
//   AF1 fL=fB*AF1_(0.5)+(fR*AF1_(0.5)+fG);
//   AF1 hL=hB*AF1_(0.5)+(hR*AF1_(0.5)+hG);
//   // Noise detection.
//   AF1 nz=AF1_(0.25)*bL+AF1_(0.25)*dL+AF1_(0.25)*fL+AF1_(0.25)*hL-eL;
//   nz=ASatF1(abs(nz)*APrxMedRcpF1(AMax3F1(AMax3F1(bL,dL,eL),fL,hL)-AMin3F1(AMin3F1(bL,dL,eL),fL,hL)));
//   nz=AF1_(-0.5)*nz+AF1_(1.0);
//   // Min and max of ring.
//   AF1 mn4R=min(AMin3F1(bR,dR,fR),hR);
//   AF1 mn4G=min(AMin3F1(bG,dG,fG),hG);
//   AF1 mn4B=min(AMin3F1(bB,dB,fB),hB);
//   AF1 mx4R=max(AMax3F1(bR,dR,fR),hR);
//   AF1 mx4G=max(AMax3F1(bG,dG,fG),hG);
//   AF1 mx4B=max(AMax3F1(bB,dB,fB),hB);
//   // Immediate constants for peak range.
//   AF2 peakC=AF2(1.0,-1.0*4.0);
//   // Limiters, these need to be high precision RCPs.
//   AF1 hitMinR=min(mn4R,eR)*ARcpF1(AF1_(4.0)*mx4R);
//   AF1 hitMinG=min(mn4G,eG)*ARcpF1(AF1_(4.0)*mx4G);
//   AF1 hitMinB=min(mn4B,eB)*ARcpF1(AF1_(4.0)*mx4B);
//   AF1 hitMaxR=(peakC.x-max(mx4R,eR))*ARcpF1(AF1_(4.0)*mn4R+peakC.y);
//   AF1 hitMaxG=(peakC.x-max(mx4G,eG))*ARcpF1(AF1_(4.0)*mn4G+peakC.y);
//   AF1 hitMaxB=(peakC.x-max(mx4B,eB))*ARcpF1(AF1_(4.0)*mn4B+peakC.y);
//   AF1 lobeR=max(-hitMinR,hitMaxR);
//   AF1 lobeG=max(-hitMinG,hitMaxG);
//   AF1 lobeB=max(-hitMinB,hitMaxB);
//   AF1 lobe=max(AF1_(-FSR_RCAS_LIMIT),min(AMax3F1(lobeR,lobeG,lobeB),AF1_(0.0)))*con.x;
//   // Apply noise removal.
//   #ifdef FSR_RCAS_DENOISE
//     lobe*=nz;
//   #endif
//   // Resolve, which needs the medium precision rcp approximation to avoid visible tonality changes.
//   AF1 rcpL=APrxMedRcpF1(AF1_(4.0)*lobe+AF1_(1.0));
//   pixR=(lobe*bR+lobe*dR+lobe*hR+lobe*fR+eR)*rcpL;
//   pixG=(lobe*bG+lobe*dG+lobe*hG+lobe*fG+eG)*rcpL;
//   pixB=(lobe*bB+lobe*dB+lobe*hB+lobe*fB+eB)*rcpL;
//   return;
// } 

// )"
