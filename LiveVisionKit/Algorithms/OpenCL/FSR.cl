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

//----------------------------------------------------------------------------------------------------------------------

void gather_four(
    __global uchar* src, int src_step, int src_offset, int2 src_size,
    int2 coord, float4* r, float4* g, float4* b
)
{   
    // Gather4 pixel ordering
    //      +---+---+
    //      | 4 | 3 |
    //      +---+---+
    //      | 1 | 2 |
    //      +---+---+
    //
    // Coord refers to top left of the gather4.

    // Clamp coords to avoid accessing memory that is out of bounds, this
    // can lead to slightly incorrect colouring on the edges but is negligible. 
    coord.x = clamp(coord.x, 0, src_size.x - 2);
    coord.y = clamp(coord.y, 0, src_size.y - 2);

    // Read source pixels in batches and normalize to float type
    int src_index = coord.y * src_step + (3 * coord.x) + src_offset;
    float8 top = convert_float8(vload8(0, src + src_index)) * 0.00392156862f;
    float8 bot = convert_float8(vload8(0, src + src_index + src_step)) * 0.00392156862f;

    *r = (float4)(bot.s0, bot.s3, top.s3, top.s0);
    *g = (float4)(bot.s1, bot.s4, top.s4, top.s1);
    *b = (float4)(bot.s2, bot.s5, top.s5, top.s2);
}

)" R"(
    
//----------------------------------------------------------------------------------------------------------------------

__kernel void easu(
    __global uchar* src, int src_step, int src_offset,
    __global uchar* dst, int dst_step, int dst_offset, 
    int2 src_size, float2 inv_scale, float2 warp_offset 
)
{
    int x = get_global_id(0), y = get_global_id(1);

    // 12-tap kernel.
    //    b c
    //  e f g h
    //  i j k l
    //    n o
    
    // Get position of 'f'.
    float2 pp = (float2)(x, y) * inv_scale.xy + warp_offset; 
    float2 fp = floor(pp);
    pp -= fp;

    // Centers of gather4
    //      +---+---+
    //      |   |   |
    //      +--(0)--+
    //      | b | c |
    //  +---+---+---+---+
    //  | e | f | g | h |
    //  +--(1)--+--(2)--+
    //  | i | j | k | l |
    //  +---+---+---+---+
    //      | n | o |
    //      +--(3)--+
    //      |   |   |
    //      +---+---+

    // NOTE: these coords refer to top left pixel of gather4s
    int2 p0 = convert_int2(fp) + (int2)(0,-1);
    int2 p1 = p0 + (int2)(-1, 2);
    int2 p2 = p0 + (int2)( 1, 2);
    int2 p3 = p0 + (int2)( 0, 4);

    float4 bczzR, bczzG, bczzB;
    gather_four(src, src_step, src_offset, src_size, p0, &bczzR, &bczzG, &bczzB);

    float4 ijfeR, ijfeG, ijfeB;
    gather_four(src, src_step, src_offset, src_size, p1, &ijfeR, &ijfeG, &ijfeB);

    float4 klhgR, klhgG, klhgB;
    gather_four(src, src_step, src_offset, src_size, p2, &klhgR, &klhgG, &klhgB);

    float4 zzonR, zzonG, zzonB;
    gather_four(src, src_step, src_offset, src_size, p3, &zzonR, &zzonG, &zzonB);

    // Simplest multi-channel approximate luma possible (luma times 2, in 2 FMA/MAD).
    float4 bczzL = bczzB * (float4)(0.5) + (bczzR * (float4)(0.5) + bczzG);
    float4 ijfeL = ijfeB * (float4)(0.5) + (ijfeR * (float4)(0.5) + ijfeG);
    float4 klhgL = klhgB * (float4)(0.5) + (klhgR * (float4)(0.5) + klhgG);
    float4 zzonL = zzonB * (float4)(0.5) + (zzonR * (float4)(0.5) + zzonG);

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
    float3 aC =(float3)(0.0f);
    float aW = 0.0f;

    easu_tap(&aC,&aW,(float2)( 0.0,-1.0)-pp,dir,len2,lob,clp,(float3)(bczzR.x,bczzG.x,bczzB.x)); // b
    easu_tap(&aC,&aW,(float2)( 1.0,-1.0)-pp,dir,len2,lob,clp,(float3)(bczzR.y,bczzG.y,bczzB.y)); // c
    easu_tap(&aC,&aW,(float2)(-1.0, 1.0)-pp,dir,len2,lob,clp,(float3)(ijfeR.x,ijfeG.x,ijfeB.x)); // i
    easu_tap(&aC,&aW,(float2)( 0.0, 1.0)-pp,dir,len2,lob,clp,(float3)(ijfeR.y,ijfeG.y,ijfeB.y)); // j
    easu_tap(&aC,&aW,(float2)( 0.0, 0.0)-pp,dir,len2,lob,clp,(float3)(ijfeR.z,ijfeG.z,ijfeB.z)); // f
    easu_tap(&aC,&aW,(float2)(-1.0, 0.0)-pp,dir,len2,lob,clp,(float3)(ijfeR.w,ijfeG.w,ijfeB.w)); // e
    easu_tap(&aC,&aW,(float2)( 1.0, 1.0)-pp,dir,len2,lob,clp,(float3)(klhgR.x,klhgG.x,klhgB.x)); // k
    easu_tap(&aC,&aW,(float2)( 2.0, 1.0)-pp,dir,len2,lob,clp,(float3)(klhgR.y,klhgG.y,klhgB.y)); // l
    easu_tap(&aC,&aW,(float2)( 2.0, 0.0)-pp,dir,len2,lob,clp,(float3)(klhgR.z,klhgG.z,klhgB.z)); // h
    easu_tap(&aC,&aW,(float2)( 1.0, 0.0)-pp,dir,len2,lob,clp,(float3)(klhgR.w,klhgG.w,klhgB.w)); // g
    easu_tap(&aC,&aW,(float2)( 0.0, 2.0)-pp,dir,len2,lob,clp,(float3)(zzonR.w,zzonG.w,zzonB.w)); // n
    easu_tap(&aC,&aW,(float2)( 1.0, 2.0)-pp,dir,len2,lob,clp,(float3)(zzonR.z,zzonG.z,zzonB.z)); // o

    // Normalize and dering.
    float3 fpx = min(ma4, max(mi4, aC* (float3)(native_recip(aW))));
    uchar3 px = convert_uchar3(fpx * 255.0f);

    // Write pixel.
    int dst_index = y * dst_step + (3 * x) + dst_offset;
    vstore3(px, 0, dst + dst_index);
}

// )" R"(


__kernel void easu_warp(
    __global uchar* src, int src_step, int src_offset,
    __global uchar* dst, int dst_step, int dst_offset,
    __global uchar* map, int map_step, int map_offset,
    int2 src_size, float2 inv_scale 
)
{
    int x = get_global_id(0), y = get_global_id(1);

    int map_index = y * map_step + (8 * x) + 8 * map_offset;
    float2 warp_offset = as_float2(vload8(0, map + map_index));

    easu(src, src_step, src_offset, dst, dst_step, dst_offset, src_size, inv_scale, warp_offset);
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

)"
