//==============================================================================================================================
//
//                                               [A] SHADER PORTABILITY 1.20210629 (PRUNED VERSION)
//
//==============================================================================================================================
// ABOUT
// ===========
//
// This is a copy of AMD's FSR provided ffx_a.h file which has been significantly pruned for use by OBS's graphics sub-system.
// Pruning is required as not all functionality used by the original file is supported. 
//
// Please see the original ffx_a.h file for more information. 
//
// AMD LICENSE
// ===========
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
// MIT LICENSE
// ===========
// Copyright (c) 2014 Michal Drobot (for concepts used in "FLOAT APPROXIMATIONS").
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
// files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
// modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// ==============================================================================================================================



//==============================================================================================================================
//
//
//                                                            HLSL
//
//
//==============================================================================================================================
#define AP1 bool
#define AP2 bool2
#define AP3 bool3
#define AP4 bool4
//------------------------------------------------------------------------------------------------------------------------------
#define AF1 float
#define AF2 float2
#define AF3 float3
#define AF4 float4
//------------------------------------------------------------------------------------------------------------------------------
#define AU1 uint
#define AU2 uint2
#define AU3 uint3
#define AU4 uint4
//------------------------------------------------------------------------------------------------------------------------------
#define ASU1 int
#define ASU2 int2
#define ASU3 int3
#define ASU4 int4
//==============================================================================================================================
#define AF1_AU1(x) asfloat(AU1(x))
#define AF2_AU2(x) asfloat(AU2(x))
#define AF3_AU3(x) asfloat(AU3(x))
#define AF4_AU4(x) asfloat(AU4(x))
//------------------------------------------------------------------------------------------------------------------------------
#define AU1_AF1(x) asuint(AF1(x))
#define AU2_AF2(x) asuint(AF2(x))
#define AU3_AF3(x) asuint(AF3(x))
#define AU4_AF4(x) asuint(AF4(x))
//------------------------------------------------------------------------------------------------------------------------------
AU1 AU1_AH1_AF1_x(AF1 a){return f32tof16(a);}
#define AU1_AH1_AF1(a) AU1_AH1_AF1_x(AF1(a))
//------------------------------------------------------------------------------------------------------------------------------
AU1 AU1_AH2_AF2_x(AF2 a){return f32tof16(a.x)|(f32tof16(a.y)<<16);}
#define AU1_AH2_AF2(a) AU1_AH2_AF2_x(AF2(a)) 
#define AU1_AB4Unorm_AF4(x) D3DCOLORtoUBYTE4(AF4(x))
//------------------------------------------------------------------------------------------------------------------------------
AF2 AF2_AH2_AU1_x(AU1 x){return AF2(f16tof32(x&0xFFFF),f16tof32(x>>16));}
#define AF2_AH2_AU1(x) AF2_AH2_AU1_x(AU1(x))
//==============================================================================================================================
AF1 AF1_x(AF1 a){return AF1(a);}
AF2 AF2_x(AF1 a){return AF2(a,a);}
AF3 AF3_x(AF1 a){return AF3(a,a,a);}
AF4 AF4_x(AF1 a){return AF4(a,a,a,a);}
#define AF1_(a) AF1_x(AF1(a))
#define AF2_(a) AF2_x(AF1(a))
#define AF3_(a) AF3_x(AF1(a))
#define AF4_(a) AF4_x(AF1(a))
//------------------------------------------------------------------------------------------------------------------------------
AU1 AU1_x(AU1 a){return AU1(a);}
AU2 AU2_x(AU1 a){return AU2(a,a);}
AU3 AU3_x(AU1 a){return AU3(a,a,a);}
AU4 AU4_x(AU1 a){return AU4(a,a,a,a);}
#define AU1_(a) AU1_x(AU1(a))
#define AU2_(a) AU2_x(AU1(a))
#define AU3_(a) AU3_x(AU1(a))
#define AU4_(a) AU4_x(AU1(a))
//==============================================================================================================================
AU1 AAbsSU1(AU1 a){return AU1(abs(ASU1(a)));}
AU2 AAbsSU2(AU2 a){return AU2(abs(ASU2(a)));}
AU3 AAbsSU3(AU3 a){return AU3(abs(ASU3(a)));}
AU4 AAbsSU4(AU4 a){return AU4(abs(ASU4(a)));}
//------------------------------------------------------------------------------------------------------------------------------
AU1 ABfe(AU1 src,AU1 off,AU1 bits){AU1 mask=(1u<<bits)-1;return (src>>off)&mask;}
AU1 ABfi(AU1 src,AU1 ins,AU1 mask){return (ins&mask)|(src&(~mask));}
AU1 ABfiM(AU1 src,AU1 ins,AU1 bits){AU1 mask=(1u<<bits)-1;return (ins&mask)|(src&(~mask));}
//------------------------------------------------------------------------------------------------------------------------------
AF1 AClampF1(AF1 x,AF1 n,AF1 m){return max(n,min(x,m));}
AF2 AClampF2(AF2 x,AF2 n,AF2 m){return max(n,min(x,m));}
AF3 AClampF3(AF3 x,AF3 n,AF3 m){return max(n,min(x,m));}
AF4 AClampF4(AF4 x,AF4 n,AF4 m){return max(n,min(x,m));}
//------------------------------------------------------------------------------------------------------------------------------
AF1 AFractF1(AF1 x){return x-floor(x);}
AF2 AFractF2(AF2 x){return x-floor(x);}
AF3 AFractF3(AF3 x){return x-floor(x);}
AF4 AFractF4(AF4 x){return x-floor(x);}
//------------------------------------------------------------------------------------------------------------------------------
AF1 ALerpF1(AF1 x,AF1 y,AF1 a){return lerp(x,y,a);}
AF2 ALerpF2(AF2 x,AF2 y,AF2 a){return lerp(x,y,a);}
AF3 ALerpF3(AF3 x,AF3 y,AF3 a){return lerp(x,y,a);}
AF4 ALerpF4(AF4 x,AF4 y,AF4 a){return lerp(x,y,a);}
//------------------------------------------------------------------------------------------------------------------------------
AF1 AMax3F1(AF1 x,AF1 y,AF1 z){return max(x,max(y,z));}
AF2 AMax3F2(AF2 x,AF2 y,AF2 z){return max(x,max(y,z));}
AF3 AMax3F3(AF3 x,AF3 y,AF3 z){return max(x,max(y,z));}
AF4 AMax3F4(AF4 x,AF4 y,AF4 z){return max(x,max(y,z));}
//------------------------------------------------------------------------------------------------------------------------------
AU1 AMax3SU1(AU1 x,AU1 y,AU1 z){return AU1(max(ASU1(x),max(ASU1(y),ASU1(z))));}
AU2 AMax3SU2(AU2 x,AU2 y,AU2 z){return AU2(max(ASU2(x),max(ASU2(y),ASU2(z))));}
AU3 AMax3SU3(AU3 x,AU3 y,AU3 z){return AU3(max(ASU3(x),max(ASU3(y),ASU3(z))));}
AU4 AMax3SU4(AU4 x,AU4 y,AU4 z){return AU4(max(ASU4(x),max(ASU4(y),ASU4(z))));}
//------------------------------------------------------------------------------------------------------------------------------
AU1 AMax3U1(AU1 x,AU1 y,AU1 z){return max(x,max(y,z));}
AU2 AMax3U2(AU2 x,AU2 y,AU2 z){return max(x,max(y,z));}
AU3 AMax3U3(AU3 x,AU3 y,AU3 z){return max(x,max(y,z));}
AU4 AMax3U4(AU4 x,AU4 y,AU4 z){return max(x,max(y,z));}
//------------------------------------------------------------------------------------------------------------------------------
AU1 AMaxSU1(AU1 a,AU1 b){return AU1(max(ASU1(a),ASU1(b)));}
AU2 AMaxSU2(AU2 a,AU2 b){return AU2(max(ASU2(a),ASU2(b)));}
AU3 AMaxSU3(AU3 a,AU3 b){return AU3(max(ASU3(a),ASU3(b)));}
AU4 AMaxSU4(AU4 a,AU4 b){return AU4(max(ASU4(a),ASU4(b)));}
//------------------------------------------------------------------------------------------------------------------------------
AF1 AMed3F1(AF1 x,AF1 y,AF1 z){return max(min(x,y),min(max(x,y),z));}
AF2 AMed3F2(AF2 x,AF2 y,AF2 z){return max(min(x,y),min(max(x,y),z));}
AF3 AMed3F3(AF3 x,AF3 y,AF3 z){return max(min(x,y),min(max(x,y),z));}
AF4 AMed3F4(AF4 x,AF4 y,AF4 z){return max(min(x,y),min(max(x,y),z));}
//------------------------------------------------------------------------------------------------------------------------------
AF1 AMin3F1(AF1 x,AF1 y,AF1 z){return min(x,min(y,z));}
AF2 AMin3F2(AF2 x,AF2 y,AF2 z){return min(x,min(y,z));}
AF3 AMin3F3(AF3 x,AF3 y,AF3 z){return min(x,min(y,z));}
AF4 AMin3F4(AF4 x,AF4 y,AF4 z){return min(x,min(y,z));}
//------------------------------------------------------------------------------------------------------------------------------
AU1 AMin3SU1(AU1 x,AU1 y,AU1 z){return AU1(min(ASU1(x),min(ASU1(y),ASU1(z))));}
AU2 AMin3SU2(AU2 x,AU2 y,AU2 z){return AU2(min(ASU2(x),min(ASU2(y),ASU2(z))));}
AU3 AMin3SU3(AU3 x,AU3 y,AU3 z){return AU3(min(ASU3(x),min(ASU3(y),ASU3(z))));}
AU4 AMin3SU4(AU4 x,AU4 y,AU4 z){return AU4(min(ASU4(x),min(ASU4(y),ASU4(z))));}
//------------------------------------------------------------------------------------------------------------------------------
AU1 AMin3U1(AU1 x,AU1 y,AU1 z){return min(x,min(y,z));}
AU2 AMin3U2(AU2 x,AU2 y,AU2 z){return min(x,min(y,z));}
AU3 AMin3U3(AU3 x,AU3 y,AU3 z){return min(x,min(y,z));}
AU4 AMin3U4(AU4 x,AU4 y,AU4 z){return min(x,min(y,z));}
//------------------------------------------------------------------------------------------------------------------------------
AU1 AMinSU1(AU1 a,AU1 b){return AU1(min(ASU1(a),ASU1(b)));}
AU2 AMinSU2(AU2 a,AU2 b){return AU2(min(ASU2(a),ASU2(b)));}
AU3 AMinSU3(AU3 a,AU3 b){return AU3(min(ASU3(a),ASU3(b)));}
AU4 AMinSU4(AU4 a,AU4 b){return AU4(min(ASU4(a),ASU4(b)));}
//------------------------------------------------------------------------------------------------------------------------------
AF1 ANCosF1(AF1 x){return cos(x*AF1_(A_2PI));}
AF2 ANCosF2(AF2 x){return cos(x*AF2_(A_2PI));}
AF3 ANCosF3(AF3 x){return cos(x*AF3_(A_2PI));}
AF4 ANCosF4(AF4 x){return cos(x*AF4_(A_2PI));}
//------------------------------------------------------------------------------------------------------------------------------
AF1 ANSinF1(AF1 x){return sin(x*AF1_(A_2PI));}
AF2 ANSinF2(AF2 x){return sin(x*AF2_(A_2PI));}
AF3 ANSinF3(AF3 x){return sin(x*AF3_(A_2PI));}
AF4 ANSinF4(AF4 x){return sin(x*AF4_(A_2PI));}
//------------------------------------------------------------------------------------------------------------------------------
AF1 ARcpF1(AF1 x){return rcp(x);}
AF2 ARcpF2(AF2 x){return rcp(x);}
AF3 ARcpF3(AF3 x){return rcp(x);}
AF4 ARcpF4(AF4 x){return rcp(x);}
//------------------------------------------------------------------------------------------------------------------------------
AF1 ARsqF1(AF1 x){return rsqrt(x);}
AF2 ARsqF2(AF2 x){return rsqrt(x);}
AF3 ARsqF3(AF3 x){return rsqrt(x);}
AF4 ARsqF4(AF4 x){return rsqrt(x);}
//------------------------------------------------------------------------------------------------------------------------------
AF1 ASatF1(AF1 x){return saturate(x);}
AF2 ASatF2(AF2 x){return saturate(x);}
AF3 ASatF3(AF3 x){return saturate(x);}
AF4 ASatF4(AF4 x){return saturate(x);}
//------------------------------------------------------------------------------------------------------------------------------
AU1 AShrSU1(AU1 a,AU1 b){return AU1(ASU1(a)>>ASU1(b));}
AU2 AShrSU2(AU2 a,AU2 b){return AU2(ASU2(a)>>ASU2(b));}
AU3 AShrSU3(AU3 a,AU3 b){return AU3(ASU3(a)>>ASU3(b));}
AU4 AShrSU4(AU4 a,AU4 b){return AU4(ASU4(a)>>ASU4(b));}



//==============================================================================================================================
//
//
//                                                          GPU COMMON
//
//
//==============================================================================================================================

// Negative and positive infinity.
#define A_INFP_F AF1_AU1(0x7f800000u)
#define A_INFN_F AF1_AU1(0xff800000u)
//------------------------------------------------------------------------------------------------------------------------------
// Copy sign from 's' to positive 'd'.
AF1 ACpySgnF1(AF1 d,AF1 s){return AF1_AU1(AU1_AF1(d)|(AU1_AF1(s)&AU1_(0x80000000u)));}
AF2 ACpySgnF2(AF2 d,AF2 s){return AF2_AU2(AU2_AF2(d)|(AU2_AF2(s)&AU2_(0x80000000u)));}
AF3 ACpySgnF3(AF3 d,AF3 s){return AF3_AU3(AU3_AF3(d)|(AU3_AF3(s)&AU3_(0x80000000u)));}
AF4 ACpySgnF4(AF4 d,AF4 s){return AF4_AU4(AU4_AF4(d)|(AU4_AF4(s)&AU4_(0x80000000u)));}
//------------------------------------------------------------------------------------------------------------------------------
// Single operation to return (useful to create a mask to use in lerp for branch free logic),
//  m=NaN := 0
//  m>=0  := 0
//  m<0   := 1
// Uses the following useful floating point logic,
//  saturate(+a*(-INF)==-INF) := 0
//  saturate( 0*(-INF)== NaN) := 0
//  saturate(-a*(-INF)==+INF) := 1
AF1 ASignedF1(AF1 m){return ASatF1(m*AF1_(A_INFN_F));}
AF2 ASignedF2(AF2 m){return ASatF2(m*AF2_(A_INFN_F));}
AF3 ASignedF3(AF3 m){return ASatF3(m*AF3_(A_INFN_F));}
AF4 ASignedF4(AF4 m){return ASatF4(m*AF4_(A_INFN_F));}
//------------------------------------------------------------------------------------------------------------------------------
AF1 AGtZeroF1(AF1 m){return ASatF1(m*AF1_(A_INFP_F));}
AF2 AGtZeroF2(AF2 m){return ASatF2(m*AF2_(A_INFP_F));}
AF3 AGtZeroF3(AF3 m){return ASatF3(m*AF3_(A_INFP_F));}
AF4 AGtZeroF4(AF4 m){return ASatF4(m*AF4_(A_INFP_F));}



//==============================================================================================================================
//                                                    FLOAT APPROXIMATIONS
//------------------------------------------------------------------------------------------------------------------------------
// Michal Drobot has an excellent presentation on these: "Low Level Optimizations For GCN",
//  - Idea dates back to SGI, then to Quake 3, etc.
//  - https://michaldrobot.files.wordpress.com/2014/05/gcn_alu_opt_digitaldragons2014.pdf
//     - sqrt(x)=rsqrt(x)*x
//     - rcp(x)=rsqrt(x)*rsqrt(x) for positive x
//  - https://github.com/michaldrobot/ShaderFastLibs/blob/master/ShaderFastMathLib.h
//------------------------------------------------------------------------------------------------------------------------------
// These below are from perhaps less complete searching for optimal.
// Used FP16 normal range for testing with +4096 32-bit step size for sampling error.
// So these match up well with the half approximations.
//==============================================================================================================================
 AF1 APrxLoSqrtF1(AF1 a){return AF1_AU1((AU1_AF1(a)>>AU1_(1))+AU1_(0x1fbc4639));}
 AF1 APrxLoRcpF1(AF1 a){return AF1_AU1(AU1_(0x7ef07ebb)-AU1_AF1(a));}
 AF1 APrxMedRcpF1(AF1 a){AF1 b=AF1_AU1(AU1_(0x7ef19fff)-AU1_AF1(a));return b*(-b*a+AF1_(2.0));}
 AF1 APrxLoRsqF1(AF1 a){return AF1_AU1(AU1_(0x5f347d74)-(AU1_AF1(a)>>AU1_(1)));}
//------------------------------------------------------------------------------------------------------------------------------
 AF2 APrxLoSqrtF2(AF2 a){return AF2_AU2((AU2_AF2(a)>>AU2_(1))+AU2_(0x1fbc4639));}
 AF2 APrxLoRcpF2(AF2 a){return AF2_AU2(AU2_(0x7ef07ebb)-AU2_AF2(a));}
 AF2 APrxMedRcpF2(AF2 a){AF2 b=AF2_AU2(AU2_(0x7ef19fff)-AU2_AF2(a));return b*(-b*a+AF2_(2.0));}
 AF2 APrxLoRsqF2(AF2 a){return AF2_AU2(AU2_(0x5f347d74)-(AU2_AF2(a)>>AU2_(1)));}
//------------------------------------------------------------------------------------------------------------------------------
 AF3 APrxLoSqrtF3(AF3 a){return AF3_AU3((AU3_AF3(a)>>AU3_(1))+AU3_(0x1fbc4639));}
 AF3 APrxLoRcpF3(AF3 a){return AF3_AU3(AU3_(0x7ef07ebb)-AU3_AF3(a));}
 AF3 APrxMedRcpF3(AF3 a){AF3 b=AF3_AU3(AU3_(0x7ef19fff)-AU3_AF3(a));return b*(-b*a+AF3_(2.0));}
 AF3 APrxLoRsqF3(AF3 a){return AF3_AU3(AU3_(0x5f347d74)-(AU3_AF3(a)>>AU3_(1)));}
//------------------------------------------------------------------------------------------------------------------------------
 AF4 APrxLoSqrtF4(AF4 a){return AF4_AU4((AU4_AF4(a)>>AU4_(1))+AU4_(0x1fbc4639));}
 AF4 APrxLoRcpF4(AF4 a){return AF4_AU4(AU4_(0x7ef07ebb)-AU4_AF4(a));}
 AF4 APrxMedRcpF4(AF4 a){AF4 b=AF4_AU4(AU4_(0x7ef19fff)-AU4_AF4(a));return b*(-b*a+AF4_(2.0));}
 AF4 APrxLoRsqF4(AF4 a){return AF4_AU4(AU4_(0x5f347d74)-(AU4_AF4(a)>>AU4_(1)));}



//==============================================================================================================================
//
//
//                                                     GPU/CPU PORTABILITY
//
//
//------------------------------------------------------------------------------------------------------------------------------
// This is the GPU implementation.
// See the CPU implementation for docs.
//==============================================================================================================================


//==============================================================================================================================
//                                     VECTOR ARGUMENT/RETURN/INITIALIZATION PORTABILITY
//==============================================================================================================================
 #define retAD2 AD2
 #define retAD3 AD3
 #define retAD4 AD4
 #define retAF2 AF2
 #define retAF3 AF3
 #define retAF4 AF4
 #define retAL2 AL2
 #define retAL3 AL3
 #define retAL4 AL4
 #define retAU2 AU2
 #define retAU3 AU3
 #define retAU4 AU4
//------------------------------------------------------------------------------------------------------------------------------
 #define inAD2 in AD2
 #define inAD3 in AD3
 #define inAD4 in AD4
 #define inAF2 in AF2
 #define inAF3 in AF3
 #define inAF4 in AF4
 #define inAL2 in AL2
 #define inAL3 in AL3
 #define inAL4 in AL4
 #define inAU2 in AU2
 #define inAU3 in AU3
 #define inAU4 in AU4
//------------------------------------------------------------------------------------------------------------------------------
 #define inoutAD2 inout AD2
 #define inoutAD3 inout AD3
 #define inoutAD4 inout AD4
 #define inoutAF2 inout AF2
 #define inoutAF3 inout AF3
 #define inoutAF4 inout AF4
 #define inoutAL2 inout AL2
 #define inoutAL3 inout AL3
 #define inoutAL4 inout AL4
 #define inoutAU2 inout AU2
 #define inoutAU3 inout AU3
 #define inoutAU4 inout AU4
//------------------------------------------------------------------------------------------------------------------------------
 #define outAD2 out AD2
 #define outAD3 out AD3
 #define outAD4 out AD4
 #define outAF2 out AF2
 #define outAF3 out AF3
 #define outAF4 out AF4
 #define outAL2 out AL2
 #define outAL3 out AL3
 #define outAL4 out AL4
 #define outAU2 out AU2
 #define outAU3 out AU3
 #define outAU4 out AU4
//------------------------------------------------------------------------------------------------------------------------------
 #define varAD2(x) AD2 x
 #define varAD3(x) AD3 x
 #define varAD4(x) AD4 x
 #define varAF2(x) AF2 x
 #define varAF3(x) AF3 x
 #define varAF4(x) AF4 x
 #define varAL2(x) AL2 x
 #define varAL3(x) AL3 x
 #define varAL4(x) AL4 x
 #define varAU2(x) AU2 x
 #define varAU3(x) AU3 x
 #define varAU4(x) AU4 x
//------------------------------------------------------------------------------------------------------------------------------
 #define initAD2(x,y) AD2(x,y)
 #define initAD3(x,y,z) AD3(x,y,z)
 #define initAD4(x,y,z,w) AD4(x,y,z,w)
 #define initAF2(x,y) AF2(x,y)
 #define initAF3(x,y,z) AF3(x,y,z)
 #define initAF4(x,y,z,w) AF4(x,y,z,w)
 #define initAL2(x,y) AL2(x,y)
 #define initAL3(x,y,z) AL3(x,y,z)
 #define initAL4(x,y,z,w) AL4(x,y,z,w)
 #define initAU2(x,y) AU2(x,y)
 #define initAU3(x,y,z) AU3(x,y,z)
 #define initAU4(x,y,z,w) AU4(x,y,z,w)


//==============================================================================================================================
//                                                     SCALAR RETURN OPS
//==============================================================================================================================
 #define AAbsD1(a) abs(AD1(a))
 #define AAbsF1(a) abs(AF1(a))
//------------------------------------------------------------------------------------------------------------------------------
 #define ACosD1(a) cos(AD1(a))
 #define ACosF1(a) cos(AF1(a))
//------------------------------------------------------------------------------------------------------------------------------
 #define ADotD2(a,b) dot(AD2(a),AD2(b))
 #define ADotD3(a,b) dot(AD3(a),AD3(b))
 #define ADotD4(a,b) dot(AD4(a),AD4(b))
 #define ADotF2(a,b) dot(AF2(a),AF2(b))
 #define ADotF3(a,b) dot(AF3(a),AF3(b))
 #define ADotF4(a,b) dot(AF4(a),AF4(b))
//------------------------------------------------------------------------------------------------------------------------------
 #define AExp2D1(a) exp2(AD1(a))
 #define AExp2F1(a) exp2(AF1(a))
//------------------------------------------------------------------------------------------------------------------------------
 #define AFloorD1(a) floor(AD1(a))
 #define AFloorF1(a) floor(AF1(a))
//------------------------------------------------------------------------------------------------------------------------------
 #define ALog2D1(a) log2(AD1(a))
 #define ALog2F1(a) log2(AF1(a))
//------------------------------------------------------------------------------------------------------------------------------
 #define AMaxD1(a,b) max(a,b)
 #define AMaxF1(a,b) max(a,b)
 #define AMaxL1(a,b) max(a,b)
 #define AMaxU1(a,b) max(a,b)
//------------------------------------------------------------------------------------------------------------------------------
 #define AMinD1(a,b) min(a,b)
 #define AMinF1(a,b) min(a,b)
 #define AMinL1(a,b) min(a,b)
 #define AMinU1(a,b) min(a,b)
//------------------------------------------------------------------------------------------------------------------------------
 #define ASinD1(a) sin(AD1(a))
 #define ASinF1(a) sin(AF1(a))
//------------------------------------------------------------------------------------------------------------------------------
 #define ASqrtD1(a) sqrt(AD1(a))
 #define ASqrtF1(a) sqrt(AF1(a))


//==============================================================================================================================
//                                               SCALAR RETURN OPS - DEPENDENT
//==============================================================================================================================
 #define APowD1(a,b) pow(AD1(a),AF1(b))
 #define APowF1(a,b) pow(AF1(a),AF1(b))


//==============================================================================================================================
//                                                         VECTOR OPS
//------------------------------------------------------------------------------------------------------------------------------
// These are added as needed for production or prototyping, so not necessarily a complete set.
// They follow a convention of taking in a destination and also returning the destination value to increase utility.
//==============================================================================================================================
 AF2 opAAbsF2(outAF2 d,inAF2 a){d=abs(a);return d;}
 AF3 opAAbsF3(outAF3 d,inAF3 a){d=abs(a);return d;}
 AF4 opAAbsF4(outAF4 d,inAF4 a){d=abs(a);return d;}
//------------------------------------------------------------------------------------------------------------------------------
 AF2 opAAddF2(outAF2 d,inAF2 a,inAF2 b){d=a+b;return d;}
 AF3 opAAddF3(outAF3 d,inAF3 a,inAF3 b){d=a+b;return d;}
 AF4 opAAddF4(outAF4 d,inAF4 a,inAF4 b){d=a+b;return d;}
//------------------------------------------------------------------------------------------------------------------------------
 AF2 opAAddOneF2(outAF2 d,inAF2 a,AF1 b){d=a+AF2_(b);return d;}
 AF3 opAAddOneF3(outAF3 d,inAF3 a,AF1 b){d=a+AF3_(b);return d;}
 AF4 opAAddOneF4(outAF4 d,inAF4 a,AF1 b){d=a+AF4_(b);return d;}
//------------------------------------------------------------------------------------------------------------------------------
 AF2 opACpyF2(outAF2 d,inAF2 a){d=a;return d;}
 AF3 opACpyF3(outAF3 d,inAF3 a){d=a;return d;}
 AF4 opACpyF4(outAF4 d,inAF4 a){d=a;return d;}
//------------------------------------------------------------------------------------------------------------------------------
 AF2 opALerpF2(outAF2 d,inAF2 a,inAF2 b,inAF2 c){d=ALerpF2(a,b,c);return d;}
 AF3 opALerpF3(outAF3 d,inAF3 a,inAF3 b,inAF3 c){d=ALerpF3(a,b,c);return d;}
 AF4 opALerpF4(outAF4 d,inAF4 a,inAF4 b,inAF4 c){d=ALerpF4(a,b,c);return d;}
//------------------------------------------------------------------------------------------------------------------------------
 AF2 opALerpOneF2(outAF2 d,inAF2 a,inAF2 b,AF1 c){d=ALerpF2(a,b,AF2_(c));return d;}
 AF3 opALerpOneF3(outAF3 d,inAF3 a,inAF3 b,AF1 c){d=ALerpF3(a,b,AF3_(c));return d;}
 AF4 opALerpOneF4(outAF4 d,inAF4 a,inAF4 b,AF1 c){d=ALerpF4(a,b,AF4_(c));return d;}
//------------------------------------------------------------------------------------------------------------------------------
 AF2 opAMaxF2(outAF2 d,inAF2 a,inAF2 b){d=max(a,b);return d;}
 AF3 opAMaxF3(outAF3 d,inAF3 a,inAF3 b){d=max(a,b);return d;}
 AF4 opAMaxF4(outAF4 d,inAF4 a,inAF4 b){d=max(a,b);return d;}
//------------------------------------------------------------------------------------------------------------------------------
 AF2 opAMinF2(outAF2 d,inAF2 a,inAF2 b){d=min(a,b);return d;}
 AF3 opAMinF3(outAF3 d,inAF3 a,inAF3 b){d=min(a,b);return d;}
 AF4 opAMinF4(outAF4 d,inAF4 a,inAF4 b){d=min(a,b);return d;}
//------------------------------------------------------------------------------------------------------------------------------
 AF2 opAMulF2(outAF2 d,inAF2 a,inAF2 b){d=a*b;return d;}
 AF3 opAMulF3(outAF3 d,inAF3 a,inAF3 b){d=a*b;return d;}
 AF4 opAMulF4(outAF4 d,inAF4 a,inAF4 b){d=a*b;return d;}
//------------------------------------------------------------------------------------------------------------------------------
 AF2 opAMulOneF2(outAF2 d,inAF2 a,AF1 b){d=a*AF2_(b);return d;}
 AF3 opAMulOneF3(outAF3 d,inAF3 a,AF1 b){d=a*AF3_(b);return d;}
 AF4 opAMulOneF4(outAF4 d,inAF4 a,AF1 b){d=a*AF4_(b);return d;}
//------------------------------------------------------------------------------------------------------------------------------
 AF2 opANegF2(outAF2 d,inAF2 a){d=-a;return d;}
 AF3 opANegF3(outAF3 d,inAF3 a){d=-a;return d;}
 AF4 opANegF4(outAF4 d,inAF4 a){d=-a;return d;}
//------------------------------------------------------------------------------------------------------------------------------
 AF2 opARcpF2(outAF2 d,inAF2 a){d=ARcpF2(a);return d;}
 AF3 opARcpF3(outAF3 d,inAF3 a){d=ARcpF3(a);return d;}
 AF4 opARcpF4(outAF4 d,inAF4 a){d=ARcpF4(a);return d;}