//==============================================================================================================================
//
//                                               [A] SHADER PORTABILITY 1.20210629 (MODIFIED)
//
//==============================================================================================================================
// ABOUT
// ===========
//
// This is a copy of AMD's FSR provided ffx_a.h file which has been significantly pruned for use by OBS's graphics sub-system.
// Modification is required as the shader parser does not support all functionality that is used by the original file, namely:
//  
//   > The graphics sub-subsystem treats shaders as written in HLSL, which are then converted to GLSL in a very basic manner. 
//     So only the HLSL parts of the file are required.  
//
//   > #if macros are not supported, but #ifdef is.
//
//   > Computations within #define macros are not supported, so they must be resolved manually.
//   
//   > Many HLSL-GLSL conversions are not supported by the OBS shader parser, such as:
//
//          - uint* aren't converted to equivalent uvec*, so AU* types aren't supported.
//
//          - HLSL rcp function does not have an equivalent in GLSL, so ARcpF* functions have been re-written 
//            to perform the reciprocal manually.
//
//          - as* type bit re-interpretation casts aren't converted to their GLSL equivalent. Within the current FSR implementation
//            these are only used for converting the input constants to floats, and for the fast float approximation functions.
//            The former is resolved by re-interpreting the bits on the CPU before the constants are passed to the shaders as floats.
//            The latter is resolved at a performance cost by not using the approximations.
//            
// Many of the HLSL defines left in are unsupported but aren't used by this version of FSR, so aren't causing issues.
//
// Please see the original ffx_a.h file for proper implementation. 
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
#define AU1 uint  //NOTE: not supported in HLSL-GLSL conversion, necessary functions which use this must be modified.
#define AU2 uint2 //NOTE: not supported in HLSL-GLSL conversion, necessary functions which use this must be modified.
#define AU3 uint3 //NOTE: not supported in HLSL-GLSL conversion, necessary functions which use this must be modified.
#define AU4 uint4 //NOTE: not supported in HLSL-GLSL conversion, necessary functions which use this must be modified.
//------------------------------------------------------------------------------------------------------------------------------
#define ASU1 int
#define ASU2 int2
#define ASU3 int3
#define ASU4 int4
//==============================================================================================================================
#define AF1_AU1(x) asfloat(AU1(x)) //NOTE: not supported in HLSL-GLSL conversion, necessary functions which use this must be modified.
#define AF2_AU2(x) asfloat(AU2(x)) //NOTE: not supported in HLSL-GLSL conversion, necessary functions which use this must be modified.
#define AF3_AU3(x) asfloat(AU3(x)) //NOTE: not supported in HLSL-GLSL conversion, necessary functions which use this must be modified.
#define AF4_AU4(x) asfloat(AU4(x)) //NOTE: not supported in HLSL-GLSL conversion, necessary functions which use this must be modified.
//------------------------------------------------------------------------------------------------------------------------------
#define AU1_AF1(x) asuint(AF1(x)) //NOTE: not supported in HLSL-GLSL conversion, necessary functions which use this must be modified.
#define AU2_AF2(x) asuint(AF2(x)) //NOTE: not supported in HLSL-GLSL conversion, necessary functions which use this must be modified.
#define AU3_AF3(x) asuint(AF3(x)) //NOTE: not supported in HLSL-GLSL conversion, necessary functions which use this must be modified.
#define AU4_AF4(x) asuint(AF4(x)) //NOTE: not supported in HLSL-GLSL conversion, necessary functions which use this must be modified.
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
AF1 ARcpF1(AF1 x){return AF1_x(1.0f)/x;} //NOTE: rcp() not supported in HLSL-GLSL conversion, changed to manual calculation
AF2 ARcpF2(AF2 x){return AF2_x(1.0f)/x;} //NOTE: rcp() not supported in HLSL-GLSL conversion, changed to manual calculation
AF3 ARcpF3(AF3 x){return AF3_x(1.0f)/x;} //NOTE: rcp() not supported in HLSL-GLSL conversion, changed to manual calculation
AF4 ARcpF4(AF4 x){return AF4_x(1.0f)/x;} //NOTE: rcp() not supported in HLSL-GLSL conversion, changed to manual calculation
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
 AF1 APrxLoSqrtF1(AF1 a){return sqrt(a);}   //NOTE: Not supported in HLSL-GLSL conversion, changed to non-approx calculation
 AF1 APrxLoRcpF1(AF1 a){return ARcpF1(a);}  //NOTE: Not supported in HLSL-GLSL conversion, changed to non-approx calculation
 AF1 APrxMedRcpF1(AF1 a){return ARcpF1(a);} //NOTE: Not supported in HLSL-GLSL conversion, changed to non-approx calculation
 AF1 APrxLoRsqF1(AF1 a){return ARsqF1(a);}  //NOTE: Not supported in HLSL-GLSL conversion, changed to non-approx calculation
//------------------------------------------------------------------------------------------------------------------------------
 AF1 APrxLoSqrtF2(AF2 a){return sqrt(a);}   //NOTE: Not supported in HLSL-GLSL conversion, changed to non-approx calculation
 AF1 APrxLoRcpF2(AF2 a){return ARcpF2(a);}  //NOTE: Not supported in HLSL-GLSL conversion, changed to non-approx calculation
 AF1 APrxMedRcpF2(AF2 a){return ARcpF2(a);} //NOTE: Not supported in HLSL-GLSL conversion, changed to non-approx calculation
 AF1 APrxLoRsqF2(AF2 a){return ARsqF2(a);}  //NOTE: Not supported in HLSL-GLSL conversion, changed to non-approx calculation
//------------------------------------------------------------------------------------------------------------------------------
 AF3 APrxLoSqrtF3(AF3 a){return sqrt(a);}   //NOTE: Not supported in HLSL-GLSL conversion, changed to non-approx calculation
 AF3 APrxLoRcpF3(AF3 a){return ARcpF3(a);}  //NOTE: Not supported in HLSL-GLSL conversion, changed to non-approx calculation
 AF3 APrxMedRcpF3(AF3 a){return ARcpF3(a);} //NOTE: Not supported in HLSL-GLSL conversion, changed to non-approx calculation
 AF3 APrxLoRsqF3(AF3 a){return ARsqF3(a);}  //NOTE: Not supported in HLSL-GLSL conversion, changed to non-approx calculation
//------------------------------------------------------------------------------------------------------------------------------
 AF4 APrxLoSqrtF4(AF4 a){return sqrt(a);}   //NOTE: Not supported in HLSL-GLSL conversion, changed to non-approx calculation
 AF4 APrxLoRcpF4(AF4 a){return ARcpF4(a);}  //NOTE: Not supported in HLSL-GLSL conversion, changed to non-approx calculation
 AF4 APrxMedRcpF4(AF4 a){return ARcpF4(a);} //NOTE: Not supported in HLSL-GLSL conversion, changed to non-approx calculation
 AF4 APrxLoRsqF4(AF4 a){return ARsqF4(a);}  //NOTE: Not supported in HLSL-GLSL conversion, changed to non-approx calculation

