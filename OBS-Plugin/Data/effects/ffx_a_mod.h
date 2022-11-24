//==============================================================================================================================
//
//                                               [A] SHADER PORTABILITY 1.20210629 (MODIFIED)
//
//==============================================================================================================================
// ABOUT
// ===========
//
// This is a copy of AMD's FSR provided ffx_a.h file which has been modified/pruned for use by OBS's graphics system.
// 
// The following modifications have been made:
//
//   > The file has been simplified by removing everything except what is strictly necessary for FSR & CAS to  
//     aid in testing and compatibility with OBS's shader parser. Mainly the HLSL implementation has been left in as
//     OBS' shaders are written in HLSL, with built in conversion to GLSL when needed. 
//
//   > Certain required functionality which OBS' shader parser does not properly convert to its GLSL counter part 
//     has been converted manually. The OBS define _OPENGL is used to switch between the two.
//
//   > #if macros are not supported so have been swapped for simple #ifdef HLSL and #ifdef GLSL macros.
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
//                                                           MAIN DEFINES 
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
#ifdef _OPENGL
    #define AU1 uint
    #define AU2 uvec2
    #define AU3 uvec3
    #define AU4 uvec4

    #define AF1_AU1(x) uintBitsToFloat(x)
    #define AF2_AU2(x) uintBitsToFloat(x)
    #define AF3_AU3(x) uintBitsToFloat(x)
    #define AF4_AU4(x) uintBitsToFloat(x)

    #define AU1_AF1(x) floatBitsToUint(x)
    #define AU2_AF2(x) floatBitsToUint(x)
    #define AU3_AF3(x) floatBitsToUint(x)
    #define AU4_AF4(x) floatBitsToUint(x)
#else
    #define AU1 uint
    #define AU2 uvec2
    #define AU3 uvec3
    #define AU4 uvec4

    #define AF1_AU1(x) asfloat(AU1(x))
    #define AF2_AU2(x) asfloat(AU2(x))
    #define AF3_AU3(x) asfloat(AU3(x))
    #define AF4_AU4(x) asfloat(AU4(x))

    #define AU1_AF1(x) asuint(AF1(x))
    #define AU2_AF2(x) asuint(AF2(x))
    #define AU3_AF3(x) asuint(AF3(x))
    #define AU4_AF4(x) asuint(AF4(x))
#endif 
//------------------------------------------------------------------------------------------------------------------------------
#define ASU1 int
#define ASU2 int2
#define ASU3 int3
#define ASU4 int4
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
AF1 AMax3F1(AF1 x,AF1 y,AF1 z){return max(x,max(y,z));}
AF2 AMax3F2(AF2 x,AF2 y,AF2 z){return max(x,max(y,z));}
AF3 AMax3F3(AF3 x,AF3 y,AF3 z){return max(x,max(y,z));}
AF4 AMax3F4(AF4 x,AF4 y,AF4 z){return max(x,max(y,z));}
//------------------------------------------------------------------------------------------------------------------------------
AF1 AMin3F1(AF1 x,AF1 y,AF1 z){return min(x,min(y,z));}
AF2 AMin3F2(AF2 x,AF2 y,AF2 z){return min(x,min(y,z));}
AF3 AMin3F3(AF3 x,AF3 y,AF3 z){return min(x,min(y,z));}
AF4 AMin3F4(AF4 x,AF4 y,AF4 z){return min(x,min(y,z));}
//------------------------------------------------------------------------------------------------------------------------------
#ifdef _OPENGL
    AF1 ARcpF1(AF1 x){return AF1_x(1.0f)/x;}
    AF2 ARcpF2(AF2 x){return AF2_x(1.0f)/x;} 
    AF3 ARcpF3(AF3 x){return AF3_x(1.0f)/x;} 
    AF4 ARcpF4(AF4 x){return AF4_x(1.0f)/x;} 
#else
    AF1 ARcpF1(AF1 x){return rcp(x);}
    AF2 ARcpF2(AF2 x){return rcp(x);}
    AF3 ARcpF3(AF3 x){return rcp(x);}
    AF4 ARcpF4(AF4 x){return rcp(x);}
#endif 
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