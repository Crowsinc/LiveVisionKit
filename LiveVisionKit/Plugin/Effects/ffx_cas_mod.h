//==============================================================================================================================
//
//                                 [CAS] FIDELITY FX - CONSTRAST ADAPTIVE SHARPENING 1.20190610 (MODIFIED)
//
//==============================================================================================================================
// ABOUT
// ===========
//
// This is a copy of AMD's CAS provided ffx_cas.h file which has been modified/pruned for use by OBS's graphics system.
// 
// The following modifications have been made:
//
//   > The file has been simplified by removing everything except for the core unpacked 32bit CAS implementation. 
//     The implementation itself has also been stripped of all scaling and non-sharpening code for simplicity
//     in debugging and to maximise compatibility.
//
//   > The AU4 constants have been swapped for AF4 for better OBS compatibility. All instances are ultimately converted
//     to floats for use anyway. So they are not necessary in this version of CAS, however this may change in the future.   
// 
// Please see the original ffx_cas.h file for proper implementation. 
//
// AMD LICENSE
// ===========
// Copyright (c) 2017-2019 Advanced Micro Devices, Inc. All rights reserved.
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
//==============================================================================================================================
//
//                                   Fidelity FX Contrast Adaptive Sharpening (SHARPENING ONLY)
//
//                                                   NON-PACKED 32-BIT VERSION
//
//==============================================================================================================================

 void CasFilter(
 out AF1 pixR, // Output values, non-vector so port between CasFilter() and CasFilterH() is easy.
 out AF1 pixG,
 out AF1 pixB,
 AF2 ip, // Integer pixel position in output.
 AF4 const1)
{ 
//------------------------------------------------------------------------------------------------------------------------------
  // No scaling algorithm uses minimal 3x3 pixel neighborhood.
  // a b c 
  // d e f
  // g h i
   ASU2 sp=ASU2(ip);
   AF3 a=CasLoad(sp+ASU2(-1,-1));
   AF3 b=CasLoad(sp+ASU2( 0,-1));
   AF3 c=CasLoad(sp+ASU2( 1,-1));
   AF3 d=CasLoad(sp+ASU2(-1, 0));
   AF3 e=CasLoad(sp);
   AF3 f=CasLoad(sp+ASU2( 1, 0));
   AF3 g=CasLoad(sp+ASU2(-1, 1));
   AF3 h=CasLoad(sp+ASU2( 0, 1));
   AF3 i=CasLoad(sp+ASU2( 1, 1));
  // Run optional input transform.
   CasInput(a.r,a.g,a.b);
   CasInput(b.r,b.g,b.b);
   CasInput(c.r,c.g,c.b);
   CasInput(d.r,d.g,d.b);
   CasInput(e.r,e.g,e.b);
   CasInput(f.r,f.g,f.b);
   CasInput(g.r,g.g,g.b);
   CasInput(h.r,h.g,h.b);
   CasInput(i.r,i.g,i.b);
   // Soft min and max.
   //  a b c             b
   //  d e f * 0.5  +  d e f * 0.5
   //  g h i             h
   // These are 2.0x bigger (factored out the extra multiply).
   AF1 mnR=AMin3F1(AMin3F1(d.r,e.r,f.r),b.r,h.r);
   AF1 mnG=AMin3F1(AMin3F1(d.g,e.g,f.g),b.g,h.g);
   AF1 mnB=AMin3F1(AMin3F1(d.b,e.b,f.b),b.b,h.b);
   #ifdef CAS_BETTER_DIAGONALS
    AF1 mnR2=AMin3F1(AMin3F1(mnR,a.r,c.r),g.r,i.r);
    AF1 mnG2=AMin3F1(AMin3F1(mnG,a.g,c.g),g.g,i.g);
    AF1 mnB2=AMin3F1(AMin3F1(mnB,a.b,c.b),g.b,i.b);
    mnR=mnR+mnR2;
    mnG=mnG+mnG2;
    mnB=mnB+mnB2;
   #endif
   AF1 mxR=AMax3F1(AMax3F1(d.r,e.r,f.r),b.r,h.r);
   AF1 mxG=AMax3F1(AMax3F1(d.g,e.g,f.g),b.g,h.g);
   AF1 mxB=AMax3F1(AMax3F1(d.b,e.b,f.b),b.b,h.b);
   #ifdef CAS_BETTER_DIAGONALS
    AF1 mxR2=AMax3F1(AMax3F1(mxR,a.r,c.r),g.r,i.r);
    AF1 mxG2=AMax3F1(AMax3F1(mxG,a.g,c.g),g.g,i.g);
    AF1 mxB2=AMax3F1(AMax3F1(mxB,a.b,c.b),g.b,i.b);
    mxR=mxR+mxR2;
    mxG=mxG+mxG2;
    mxB=mxB+mxB2;
   #endif
   // Smooth minimum distance to signal limit divided by smooth max.
   #ifdef CAS_GO_SLOWER
    AF1 rcpMR=ARcpF1(mxR);
    AF1 rcpMG=ARcpF1(mxG);
    AF1 rcpMB=ARcpF1(mxB);
   #else
    AF1 rcpMR=APrxLoRcpF1(mxR);
    AF1 rcpMG=APrxLoRcpF1(mxG);
    AF1 rcpMB=APrxLoRcpF1(mxB);
   #endif
   #ifdef CAS_BETTER_DIAGONALS
    AF1 ampR=ASatF1(min(mnR,AF1_(2.0)-mxR)*rcpMR);
    AF1 ampG=ASatF1(min(mnG,AF1_(2.0)-mxG)*rcpMG);
    AF1 ampB=ASatF1(min(mnB,AF1_(2.0)-mxB)*rcpMB);
   #else
    AF1 ampR=ASatF1(min(mnR,AF1_(1.0)-mxR)*rcpMR);
    AF1 ampG=ASatF1(min(mnG,AF1_(1.0)-mxG)*rcpMG);
    AF1 ampB=ASatF1(min(mnB,AF1_(1.0)-mxB)*rcpMB);
   #endif
   // Shaping amount of sharpening.
   #ifdef CAS_GO_SLOWER
    ampR=sqrt(ampR);
    ampG=sqrt(ampG);
    ampB=sqrt(ampB);
   #else
    ampR=APrxLoSqrtF1(ampR);
    ampG=APrxLoSqrtF1(ampG);
    ampB=APrxLoSqrtF1(ampB);
   #endif
   // Filter shape.
   //  0 w 0
   //  w 1 w
   //  0 w 0
   AF1 peak=const1.x;
   AF1 wR=ampR*peak;
   AF1 wG=ampG*peak;
   AF1 wB=ampB*peak;
   // Filter.
   #ifndef CAS_SLOW
    // Using green coef only, depending on dead code removal to strip out the extra overhead.
    #ifdef CAS_GO_SLOWER
     AF1 rcpWeight=ARcpF1(AF1_(1.0)+AF1_(4.0)*wG);
    #else
     AF1 rcpWeight=APrxMedRcpF1(AF1_(1.0)+AF1_(4.0)*wG);
    #endif
    pixR=ASatF1((b.r*wG+d.r*wG+f.r*wG+h.r*wG+e.r)*rcpWeight);
    pixG=ASatF1((b.g*wG+d.g*wG+f.g*wG+h.g*wG+e.g)*rcpWeight);
    pixB=ASatF1((b.b*wG+d.b*wG+f.b*wG+h.b*wG+e.b)*rcpWeight);
   #else
    #ifdef CAS_GO_SLOWER
     AF1 rcpWeightR=ARcpF1(AF1_(1.0)+AF1_(4.0)*wR);
     AF1 rcpWeightG=ARcpF1(AF1_(1.0)+AF1_(4.0)*wG);
     AF1 rcpWeightB=ARcpF1(AF1_(1.0)+AF1_(4.0)*wB);
    #else
     AF1 rcpWeightR=APrxMedRcpF1(AF1_(1.0)+AF1_(4.0)*wR);
     AF1 rcpWeightG=APrxMedRcpF1(AF1_(1.0)+AF1_(4.0)*wG);
     AF1 rcpWeightB=APrxMedRcpF1(AF1_(1.0)+AF1_(4.0)*wB);
    #endif
    pixR=ASatF1((b.r*wR+d.r*wR+f.r*wR+h.r*wR+e.r)*rcpWeightR);
    pixG=ASatF1((b.g*wG+d.g*wG+f.g*wG+h.g*wG+e.g)*rcpWeightG);
    pixB=ASatF1((b.b*wB+d.b*wB+f.b*wB+h.b*wB+e.b)*rcpWeightB);
   #endif
   return;
}

