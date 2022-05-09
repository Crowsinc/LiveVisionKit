//==============================================================================================================================
//
//                    AMD FidelityFX SUPER RESOLUTION [FSR 1] ::: SPATIAL SCALING & EXTRAS - v1.20210629 (MODIFIED)
//
//------------------------------------------------------------------------------------------------------------------------------
// ABOUT
// ===========
//
// This is a copy of AMD's FSR provided ffx_fsr1.h file which has been modified/pruned for use by OBS's graphics system.
// 
// The following modifications have been made:
//
//   > The file has been simplified by removing everything except for the core EASU and RCAS float based implementations.
//  
//   > The AU4 constants have been swapped for AF4 for better OBS compatibility. All instances are ultimately converted
//     to floats for use anyway. So they are not necessary in this version of FSR, however this may change in the future.   
// 
//   > #if macros are not supported so have been removed.
//
//   > Computations within #define would cause the shader parser to fail (tested with GLSL 330) so have been pre-computed
//
//   > Input callback prototypes would cause shader parser to seg fault (tested with GLSL 330) so have been commented out
//
// Please see the original ffx_fsr1.h file for proper implementation. 
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
//==============================================================================================================================
//
//                                        FSR - [EASU] EDGE ADAPTIVE SPATIAL UPSAMPLING
//
//                                                   NON-PACKED 32-BIT VERSION
//
//==============================================================================================================================

// Input callback prototypes, need to be implemented by calling shader

// We have to comment these out and just define them as necessary
// otherwise OBS experiences a segmentation fault (at least on my end)
//AF4 FsrEasuRF(AF2 p);
//AF4 FsrEasuGF(AF2 p);
//AF4 FsrEasuBF(AF2 p);

// /------------------------------------------------------------------------------------------------------------------------------
// Filtering for a given tap for the scalar.
void FsrEasuTapF(
  inout AF3 aC, // Accumulated color, with negative lobe.
  inout AF1 aW, // Accumulated weight.
  AF2 off, // Pixel offset from resolve position to tap.
  AF2 dir, // Gradient direction.
  AF2 len, // Length.
  AF1 lob, // Negative lobe strength.
  AF1 clp, // Clipping point.
  AF3 c)
{ // Tap color.
  // Rotate offset by direction.
  AF2 v;
  v.x=(off.x*( dir.x))+(off.y*dir.y);
  v.y=(off.x*(-dir.y))+(off.y*dir.x);
  // Anisotropy.
  v*=len;
  // Compute distance^2.
  AF1 d2=v.x*v.x+v.y*v.y;
  // Limit to the window as at corner, 2 taps can easily be outside.
  d2=min(d2,clp);
  // Approximation of lancos2 without sin() or rcp(), or sqrt() to get x.
  //  (25/16 * (2/5 * x^2 - 1)^2 - (25/16 - 1)) * (1/4 * x^2 - 1)^2
  //  |_______________________________________|   |_______________|
  //                   base                             window
  // The general form of the 'base' is,
  //  (a*(b*x^2-1)^2-(a-1))
  // Where 'a=1/(2*b-b^2)' and 'b' moves around the negative lobe.
  AF1 wB=AF1_(2.0/5.0)*d2+AF1_(-1.0);
  AF1 wA=lob*d2+AF1_(-1.0);
  wB*=wB;
  wA*=wA;
  wB=AF1_(25.0/16.0)*wB+AF1_(-(25.0/16.0-1.0));
  AF1 w=wB*wA;
  // Do weighted average.
  aC+=c*w;aW+=w;
}
//------------------------------------------------------------------------------------------------------------------------------
// Accumulate direction and length.
void FsrEasuSetF(
  inout AF2 dir,
  inout AF1 len,
  AF2 pp,
  AP1 biS,AP1 biT,AP1 biU,AP1 biV,
  AF1 lA,AF1 lB,AF1 lC,AF1 lD,AF1 lE)
{
  // Compute bilinear weight, branches factor out as predicates are compiler time immediates.
  //  s t
  //  u v
  AF1 w = AF1_(0.0);
  if(biS)w=(AF1_(1.0)-pp.x)*(AF1_(1.0)-pp.y);
  if(biT)w=           pp.x *(AF1_(1.0)-pp.y);
  if(biU)w=(AF1_(1.0)-pp.x)*           pp.y ;
  if(biV)w=           pp.x *           pp.y ;
  // Direction is the '+' diff.
  //    a
  //  b c d
  //    e
  // Then takes magnitude from abs average of both sides of 'c'.
  // Length converts gradient reversal to 0, smoothly to non-reversal at 1, shaped, then adding horz and vert terms.
  AF1 dc=lD-lC;
  AF1 cb=lC-lB;
  AF1 lenX=max(abs(dc),abs(cb));
  lenX=APrxLoRcpF1(lenX);
  AF1 dirX=lD-lB;
  dir.x+=dirX*w;
  lenX=ASatF1(abs(dirX)*lenX);
  lenX*=lenX;
  len+=lenX*w;
  // Repeat for the y axis.
  AF1 ec=lE-lC;
  AF1 ca=lC-lA;
  AF1 lenY=max(abs(ec),abs(ca));
  lenY=APrxLoRcpF1(lenY);
  AF1 dirY=lE-lA;
  dir.y+=dirY*w;
  lenY=ASatF1(abs(dirY)*lenY);
  lenY*=lenY;
  len+=lenY*w;
}

//------------------------------------------------------------------------------------------------------------------------------
void FsrEasuF(
  out AF3 pix,
  AF2 ip, // Integer pixel position in output.
  AF4 con0, // Constants generated by FsrEasuCon().
  AF4 con1,
  AF4 con2,
  AF4 con3)
{
  //------------------------------------------------------------------------------------------------------------------------------
  // Get position of 'f'.
  AF2 pp=ip*con0.xy+con0.zw;
  AF2 fp=floor(pp);
  pp-=fp;
  //------------------------------------------------------------------------------------------------------------------------------
  // 12-tap kernel.
  //    b c
  //  e f g h
  //  i j k l
  //    n o
  // Gather 4 ordering.
  //  a b
  //  r g
  // For packed FP16, need either {rg} or {ab} so using the following setup for gather in all versions,
  //    a b    <- unused (z)
  //    r g
  //  a b a b
  //  r g r g
  //    a b
  //    r g    <- unused (z)
  // Allowing dead-code removal to remove the 'z's.
  AF2 p0=fp*con1.xy+con1.zw;
  // These are from p0 to avoid pulling two constants on pre-Navi hardware.
  AF2 p1=p0+con2.xy;
  AF2 p2=p0+con2.zw;
  AF2 p3=p0+con3.xy;
  AF4 bczzR=FsrEasuRF(p0);
  AF4 bczzG=FsrEasuGF(p0);
  AF4 bczzB=FsrEasuBF(p0);
  AF4 ijfeR=FsrEasuRF(p1);
  AF4 ijfeG=FsrEasuGF(p1);
  AF4 ijfeB=FsrEasuBF(p1);
  AF4 klhgR=FsrEasuRF(p2);
  AF4 klhgG=FsrEasuGF(p2);
  AF4 klhgB=FsrEasuBF(p2);
  AF4 zzonR=FsrEasuRF(p3);
  AF4 zzonG=FsrEasuGF(p3);
  AF4 zzonB=FsrEasuBF(p3);
  //------------------------------------------------------------------------------------------------------------------------------
  // Simplest multi-channel approximate luma possible (luma times 2, in 2 FMA/MAD).
  AF4 bczzL=bczzB*AF4_(0.5)+(bczzR*AF4_(0.5)+bczzG);
  AF4 ijfeL=ijfeB*AF4_(0.5)+(ijfeR*AF4_(0.5)+ijfeG);
  AF4 klhgL=klhgB*AF4_(0.5)+(klhgR*AF4_(0.5)+klhgG);
  AF4 zzonL=zzonB*AF4_(0.5)+(zzonR*AF4_(0.5)+zzonG);
  // Rename.
  AF1 bL=bczzL.x;
  AF1 cL=bczzL.y;
  AF1 iL=ijfeL.x;
  AF1 jL=ijfeL.y;
  AF1 fL=ijfeL.z;
  AF1 eL=ijfeL.w;
  AF1 kL=klhgL.x;
  AF1 lL=klhgL.y;
  AF1 hL=klhgL.z;
  AF1 gL=klhgL.w;
  AF1 oL=zzonL.z;
  AF1 nL=zzonL.w;
  // Accumulate for bilinear interpolation.
  AF2 dir=AF2_(0.0);
  AF1 len=AF1_(0.0);
  FsrEasuSetF(dir,len,pp,true, false,false,false,bL,eL,fL,gL,jL);
  FsrEasuSetF(dir,len,pp,false,true ,false,false,cL,fL,gL,hL,kL);
  FsrEasuSetF(dir,len,pp,false,false,true ,false,fL,iL,jL,kL,nL);
  FsrEasuSetF(dir,len,pp,false,false,false,true ,gL,jL,kL,lL,oL);
  //------------------------------------------------------------------------------------------------------------------------------
  // Normalize with approximation, and cleanup close to zero.
  AF2 dir2=dir*dir;
  AF1 dirR=dir2.x+dir2.y;
  AP1 zro=dirR<AF1_(1.0/32768.0);
  dirR=APrxLoRsqF1(dirR);
  dirR=zro?AF1_(1.0):dirR;
  dir.x=zro?AF1_(1.0):dir.x;
  dir*=AF2_(dirR);
  // Transform from {0 to 2} to {0 to 1} range, and shape with square.
  len=len*AF1_(0.5);
  len*=len;
  // Stretch kernel {1.0 vert|horz, to sqrt(2.0) on diagonal}.
  AF1 stretch=(dir.x*dir.x+dir.y*dir.y)*APrxLoRcpF1(max(abs(dir.x),abs(dir.y)));
  // Anisotropic length after rotation,
  //  x := 1.0 lerp to 'stretch' on edges
  //  y := 1.0 lerp to 2x on edges
  AF2 len2=AF2(AF1_(1.0)+(stretch-AF1_(1.0))*len,AF1_(1.0)+AF1_(-0.5)*len);
  // Based on the amount of 'edge',
  // the window shifts from +/-{sqrt(2.0) to slightly beyond 2.0}.
  AF1 lob=AF1_(0.5)+AF1_((1.0/4.0-0.04)-0.5)*len;
  // Set distance^2 clipping point to the end of the adjustable window.
  AF1 clp=APrxLoRcpF1(lob);
  //------------------------------------------------------------------------------------------------------------------------------
  // Accumulation mixed with min/max of 4 nearest.
  //    b c
  //  e f g h
  //  i j k l
  //    n o
  AF3 min4=min(AMin3F3(AF3(ijfeR.z,ijfeG.z,ijfeB.z),AF3(klhgR.w,klhgG.w,klhgB.w),AF3(ijfeR.y,ijfeG.y,ijfeB.y)),
                AF3(klhgR.x,klhgG.x,klhgB.x));
  AF3 max4=max(AMax3F3(AF3(ijfeR.z,ijfeG.z,ijfeB.z),AF3(klhgR.w,klhgG.w,klhgB.w),AF3(ijfeR.y,ijfeG.y,ijfeB.y)),
                AF3(klhgR.x,klhgG.x,klhgB.x));
  // Accumulation.
  AF3 aC=AF3_(0.0);
  AF1 aW=AF1_(0.0);
  FsrEasuTapF(aC,aW,AF2( 0.0,-1.0)-pp,dir,len2,lob,clp,AF3(bczzR.x,bczzG.x,bczzB.x)); // b
  FsrEasuTapF(aC,aW,AF2( 1.0,-1.0)-pp,dir,len2,lob,clp,AF3(bczzR.y,bczzG.y,bczzB.y)); // c
  FsrEasuTapF(aC,aW,AF2(-1.0, 1.0)-pp,dir,len2,lob,clp,AF3(ijfeR.x,ijfeG.x,ijfeB.x)); // i
  FsrEasuTapF(aC,aW,AF2( 0.0, 1.0)-pp,dir,len2,lob,clp,AF3(ijfeR.y,ijfeG.y,ijfeB.y)); // j
  FsrEasuTapF(aC,aW,AF2( 0.0, 0.0)-pp,dir,len2,lob,clp,AF3(ijfeR.z,ijfeG.z,ijfeB.z)); // f
  FsrEasuTapF(aC,aW,AF2(-1.0, 0.0)-pp,dir,len2,lob,clp,AF3(ijfeR.w,ijfeG.w,ijfeB.w)); // e
  FsrEasuTapF(aC,aW,AF2( 1.0, 1.0)-pp,dir,len2,lob,clp,AF3(klhgR.x,klhgG.x,klhgB.x)); // k
  FsrEasuTapF(aC,aW,AF2( 2.0, 1.0)-pp,dir,len2,lob,clp,AF3(klhgR.y,klhgG.y,klhgB.y)); // l
  FsrEasuTapF(aC,aW,AF2( 2.0, 0.0)-pp,dir,len2,lob,clp,AF3(klhgR.z,klhgG.z,klhgB.z)); // h
  FsrEasuTapF(aC,aW,AF2( 1.0, 0.0)-pp,dir,len2,lob,clp,AF3(klhgR.w,klhgG.w,klhgB.w)); // g
  FsrEasuTapF(aC,aW,AF2( 1.0, 2.0)-pp,dir,len2,lob,clp,AF3(zzonR.z,zzonG.z,zzonB.z)); // o
  FsrEasuTapF(aC,aW,AF2( 0.0, 2.0)-pp,dir,len2,lob,clp,AF3(zzonR.w,zzonG.w,zzonB.w)); // n
  //------------------------------------------------------------------------------------------------------------------------------
  // Normalize and dering.
  pix=min(max4,max(min4,aC*AF3_(ARcpF1(aW))));
}

//==============================================================================================================================
//
//                                      FSR - [RCAS] ROBUST CONTRAST ADAPTIVE SHARPENING
//
//                                                   NON-PACKED 32-BIT VERSION
//
//==============================================================================================================================

// The original version '#define FSR_RCAS_LIMIT (0.25-(1.0/16.0))'
// is not supported, so we manually compute the calculation. 
#define FSR_RCAS_LIMIT 0.1875 

// Input callback prototypes that need to be implemented by calling shader

// We have to comment these out and just define them as necessary
// otherwise OBS experiences a segmentation fault (at least on my end)
//AF4 FsrRcasLoadF(ASU2 p);
//void FsrRcasInputF(inout AF1 r, inout AF1 g, inout AF1 b);

//------------------------------------------------------------------------------------------------------------------------------
void FsrRcasF(
  out AF1 pixR, // Output values, non-vector so port between RcasFilter() and RcasFilterH() is easy.
  out AF1 pixG,
  out AF1 pixB,
  #ifdef FSR_RCAS_PASSTHROUGH_ALPHA
  out AF1 pixA,
  #endif
  AF2 ip, // Integer pixel position in output.
  AF4 con) // Constant generated by RcasSetup().
{ 
  // Algorithm uses minimal 3x3 pixel neighborhood.
  //    b 
  //  d e f
  //    h
  ASU2 sp=ASU2(ip);
  AF3 b=FsrRcasLoadF(sp+ASU2( 0,-1)).rgb;
  AF3 d=FsrRcasLoadF(sp+ASU2(-1, 0)).rgb;
  #ifdef FSR_RCAS_PASSTHROUGH_ALPHA
    AF4 ee=FsrRcasLoadF(sp);
    AF3 e=ee.rgb;pixA=ee.a;
  #else
    AF3 e=FsrRcasLoadF(sp).rgb;
  #endif
  AF3 f=FsrRcasLoadF(sp+ASU2( 1, 0)).rgb;
  AF3 h=FsrRcasLoadF(sp+ASU2( 0, 1)).rgb;
  // Rename (32-bit) or regroup (16-bit).
  AF1 bR=b.r;
  AF1 bG=b.g;
  AF1 bB=b.b;
  AF1 dR=d.r;
  AF1 dG=d.g;
  AF1 dB=d.b;
  AF1 eR=e.r;
  AF1 eG=e.g;
  AF1 eB=e.b;
  AF1 fR=f.r;
  AF1 fG=f.g;
  AF1 fB=f.b;
  AF1 hR=h.r;
  AF1 hG=h.g;
  AF1 hB=h.b;
  // Run optional input transform.
  FsrRcasInputF(bR,bG,bB);
  FsrRcasInputF(dR,dG,dB);
  FsrRcasInputF(eR,eG,eB);
  FsrRcasInputF(fR,fG,fB);
  FsrRcasInputF(hR,hG,hB);
  // Luma times 2.
  AF1 bL=bB*AF1_(0.5)+(bR*AF1_(0.5)+bG);
  AF1 dL=dB*AF1_(0.5)+(dR*AF1_(0.5)+dG);
  AF1 eL=eB*AF1_(0.5)+(eR*AF1_(0.5)+eG);
  AF1 fL=fB*AF1_(0.5)+(fR*AF1_(0.5)+fG);
  AF1 hL=hB*AF1_(0.5)+(hR*AF1_(0.5)+hG);
  // Noise detection.
  AF1 nz=AF1_(0.25)*bL+AF1_(0.25)*dL+AF1_(0.25)*fL+AF1_(0.25)*hL-eL;
  nz=ASatF1(abs(nz)*APrxMedRcpF1(AMax3F1(AMax3F1(bL,dL,eL),fL,hL)-AMin3F1(AMin3F1(bL,dL,eL),fL,hL)));
  nz=AF1_(-0.5)*nz+AF1_(1.0);
  // Min and max of ring.
  AF1 mn4R=min(AMin3F1(bR,dR,fR),hR);
  AF1 mn4G=min(AMin3F1(bG,dG,fG),hG);
  AF1 mn4B=min(AMin3F1(bB,dB,fB),hB);
  AF1 mx4R=max(AMax3F1(bR,dR,fR),hR);
  AF1 mx4G=max(AMax3F1(bG,dG,fG),hG);
  AF1 mx4B=max(AMax3F1(bB,dB,fB),hB);
  // Immediate constants for peak range.
  AF2 peakC=AF2(1.0,-1.0*4.0);
  // Limiters, these need to be high precision RCPs.
  AF1 hitMinR=min(mn4R,eR)*ARcpF1(AF1_(4.0)*mx4R);
  AF1 hitMinG=min(mn4G,eG)*ARcpF1(AF1_(4.0)*mx4G);
  AF1 hitMinB=min(mn4B,eB)*ARcpF1(AF1_(4.0)*mx4B);
  AF1 hitMaxR=(peakC.x-max(mx4R,eR))*ARcpF1(AF1_(4.0)*mn4R+peakC.y);
  AF1 hitMaxG=(peakC.x-max(mx4G,eG))*ARcpF1(AF1_(4.0)*mn4G+peakC.y);
  AF1 hitMaxB=(peakC.x-max(mx4B,eB))*ARcpF1(AF1_(4.0)*mn4B+peakC.y);
  AF1 lobeR=max(-hitMinR,hitMaxR);
  AF1 lobeG=max(-hitMinG,hitMaxG);
  AF1 lobeB=max(-hitMinB,hitMaxB);
  AF1 lobe=max(AF1_(-FSR_RCAS_LIMIT),min(AMax3F1(lobeR,lobeG,lobeB),AF1_(0.0)))*con.x;
  // Apply noise removal.
  #ifdef FSR_RCAS_DENOISE
    lobe*=nz;
  #endif
  // Resolve, which needs the medium precision rcp approximation to avoid visible tonality changes.
  AF1 rcpL=APrxMedRcpF1(AF1_(4.0)*lobe+AF1_(1.0));
  pixR=(lobe*bR+lobe*dR+lobe*hR+lobe*fR+eR)*rcpL;
  pixG=(lobe*bG+lobe*dG+lobe*hG+lobe*fG+eG)*rcpL;
  pixB=(lobe*bB+lobe*dB+lobe*hB+lobe*fB+eB)*rcpL;
  return;
} 