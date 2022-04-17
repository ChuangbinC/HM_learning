/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2020, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     TEncRateCtrl.cpp
    \brief    Rate control manager class
*/
#include "TEncRateCtrl.h"
#include "../TLibCommon/TComPic.h"
#include "../TLibCommon/TComChromaFormat.h"

#include <cmath>

using namespace std;

//sequence level
TEncRCSeq::TEncRCSeq()
{
  m_totalFrames         = 0;
  m_targetRate          = 0;
  m_frameRate           = 0;
  m_targetBits          = 0;
  m_GOPSize             = 0;
  m_picWidth            = 0;
  m_picHeight           = 0;
  m_LCUWidth            = 0;
  m_LCUHeight           = 0;
  m_numberOfLevel       = 0;
  m_numberOfLCU         = 0;
  m_averageBits         = 0;
  m_bitsRatio           = NULL;
  m_GOPID2Level         = NULL;
  m_picPara             = NULL;
  m_LCUPara             = NULL;
  m_numberOfPixel       = 0;
  m_framesLeft          = 0;
  m_bitsLeft            = 0;
  m_useLCUSeparateModel = false;
  m_adaptiveBit         = 0;
  m_lastLambda          = 0.0;
}

TEncRCSeq::~TEncRCSeq()
{
  destroy();
}

Void TEncRCSeq::create( Int totalFrames, Int targetBitrate, Int frameRate, Int GOPSize, Int picWidth, Int picHeight, Int LCUWidth, Int LCUHeight, Int numberOfLevel, Bool useLCUSeparateModel, Int adaptiveBit )
{
  destroy();
  m_totalFrames         = totalFrames;
  m_targetRate          = targetBitrate;
  m_frameRate           = frameRate;
  m_GOPSize             = GOPSize;
  m_picWidth            = picWidth;
  m_picHeight           = picHeight;
  m_LCUWidth            = LCUWidth;
  m_LCUHeight           = LCUHeight;
  m_numberOfLevel       = numberOfLevel;
  m_useLCUSeparateModel = useLCUSeparateModel;

  m_numberOfPixel   = m_picWidth * m_picHeight;
  m_targetBits      = (Int64)m_totalFrames * (Int64)m_targetRate / (Int64)m_frameRate; // 序列总码率（输出码流总大小）
  m_seqTargetBpp = (Double)m_targetRate / (Double)m_frameRate / (Double)m_numberOfPixel; //每个像素被分配的目标比特
  if ( m_seqTargetBpp < 0.03 )
  {
    m_alphaUpdate = 0.01;
    m_betaUpdate  = 0.005;
  }
  else if ( m_seqTargetBpp < 0.08 )
  {
    m_alphaUpdate = 0.05;
    m_betaUpdate  = 0.025;
  }
  else if ( m_seqTargetBpp < 0.2 )
  {
    m_alphaUpdate = 0.1;
    m_betaUpdate  = 0.05;
  }
  else if ( m_seqTargetBpp < 0.5 )
  {
    m_alphaUpdate = 0.2;
    m_betaUpdate  = 0.1;
  }
  else
  {
    m_alphaUpdate = 0.4;
    m_betaUpdate  = 0.2;
  }

  m_averageBits     = (Int)(m_targetBits / totalFrames); //!< 平均每帧占用的比特数
  Int picWidthInBU  = ( m_picWidth  % m_LCUWidth  ) == 0 ? m_picWidth  / m_LCUWidth  : m_picWidth  / m_LCUWidth  + 1;
  Int picHeightInBU = ( m_picHeight % m_LCUHeight ) == 0 ? m_picHeight / m_LCUHeight : m_picHeight / m_LCUHeight + 1;
  m_numberOfLCU     = picWidthInBU * picHeightInBU; //!< 一帧picture中包含的LCU数目

  m_bitsRatio   = new Int[m_GOPSize];
  for ( Int i=0; i<m_GOPSize; i++ )
  {
    m_bitsRatio[i] = 1;
  }

  m_GOPID2Level = new Int[m_GOPSize];
  for ( Int i=0; i<m_GOPSize; i++ )
  {
    m_GOPID2Level[i] = 1;
  }

  m_picPara = new TRCParameter[m_numberOfLevel];
  for ( Int i=0; i<m_numberOfLevel; i++ )
  {
    m_picPara[i].m_alpha = 0.0;
    m_picPara[i].m_beta  = 0.0;
#if JVET_K0390_RATE_CTRL
    m_picPara[i].m_validPix = -1;
#endif
#if JVET_M0600_RATE_CTRL
    m_picPara[i].m_skipRatio = 0.0;
#endif
  }

  if ( m_useLCUSeparateModel ) //!< 是否每个LCU的alpha和beta都有各自的值
  {
    m_LCUPara = new TRCParameter*[m_numberOfLevel];
    for ( Int i=0; i<m_numberOfLevel; i++ )
    {
      m_LCUPara[i] = new TRCParameter[m_numberOfLCU];
      for ( Int j=0; j<m_numberOfLCU; j++)
      {
        m_LCUPara[i][j].m_alpha = 0.0;
        m_LCUPara[i][j].m_beta  = 0.0;
#if JVET_K0390_RATE_CTRL
        m_LCUPara[i][j].m_validPix = -1;
#endif
#if JVET_M0600_RATE_CTRL
        m_LCUPara[i][j].m_skipRatio = 0.0;
#endif
      }
    }
  }

  m_framesLeft = m_totalFrames; //!< 剩余的待编码帧数
  m_bitsLeft   = m_targetBits; //!< 剩余可用的比特数
  m_adaptiveBit = adaptiveBit;
  m_lastLambda = 0.0;
}

Void TEncRCSeq::destroy()
{
  if (m_bitsRatio != NULL)
  {
    delete[] m_bitsRatio;
    m_bitsRatio = NULL;
  }

  if ( m_GOPID2Level != NULL )
  {
    delete[] m_GOPID2Level;
    m_GOPID2Level = NULL;
  }

  if ( m_picPara != NULL )
  {
    delete[] m_picPara;
    m_picPara = NULL;
  }

  if ( m_LCUPara != NULL )
  {
    for ( Int i=0; i<m_numberOfLevel; i++ )
    {
      delete[] m_LCUPara[i];
    }
    delete[] m_LCUPara;
    m_LCUPara = NULL;
  }
}

Void TEncRCSeq::initBitsRatio( Int bitsRatio[])
{
  for (Int i=0; i<m_GOPSize; i++)
  {
    m_bitsRatio[i] = bitsRatio[i];
  }
}

Void TEncRCSeq::initGOPID2Level( Int GOPID2Level[] )
{
  for ( Int i=0; i<m_GOPSize; i++ )
  {
    m_GOPID2Level[i] = GOPID2Level[i];
  }
}

Void TEncRCSeq::initPicPara( TRCParameter* picPara )
{
  assert( m_picPara != NULL );

  if ( picPara == NULL )
  {
    for ( Int i=0; i<m_numberOfLevel; i++ )
    {
      if (i>0)
      {
        m_picPara[i].m_alpha = 3.2003;
        m_picPara[i].m_beta  = -1.367;
      }
      else
      {
        m_picPara[i].m_alpha = ALPHA;
        m_picPara[i].m_beta  = BETA2;
      }
    }
  }
  else
  {
    for ( Int i=0; i<m_numberOfLevel; i++ )
    {
      m_picPara[i] = picPara[i];
    }
  }
}

Void TEncRCSeq::initLCUPara( TRCParameter** LCUPara )
{
  if ( m_LCUPara == NULL )
  {
    return;
  }
  if ( LCUPara == NULL )
  {
    for ( Int i=0; i<m_numberOfLevel; i++ )
    {
      for ( Int j=0; j<m_numberOfLCU; j++)
      {
        m_LCUPara[i][j].m_alpha = m_picPara[i].m_alpha;
        m_LCUPara[i][j].m_beta  = m_picPara[i].m_beta;
      }
    }
  }
  else
  {
    for ( Int i=0; i<m_numberOfLevel; i++ )
    {
      for ( Int j=0; j<m_numberOfLCU; j++)
      {
        m_LCUPara[i][j] = LCUPara[i][j];
      }
    }
  }
}

Void TEncRCSeq::updateAfterPic ( Int bits )
{
  m_bitsLeft -= bits;
  m_framesLeft--;
}

Void TEncRCSeq::setAllBitRatio( Double basicLambda, Double* equaCoeffA, Double* equaCoeffB )
{
  Int* bitsRatio = new Int[m_GOPSize]; //新建数组计算最终的比特率
  for ( Int i=0; i<m_GOPSize; i++ )
  {
#if JVET_K0390_RATE_CTRL
    bitsRatio[i] = (Int)( equaCoeffA[i] * pow(basicLambda, equaCoeffB[i]) * (Double)getPicPara(getGOPID2Level(i)).m_validPix); //第i帧的比特率=Bpp*总像素数
#else
    bitsRatio[i] = (Int)( equaCoeffA[i] * pow( basicLambda, equaCoeffB[i] ) * m_numberOfPixel );
#endif
  }
  initBitsRatio( bitsRatio ); //遍历整个GOP，利用 m_bitsRatio[i] = bitsRatio[i]对 m_bitsRatio进行初始化
  delete[] bitsRatio;
}

//GOP level
TEncRCGOP::TEncRCGOP()
{
  m_encRCSeq  = NULL;
  m_picTargetBitInGOP = NULL;
  m_numPic     = 0;
  m_targetBits = 0;
  m_picLeft    = 0;
  m_bitsLeft   = 0;
}

TEncRCGOP::~TEncRCGOP()
{
  destroy();
}

Void TEncRCGOP::create( TEncRCSeq* encRCSeq, Int numPic )
{
  destroy();
  Int targetBits = xEstGOPTargetBits( encRCSeq, numPic ); //获得GOP层的目标总比特数；

  if ( encRCSeq->getAdaptiveBits() > 0 && encRCSeq->getLastLambda() > 0.1 ) //采用自适应比特分配并且序列的最后一个lambda的值大于0.1
  {
    Double targetBpp = (Double)targetBits / encRCSeq->getNumPixel(); //得到目标bpp
    Double basicLambda = 0.0;
    Double* lambdaRatio = new Double[encRCSeq->getGOPSize()]; //分别新建存储lambda、alpha、beta 的值
    Double* equaCoeffA = new Double[encRCSeq->getGOPSize()];
    Double* equaCoeffB = new Double[encRCSeq->getGOPSize()];

    // 在lowdelay模式下， encRCSeq->getAdaptiveBits() == 1 
    if ( encRCSeq->getAdaptiveBits() == 1 )   // for GOP size =4, low delay case
    {
      if ( encRCSeq->getLastLambda() < 120.0 ) //对每一帧的lambda的权值进行赋值
      {
        lambdaRatio[1] = 0.725 * log( encRCSeq->getLastLambda() ) + 0.5793;
        lambdaRatio[0] = 1.3 * lambdaRatio[1];
        lambdaRatio[2] = 1.3 * lambdaRatio[1];
        lambdaRatio[3] = 1.0;
      }
      else
      {
        lambdaRatio[0] = 5.0;
        lambdaRatio[1] = 4.0;
        lambdaRatio[2] = 5.0;
        lambdaRatio[3] = 1.0;
      }
    }
    // 在random access模式下， encRCSeq->getAdaptiveBits() == 2 
    else if ( encRCSeq->getAdaptiveBits() == 2 )  // for GOP size = 8, random access case 
    {
      if ( encRCSeq->getLastLambda() < 90.0 )
      {
        lambdaRatio[0] = 1.0;
        lambdaRatio[1] = 0.725 * log( encRCSeq->getLastLambda() ) + 0.7963;
        lambdaRatio[2] = 1.3 * lambdaRatio[1];
        lambdaRatio[3] = 3.25 * lambdaRatio[1];
        lambdaRatio[4] = 3.25 * lambdaRatio[1];
        lambdaRatio[5] = 1.3  * lambdaRatio[1];
        lambdaRatio[6] = 3.25 * lambdaRatio[1];
        lambdaRatio[7] = 3.25 * lambdaRatio[1];
      }
      else
      {
        lambdaRatio[0] = 1.0;
        lambdaRatio[1] = 4.0;
        lambdaRatio[2] = 5.0;
        lambdaRatio[3] = 12.3;
        lambdaRatio[4] = 12.3;
        lambdaRatio[5] = 5.0;
        lambdaRatio[6] = 12.3;
        lambdaRatio[7] = 12.3;
      }
    }
#if JVET_K0390_RATE_CTRL
    else if (encRCSeq->getAdaptiveBits() == 3)  // for GOP size = 16, random access case
    {
      Double hierarQp = 4.2005 * log(encRCSeq->getLastLambda()) + 13.7122;  //  the qp of POC16
      Double qpLev2 = (hierarQp + 0.0) + 0.2016    * (hierarQp + 0.0) - 4.8848;
      Double qpLev3 = (hierarQp + 3.0) + 0.22286 * (hierarQp + 3.0) - 5.7476;
      Double qpLev4 = (hierarQp + 4.0) + 0.2333    * (hierarQp + 4.0) - 5.9;
      Double qpLev5 = (hierarQp + 5.0) + 0.3            * (hierarQp + 5.0) - 7.1444;

      Double lambdaLev1 = exp((hierarQp - 13.7122) / 4.2005);
      Double lambdaLev2 = exp((qpLev2 - 13.7122) / 4.2005);
      Double lambdaLev3 = exp((qpLev3 - 13.7122) / 4.2005);
      Double lambdaLev4 = exp((qpLev4 - 13.7122) / 4.2005);
      Double lambdaLev5 = exp((qpLev5 - 13.7122) / 4.2005);

      lambdaRatio[0] = 1.0;
      lambdaRatio[1] = lambdaLev2 / lambdaLev1;
      lambdaRatio[2] = lambdaLev3 / lambdaLev1;
      lambdaRatio[3] = lambdaLev4 / lambdaLev1;
      lambdaRatio[4] = lambdaLev5 / lambdaLev1;
      lambdaRatio[5] = lambdaLev5 / lambdaLev1;
      lambdaRatio[6] = lambdaLev4 / lambdaLev1;
      lambdaRatio[7] = lambdaLev5 / lambdaLev1;
      lambdaRatio[8] = lambdaLev5 / lambdaLev1;
      lambdaRatio[9] = lambdaLev3 / lambdaLev1;
      lambdaRatio[10] = lambdaLev4 / lambdaLev1;
      lambdaRatio[11] = lambdaLev5 / lambdaLev1;
      lambdaRatio[12] = lambdaLev5 / lambdaLev1;
      lambdaRatio[13] = lambdaLev4 / lambdaLev1;
      lambdaRatio[14] = lambdaLev5 / lambdaLev1;
      lambdaRatio[15] = lambdaLev5 / lambdaLev1;
#if JVET_M0600_RATE_CTRL
      const Double qdfParaLev2A = 0.5847;
      const Double qdfParaLev2B = -0.0782;
      const Double qdfParaLev3A = 0.5468;
      const Double qdfParaLev3B = -0.1364;
      const Double qdfParaLev4A = 0.6539;
      const Double qdfParaLev4B = -0.203;
      const Double qdfParaLev5A = 0.8623;
      const Double qdfParaLev5B = -0.4676;
      Double qdfLev1Lev2 = Clip3(0.12, 0.9, qdfParaLev2A * encRCSeq->getPicPara(2).m_skipRatio + qdfParaLev2B);
      Double qdfLev1Lev3 = Clip3(0.13, 0.9, qdfParaLev3A * encRCSeq->getPicPara(3).m_skipRatio + qdfParaLev3B);
      Double qdfLev1Lev4 = Clip3(0.15, 0.9, qdfParaLev4A * encRCSeq->getPicPara(4).m_skipRatio + qdfParaLev4B);
      Double qdfLev1Lev5 = Clip3(0.20, 0.9, qdfParaLev5A * encRCSeq->getPicPara(5).m_skipRatio + qdfParaLev5B);
      Double qdfLev2Lev3 = Clip3(0.09, 0.9, qdfLev1Lev3 * (1 - qdfLev1Lev2));
      Double qdfLev2Lev4 = Clip3(0.12, 0.9, qdfLev1Lev4 * (1 - qdfLev1Lev2));
      Double qdfLev2Lev5 = Clip3(0.14, 0.9, qdfLev1Lev5 * (1 - qdfLev1Lev2));
      Double qdfLev3Lev4 = Clip3(0.06, 0.9, qdfLev1Lev4 * (1 - qdfLev1Lev3));
      Double qdfLev3Lev5 = Clip3(0.09, 0.9, qdfLev1Lev5 * (1 - qdfLev1Lev3));
      Double qdfLev4Lev5 = Clip3(0.10, 0.9, qdfLev1Lev5 * (1 - qdfLev1Lev4));

      lambdaLev1 = 1 / (1 + 2 * (qdfLev1Lev2 + 2 * qdfLev1Lev3 + 4 * qdfLev1Lev4 + 8 * qdfLev1Lev5));
      lambdaLev2 = 1 / (1 + (3 * qdfLev2Lev3 + 5 * qdfLev2Lev4 + 8 * qdfLev2Lev5));
      lambdaLev3 = 1 / (1 + 2 * qdfLev3Lev4 + 4 * qdfLev3Lev5);
      lambdaLev4 = 1 / (1 + 2 * qdfLev4Lev5);
      lambdaLev5 = 1 / (1.0);

      lambdaRatio[0] = 1.0;
      lambdaRatio[1] = lambdaLev2 / lambdaLev1;
      lambdaRatio[2] = lambdaLev3 / lambdaLev1;
      lambdaRatio[3] = lambdaLev4 / lambdaLev1;
      lambdaRatio[4] = lambdaLev5 / lambdaLev1;
      lambdaRatio[5] = lambdaLev5 / lambdaLev1;
      lambdaRatio[6] = lambdaLev4 / lambdaLev1;
      lambdaRatio[7] = lambdaLev5 / lambdaLev1;
      lambdaRatio[8] = lambdaLev5 / lambdaLev1;
      lambdaRatio[9] = lambdaLev3 / lambdaLev1;
      lambdaRatio[10] = lambdaLev4 / lambdaLev1;
      lambdaRatio[11] = lambdaLev5 / lambdaLev1;
      lambdaRatio[12] = lambdaLev5 / lambdaLev1;
      lambdaRatio[13] = lambdaLev4 / lambdaLev1;
      lambdaRatio[14] = lambdaLev5 / lambdaLev1;
      lambdaRatio[15] = lambdaLev5 / lambdaLev1;
#endif
    }
#endif
    xCalEquaCoeff( encRCSeq, lambdaRatio, equaCoeffA, equaCoeffB, encRCSeq->getGOPSize() );
#if JVET_K0390_RATE_CTRL
    basicLambda = xSolveEqua(encRCSeq, targetBpp, equaCoeffA, equaCoeffB, encRCSeq->getGOPSize());
#else
    basicLambda = xSolveEqua( targetBpp, equaCoeffA, equaCoeffB, encRCSeq->getGOPSize() );
#endif
    encRCSeq->setAllBitRatio( basicLambda, equaCoeffA, equaCoeffB );

    delete []lambdaRatio;
    delete []equaCoeffA;
    delete []equaCoeffB;
  }

  //GOP中的图片目标比特
  m_picTargetBitInGOP = new Int[numPic];
  Int i;
  Int totalPicRatio = 0;
  Int currPicRatio = 0;
  for ( i=0; i<numPic; i++ ) //遍历整个GOP
  {
    totalPicRatio += encRCSeq->getBitRatio( i ); //得到整个GOP所有图片的比特数
  }
  for ( i=0; i<numPic; i++ )
  {
    currPicRatio = encRCSeq->getBitRatio( i ); //得到当前图像的比特率
    m_picTargetBitInGOP[i] = (Int)( ((Double)targetBits) * currPicRatio / totalPicRatio ); //得到当前GOP中的每一帧的目标比特
  }

  m_encRCSeq    = encRCSeq;
  m_numPic       = numPic;
  m_targetBits   = targetBits;
  m_picLeft      = m_numPic;
  m_bitsLeft     = m_targetBits;
}

Void TEncRCGOP::xCalEquaCoeff( TEncRCSeq* encRCSeq, Double* lambdaRatio, Double* equaCoeffA, Double* equaCoeffB, Int GOPSize )
{
  for ( Int i=0; i<GOPSize; i++ )
  {
    Int frameLevel = encRCSeq->getGOPID2Level(i); //调用映射函数得到帧所对应的level
    Double alpha   = encRCSeq->getPicPara(frameLevel).m_alpha; //根据framelevel调用getPicPara（）函数来得到对应的alpha和beta
    Double beta    = encRCSeq->getPicPara(frameLevel).m_beta;
    equaCoeffA[i] = pow( 1.0/alpha, 1.0/beta ) * pow( lambdaRatio[i], 1.0/beta ); //根据alpha和beta计算系数A和B
    equaCoeffB[i] = 1.0/beta;
  }
}

#if JVET_K0390_RATE_CTRL
Double TEncRCGOP::xSolveEqua(TEncRCSeq* encRCSeq, Double targetBpp, Double* equaCoeffA, Double* equaCoeffB, Int GOPSize)
#else
Double TEncRCGOP::xSolveEqua( Double targetBpp, Double* equaCoeffA, Double* equaCoeffB, Int GOPSize )
#endif
{
  Double solution = 100.0;
  Double minNumber = 0.1;
  Double maxNumber = 10000.0;
  for ( Int i=0; i<g_RCIterationNum; i++ ) //const Int g_RCIterationNum = 20;循环20次
  {
    Double fx = 0.0;
    for ( Int j=0; j<GOPSize; j++ )
    {
#if JVET_K0390_RATE_CTRL
      Double tmpBpp = equaCoeffA[j] * pow(solution, equaCoeffB[j]); 
      Double actualBpp = tmpBpp * (Double)encRCSeq->getPicPara(encRCSeq->getGOPID2Level(j)).m_validPix / (Double)encRCSeq->getNumPixel();
      fx += actualBpp;
#else
      fx += equaCoeffA[j] * pow( solution, equaCoeffB[j] ); //fx+=系数A*lambda的系数B次方，累加过后为一个GOP的计算Bpp
#endif
    }

    if ( fabs( fx - targetBpp ) < 0.000001 ) //若fx - targetBpp的绝对值小于0.000001，满足精度跳出循环
    {
      break;
    }

    if ( fx > targetBpp ) //若fx大于targetBpp，将solution赋给min，并从solution—max进行二分赋给solution
    {
      minNumber = solution;
      solution = ( solution + maxNumber ) / 2.0;
    }
    else //若fx小于targetBpp，将solution赋给max，并从min—solution进行二分赋给solution
    {
      maxNumber = solution; 
      solution = ( solution + minNumber ) / 2.0;
    }
  }

  solution = Clip3( 0.1, 10000.0, solution ); //最后结果为min（10000，max（0.1，solution））
  return solution; //返回solution作为lambda的起始值
}

Void TEncRCGOP::destroy()
{
  m_encRCSeq = NULL;
  if ( m_picTargetBitInGOP != NULL )
  {
    delete[] m_picTargetBitInGOP;
    m_picTargetBitInGOP = NULL;
  }
}

Void TEncRCGOP::updateAfterPicture( Int bitsCost )
{
  m_bitsLeft -= bitsCost;
  m_picLeft--;
}

//传递进来码控序列和GOP的大小即一个GOP中的帧数
Int TEncRCGOP::xEstGOPTargetBits( TEncRCSeq* encRCSeq, Int GOPSize )
{
  Int realInfluencePicture = min( g_RCSmoothWindowSize, encRCSeq->getFramesLeft() ); //获得实际的平滑窗口大小
  Int averageTargetBitsPerPic = (Int)( encRCSeq->getTargetBits() / encRCSeq->getTotalFrames() ); //获得平均每帧的目标比特
  Int currentTargetBitsPerPic = (Int)( ( encRCSeq->getBitsLeft() - averageTargetBitsPerPic * (encRCSeq->getFramesLeft() - realInfluencePicture) ) / realInfluencePicture );

  //当前每帧的目标比特=（视频序列剩余比特数-平均每帧目标比特*（未编码帧数-平滑窗口SW））/平滑窗口SW
  Int targetBits = currentTargetBitsPerPic * GOPSize; //当前GOP的目标比特数=当前每帧的目标比特数*一个GOP中的帧数

  // GOP每个GOP至少分配200个比特
  if ( targetBits < 200 )
  {
    targetBits = 200;   // at least allocate 200 bits for one GOP
  }

  return targetBits;
}

//picture level
TEncRCPic::TEncRCPic()
{
  m_encRCSeq = NULL;
  m_encRCGOP = NULL;

  m_frameLevel    = 0;
  m_numberOfPixel = 0;
  m_numberOfLCU   = 0;
  m_targetBits    = 0;
  m_estHeaderBits = 0;
  m_estPicQP      = 0;
  m_estPicLambda  = 0.0;

  m_LCULeft       = 0;
  m_bitsLeft      = 0;
  m_pixelsLeft    = 0;

  m_LCUs         = NULL;
  m_picActualHeaderBits = 0;
  m_picActualBits       = 0;
  m_picQP               = 0;
  m_picLambda           = 0.0;
#if JVET_K0390_RATE_CTRL
  m_picMSE = 0.0;
  m_validPixelsInPic = 0;
#endif
}

TEncRCPic::~TEncRCPic()
{
  destroy();
}

Int TEncRCPic::xEstPicTargetBits( TEncRCSeq* encRCSeq, TEncRCGOP* encRCGOP )
{
  Int targetBits        = 0;
  Int GOPbitsLeft       = encRCGOP->getBitsLeft();

  Int i;
  Int currPicPosition = encRCGOP->getNumPic()-encRCGOP->getPicLeft(); //总的帧数-剩余未编码的帧数=当前帧在序列中的位置
  Int currPicRatio    = encRCSeq->getBitRatio( currPicPosition ); //利用当前帧的位置求得当前帧的比特率
  Int totalPicRatio   = 0;
  for ( i=currPicPosition; i<encRCGOP->getNumPic(); i++ ) // 从当前帧位置进行遍历
  {
    totalPicRatio += encRCSeq->getBitRatio( i ); //所有帧的比特率之和等于剩余未编码帧的比特率之和
  }

  targetBits  = Int( ((Double)GOPbitsLeft) * currPicRatio / totalPicRatio ); //求出当前帧的目标比特=GOP剩余比特*（当前帧比特率/全部帧比特率）

  if ( targetBits < 100 )
  {
    targetBits = 100;   // at least allocate 100 bits for one picture
  }

  if ( m_encRCSeq->getFramesLeft() > 16 ) //如果剩余未编码帧的数目大于16
  {
    //目标比特=0.1*目标比特+0.9*当前帧在GOP中的目标比特
    targetBits = Int( g_RCWeightPicRargetBitInBuffer * targetBits + g_RCWeightPicTargetBitInGOP * m_encRCGOP->getTargetBitInGOP( currPicPosition ) );
  }

  return targetBits;
}

Int TEncRCPic::xEstPicHeaderBits( list<TEncRCPic*>& listPreviousPictures, Int frameLevel )
{
  Int numPreviousPics   = 0;
  Int totalPreviousBits = 0;

  list<TEncRCPic*>::iterator it;
  for ( it = listPreviousPictures.begin(); it != listPreviousPictures.end(); it++ )
  {
    if ( (*it)->getFrameLevel() == frameLevel ) //如果已编码帧和当前帧为同一个level
    {
      //将已编码帧的实际头文件比特数进行累加得到totalPreviousBits
      totalPreviousBits += (*it)->getPicActualHeaderBits();
      numPreviousPics++;
    }
  }

  Int estHeaderBits = 0;

  //如果不是第一帧
  if ( numPreviousPics > 0 )
  {
    //当前帧的头文件预测比特=已经编码帧的头文件比特总数/已经编码的帧数
    estHeaderBits = totalPreviousBits / numPreviousPics;
  }

  return estHeaderBits;
}

Int TEncRCPic::xEstPicLowerBound(TEncRCSeq* encRCSeq, TEncRCGOP* encRCGOP)
{
  Int lowerBound = 0;
  Int GOPbitsLeft = encRCGOP->getBitsLeft();

  const Int nextPicPosition = (encRCGOP->getNumPic() - encRCGOP->getPicLeft() + 1) % encRCGOP->getNumPic();
  const Int nextPicRatio = encRCSeq->getBitRatio(nextPicPosition);

  Int totalPicRatio = 0;
  for (Int i = nextPicPosition; i < encRCGOP->getNumPic(); i++)
  {
    totalPicRatio += encRCSeq->getBitRatio(i);
  }

  if (nextPicPosition == 0)
  {
    GOPbitsLeft = encRCGOP->getTargetBits();
  }
  else
  {
    GOPbitsLeft -= m_targetBits;
  }

  lowerBound = Int(((Double)GOPbitsLeft) * nextPicRatio / totalPicRatio);

  if (lowerBound < 100)
  {
    lowerBound = 100;   // at least allocate 100 bits for one picture
  }

  if (m_encRCSeq->getFramesLeft() > 16)
  {
    lowerBound = Int(g_RCWeightPicRargetBitInBuffer * lowerBound + g_RCWeightPicTargetBitInGOP * m_encRCGOP->getTargetBitInGOP(nextPicPosition));
  }

  return lowerBound;
}

Void TEncRCPic::addToPictureLsit( list<TEncRCPic*>& listPreviousPictures )
{
  if ( listPreviousPictures.size() > g_RCMaxPicListSize )
  {
    TEncRCPic* p = listPreviousPictures.front();
    listPreviousPictures.pop_front();
    p->destroy();
    delete p;
  }

  listPreviousPictures.push_back( this );
}

Void TEncRCPic::create( TEncRCSeq* encRCSeq, TEncRCGOP* encRCGOP, Int frameLevel, list<TEncRCPic*>& listPreviousPictures )
{
  destroy();
  m_encRCSeq = encRCSeq;
  m_encRCGOP = encRCGOP;

  Int targetBits    = xEstPicTargetBits( encRCSeq, encRCGOP );
  Int estHeaderBits = xEstPicHeaderBits( listPreviousPictures, frameLevel );

  if ( targetBits < estHeaderBits + 100 )
  {
    targetBits = estHeaderBits + 100;   // at least allocate 100 bits for picture data
  }

  m_frameLevel       = frameLevel;
  m_numberOfPixel    = encRCSeq->getNumPixel();
  m_numberOfLCU      = encRCSeq->getNumberOfLCU();
  m_estPicLambda     = 100.0;
  m_targetBits       = targetBits;
  m_estHeaderBits    = estHeaderBits;
  m_bitsLeft         = m_targetBits;
  Int picWidth       = encRCSeq->getPicWidth();
  Int picHeight      = encRCSeq->getPicHeight();
  Int LCUWidth       = encRCSeq->getLCUWidth();
  Int LCUHeight      = encRCSeq->getLCUHeight();
  Int picWidthInLCU  = ( picWidth  % LCUWidth  ) == 0 ? picWidth  / LCUWidth  : picWidth  / LCUWidth  + 1;
  Int picHeightInLCU = ( picHeight % LCUHeight ) == 0 ? picHeight / LCUHeight : picHeight / LCUHeight + 1;
  m_lowerBound       = xEstPicLowerBound( encRCSeq, encRCGOP );

  m_LCULeft         = m_numberOfLCU;
  m_bitsLeft       -= m_estHeaderBits;
  m_pixelsLeft      = m_numberOfPixel;

  m_LCUs           = new TRCLCU[m_numberOfLCU];
  Int i, j;
  Int LCUIdx;
  for ( i=0; i<picWidthInLCU; i++ )
  {
    for ( j=0; j<picHeightInLCU; j++ )
    {
      LCUIdx = j*picWidthInLCU + i;
      m_LCUs[LCUIdx].m_actualBits = 0;
#if JVET_K0390_RATE_CTRL
      m_LCUs[LCUIdx].m_actualSSE = 0.0;
      m_LCUs[LCUIdx].m_actualMSE = 0.0;
#endif
      m_LCUs[LCUIdx].m_QP         = 0;
      m_LCUs[LCUIdx].m_lambda     = 0.0;
      m_LCUs[LCUIdx].m_targetBits = 0;
      m_LCUs[LCUIdx].m_bitWeight  = 1.0;
      Int currWidth  = ( (i == picWidthInLCU -1) ? picWidth  - LCUWidth *(picWidthInLCU -1) : LCUWidth  );
      Int currHeight = ( (j == picHeightInLCU-1) ? picHeight - LCUHeight*(picHeightInLCU-1) : LCUHeight );
      m_LCUs[LCUIdx].m_numberOfPixel = currWidth * currHeight;
    }
  }
  m_picActualHeaderBits = 0;
  m_picActualBits       = 0;
  m_picQP               = 0;
  m_picLambda           = 0.0;
}

Void TEncRCPic::destroy()
{
  if( m_LCUs != NULL )
  {
    delete[] m_LCUs;
    m_LCUs = NULL;
  }
  m_encRCSeq = NULL;
  m_encRCGOP = NULL;
}


Double TEncRCPic::estimatePicLambda( list<TEncRCPic*>& listPreviousPictures, SliceType eSliceType)
{
  Double alpha         = m_encRCSeq->getPicPara( m_frameLevel ).m_alpha;
  Double beta          = m_encRCSeq->getPicPara( m_frameLevel ).m_beta;
  Double bpp       = (Double)m_targetBits/(Double)m_numberOfPixel;
  
#if JVET_K0390_RATE_CTRL
  Int lastPicValPix = 0;
  if (listPreviousPictures.size() > 0)
  {
    lastPicValPix = m_encRCSeq->getPicPara(m_frameLevel).m_validPix;
  }
  if (lastPicValPix > 0)
  {
    bpp = (Double)m_targetBits / (Double)lastPicValPix;
  }
#endif
  
  Double estLambda;
  if (eSliceType == I_SLICE)
  {
    estLambda = calculateLambdaIntra(alpha, beta, pow(m_totalCostIntra/(Double)m_numberOfPixel, BETA1), bpp); //利用I帧的lambda计算公式计算出来估计lambda
  }
  else
  {
    estLambda = alpha * pow( bpp, beta ); //直接利用公式计算
  }

  Double lastLevelLambda = -1.0; //定义上一个同level的lambda、上一帧的lambda和上一个有效的lambda，并分别初始化
  Double lastPicLambda   = -1.0;
  Double lastValidLambda = -1.0;
  list<TEncRCPic*>::iterator it;
  for ( it = listPreviousPictures.begin(); it != listPreviousPictures.end(); it++ ) //从头到尾遍历已经编码完成的帧
  {
    if ( (*it)->getFrameLevel() == m_frameLevel ) //如果已经编码完成的帧的level和当前帧的level是一样的
    {
      lastLevelLambda = (*it)->getPicActualLambda(); //把那一帧的实际的帧lambda赋给lastLevelLambda
    }
    lastPicLambda     = (*it)->getPicActualLambda(); //把已编码帧的实际lambda赋给lastPicLambda

    if ( lastPicLambda > 0.0 ) //如果lastPicLambda有效即>0
    {
      lastValidLambda = lastPicLambda; //即把lastPicLambda赋给lastValidLambda
    }
  }

  if ( lastLevelLambda > 0.0 ) //如果同level的上一个lambda有效即>0
  {
    lastLevelLambda = Clip3( 0.1, 10000.0, lastLevelLambda ); //对lastLevelLambda和当前帧的估计lambda进行修正
    estLambda = Clip3( lastLevelLambda * pow( 2.0, -3.0/3.0 ), lastLevelLambda * pow( 2.0, 3.0/3.0 ), estLambda );
  }

  if ( lastPicLambda > 0.0 ) //如果上一个帧的lambda有效
  {
    lastPicLambda = Clip3( 0.1, 2000.0, lastPicLambda ); //对lastPicLambda和当前帧的估计lambda进行修正
    estLambda = Clip3( lastPicLambda * pow( 2.0, -10.0/3.0 ), lastPicLambda * pow( 2.0, 10.0/3.0 ), estLambda );
  }
  else if ( lastValidLambda > 0.0 ) //如果上一个帧的lambda无效，但是lastValidLambda有效
  {
    lastValidLambda = Clip3( 0.1, 2000.0, lastValidLambda ); //对lastValidLambda和当前帧的估计lambda进行修正
    estLambda = Clip3( lastValidLambda * pow(2.0, -10.0/3.0), lastValidLambda * pow(2.0, 10.0/3.0), estLambda );
  }
  else // 如果上一个帧的lambda无效，并且lastValidLambda也无效
  {
    estLambda = Clip3( 0.1, 10000.0, estLambda ); //对估计lambda进行修正
  }

  if ( estLambda < 0.1 )
  {
    estLambda = 0.1; //一个帧的估计lambda至少为0.1
  }
#if JVET_K0390_RATE_CTRL
  //Avoid different results in different platforms. The problem is caused by the different results of pow() in different platforms.
  estLambda = Double(int64_t(estLambda * (Double)LAMBDA_PREC + 0.5)) / (Double)LAMBDA_PREC; 
#endif
  m_estPicLambda = estLambda; //将估计lambda赋给当前帧的估计lambda

  Double totalWeight = 0.0;
  // initial BU bit allocation weight
  for ( Int i=0; i<m_numberOfLCU; i++ ) //对所有的LCU进行遍历
  {
    Double alphaLCU, betaLCU;
    if ( m_encRCSeq->getUseLCUSeparateModel() ) //如果每个LCU有单独的alpha和beta值
    {
      alphaLCU = m_encRCSeq->getLCUPara( m_frameLevel, i ).m_alpha; //利用framelevel和系数i初始化
      betaLCU  = m_encRCSeq->getLCUPara( m_frameLevel, i ).m_beta;
    }
    else //否则利用framelevel进行初始化
    {
      alphaLCU = m_encRCSeq->getPicPara( m_frameLevel ).m_alpha;
      betaLCU  = m_encRCSeq->getPicPara( m_frameLevel ).m_beta;
    }

    m_LCUs[i].m_bitWeight =  m_LCUs[i].m_numberOfPixel * pow( estLambda/alphaLCU, 1.0/betaLCU );//计算出每个LCU所占的比重

    if ( m_LCUs[i].m_bitWeight < 0.01 )
    {
      m_LCUs[i].m_bitWeight = 0.01; //每个LCU的比重最小为0.01
    }
    totalWeight += m_LCUs[i].m_bitWeight; //对每个LCU的比重进行累加，得到总的比重
  }
  for ( Int i=0; i<m_numberOfLCU; i++ ) //遍历所有的LCU
  {
    Double BUTargetBits = m_targetBits * m_LCUs[i].m_bitWeight / totalWeight; //获得每个LCU的目标比特并赋给每个LCU
    m_LCUs[i].m_bitWeight = BUTargetBits;
  }

  return estLambda;
}

Int TEncRCPic::estimatePicQP( Double lambda, list<TEncRCPic*>& listPreviousPictures )
{
  Int QP = Int( 4.2005 * log( lambda ) + 13.7122 + 0.5 ); //首先通过传进来的lambda值进行QP的计算

  Int lastLevelQP = g_RCInvalidQPValue; //对同层的上一个QP、上一帧的QP和上一个有效的QP进行新建并初始化
  Int lastPicQP   = g_RCInvalidQPValue;
  Int lastValidQP = g_RCInvalidQPValue;
  list<TEncRCPic*>::iterator it;
  for ( it = listPreviousPictures.begin(); it != listPreviousPictures.end(); it++ ) //从已编码的第一帧遍历到最后一帧 
  {
    if ( (*it)->getFrameLevel() == m_frameLevel ) //如果framelevel相同
    {
      lastLevelQP = (*it)->getPicActualQP(); //把遍历的帧的实际QP值赋给 lastLevelQP
    }
    lastPicQP = (*it)->getPicActualQP(); //把遍历的帧的实际QP个上一帧的QP
    if ( lastPicQP > g_RCInvalidQPValue ) //若上一帧的QP有效
    {
      lastValidQP = lastPicQP; //上一帧的QP记为上一个有效QP
    }
  }

  if ( lastLevelQP > g_RCInvalidQPValue ) //如果同层的上一个QP有效
  {
    QP = Clip3( lastLevelQP - 3, lastLevelQP + 3, QP ); //修正QP
  }

  if( lastPicQP > g_RCInvalidQPValue ) //如果上一帧的QP有效
  {
    QP = Clip3( lastPicQP - 10, lastPicQP + 10, QP ); //修正QP
  }
  else if( lastValidQP > g_RCInvalidQPValue ) //如果上一帧的QP无效，但上一个有效QP是有效的
  {
    QP = Clip3( lastValidQP - 10, lastValidQP + 10, QP ); //修正QP
  }

  return QP;
}

Double TEncRCPic::getLCUTargetBpp(SliceType eSliceType)
{
  // 返回已经编码的LCU的数目
  Int   LCUIdx    = getLCUCoded();
  Double bpp      = -1.0;
  Int avgBits     = 0;

  if (eSliceType == I_SLICE)
  {
    // 剩余LCU的个数=LCU的总个数-已经编码的LCU的个数+1
    Int noOfLCUsLeft = m_numberOfLCU - LCUIdx + 1;
    // 比特率窗，最小为4，不能小于GOP的size
    Int bitrateWindow = min(4,noOfLCUsLeft);
    // 返回TRCLCU类型的结构体，编号为LCUIdx的LCU块的比重值为MAD
    Double MAD      = getLCU(LCUIdx).m_costIntra;

    //如果剩余帧内LCU的比重>0.1
    if (m_remainingCostIntra > 0.1 )
    {
      Double weightedBitsLeft = (m_bitsLeft*bitrateWindow+(m_bitsLeft-getLCU(LCUIdx).m_targetBitsLeft)*noOfLCUsLeft)/(Double)bitrateWindow;
      //剩余比特
      avgBits = Int( MAD*weightedBitsLeft/m_remainingCostIntra );
    }
    else //如果剩余帧内消耗比重<=0.1
    {
      avgBits = Int( m_bitsLeft / m_LCULeft ); //平均每个LCU的比特数=剩余比特/剩余LCU个数
    }
    //m_remainingCostIntra =m_remainingCostIntra -MAD  更新剩余LCU所占的比重
    m_remainingCostIntra -= MAD;
  }
  else
  {
    Double totalWeight = 0;
    //从当前的LCU向后进行遍历
    for ( Int i=LCUIdx; i<m_numberOfLCU; i++ )
    {
        //计算出包括当前LCU在内的剩余LCU所需要的比特和
      totalWeight += m_LCUs[i].m_bitWeight;
    }
    //真正有用的（影响大的）LCU数目=平滑窗口和剩余LCU数目的最小值
    Int realInfluenceLCU = min( g_RCLCUSmoothWindowSize, getLCULeft() );
    //平均每个LCU的比特数=当前LCU的需要比特-（包括当前LCU在内的剩余LCU所需要的比特和-实际剩余的比特）/有用的LCU数目+0.5
    avgBits = (Int)( m_LCUs[LCUIdx].m_bitWeight - ( totalWeight - m_bitsLeft ) / realInfluenceLCU + 0.5 );
  }

  //平均每个LCU至少分配1个比特
  if ( avgBits < 1 )
  {
    avgBits = 1;
  }
  //计算当前LCU的Bpp值 
  bpp = ( Double )avgBits/( Double )m_LCUs[ LCUIdx ].m_numberOfPixel;
  //平均比特即作为当前LCU的目标比特
  m_LCUs[ LCUIdx ].m_targetBits = avgBits;

  return bpp;
}

Double TEncRCPic::getLCUEstLambda( Double bpp )
{
  //获得已经编码的帧数
  Int   LCUIdx = getLCUCoded();
  Double alpha;
  Double beta;
  //如果每个LCU的alpha和beta互相独立
  if ( m_encRCSeq->getUseLCUSeparateModel() )
  {
    //获得当前LCU的alpha和beta参数
    alpha = m_encRCSeq->getLCUPara( m_frameLevel, LCUIdx ).m_alpha;
    beta  = m_encRCSeq->getLCUPara( m_frameLevel, LCUIdx ).m_beta;
  }
  else
  {
    //否则使用图片级的alpha和beta参数
    alpha = m_encRCSeq->getPicPara( m_frameLevel ).m_alpha;
    beta  = m_encRCSeq->getPicPara( m_frameLevel ).m_beta;
  }

  //新建估计lambda变量，并且计算赋值
  Double estLambda = alpha * pow( bpp, beta );
  //for Lambda clip, picture level clip
  Double clipPicLambda = m_estPicLambda;//得到图片的估计lambda

  //for Lambda clip, LCU level clip
  Double clipNeighbourLambda = -1.0;//新建邻域lambda变量，并初始化
  for ( Int i=LCUIdx - 1; i>=0; i-- ) //从当前LCU向前进行遍历
  {
    if ( m_LCUs[i].m_lambda > 0 )
    {
      clipNeighbourLambda = m_LCUs[i].m_lambda; //若lambda>0，则有效lambda的值就赋给邻域lambda变量
      break;
    }
  }

  if ( clipNeighbourLambda > 0.0 ) //若邻域lambda>0
  {
      //对估计lambda进行修正
    estLambda = Clip3( clipNeighbourLambda * pow( 2.0, -1.0/3.0 ), clipNeighbourLambda * pow( 2.0, 1.0/3.0 ), estLambda );
  }

  if ( clipPicLambda > 0.0 ) //若图片层估计lambda>0
  {
    estLambda = Clip3( clipPicLambda * pow( 2.0, -2.0/3.0 ), clipPicLambda * pow( 2.0, 2.0/3.0 ), estLambda );
  }
  else //否则用常数进行修正
  {
    estLambda = Clip3( 10.0, 1000.0, estLambda );
  }

  if ( estLambda < 0.1 ) //估计的LCUlambda至少为0.1
  {
    estLambda = 0.1;
  }
#if JVET_K0390_RATE_CTRL
  //Avoid different results in different platforms. The problem is caused by the different results of pow() in different platforms.
  estLambda = Double(int64_t(estLambda * (Double)LAMBDA_PREC + 0.5)) / (Double)LAMBDA_PREC;
#endif
  return estLambda;
}

Int TEncRCPic::getLCUEstQP( Double lambda, Int clipPicQP )
{
  Int LCUIdx = getLCUCoded();
  Int estQP = Int( 4.2005 * log( lambda ) + 13.7122 + 0.5 ); //根据传入的LCU的估计lambda对QP进行估计

  //for Lambda clip, LCU level clip
  Int clipNeighbourQP = g_RCInvalidQPValue;// 新建邻域QP变量并且初始化
  for ( Int i=LCUIdx - 1; i>=0; i-- ) //从当前LCU向前遍历
  {
    if ( (getLCU(i)).m_QP > g_RCInvalidQPValue )
    {
      clipNeighbourQP = getLCU(i).m_QP; //若为有效的QP则赋给邻域QP
      break;
    }
  }

  if ( clipNeighbourQP > g_RCInvalidQPValue )
  {
    estQP = Clip3( clipNeighbourQP - 1, clipNeighbourQP + 1, estQP ); //对预测QP进行修正
  }

  estQP = Clip3( clipPicQP - 2, clipPicQP + 2, estQP ); //根据图片层的QP再次进行修正

  return estQP;
}

#if JVET_M0600_RATE_CTRL
Void TEncRCPic::updateAfterCTU(Int LCUIdx, Int bits, Int QP, Double lambda, Double skipRatio, Bool updateLCUParameter)
#else
Void TEncRCPic::updateAfterCTU( Int LCUIdx, Int bits, Int QP, Double lambda, Bool updateLCUParameter )
#endif
{
  //对当前的LCU的实际编码比特、QP和lambda进行初始化
  m_LCUs[LCUIdx].m_actualBits = bits;
  m_LCUs[LCUIdx].m_QP         = QP;
  m_LCUs[LCUIdx].m_lambda     = lambda;
#if JVET_K0390_RATE_CTRL
  m_LCUs[LCUIdx].m_actualSSE = m_LCUs[LCUIdx].m_actualMSE * m_LCUs[LCUIdx].m_numberOfPixel;
#endif

  m_LCULeft--; //当前LCU已经编码完成，未编码LCU数目=原数目-1
  m_bitsLeft   -= bits; //剩余比特=原剩余比特-当前LCU的实际编码比特
  m_pixelsLeft -= m_LCUs[LCUIdx].m_numberOfPixel; //剩余像素=原剩余像素-当前LCU的像素

  if ( !updateLCUParameter ) //如果LCU的参数不需要更新，直接返回
  {
    return;
  }

  if ( !m_encRCSeq->getUseLCUSeparateModel() ) //如果LCU的参数不是独立的，直接返回
  {
    return;
  }

  Double alpha = m_encRCSeq->getLCUPara( m_frameLevel, LCUIdx ).m_alpha;
  Double beta  = m_encRCSeq->getLCUPara( m_frameLevel, LCUIdx ).m_beta;

  Int LCUActualBits   = m_LCUs[LCUIdx].m_actualBits; //获得当前LCU的实际编码比特
  Int LCUTotalPixels  = m_LCUs[LCUIdx].m_numberOfPixel; //获得当前LCU的实际像素数
  Double bpp         = ( Double )LCUActualBits/( Double )LCUTotalPixels; //获得当前LCU的实际Bpp
  Double calLambda   = alpha * pow( bpp, beta ); //实际lambda
  Double inputLambda = m_LCUs[LCUIdx].m_lambda; //获得当前LCU的lambda，即原来（old）的lambda

  if( inputLambda < 0.01 || calLambda < 0.01 || bpp < 0.0001 )
  {
    alpha *= ( 1.0 - m_encRCSeq->getAlphaUpdate() / 2.0 );
    beta  *= ( 1.0 - m_encRCSeq->getBetaUpdate() / 2.0 );

    alpha = Clip3( g_RCAlphaMinValue, g_RCAlphaMaxValue, alpha );
    beta  = Clip3( g_RCBetaMinValue,  g_RCBetaMaxValue,  beta  );

    TRCParameter rcPara;
    rcPara.m_alpha = alpha;
    rcPara.m_beta  = beta;
#if JVET_M0600_RATE_CTRL
    rcPara.m_skipRatio = skipRatio;
#endif
#if JVET_K0390_RATE_CTRL
    if (QP == g_RCInvalidQPValue && m_encRCSeq->getAdaptiveBits() == 1)
    {
      rcPara.m_validPix = 0;
    }
    else
    {
      rcPara.m_validPix = LCUTotalPixels;
    }

    Double MSE = m_LCUs[LCUIdx].m_actualMSE;
    Double updatedK = bpp * inputLambda / MSE;
    Double updatedC = MSE / pow(bpp, -updatedK);
    rcPara.m_alpha = updatedC * updatedK;
    rcPara.m_beta = -updatedK - 1.0;

    if (bpp > 0 && updatedK > 0.0001)
    {
      m_encRCSeq->setLCUPara(m_frameLevel, LCUIdx, rcPara);
    }
    else
    {
      rcPara.m_alpha = Clip3(0.0001, g_RCAlphaMaxValue, rcPara.m_alpha);
      m_encRCSeq->setLCUPara(m_frameLevel, LCUIdx, rcPara);
    }
#else
    m_encRCSeq->setLCUPara( m_frameLevel, LCUIdx, rcPara );
#endif
    return;
  }

  calLambda = Clip3( inputLambda / 10.0, inputLambda * 10.0, calLambda );
  //P360页更新公式
  alpha += m_encRCSeq->getAlphaUpdate() * ( log( inputLambda ) - log( calLambda ) ) * alpha;
  Double lnbpp = log( bpp );
  lnbpp = Clip3( -5.0, -0.1, lnbpp );
  //P360页更新公式
  beta  += m_encRCSeq->getBetaUpdate() * ( log( inputLambda ) - log( calLambda ) ) * lnbpp;

  alpha = Clip3( g_RCAlphaMinValue, g_RCAlphaMaxValue, alpha );
  beta  = Clip3( g_RCBetaMinValue,  g_RCBetaMaxValue,  beta  );

  TRCParameter rcPara;
  rcPara.m_alpha = alpha;
  rcPara.m_beta  = beta;
#if JVET_M0600_RATE_CTRL
  rcPara.m_skipRatio = skipRatio;
#endif
#if JVET_K0390_RATE_CTRL
  if (QP == g_RCInvalidQPValue && m_encRCSeq->getAdaptiveBits() == 1)
  {
    rcPara.m_validPix = 0;
  }
  else
  {
    rcPara.m_validPix = LCUTotalPixels;
  }

  Double MSE = m_LCUs[LCUIdx].m_actualMSE;
  Double updatedK = bpp * inputLambda / MSE;
  Double updatedC = MSE / pow(bpp, -updatedK);
  rcPara.m_alpha = updatedC * updatedK;
  rcPara.m_beta = -updatedK - 1.0;

  if (bpp > 0 && updatedK > 0.0001)
  {
    m_encRCSeq->setLCUPara(m_frameLevel, LCUIdx, rcPara);
  }
  else
  {
    rcPara.m_alpha = Clip3(0.0001, g_RCAlphaMaxValue, rcPara.m_alpha);
    m_encRCSeq->setLCUPara(m_frameLevel, LCUIdx, rcPara);
  }
#else
  m_encRCSeq->setLCUPara( m_frameLevel, LCUIdx, rcPara );
#endif
}

Double TEncRCPic::calAverageQP()
{
  Int totalQPs = 0;
  Int numTotalLCUs = 0;

  Int i;
  for ( i=0; i<m_numberOfLCU; i++ )
  {
    if ( m_LCUs[i].m_QP > 0 )
    {
      totalQPs += m_LCUs[i].m_QP;
      numTotalLCUs++;
    }
  }

  Double avgQP = 0.0;
  if ( numTotalLCUs == 0 )
  {
    avgQP = g_RCInvalidQPValue;
  }
  else
  {
    avgQP = ((Double)totalQPs) / ((Double)numTotalLCUs);
  }
  return avgQP;
}

Double TEncRCPic::calAverageLambda()
{
  Double totalLambdas = 0.0;
  Int numTotalLCUs = 0;
#if JVET_K0390_RATE_CTRL
  Double totalSSE = 0.0;
  Int totalPixels = 0;
#endif
  Int i;
  for ( i=0; i<m_numberOfLCU; i++ )
  {
    if ( m_LCUs[i].m_lambda > 0.01 )
    {
#if JVET_K0390_RATE_CTRL
      if (m_LCUs[i].m_QP > 0 || m_encRCSeq->getAdaptiveBits() != 1)
      {
        m_validPixelsInPic += m_LCUs[i].m_numberOfPixel;

        totalLambdas += log(m_LCUs[i].m_lambda);
        numTotalLCUs++;
      }
#else
      totalLambdas += log( m_LCUs[i].m_lambda );
      numTotalLCUs++;
#endif

#if JVET_K0390_RATE_CTRL
      if (m_LCUs[i].m_QP > 0 || m_encRCSeq->getAdaptiveBits() != 1)
      {
        totalSSE += m_LCUs[i].m_actualSSE;
        totalPixels += m_LCUs[i].m_numberOfPixel;
      }
#endif
    }
  }
#if JVET_K0390_RATE_CTRL
  setPicMSE(totalPixels > 0 ? totalSSE / (Double)totalPixels : 1.0); //1.0 is useless in the following process, just to make sure the divisor not be 0
#endif

  Double avgLambda;
  if( numTotalLCUs == 0 )
  {
    avgLambda = -1.0;
  }
  else
  {
    avgLambda = pow( 2.7183, totalLambdas / numTotalLCUs );
  }
  return avgLambda;
}


Void TEncRCPic::updateAfterPicture( Int actualHeaderBits, Int actualTotalBits, Double averageQP, Double averageLambda, SliceType eSliceType)
{
  m_picActualHeaderBits = actualHeaderBits; //新建并初始化该图像的头信息比特和实际编码比特
  m_picActualBits       = actualTotalBits;
  if ( averageQP > 0.0 ) //若传入的QP>0，
  {
    m_picQP             = Int( averageQP + 0.5 );
  }
  else // 否则
  {
    m_picQP             = g_RCInvalidQPValue; //QP=-99
  }
  m_picLambda           = averageLambda; //新建该帧的lambda并初始化

  Double alpha = m_encRCSeq->getPicPara( m_frameLevel ).m_alpha; //获得原来的alpha和beta的值
  Double beta  = m_encRCSeq->getPicPara( m_frameLevel ).m_beta;
#if JVET_M0600_RATE_CTRL //calculate the skipRatio of picture
  Double skipRatio = 0;
  Int numOfSkipPixel = 0;
  for (Int LCUIdx = 0; LCUIdx < m_numberOfLCU; LCUIdx++)
  {
    numOfSkipPixel += Int(m_encRCSeq->getLCUPara(m_frameLevel, LCUIdx).m_skipRatio*m_LCUs[LCUIdx].m_numberOfPixel);
  }
  skipRatio = (Double)numOfSkipPixel / (Double)m_numberOfPixel;
#endif
  if (eSliceType == I_SLICE) //如果是I帧，利用update函数进行更新
  {
    updateAlphaBetaIntra(&alpha, &beta);
  }
  else //如果不是I帧
  {
    // update parameters
    Double picActualBits = ( Double )m_picActualBits; //获得图像实际编码比特、Bpp
#if JVET_K0390_RATE_CTRL
    Double picActualBpp = picActualBits / (Double)m_validPixelsInPic;
#else
    Double picActualBpp  = picActualBits/(Double)m_numberOfPixel;
#endif
    Double calLambda     = alpha * pow( picActualBpp, beta ); //实际的lambda
    Double inputLambda   = m_picLambda;

    if ( inputLambda < 0.01 || calLambda < 0.01 || picActualBpp < 0.0001 )
    {
      alpha *= ( 1.0 - m_encRCSeq->getAlphaUpdate() / 2.0 );
      beta  *= ( 1.0 - m_encRCSeq->getBetaUpdate() / 2.0 );

      alpha = Clip3( g_RCAlphaMinValue, g_RCAlphaMaxValue, alpha );
      beta  = Clip3( g_RCBetaMinValue,  g_RCBetaMaxValue,  beta  );

      TRCParameter rcPara;
      rcPara.m_alpha = alpha;
      rcPara.m_beta  = beta;
#if JVET_M0600_RATE_CTRL
      rcPara.m_skipRatio = skipRatio;
#endif
#if JVET_K0390_RATE_CTRL
      Double avgMSE = getPicMSE();
      Double updatedK = picActualBpp * averageLambda / avgMSE;
      Double updatedC = avgMSE / pow(picActualBpp, -updatedK);

      if (m_frameLevel > 0)  //only use for level > 0
      {
        rcPara.m_alpha = updatedC * updatedK;
        rcPara.m_beta = -updatedK - 1.0;
      }

      rcPara.m_validPix = m_validPixelsInPic;

      if (m_validPixelsInPic > 0)
      {
        m_encRCSeq->setPicPara(m_frameLevel, rcPara);
      }
#else
      m_encRCSeq->setPicPara( m_frameLevel, rcPara );
#endif

      return;
    }

    calLambda = Clip3( inputLambda / 10.0, inputLambda * 10.0, calLambda );
    alpha += m_encRCSeq->getAlphaUpdate() * ( log( inputLambda ) - log( calLambda ) ) * alpha;
    Double lnbpp = log( picActualBpp );
    lnbpp = Clip3( -5.0, -0.1, lnbpp );

    beta  += m_encRCSeq->getBetaUpdate() * ( log( inputLambda ) - log( calLambda ) ) * lnbpp;

    alpha = Clip3( g_RCAlphaMinValue, g_RCAlphaMaxValue, alpha );
    beta  = Clip3( g_RCBetaMinValue,  g_RCBetaMaxValue,  beta  );
  }

  TRCParameter rcPara;
  rcPara.m_alpha = alpha;
  rcPara.m_beta  = beta;
#if JVET_M0600_RATE_CTRL
  rcPara.m_skipRatio = skipRatio;

#endif
#if JVET_K0390_RATE_CTRL
  Double picActualBpp = (Double)m_picActualBits / (Double)m_validPixelsInPic;

  Double avgMSE = getPicMSE();
  Double updatedK = picActualBpp * averageLambda / avgMSE;
  Double updatedC = avgMSE / pow(picActualBpp, -updatedK);
  if (m_frameLevel > 0)  //only use for level > 0
  {
    rcPara.m_alpha = updatedC * updatedK;
    rcPara.m_beta = -updatedK - 1.0;
  }

  rcPara.m_validPix = m_validPixelsInPic;

  if (m_validPixelsInPic > 0)
  {
    m_encRCSeq->setPicPara(m_frameLevel, rcPara);
  }
#else
  m_encRCSeq->setPicPara( m_frameLevel, rcPara );
#endif

  if ( m_frameLevel == 1 )
  {
    Double currLambda = Clip3( 0.1, 10000.0, m_picLambda );
    Double updateLastLambda = g_RCWeightHistoryLambda * m_encRCSeq->getLastLambda() + g_RCWeightCurrentLambda * currLambda;
    //利用该公式获得lastlambda，并传递更新
    m_encRCSeq->setLastLambda( updateLastLambda );
  }
}

Int TEncRCPic::getRefineBitsForIntra( Int orgBits )
{
  Double alpha=0.25, beta=0.5582;
  Int iIntraBits;

  if (orgBits*40 < m_numberOfPixel)
  {
    alpha=0.25;
  }
  else
  {
    alpha=0.30;
  }

  iIntraBits = (Int)(alpha* pow(m_totalCostIntra*4.0/(Double)orgBits, beta)*(Double)orgBits+0.5);

  return iIntraBits;
}

Double TEncRCPic::calculateLambdaIntra(Double alpha, Double beta, Double MADPerPixel, Double bitsPerPixel)
{
  return ( (alpha/256.0) * pow( MADPerPixel/bitsPerPixel, beta ) );
}

Void TEncRCPic::updateAlphaBetaIntra(Double *alpha, Double *beta)
{
  Double lnbpp = log(pow(m_totalCostIntra / (Double)m_numberOfPixel, BETA1));
  Double diffLambda = (*beta)*(log((Double)m_picActualBits)-log((Double)m_targetBits));

  diffLambda = Clip3(-0.125, 0.125, 0.25*diffLambda);
  *alpha    =  (*alpha) * exp(diffLambda);
  *beta     =  (*beta) + diffLambda / lnbpp;
}


Void TEncRCPic::getLCUInitTargetBits()
{
  Int iAvgBits     = 0;

  m_remainingCostIntra = m_totalCostIntra;
  for (Int i=m_numberOfLCU-1; i>=0; i--)
  {
    iAvgBits += Int(m_targetBits * getLCU(i).m_costIntra/m_totalCostIntra);
    getLCU(i).m_targetBitsLeft = iAvgBits;
  }
}


Double TEncRCPic::getLCUEstLambdaAndQP(Double bpp, Int clipPicQP, Int *estQP)
{
  Int   LCUIdx = getLCUCoded(); //获得已经编码完的帧数

  Double   alpha = m_encRCSeq->getPicPara( m_frameLevel ).m_alpha;
  Double   beta  = m_encRCSeq->getPicPara( m_frameLevel ).m_beta;

  //每个像素在帧内的比重=当前LCU在帧内的比重/总像素数
  Double costPerPixel = getLCU(LCUIdx).m_costIntra/(Double)getLCU(LCUIdx).m_numberOfPixel;
  //对比重进行修正
  costPerPixel = pow(costPerPixel, BETA1);
  //通过计算帧内lambda的函数求出估计lambda
  Double estLambda = calculateLambdaIntra(alpha, beta, costPerPixel, bpp);

  // 定义变量存储邻域QP
  Int clipNeighbourQP = g_RCInvalidQPValue;
  for (Int i=LCUIdx-1; i>=0; i--)
  {
      //从当前LCU向前遍历，若QP为有效值
    if ((getLCU(i)).m_QP > g_RCInvalidQPValue)
    {
        //将该邻域QP赋给变量并跳出循环
      clipNeighbourQP = getLCU(i).m_QP;
      break;
    }
  }
  //clipPicQP为slice的QP值
  Int minQP = clipPicQP - 2;
  Int maxQP = clipPicQP + 2;

  //若邻域QP有效，对Max和minQP进行修正
  if ( clipNeighbourQP > g_RCInvalidQPValue )
  {
    maxQP = min(clipNeighbourQP + 1, maxQP);
    minQP = max(clipNeighbourQP - 1, minQP);
  }

  //通过maxQP和minQP分别计算Maxlambda和minlambda
  Double maxLambda=exp(((Double)(maxQP+0.49)-13.7122)/4.2005);
  Double minLambda=exp(((Double)(minQP-0.49)-13.7122)/4.2005);

  //选出最适合的作为当前LCU的估计lambda
  estLambda = Clip3(minLambda, maxLambda, estLambda);
#if JVET_K0390_RATE_CTRL
  //Avoid different results in different platforms. The problem is caused by the different results of pow() in different platforms.
  estLambda = Double(int64_t(estLambda * (Double)LAMBDA_PREC + 0.5)) / (Double)LAMBDA_PREC;
#endif

  //根据得到的估计lambda对传进来的估计QP进行修正
  *estQP = Int( 4.2005 * log(estLambda) + 13.7122 + 0.5 );
  *estQP = Clip3(minQP, maxQP, *estQP);

  return estLambda;
}

TEncRateCtrl::TEncRateCtrl()
{
  m_encRCSeq = NULL;
  m_encRCGOP = NULL;
  m_encRCPic = NULL;
}

TEncRateCtrl::~TEncRateCtrl()
{
  destroy();
}

Void TEncRateCtrl::destroy()
{
  if ( m_encRCSeq != NULL )
  {
    delete m_encRCSeq;
    m_encRCSeq = NULL;
  }
  if ( m_encRCGOP != NULL )
  {
    delete m_encRCGOP;
    m_encRCGOP = NULL;
  }
  while ( m_listRCPictures.size() > 0 )
  {
    TEncRCPic* p = m_listRCPictures.front();
    m_listRCPictures.pop_front();
    delete p;
  }
}

Void TEncRateCtrl::init( Int totalFrames, Int targetBitrate, Int frameRate, Int GOPSize, Int picWidth, Int picHeight, Int LCUWidth, Int LCUHeight, Int keepHierBits, Bool useLCUSeparateModel, GOPEntry  GOPList[MAX_GOP] )
{
  destroy();

  Bool isLowdelay = true; //lowdelay的判断标志位
  for ( Int i=0; i<GOPSize-1; i++ ) //GOPList[MAX_GOP]GOP列表，MAX_GOP是GOP分层结构大小的最大值，为64
  {
    if ( GOPList[i].m_POC > GOPList[i+1].m_POC ) //.m_POC是GOPEntry结构体的成员。如果编码顺序对应的播放顺序大于下一帧的播放顺序，就不是lowdelay模式
    {
      isLowdelay = false;
      break;
    }
  }

  Int numberOfLevel = 1;
  Int adaptiveBit = 0;
  if ( keepHierBits > 0 ) //分层结构
  {
    numberOfLevel = Int( log((Double)GOPSize)/log(2.0) + 0.5 ) + 1;
  }
#if JVET_K0390_RATE_CTRL
  if (!isLowdelay && (GOPSize == 16 || GOPSize == 8))
#else
  if ( !isLowdelay && GOPSize == 8 )
#endif
  {
    numberOfLevel = Int( log((Double)GOPSize)/log(2.0) + 0.5 ) + 1;
  }
  numberOfLevel++;    // intra picture
  numberOfLevel++;    // non-reference picture


  Int* bitsRatio;
  bitsRatio = new Int[ GOPSize ]; //新建数组，指向第一个数据
  for ( Int i=0; i<GOPSize; i++ )
  {
    bitsRatio[i] = 10;
    if ( !GOPList[i].m_refPic ) //如果是参考图片
    {
      bitsRatio[i] = 2;
    }
  }

  if ( keepHierBits > 0 )
  {
    Double bpp = (Double)( targetBitrate / (Double)( frameRate*picWidth*picHeight ) ); //计算bpp
    if ( GOPSize == 4 && isLowdelay ) //如果GOPSize==4并且是lowdelay模式，按照358页表12.1进行权值分配
    {
      if ( bpp > 0.2 )
      {
        bitsRatio[0] = 2;
        bitsRatio[1] = 3;
        bitsRatio[2] = 2;
        bitsRatio[3] = 6;
      }
      else if( bpp > 0.1 )
      {
        bitsRatio[0] = 2;
        bitsRatio[1] = 3;
        bitsRatio[2] = 2;
        bitsRatio[3] = 10;
      }
      else if ( bpp > 0.05 )
      {
        bitsRatio[0] = 2;
        bitsRatio[1] = 3;
        bitsRatio[2] = 2;
        bitsRatio[3] = 12;
      }
      else
      {
        bitsRatio[0] = 2;
        bitsRatio[1] = 3;
        bitsRatio[2] = 2;
        bitsRatio[3] = 14;
      }

      if ( keepHierBits == 2 ) //若keepHierBits==2，adaptiveBit赋值为1
      {
        adaptiveBit = 1;
      }
    }
    else if ( GOPSize == 8 && !isLowdelay ) //如果GOPSize==8并且不是lowdelay模式，按照358页表12.2进行权值分配
    {
      if ( bpp > 0.2 )
      {
        bitsRatio[0] = 15;
        bitsRatio[1] = 5;
        bitsRatio[2] = 4;
        bitsRatio[3] = 1;
        bitsRatio[4] = 1;
        bitsRatio[5] = 4;
        bitsRatio[6] = 1;
        bitsRatio[7] = 1;
      }
      else if ( bpp > 0.1 )
      {
        bitsRatio[0] = 20;
        bitsRatio[1] = 6;
        bitsRatio[2] = 4;
        bitsRatio[3] = 1;
        bitsRatio[4] = 1;
        bitsRatio[5] = 4;
        bitsRatio[6] = 1;
        bitsRatio[7] = 1;
      }
      else if ( bpp > 0.05 )
      {
        bitsRatio[0] = 25;
        bitsRatio[1] = 7;
        bitsRatio[2] = 4;
        bitsRatio[3] = 1;
        bitsRatio[4] = 1;
        bitsRatio[5] = 4;
        bitsRatio[6] = 1;
        bitsRatio[7] = 1;
      }
      else
      {
        bitsRatio[0] = 30;
        bitsRatio[1] = 8;
        bitsRatio[2] = 4;
        bitsRatio[3] = 1;
        bitsRatio[4] = 1;
        bitsRatio[5] = 4;
        bitsRatio[6] = 1;
        bitsRatio[7] = 1;
      }

      if ( keepHierBits == 2 )
      {
        adaptiveBit = 2;
      }
    }
#if JVET_K0390_RATE_CTRL
    else if (GOPSize == 16 && !isLowdelay)
    {
      if (bpp > 0.2)
      {
        bitsRatio[0] = 10;
        bitsRatio[1] = 8;
        bitsRatio[2] = 4;
        bitsRatio[3] = 2;
        bitsRatio[4] = 1;
        bitsRatio[5] = 1;
        bitsRatio[6] = 2;
        bitsRatio[7] = 1;
        bitsRatio[8] = 1;
        bitsRatio[9] = 4;
        bitsRatio[10] = 2;
        bitsRatio[11] = 1;
        bitsRatio[12] = 1;
        bitsRatio[13] = 2;
        bitsRatio[14] = 1;
        bitsRatio[15] = 1;
      }
      else if (bpp > 0.1)
      {
        bitsRatio[0] = 15;
        bitsRatio[1] = 9;
        bitsRatio[2] = 4;
        bitsRatio[3] = 2;
        bitsRatio[4] = 1;
        bitsRatio[5] = 1;
        bitsRatio[6] = 2;
        bitsRatio[7] = 1;
        bitsRatio[8] = 1;
        bitsRatio[9] = 4;
        bitsRatio[10] = 2;
        bitsRatio[11] = 1;
        bitsRatio[12] = 1;
        bitsRatio[13] = 2;
        bitsRatio[14] = 1;
        bitsRatio[15] = 1;
      }
      else if (bpp > 0.05)
      {
        bitsRatio[0] = 40;
        bitsRatio[1] = 17;
        bitsRatio[2] = 7;
        bitsRatio[3] = 2;
        bitsRatio[4] = 1;
        bitsRatio[5] = 1;
        bitsRatio[6] = 2;
        bitsRatio[7] = 1;
        bitsRatio[8] = 1;
        bitsRatio[9] = 7;
        bitsRatio[10] = 2;
        bitsRatio[11] = 1;
        bitsRatio[12] = 1;
        bitsRatio[13] = 2;
        bitsRatio[14] = 1;
        bitsRatio[15] = 1;
      }
      else
      {
        bitsRatio[0] = 40;
        bitsRatio[1] = 15;
        bitsRatio[2] = 6;
        bitsRatio[3] = 3;
        bitsRatio[4] = 1;
        bitsRatio[5] = 1;
        bitsRatio[6] = 3;
        bitsRatio[7] = 1;
        bitsRatio[8] = 1;
        bitsRatio[9] = 6;
        bitsRatio[10] = 3;
        bitsRatio[11] = 1;
        bitsRatio[12] = 1;
        bitsRatio[13] = 3;
        bitsRatio[14] = 1;
        bitsRatio[15] = 1;
      }

      if (keepHierBits == 2)
      {
        adaptiveBit = 3;
      }
    }
#endif
    else
    {
      printf( "\n hierarchical bit allocation is not support for the specified coding structure currently.\n" );
    }
  }

  Int* GOPID2Level = new Int[ GOPSize ]; //根据在GOP中的id确定所属的分层
  for ( Int i=0; i<GOPSize; i++ )
  {
    GOPID2Level[i] = 1;
    if ( !GOPList[i].m_refPic )
    {
      GOPID2Level[i] = 2;
    }
  }

  if ( keepHierBits > 0 )
  {
    if ( GOPSize == 4 && isLowdelay )
    {
      GOPID2Level[0] = 3;
      GOPID2Level[1] = 2;
      GOPID2Level[2] = 3;
      GOPID2Level[3] = 1;
    }
    else if ( GOPSize == 8 && !isLowdelay )
    {
      GOPID2Level[0] = 1;
      GOPID2Level[1] = 2;
      GOPID2Level[2] = 3;
      GOPID2Level[3] = 4;
      GOPID2Level[4] = 4;
      GOPID2Level[5] = 3;
      GOPID2Level[6] = 4;
      GOPID2Level[7] = 4;
    }
#if JVET_K0390_RATE_CTRL
    else if (GOPSize == 16 && !isLowdelay)
    {
      GOPID2Level[0] = 1;
      GOPID2Level[1] = 2;
      GOPID2Level[2] = 3;
      GOPID2Level[3] = 4;
      GOPID2Level[4] = 5;
      GOPID2Level[5] = 5;
      GOPID2Level[6] = 4;
      GOPID2Level[7] = 5;
      GOPID2Level[8] = 5;
      GOPID2Level[9] = 3;
      GOPID2Level[10] = 4;
      GOPID2Level[11] = 5;
      GOPID2Level[12] = 5;
      GOPID2Level[13] = 4;
      GOPID2Level[14] = 5;
      GOPID2Level[15] = 5;
    }
#endif
  }

  if ( !isLowdelay && GOPSize == 8 )
  {
    GOPID2Level[0] = 1;
    GOPID2Level[1] = 2;
    GOPID2Level[2] = 3;
    GOPID2Level[3] = 4;
    GOPID2Level[4] = 4;
    GOPID2Level[5] = 3;
    GOPID2Level[6] = 4;
    GOPID2Level[7] = 4;
  }
#if JVET_K0390_RATE_CTRL
  else if (GOPSize == 16 && !isLowdelay)
  {
    GOPID2Level[0] = 1;
    GOPID2Level[1] = 2;
    GOPID2Level[2] = 3;
    GOPID2Level[3] = 4;
    GOPID2Level[4] = 5;
    GOPID2Level[5] = 5;
    GOPID2Level[6] = 4;
    GOPID2Level[7] = 5;
    GOPID2Level[8] = 5;
    GOPID2Level[9] = 3;
    GOPID2Level[10] = 4;
    GOPID2Level[11] = 5;
    GOPID2Level[12] = 5;
    GOPID2Level[13] = 4;
    GOPID2Level[14] = 5;
    GOPID2Level[15] = 5;
  }
#endif

  //!< 序列级RC参数的初始化
  m_encRCSeq = new TEncRCSeq;
  m_encRCSeq->create( totalFrames, targetBitrate, frameRate, GOPSize, picWidth, picHeight, LCUWidth, LCUHeight, numberOfLevel, useLCUSeparateModel, adaptiveBit );
  m_encRCSeq->initBitsRatio( bitsRatio ); //!< 每幅picture的权值
  m_encRCSeq->initGOPID2Level( GOPID2Level ); //设置GOP的Id到Level的映射
  m_encRCSeq->initPicPara(); //帧级别的alpha，beta的参数设置
  if ( useLCUSeparateModel )
  {
    m_encRCSeq->initLCUPara(); //LCU级别的alpha，beta的参数设置
  }
  m_CpbSaturationEnabled = false;
  m_cpbSize              = targetBitrate;
  m_cpbState             = (UInt)(m_cpbSize*0.5f);
  m_bufferingRate        = (Int)(targetBitrate / frameRate);

  delete[] bitsRatio;
  delete[] GOPID2Level;
}

Void TEncRateCtrl::initRCPic( Int frameLevel )
{
  m_encRCPic = new TEncRCPic;
  m_encRCPic->create( m_encRCSeq, m_encRCGOP, frameLevel, m_listRCPictures );
}

Void TEncRateCtrl::initRCGOP( Int numberOfPictures )
{
  m_encRCGOP = new TEncRCGOP;
  m_encRCGOP->create( m_encRCSeq, numberOfPictures );
}

Int  TEncRateCtrl::updateCpbState(Int actualBits)
{
  Int cpbState = 1;

  m_cpbState -= actualBits;
  if (m_cpbState < 0)
  {
    cpbState = -1;
  }

  m_cpbState += m_bufferingRate;
  if (m_cpbState > m_cpbSize)
  {
    cpbState = 0;
  }

  return cpbState;
}

Void TEncRateCtrl::initHrdParam(const TComHRD* pcHrd, Int iFrameRate, Double fInitialCpbFullness)
{
  m_CpbSaturationEnabled = true;
  m_cpbSize = (pcHrd->getCpbSizeValueMinus1(0, 0, 0) + 1) << (4 + pcHrd->getCpbSizeScale());
  m_cpbState = (UInt)(m_cpbSize*fInitialCpbFullness);
  m_bufferingRate = (UInt)(((pcHrd->getBitRateValueMinus1(0, 0, 0) + 1) << (6 + pcHrd->getBitRateScale())) / iFrameRate);
  printf("\nHRD - [Initial CPB state %6d] [CPB Size %6d] [Buffering Rate %6d]\n", m_cpbState, m_cpbSize, m_bufferingRate);
}

Void TEncRateCtrl::destroyRCGOP()
{
  delete m_encRCGOP;
  m_encRCGOP = NULL;
}
