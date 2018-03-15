/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2015, ITU/ISO/IEC
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

/** \file     TEncCu.h
    \brief    Coding Unit (CU) encoder class (header)
*/

#ifndef __TENCCU__
#define __TENCCU__

// Include files
 


#include "TLibCommon/CommonDef.h"
#include "TLibCommon/TComYuv.h"
#include "TLibCommon/TComPrediction.h"
#include "TLibCommon/TComTrQuant.h"
#include "TLibCommon/TComBitCounter.h"
#include "TLibCommon/TComDataCU.h"

#include "TEncEntropy.h"
#include "TEncSearch.h"
#include "TEncRateCtrl.h"
//! \ingroup TLibEncoder
//! \{

//rhevc
#include <iostream>
#include <dlib/svm.h>
#include <dlib/rand.h>
#include <vector>
#include <sys/time.h>

#define FEATURES_NUMBER 3
#define REDUCED_FEATURES_NUMBER 6
#define SVM_ONLINE 1

typedef dlib::matrix<double, FEATURES_NUMBER, 1> sample_type;
typedef dlib::radial_basis_kernel<sample_type> kernel_type;

typedef dlib::matrix<double, REDUCED_FEATURES_NUMBER, 1> reduced_sample_type;
typedef dlib::radial_basis_kernel<reduced_sample_type> reduced_kernel_type;


    // Here we are making an instance of the normalized_function object.  This
    // object provides a convenient way to store the vector normalization
    // information along with the decision function we are going to learn.  
   
//--

class TEncTop;
class TEncSbac;
class TEncCavlc;
class TEncSlice;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// CU encoder class
class TEncCu
{
private:

  TComDataCU**            m_ppcBestCU;      ///< Best CUs in each depth
  TComDataCU**            m_ppcTempCU;      ///< Temporary CUs in each depth
  UChar                   m_uhTotalDepth;

  TComYuv**               m_ppcPredYuvBest; ///< Best Prediction Yuv for each depth
  TComYuv**               m_ppcResiYuvBest; ///< Best Residual Yuv for each depth
  TComYuv**               m_ppcRecoYuvBest; ///< Best Reconstruction Yuv for each depth
  TComYuv**               m_ppcPredYuvTemp; ///< Temporary Prediction Yuv for each depth
  TComYuv**               m_ppcResiYuvTemp; ///< Temporary Residual Yuv for each depth
  TComYuv**               m_ppcRecoYuvTemp; ///< Temporary Reconstruction Yuv for each depth
  TComYuv**               m_ppcOrigYuv;     ///< Original Yuv for each depth

  //  Data : encoder control
  Bool                    m_bEncodeDQP;
  Bool                    m_bFastDeltaQP;
  Bool                    m_stillToCodeChromaQpOffsetFlag; //indicates whether chroma QP offset flag needs to coded at this particular CU granularity.
  Int                     m_cuChromaQpOffsetIdxPlus1; // if 0, then cu_chroma_qp_offset_flag will be 0, otherwise cu_chroma_qp_offset_flag will be 1.

  //  Access channel
  TEncCfg*                m_pcEncCfg;
  TEncSearch*             m_pcPredSearch;
  TComTrQuant*            m_pcTrQuant;
  TComRdCost*             m_pcRdCost;

  TEncEntropy*            m_pcEntropyCoder;
  TEncBinCABAC*           m_pcBinCABAC;

  // SBAC RD
  TEncSbac***             m_pppcRDSbacCoder;
  TEncSbac*               m_pcRDGoOnSbacCoder;
  TEncRateCtrl*           m_pcRateCtrl;
  //rhevc
  Int                     rhevc_countInter;
  Int                     rhevc_countIntra;
  dlib::svm_c_trainer<reduced_kernel_type>* r_trainer;
  dlib::svm_c_trainer<kernel_type>*         trainer;
  dlib::decision_function<kernel_type>* learned_function;
  std::vector<double> classification;;
  double best_c, best_gamma;
  sample_type samp;
  bool ativeValidation;
  double tempo_inicial;
  bool chave;
  bool is2Nx2N;
  bool isTraining;
  std::vector<sample_type> lastSample;
  std::vector<sample_type> lastSampleTime;
  int validationSize;

  struct timeval start, end;

  //--

public:
  Int cont_train;
  Int contteste;
  //rhevc
  void writeDataFileLibSVMFormat(double *features, int features_number, double *labels, int labels_number, bool isTraining);
  //--
  /// copy parameters from encoder class
  Void  init                ( TEncTop* pcEncTop );

  /// create internal buffers
  Void  create              ( UChar uhTotalDepth, UInt iMaxWidth, UInt iMaxHeight, ChromaFormat chromaFormat );

  /// destroy internal buffers
  Void  destroy             ();

  /// CTU analysis function
  Void  compressCtu         ( TComDataCU*  pCtu );

  /// CTU encoding function
  Void  encodeCtu           ( TComDataCU*  pCtu );

  Int   updateCtuDataISlice ( TComDataCU* pCtu, Int width, Int height );

  Void setFastDeltaQp       ( Bool b)                 { m_bFastDeltaQP = b;         }

protected:
  Void  finishCU            ( TComDataCU*  pcCU, UInt uiAbsPartIdx );
#if AMP_ENC_SPEEDUP
  Void  xCompressCU         ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, const UInt uiDepth DEBUG_STRING_FN_DECLARE(sDebug), PartSize eParentPartSize = NUMBER_OF_PART_SIZES );
#else
  Void  xCompressCU         ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, const UInt uiDepth        );
#endif
  Void  xEncodeCU           ( TComDataCU*  pcCU, UInt uiAbsPartIdx,           UInt uiDepth        );

  Int   xComputeQP          ( TComDataCU* pcCU, UInt uiDepth );
  Void  xCheckBestMode      ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, UInt uiDepth DEBUG_STRING_FN_DECLARE(sParent) DEBUG_STRING_FN_DECLARE(sTest) DEBUG_STRING_PASS_INTO(Bool bAddSizeInfo=true));

  Void  xCheckRDCostMerge2Nx2N( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU DEBUG_STRING_FN_DECLARE(sDebug), Bool *earlyDetectionSkipMode );

#if AMP_MRG
  Void  xCheckRDCostInter   ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize DEBUG_STRING_FN_DECLARE(sDebug), Bool bUseMRG = false  );
#else
  Void  xCheckRDCostInter   ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU, PartSize ePartSize  );
#endif

  Void  xCheckRDCostIntra   ( TComDataCU *&rpcBestCU,
                              TComDataCU *&rpcTempCU,
                              Double      &cost,
                              PartSize     ePartSize
                              DEBUG_STRING_FN_DECLARE(sDebug)
                            );

  Void  xCheckDQP           ( TComDataCU*  pcCU );

  Void  xCheckIntraPCM      ( TComDataCU*& rpcBestCU, TComDataCU*& rpcTempCU                      );
  Void  xCopyAMVPInfo       ( AMVPInfo* pSrc, AMVPInfo* pDst );
  Void  xCopyYuv2Pic        (TComPic* rpcPic, UInt uiCUAddr, UInt uiAbsPartIdx, UInt uiDepth, UInt uiSrcDepth );
  Void  xCopyYuv2Tmp        ( UInt uhPartUnitIdx, UInt uiDepth );

  Bool getdQPFlag           ()                        { return m_bEncodeDQP;        }
  Void setdQPFlag           ( Bool b )                { m_bEncodeDQP = b;           }

  Bool getFastDeltaQp       () const                  { return m_bFastDeltaQP;      }

  Bool getCodeChromaQpAdjFlag() { return m_stillToCodeChromaQpOffsetFlag; }
  Void setCodeChromaQpAdjFlag( Bool b ) { m_stillToCodeChromaQpOffsetFlag = b; }

#if ADAPTIVE_QP_SELECTION
  // Adaptive reconstruction level (ARL) statistics collection functions
  Void xCtuCollectARLStats(TComDataCU* pCtu);
  Int  xTuCollectARLStats(TCoeff* rpcCoeff, TCoeff* rpcArlCoeff, Int NumCoeffInCU, Double* cSum, UInt* numSamples );
#endif

#if AMP_ENC_SPEEDUP
#if AMP_MRG
  Void deriveTestModeAMP (TComDataCU *pcBestCU, PartSize eParentPartSize, Bool &bTestAMP_Hor, Bool &bTestAMP_Ver, Bool &bTestMergeAMP_Hor, Bool &bTestMergeAMP_Ver);
#else
  Void deriveTestModeAMP (TComDataCU *pcBestCU, PartSize eParentPartSize, Bool &bTestAMP_Hor, Bool &bTestAMP_Ver);
#endif
#endif

  Void  xFillPCMBuffer     ( TComDataCU* pCU, TComYuv* pOrgYuv );
};

//! \}

#endif // __TENCMB__

// if( rpcBestCU->getSlice()->getSliceType() != I_SLICE && chave && cont_train%100) { 
//       dlib::svm_c_trainer<kernel_type>        trainer2;
//       if(((TEncTop*) m_pcEncCfg)->isTraining()){     
//           // dlib::matrix<double> best_result(2,1);
          // dlib::matrix<double> result(2,1);
          // for (double gamma = 0.0001; gamma <= 1; gamma *= 10)       {
          //     for (double C = 1; C <= 10000; C *= 10) {
          //         dlib::svm_c_trainer<kernel_type>        temp_trainer;
          //         temp_trainer.set_kernel(kernel_type(gamma));
          //         temp_trainer.set_c(C);
          //         result = cross_validate_trainer(temp_trainer, lastSample, classification, 3);
          //         if (dlib::sum(result) > dlib::sum(best_result)) {
          //             best_result = result;
          //             best_gamma = gamma;
          //             best_c = C;
          //         }
          //     }
          // }
          
  //         trainer->set_kernel(kernel_type(best_gamma));
  //         trainer->set_c(best_c);
          

  //         learned_function = trainer->train(lastSample, classification);
          
  //         for (int i = 0; i < classification.size(); ++i) {
  //             ((TEncTop*) m_pcEncCfg)->calcSVMClassError((int)classification[i], (learned_function(lastSample[i]))> 0 ? 1 : -1);
  //         }

  //         if(cont_train % 15000 == 0){

  //             cout << "Beste Gamma: " << best_gamma << " -- Best C: " << best_c << "size " << lastSample.size() <<  endl;

  //             double nErrors = ((TEncTop*)m_pcEncCfg)->getnErrorClassification();
  //             //cout << samp << endl;
  //             cout << "This is a " << classification[0] << " example, its SVM C output is: " << learned_function(samp) << " " << cont_train << endl;
  //             m_pcEncCfg->getOfstream() << (nErrors / (double)cont_train)*100 << endl;
  //             lastSample.erase (lastSample.begin(), lastSample.end());
  //             classification.erase (classification.begin(), classification.end());  
              
  //             gettimeofday(&end, NULL);
  //             double temp = ((end.tv_sec  - start.tv_sec) * 1000000u + end.tv_usec - start.tv_usec) / 1.e6;
  //             cout << "Amostra: " << cont_train << " -- Tempo: " <<temp << " -- Error Ratio: " << (nErrors / (double)cont_train)*100 << "%" << endl;
  //             cout << "-------------------------------------------" << endl;
  //         }else{
  //             double nErrors = ((TEncTop*)m_pcEncCfg)->getnErrorClassification();
  //             m_pcEncCfg->getOfstream() << (nErrors / (double)cont_train)*100 << endl;
  //             lastSample.erase (lastSample.begin(), lastSample.end());
  //             classification.erase (classification.begin(), classification.end());  
  //            // gettimeofday(&end, NULL);
  //             //double temp = ((end.tv_sec  - start.tv_sec) * 1000000u + end.tv_usec - start.tv_usec) / 1.e6;
  //         }
  //     }      
  // }