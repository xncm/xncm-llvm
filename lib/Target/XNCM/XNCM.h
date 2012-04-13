//==-- XNCM.h - Top-level interface for XNCM representation --*- C++ -*-==//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in
// the LLVM XNCM backend.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TARGET_XNCM_H
#define LLVM_TARGET_XNCM_H

#include "MCTargetDesc/XNCMMCTargetDesc.h"
#include "llvm/Target/TargetMachine.h"

namespace XNCMCC {
  // XNCM specific condition code.
  enum CondCodes {
    COND_E  = 0,  // aka COND_Z
    COND_NE = 1,  // aka COND_NZ
    COND_HS = 2,  // aka COND_C
    COND_LO = 3,  // aka COND_NC
    COND_GE = 4,
    COND_L  = 5,

    COND_INVALID = -1
  };
}

namespace llvm {
  class XNCMTargetMachine;
  class FunctionPass;
  class formatted_raw_ostream;

  FunctionPass *createXNCMISelDag(XNCMTargetMachine &TM,
                                    CodeGenOpt::Level OptLevel);

  FunctionPass *createXNCMBranchSelectionPass();

} // end namespace llvm;

#endif
