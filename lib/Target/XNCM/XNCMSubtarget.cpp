//===-- XNCMSubtarget.cpp - XNCM Subtarget Information ----------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the XNCM specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "XNCMSubtarget.h"
#include "XNCM.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "XNCMGenSubtargetInfo.inc"

using namespace llvm;

void XNCMSubtarget::anchor() { }

XNCMSubtarget::XNCMSubtarget(const std::string &TT,
                                 const std::string &CPU,
                                 const std::string &FS) :
  XNCMGenSubtargetInfo(TT, CPU, FS) {
  std::string CPUName = "generic";

  // Parse features string.
  ParseSubtargetFeatures(CPUName, FS);
}
