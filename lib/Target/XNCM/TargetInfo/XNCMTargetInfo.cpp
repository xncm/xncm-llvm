//===-- XNCMTargetInfo.cpp - XNCM Target Implementation ---------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "XNCM.h"
#include "llvm/Module.h"
#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

Target llvm::TheXNCMTarget;

extern "C" void LLVMInitializeXNCMTargetInfo() { 
  RegisterTarget<Triple::xncm> 
    X(TheXNCMTarget, "xncm", "XNCM [experimental]");
}
