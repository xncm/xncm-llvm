//===-- XNCMSelectionDAGInfo.cpp - XNCM SelectionDAG Info -------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the XNCMSelectionDAGInfo class.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "xncm-selectiondag-info"
#include "XNCMTargetMachine.h"
using namespace llvm;

XNCMSelectionDAGInfo::XNCMSelectionDAGInfo(const XNCMTargetMachine &TM)
  : TargetSelectionDAGInfo(TM) {
}

XNCMSelectionDAGInfo::~XNCMSelectionDAGInfo() {
}
