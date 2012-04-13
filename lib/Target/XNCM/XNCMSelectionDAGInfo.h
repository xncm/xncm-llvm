//===-- XNCMSelectionDAGInfo.h - XNCM SelectionDAG Info -----*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the XNCM subclass for TargetSelectionDAGInfo.
//
//===----------------------------------------------------------------------===//

#ifndef XNCMSELECTIONDAGINFO_H
#define XNCMSELECTIONDAGINFO_H

#include "llvm/Target/TargetSelectionDAGInfo.h"

namespace llvm {

class XNCMTargetMachine;

class XNCMSelectionDAGInfo : public TargetSelectionDAGInfo {
public:
  explicit XNCMSelectionDAGInfo(const XNCMTargetMachine &TM);
  ~XNCMSelectionDAGInfo();
};

}

#endif
