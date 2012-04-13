//===- XNCMMachineFuctionInfo.h - XNCM machine function info -*- C++ -*-==//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares XNCM-specific per-machine-function information.
//
//===----------------------------------------------------------------------===//

#ifndef XNCMMACHINEFUNCTIONINFO_H
#define XNCMMACHINEFUNCTIONINFO_H

#include "llvm/CodeGen/MachineFunction.h"

namespace llvm {

/// XNCMMachineFunctionInfo - This class is derived from MachineFunction and
/// contains private XNCM target-specific information for each MachineFunction.
class XNCMMachineFunctionInfo : public MachineFunctionInfo {
  virtual void anchor();

  /// CalleeSavedFrameSize - Size of the callee-saved register portion of the
  /// stack frame in bytes.
  unsigned CalleeSavedFrameSize;

  /// ReturnAddrIndex - FrameIndex for return slot.
  int ReturnAddrIndex;

public:
  XNCMMachineFunctionInfo() : CalleeSavedFrameSize(0) {}

  explicit XNCMMachineFunctionInfo(MachineFunction &MF)
    : CalleeSavedFrameSize(0), ReturnAddrIndex(0) {}

  unsigned getCalleeSavedFrameSize() const { return CalleeSavedFrameSize; }
  void setCalleeSavedFrameSize(unsigned bytes) { CalleeSavedFrameSize = bytes; }

  int getRAIndex() const { return ReturnAddrIndex; }
  void setRAIndex(int Index) { ReturnAddrIndex = Index; }
};

} // End llvm namespace

#endif
