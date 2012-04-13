//===-- XNCMTargetMachine.h - Define TargetMachine for XNCM -*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the XNCM specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//


#ifndef LLVM_TARGET_XNCM_TARGETMACHINE_H
#define LLVM_TARGET_XNCM_TARGETMACHINE_H

#include "XNCMInstrInfo.h"
#include "XNCMISelLowering.h"
#include "XNCMFrameLowering.h"
#include "XNCMSelectionDAGInfo.h"
#include "XNCMRegisterInfo.h"
#include "XNCMSubtarget.h"
#include "llvm/Target/TargetData.h"
#include "llvm/Target/TargetFrameLowering.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {

/// XNCMTargetMachine
///
class XNCMTargetMachine : public LLVMTargetMachine {
  XNCMSubtarget        Subtarget;
  const TargetData       DataLayout;       // Calculates type size & alignment
  XNCMInstrInfo        InstrInfo;
  XNCMTargetLowering   TLInfo;
  XNCMSelectionDAGInfo TSInfo;
  XNCMFrameLowering    FrameLowering;

public:
  XNCMTargetMachine(const Target &T, StringRef TT,
                      StringRef CPU, StringRef FS, const TargetOptions &Options,
                      Reloc::Model RM, CodeModel::Model CM,
                      CodeGenOpt::Level OL);

  virtual const TargetFrameLowering *getFrameLowering() const {
    return &FrameLowering;
  }
  virtual const XNCMInstrInfo *getInstrInfo() const  { return &InstrInfo; }
  virtual const TargetData *getTargetData() const     { return &DataLayout;}
  virtual const XNCMSubtarget *getSubtargetImpl() const { return &Subtarget; }

  virtual const TargetRegisterInfo *getRegisterInfo() const {
    return &InstrInfo.getRegisterInfo();
  }

  virtual const XNCMTargetLowering *getTargetLowering() const {
    return &TLInfo;
  }

  virtual const XNCMSelectionDAGInfo* getSelectionDAGInfo() const {
    return &TSInfo;
  }

  virtual TargetPassConfig *createPassConfig(PassManagerBase &PM);
}; // XNCMTargetMachine.

} // end namespace llvm

#endif // LLVM_TARGET_XNCM_TARGETMACHINE_H
