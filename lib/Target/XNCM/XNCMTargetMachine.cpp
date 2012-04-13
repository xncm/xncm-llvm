//===-- XNCMTargetMachine.cpp - Define TargetMachine for XNCM ---------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Top-level implementation for the XNCM target.
//
//===----------------------------------------------------------------------===//

#include "XNCMTargetMachine.h"
#include "XNCM.h"
#include "llvm/PassManager.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/Support/TargetRegistry.h"
using namespace llvm;

extern "C" void LLVMInitializeXNCMTarget() {
  // Register the target.
  RegisterTargetMachine<XNCMTargetMachine> X(TheXNCMTarget);
}

XNCMTargetMachine::XNCMTargetMachine(const Target &T,
                                         StringRef TT,
                                         StringRef CPU,
                                         StringRef FS,
                                         const TargetOptions &Options,
                                         Reloc::Model RM, CodeModel::Model CM,
                                         CodeGenOpt::Level OL)
  : LLVMTargetMachine(T, TT, CPU, FS, Options, RM, CM, OL),
    Subtarget(TT, CPU, FS),
    // FIXME: Check TargetData string.
    DataLayout("e-p:16:16:16-i8:8:8-i16:16:16-i32:16:32-n8:16"),
    InstrInfo(*this), TLInfo(*this), TSInfo(*this),
    FrameLowering(Subtarget) { }

namespace {
/// XNCM Code Generator Pass Configuration Options.
class XNCMPassConfig : public TargetPassConfig {
public:
  XNCMPassConfig(XNCMTargetMachine *TM, PassManagerBase &PM)
    : TargetPassConfig(TM, PM) {}

  XNCMTargetMachine &getXNCMTargetMachine() const {
    return getTM<XNCMTargetMachine>();
  }

  virtual bool addInstSelector();
  virtual bool addPreEmitPass();
};
} // namespace

TargetPassConfig *XNCMTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new XNCMPassConfig(this, PM);
}

bool XNCMPassConfig::addInstSelector() {
  // Install an instruction selector.
  PM.add(createXNCMISelDag(getXNCMTargetMachine(), getOptLevel()));
  return false;
}

bool XNCMPassConfig::addPreEmitPass() {
  // Must run branch selection immediately preceding the asm printer.
  PM.add(createXNCMBranchSelectionPass());
  return false;
}
