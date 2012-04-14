//===-- XNCMMCTargetDesc.cpp - XNCM Target Descriptions ---------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file provides XNCM specific target descriptions.
//
//===----------------------------------------------------------------------===//

#include "XNCMMCTargetDesc.h"
#include "XNCMMCAsmInfo.h"
#include "InstPrinter/XNCMInstPrinter.h"
#include "llvm/MC/MCCodeGenInfo.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_INSTRINFO_MC_DESC
#include "XNCMGenInstrInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "XNCMGenSubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "XNCMGenRegisterInfo.inc"

using namespace llvm;

static MCInstrInfo *createXNCMMCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitXNCMMCInstrInfo(X);
  return X;
}

static MCRegisterInfo *createXNCMMCRegisterInfo(StringRef TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitXNCMMCRegisterInfo(X, XNCM::IP);
  return X;
}

static MCSubtargetInfo *createXNCMMCSubtargetInfo(StringRef TT, StringRef CPU,
                                                    StringRef FS) {
  MCSubtargetInfo *X = new MCSubtargetInfo();
  InitXNCMMCSubtargetInfo(X, TT, CPU, FS);
  return X;
}

static MCCodeGenInfo *createXNCMMCCodeGenInfo(StringRef TT, Reloc::Model RM,
                                                CodeModel::Model CM,
                                                CodeGenOpt::Level OL) {
  MCCodeGenInfo *X = new MCCodeGenInfo();
  X->InitMCCodeGenInfo(RM, CM, OL);
  return X;
}

static MCInstPrinter *createXNCMMCInstPrinter(const Target &T,
                                                unsigned SyntaxVariant,
                                                const MCAsmInfo &MAI,
                                                const MCRegisterInfo &MRI,
                                                const MCSubtargetInfo &STI) {
  if (SyntaxVariant == 0)
    return new XNCMInstPrinter(MAI, MRI);
  return 0;
}

extern "C" void LLVMInitializeXNCMTargetMC() {
  // Register the MC asm info.
  RegisterMCAsmInfo<XNCMMCAsmInfo> X(TheXNCMTarget);

  // Register the MC codegen info.
  TargetRegistry::RegisterMCCodeGenInfo(TheXNCMTarget,
                                        createXNCMMCCodeGenInfo);

  // Register the MC instruction info.
  TargetRegistry::RegisterMCInstrInfo(TheXNCMTarget, createXNCMMCInstrInfo);

  // Register the MC register info.
  TargetRegistry::RegisterMCRegInfo(TheXNCMTarget,
                                    createXNCMMCRegisterInfo);

  // Register the MC subtarget info.
  TargetRegistry::RegisterMCSubtargetInfo(TheXNCMTarget,
                                          createXNCMMCSubtargetInfo);

  // Register the MCInstPrinter.
  TargetRegistry::RegisterMCInstPrinter(TheXNCMTarget,
                                        createXNCMMCInstPrinter);
}
