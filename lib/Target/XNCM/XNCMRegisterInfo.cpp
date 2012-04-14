//===-- XNCMRegisterInfo.cpp - XNCM Register Information --------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the XNCM implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "xncm-reg-info"

#include "XNCMRegisterInfo.h"
#include "XNCM.h"
#include "XNCMMachineFunctionInfo.h"
#include "XNCMTargetMachine.h"
#include "llvm/Function.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetOptions.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/Support/ErrorHandling.h"

#define GET_REGINFO_TARGET_DESC
#include "XNCMGenRegisterInfo.inc"

using namespace llvm;

// FIXME: Provide proper call frame setup / destroy opcodes.
XNCMRegisterInfo::XNCMRegisterInfo(XNCMTargetMachine &tm,
                                       const TargetInstrInfo &tii)
  : XNCMGenRegisterInfo(XNCM::IP), TM(tm), TII(tii) {
  StackAlign = TM.getFrameLowering()->getStackAlignment();
}

const uint16_t*
XNCMRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  static const uint16_t CalleeSavedRegs[] = {
    XNCM::Y, XNCM::YL, XNCM::YH,
    0
  };
  return CalleeSavedRegs;
}

BitVector XNCMRegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  BitVector Reserved(getNumRegs());

  // Mark 4 special registers with subregisters as reserved.
  //Reserved.set(XNCM::PCB);
  //Reserved.set(XNCM::SPB);
  //Reserved.set(XNCM::SRB);
  //Reserved.set(XNCM::CGB);
  //Reserved.set(XNCM::PCW);
  //Reserved.set(XNCM::SPW);
  //Reserved.set(XNCM::SRW);
  //Reserved.set(XNCM::CGW);

  return Reserved;
}

const TargetRegisterClass *
XNCMRegisterInfo::getPointerRegClass(unsigned Kind) const {
  return &XNCM::GR16RegClass;
}

void XNCMRegisterInfo::
eliminateCallFramePseudoInstr(MachineFunction &MF, MachineBasicBlock &MBB,
                              MachineBasicBlock::iterator I) const {
  llvm_unreachable(0);
}

void
XNCMRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                        int SPAdj, RegScavenger *RS) const {
  llvm_unreachable(0);
}

void
XNCMRegisterInfo::processFunctionBeforeFrameFinalized(MachineFunction &MF)
                                                                         const {
  const TargetFrameLowering *TFI = MF.getTarget().getFrameLowering();

  // Create a frame entry for the FPW register that must be saved.
  if (TFI->hasFP(MF)) {
    int FrameIdx = MF.getFrameInfo()->CreateFixedObject(2, -4, true);
    (void)FrameIdx;
    assert(FrameIdx == MF.getFrameInfo()->getObjectIndexBegin() &&
           "Slot for FPW register must be last in order to be found!");
  }
}

unsigned XNCMRegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  llvm_unreachable(0);
}
