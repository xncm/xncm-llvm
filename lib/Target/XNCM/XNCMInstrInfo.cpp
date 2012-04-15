//===-- XNCMInstrInfo.cpp - XNCM Instruction Information --------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the XNCM implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "XNCMInstrInfo.h"
#include "XNCM.h"
#include "XNCMMachineFunctionInfo.h"
#include "XNCMTargetMachine.h"
#include "llvm/Function.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

#define GET_INSTRINFO_CTOR
#include "XNCMGenInstrInfo.inc"

using namespace llvm;

XNCMInstrInfo::XNCMInstrInfo(XNCMTargetMachine &tm)
  : XNCMGenInstrInfo(XNCM::ADJCALLSTACKDOWN, XNCM::ADJCALLSTACKUP),
    RI(tm, *this), TM(tm) {}

void XNCMInstrInfo::storeRegToStackSlot(MachineBasicBlock &MBB,
                                          MachineBasicBlock::iterator MI,
                                    unsigned SrcReg, bool isKill, int FrameIdx,
                                          const TargetRegisterClass *RC,
                                          const TargetRegisterInfo *TRI) const {
    llvm_unreachable("Cannot store this register to stack slot!");
}

void XNCMInstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                           MachineBasicBlock::iterator MI,
                                           unsigned DestReg, int FrameIdx,
                                           const TargetRegisterClass *RC,
                                           const TargetRegisterInfo *TRI) const{
    llvm_unreachable("Cannot load this register from stack slot!");
}

void XNCMInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                                  MachineBasicBlock::iterator I, DebugLoc DL,
                                  unsigned DestReg, unsigned SrcReg,
                                  bool KillSrc) const {
  llvm_unreachable("Impossible reg-to-reg copy");
}

bool XNCMInstrInfo::
ReverseBranchCondition(SmallVectorImpl<MachineOperand> &Cond) const {
  assert(Cond.size() == 1 && "Invalid Xbranch condition!");

  XNCMCC::CondCodes CC = static_cast<XNCMCC::CondCodes>(Cond[0].getImm());

  switch (CC) {
  default: llvm_unreachable("Invalid branch condition!");
  case XNCMCC::COND_E:
    CC = XNCMCC::COND_NE;
    break;
  case XNCMCC::COND_NE:
    CC = XNCMCC::COND_E;
    break;
  case XNCMCC::COND_L:
    CC = XNCMCC::COND_GE;
    break;
  case XNCMCC::COND_GE:
    CC = XNCMCC::COND_L;
    break;
  case XNCMCC::COND_HS:
    CC = XNCMCC::COND_LO;
    break;
  case XNCMCC::COND_LO:
    CC = XNCMCC::COND_HS;
    break;
  }

  Cond[0].setImm(CC);
  return false;
}

bool XNCMInstrInfo::isUnpredicatedTerminator(const MachineInstr *MI) const {
  if (!MI->isTerminator()) return false;

  // Conditional branch is a special case.
  if (MI->isBranch() && !MI->isBarrier())
    return true;
  if (!MI->isPredicable())
    return true;
  return !isPredicated(MI);
}

/// GetInstSize - Return the number of bytes of code the specified
/// instruction may be.  This returns the maximum number of bytes.
///
unsigned XNCMInstrInfo::GetInstSizeInBytes(const MachineInstr *MI) const {
  llvm_unreachable(0);
}
