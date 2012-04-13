//===-- XNCMMCTargetDesc.h - XNCM Target Descriptions -------*- C++ -*-===//
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

#ifndef XNCMMCTARGETDESC_H
#define XNCMMCTARGETDESC_H

namespace llvm {
class MCSubtargetInfo;
class Target;
class StringRef;

extern Target TheXNCMTarget;

} // End llvm namespace

// Defines symbolic names for XNCM registers.
// This defines a mapping from register name to register number.
#define GET_REGINFO_ENUM
#include "XNCMGenRegisterInfo.inc"

// Defines symbolic names for the XNCM instructions.
#define GET_INSTRINFO_ENUM
#include "XNCMGenInstrInfo.inc"

#define GET_SUBTARGETINFO_ENUM
#include "XNCMGenSubtargetInfo.inc"

#endif
