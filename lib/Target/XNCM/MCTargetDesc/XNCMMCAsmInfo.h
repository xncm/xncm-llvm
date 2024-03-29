//===-- XNCMMCAsmInfo.h - XNCM asm properties --------------*- C++ -*--===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source 
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of the XNCMMCAsmInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef XNCMTARGETASMINFO_H
#define XNCMTARGETASMINFO_H

#include "llvm/ADT/StringRef.h"
#include "llvm/MC/MCAsmInfo.h"

namespace llvm {
  class Target;

  class XNCMMCAsmInfo : public MCAsmInfo {
    virtual void anchor();
  public:
    explicit XNCMMCAsmInfo(const Target &T, StringRef TT);
  };

} // namespace llvm

#endif
