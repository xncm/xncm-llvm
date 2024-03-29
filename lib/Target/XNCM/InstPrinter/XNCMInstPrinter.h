//= XNCMInstPrinter.h - Convert XNCM MCInst to assembly syntax -*- C++ -*-//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This class prints a XNCM MCInst to a .s file.
//
//===----------------------------------------------------------------------===//

#ifndef XNCMINSTPRINTER_H
#define XNCMINSTPRINTER_H

#include "llvm/MC/MCInstPrinter.h"

namespace llvm {
  class MCOperand;

  class XNCMInstPrinter : public MCInstPrinter {
  public:
    XNCMInstPrinter(const MCAsmInfo &MAI, const MCRegisterInfo &MRI)
        : MCInstPrinter(MAI, MRI) {}

    virtual void printInst(const MCInst *MI, raw_ostream &O, StringRef Annot);

    // Autogenerated by tblgen.
    void printInstruction(const MCInst *MI, raw_ostream &O);
    static const char *getRegisterName(unsigned RegNo);

    void printOperand(const MCInst *MI, unsigned OpNo, raw_ostream &O,
                      const char *Modifier = 0);
    void printPCRelImmOperand(const MCInst *MI, unsigned OpNo, raw_ostream &O);
    void printSrcMemOperand(const MCInst *MI, unsigned OpNo, raw_ostream &O,
                            const char *Modifier = 0);
    void printCCOperand(const MCInst *MI, unsigned OpNo, raw_ostream &O);

  };
}

#endif
