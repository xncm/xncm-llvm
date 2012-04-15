//===-- XNCMInstrFormats.td - XNCM Instruction Formats -*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source 
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Describe XNCM instructions format here
//

// Generic XNCM Format
class XNCMInst<dag outs, dag ins, string asmstr, list<dag> pattern> : Instruction {
  let Namespace = "XNCM";

  dag OutOperandList = outs;
  dag InOperandList = ins;

  let AsmString = asmstr;
  let Pattern = pattern;
}

