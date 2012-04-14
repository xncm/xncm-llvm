//==- XNCMCallingConv.td - Calling Conventions for XNCM -*- tablegen -*-==//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
// This describes the calling conventions for XNCM architecture.
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// XNCM Return Value Calling Convention
//===----------------------------------------------------------------------===//
def RetCC_XNCM : CallingConv<[
  // i8 are returned in the accumulator
  CCIfType<[i8], CCAssignToReg<[A]>>,

  // i16 are returned in XH:XL
  CCIfType<[i16], CCAssignToReg<[X]>>
]>;

//===----------------------------------------------------------------------===//
// XNCM Argument Calling Conventions
//===----------------------------------------------------------------------===//
def CC_XNCM : CallingConv<[
  // Pass first non-vararg arguments in registers
  CCIfNotVarArg<CCIfType<[i8], CCAssignToReg<[A, XL, XH]>>>,
  CCIfNotVarArg<CCIfType<[i16], CCAssignToReg<[X]>>>,

  // Pass the rest on the stack
  CCIfType<[i8], CCAssignToStack<2, 1>>,
  CCIfType<[i16], CCAssignToStack<2, 1>>
]>;

