//===-- XNCMInstrInfo.td - XNCM Instruction defs -------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source 
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file describes the XNCM instructions in TableGen format.
//
//===----------------------------------------------------------------------===//

include "XNCMInstrFormats.td"

//===----------------------------------------------------------------------===//
// Type Constraints.
//===----------------------------------------------------------------------===//
class SDTCisI8<int OpNum> : SDTCisVT<OpNum, i8>;
class SDTCisI16<int OpNum> : SDTCisVT<OpNum, i16>;

//===----------------------------------------------------------------------===//
// Type Profiles.
//===----------------------------------------------------------------------===//
def SDT_XNCMCall         : SDTypeProfile<0, -1, [SDTCisVT<0, iPTR>]>;
def SDT_XNCMCallSeqStart : SDCallSeqStart<[SDTCisVT<0, i16>]>;
def SDT_XNCMCallSeqEnd   : SDCallSeqEnd<[SDTCisVT<0, i16>, SDTCisVT<1, i16>]>;
def SDT_XNCMWrapper      : SDTypeProfile<1, 1, [SDTCisSameAs<0, 1>,
                                                  SDTCisPtrTy<0>]>;
def SDT_XNCMCmp          : SDTypeProfile<0, 2, [SDTCisSameAs<0, 1>]>;
def SDT_XNCMBrCC         : SDTypeProfile<0, 2, [SDTCisVT<0, OtherVT>,
                                                  SDTCisVT<1, i8>]>;
def SDT_XNCMSelectCC     : SDTypeProfile<1, 3, [SDTCisSameAs<0, 1>,
                                                  SDTCisSameAs<1, 2>, 
                                                  SDTCisVT<3, i8>]>;
def SDT_XNCMShift        : SDTypeProfile<1, 2, [SDTCisSameAs<0, 1>,
                                                  SDTCisI8<2>]>;

//===----------------------------------------------------------------------===//
// XNCM Specific Node Definitions.
//===----------------------------------------------------------------------===//
def XNCMretflag  : SDNode<"XNCMISD::RET_FLAG", SDTNone,
                       [SDNPHasChain, SDNPOptInGlue]>;
def XNCMretiflag : SDNode<"XNCMISD::RETI_FLAG", SDTNone,
                       [SDNPHasChain, SDNPOptInGlue]>;

def XNCMrra     : SDNode<"XNCMISD::RRA", SDTIntUnaryOp, []>;
def XNCMrla     : SDNode<"XNCMISD::RLA", SDTIntUnaryOp, []>;
def XNCMrrc     : SDNode<"XNCMISD::RRC", SDTIntUnaryOp, []>;

def XNCMcall    : SDNode<"XNCMISD::CALL", SDT_XNCMCall,
                     [SDNPHasChain, SDNPOutGlue, SDNPOptInGlue, SDNPVariadic]>;
def XNCMcallseq_start :
                 SDNode<"ISD::CALLSEQ_START", SDT_XNCMCallSeqStart,
                        [SDNPHasChain, SDNPOutGlue]>;
def XNCMcallseq_end :
                 SDNode<"ISD::CALLSEQ_END",   SDT_XNCMCallSeqEnd,
                        [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue]>;
def XNCMWrapper : SDNode<"XNCMISD::Wrapper", SDT_XNCMWrapper>;
def XNCMcmp     : SDNode<"XNCMISD::CMP", SDT_XNCMCmp, [SDNPOutGlue]>;
def XNCMbrcc    : SDNode<"XNCMISD::BR_CC", SDT_XNCMBrCC,
                            [SDNPHasChain, SDNPInGlue]>;
def XNCMselectcc: SDNode<"XNCMISD::SELECT_CC", SDT_XNCMSelectCC,
                            [SDNPInGlue]>;
def XNCMshl     : SDNode<"XNCMISD::SHL", SDT_XNCMShift, []>;
def XNCMsra     : SDNode<"XNCMISD::SRA", SDT_XNCMShift, []>;
def XNCMsrl     : SDNode<"XNCMISD::SRL", SDT_XNCMShift, []>;

//===----------------------------------------------------------------------===//
// XNCM Operand Definitions.
//===----------------------------------------------------------------------===//

// Address operands
def memsrc : Operand<i16> {
  let PrintMethod = "printSrcMemOperand";
  let MIOperandInfo = (ops GR16, i16imm);
}

def memdst : Operand<i16> {
  let PrintMethod = "printSrcMemOperand";
  let MIOperandInfo = (ops GR16, i16imm);
}

// Short jump targets have OtherVT type and are printed as pcrel imm values.
def jmptarget : Operand<OtherVT> {
  let PrintMethod = "printPCRelImmOperand";
}

// Operand for printing out a condition code.
def cc : Operand<i8> {
  let PrintMethod = "printCCOperand";
}

//===----------------------------------------------------------------------===//
// XNCM Complex Pattern Definitions.
//===----------------------------------------------------------------------===//

def addr : ComplexPattern<iPTR, 2, "SelectAddr", [], []>;

//===----------------------------------------------------------------------===//
// Pattern Fragments
def zextloadi16i8 : PatFrag<(ops node:$ptr), (i16 (zextloadi8 node:$ptr))>;
def  extloadi16i8 : PatFrag<(ops node:$ptr), (i16 ( extloadi8 node:$ptr))>;
def and_su : PatFrag<(ops node:$lhs, node:$rhs), (and node:$lhs, node:$rhs), [{
  return N->hasOneUse();
}]>;
//===----------------------------------------------------------------------===//
// Instruction list..

// ADJCALLSTACKDOWN/UP implicitly use/def SP because they may be expanded into
// a stack adjustment and the codegen must know that they may modify the stack
// pointer before prolog-epilog rewriting occurs.
let Defs = [SP] in {
	def ADJCALLSTACKDOWN : Pseudo<(outs), (ins i16imm:$amt),
				      "#ADJCALLSTACKDOWN",
				      [(XNCMcallseq_start timm:$amt)]>;
	def ADJCALLSTACKUP   : Pseudo<(outs), (ins i16imm:$amt1, i16imm:$amt2),
				      "#ADJCALLSTACKUP",
				      [(XNCMcallseq_end timm:$amt1, timm:$amt2)]>;
}

let isReturn = 1, isTerminator = 1, isBarrier = 1 in {
	def RET  : IForm16<0x0, DstReg, SrcPostInc, Size2Bytes,
                     (outs), (ins), "ret",  [(XNCMretflag)]>;
}

