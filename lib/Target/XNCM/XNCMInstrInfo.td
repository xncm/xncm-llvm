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

//===----------------------------------------------------------------------===//
// XNCM Specific Node Definitions.
//===----------------------------------------------------------------------===//
def XNCMretflag  : SDNode<"XNCMISD::RET_FLAG", SDTNone,
                       [SDNPHasChain, SDNPOptInGlue]>;
def XNCMcallseq_start :
                 SDNode<"ISD::CALLSEQ_START", SDT_XNCMCallSeqStart,
                        [SDNPHasChain, SDNPOutGlue]>;
def XNCMcallseq_end :
                 SDNode<"ISD::CALLSEQ_END",   SDT_XNCMCallSeqEnd,
                        [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue]>;

//===----------------------------------------------------------------------===//
// XNCM Operand Definitions.
//===----------------------------------------------------------------------===//

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

let Uses = [LR], isReturn = 1, isTerminator = 1, isBarrier = 1 in {
	def RET  : IForm16<0x0, DstReg, SrcPostInc, Size2Bytes,
                     (outs), (ins), "ret",  [(XNCMretflag)]>;
}

