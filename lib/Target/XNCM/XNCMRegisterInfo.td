//===-- XNCMRegisterInfo.td - XNCM Register defs -------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source 
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

let Namespace = "XNCM" in
{
	def A : Register<"a">;

	def XL : Register<"xl">;
	def XH : Register<"xh">;
	def YL : Register<"yl">;
	def YH : Register<"yh">;

	def sub_8bit_lo : SubRegIndex;
	def sub_8bit_hi : SubRegIndex;

	let SubRegIndices = [sub_8bit_lo, sub_8bit_hi], CoveredBySubRegs = 1 in {
		def X : RegisterWithSubRegs<"x", [XL, XH]>;
		def Y : RegisterWithSubRegs<"y", [YL, YH]>;
	}

	def IP : Register<"ip">;
	def SP : Register<"sp">;
	def LR : Register<"lr">;

	def GR8 : RegisterClass<"XNCM", [i8], 8, (add A, XL, XH, YL, YH)>;
	def GR16 : RegisterClass<"XNCM", [i16], 16, (add X, Y, IP, SP, LR)>;
}

