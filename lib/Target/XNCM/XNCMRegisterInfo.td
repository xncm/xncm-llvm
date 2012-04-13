//===-- XNCMRegisterInfo.td - XNCM Register defs -------*- tablegen -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source 
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Declarations that describe the XNCM register file
//===----------------------------------------------------------------------===//

class XNCMReg<bits<4> num, string n> : Register<n> {
  field bits<4> Num = num;
  let Namespace = "XNCM";
}

class XNCMRegWithSubregs<bits<4> num, string n, list<Register> subregs> 
  : RegisterWithSubRegs<n, subregs> {
  field bits<4> Num = num;
  let Namespace = "XNCM";
}

//===----------------------------------------------------------------------===//
//  Registers
//===----------------------------------------------------------------------===//

def PCB  : XNCMReg<0,  "r0">;
def SPB  : XNCMReg<1,  "r1">;
def SRB  : XNCMReg<2,  "r2">;
def CGB  : XNCMReg<3,  "r3">;
def FPB  : XNCMReg<4,  "r4">;
def R5B  : XNCMReg<5,  "r5">;
def R6B  : XNCMReg<6,  "r6">;
def R7B  : XNCMReg<7,  "r7">;
def R8B  : XNCMReg<8,  "r8">;
def R9B  : XNCMReg<9,  "r9">;
def R10B : XNCMReg<10, "r10">;
def R11B : XNCMReg<11, "r11">;
def R12B : XNCMReg<12, "r12">;
def R13B : XNCMReg<13, "r13">;
def R14B : XNCMReg<14, "r14">;
def R15B : XNCMReg<15, "r15">;

def subreg_8bit : SubRegIndex { let Namespace = "XNCM"; }

let SubRegIndices = [subreg_8bit] in {
def PCW  : XNCMRegWithSubregs<0,  "r0",  [PCB]>;
def SPW  : XNCMRegWithSubregs<1,  "r1",  [SPB]>;
def SRW  : XNCMRegWithSubregs<2,  "r2",  [SRB]>;
def CGW  : XNCMRegWithSubregs<3,  "r3",  [CGB]>;
def FPW  : XNCMRegWithSubregs<4,  "r4",  [FPB]>;
def R5W  : XNCMRegWithSubregs<5,  "r5",  [R5B]>;
def R6W  : XNCMRegWithSubregs<6,  "r6",  [R6B]>;
def R7W  : XNCMRegWithSubregs<7,  "r7",  [R7B]>;
def R8W  : XNCMRegWithSubregs<8,  "r8",  [R8B]>;
def R9W  : XNCMRegWithSubregs<9,  "r9",  [R9B]>;
def R10W : XNCMRegWithSubregs<10, "r10", [R10B]>;
def R11W : XNCMRegWithSubregs<11, "r11", [R11B]>;
def R12W : XNCMRegWithSubregs<12, "r12", [R12B]>;
def R13W : XNCMRegWithSubregs<13, "r13", [R13B]>;
def R14W : XNCMRegWithSubregs<14, "r14", [R14B]>;
def R15W : XNCMRegWithSubregs<15, "r15", [R15B]>;
}

def GR8 : RegisterClass<"XNCM", [i8], 8,
   // Volatile registers
  (add R12B, R13B, R14B, R15B, R11B, R10B, R9B, R8B, R7B, R6B, R5B,
   // Frame pointer, sometimes allocable
   FPB,
   // Volatile, but not allocable
   PCB, SPB, SRB, CGB)>;

def GR16 : RegisterClass<"XNCM", [i16], 16,
   // Volatile registers
  (add R12W, R13W, R14W, R15W, R11W, R10W, R9W, R8W, R7W, R6W, R5W,
   // Frame pointer, sometimes allocable
   FPW,
   // Volatile, but not allocable
   PCW, SPW, SRW, CGW)>
{
  let SubRegClasses = [(GR8 subreg_8bit)];
}

