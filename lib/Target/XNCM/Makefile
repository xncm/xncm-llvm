##===- lib/Target/XNCM/Makefile --------------------------*- Makefile -*-===##
# 
#                     The LLVM Compiler Infrastructure
#
# This file is distributed under the University of Illinois Open Source 
# License. See LICENSE.TXT for details.
# 
##===----------------------------------------------------------------------===##

LEVEL = ../../..
LIBRARYNAME = LLVMXNCMCodeGen
TARGET = XNCM

# Make sure that tblgen is run, first thing.
BUILT_SOURCES = XNCMGenRegisterInfo.inc XNCMGenInstrInfo.inc \
		XNCMGenAsmWriter.inc \
		XNCMGenDAGISel.inc XNCMGenCallingConv.inc \
		XNCMGenSubtargetInfo.inc

DIRS = InstPrinter TargetInfo MCTargetDesc

include $(LEVEL)/Makefile.common

