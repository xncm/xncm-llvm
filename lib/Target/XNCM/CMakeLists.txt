set(LLVM_TARGET_DEFINITIONS XNCM.td)

tablegen(LLVM XNCMGenRegisterInfo.inc -gen-register-info)
tablegen(LLVM XNCMGenInstrInfo.inc -gen-instr-info)
tablegen(LLVM XNCMGenAsmWriter.inc -gen-asm-writer)
tablegen(LLVM XNCMGenDAGISel.inc -gen-dag-isel)
tablegen(LLVM XNCMGenCallingConv.inc -gen-callingconv)
tablegen(LLVM XNCMGenSubtargetInfo.inc -gen-subtarget)
add_public_tablegen_target(XNCMCommonTableGen)

add_llvm_target(XNCMCodeGen
  XNCMISelDAGToDAG.cpp
  XNCMISelLowering.cpp
  XNCMInstrInfo.cpp
  XNCMFrameLowering.cpp
  XNCMMachineFunctionInfo.cpp
  XNCMRegisterInfo.cpp
  XNCMSubtarget.cpp
  XNCMTargetMachine.cpp
  XNCMSelectionDAGInfo.cpp
  XNCMAsmPrinter.cpp
  XNCMMCInstLower.cpp
  )

add_subdirectory(InstPrinter)
add_subdirectory(TargetInfo)
add_subdirectory(MCTargetDesc)
