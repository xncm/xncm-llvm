add_llvm_library(LLVMXNCMDesc
  XNCMMCTargetDesc.cpp
  XNCMMCAsmInfo.cpp
  )

add_dependencies(LLVMXNCMDesc XNCMCommonTableGen)
