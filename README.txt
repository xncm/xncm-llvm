
LLVM Backend for the XNCM Architecture
======================================

** This is work in progress ** come back later **

Quick build guide:

	$ mkdir obj; cd obj
	$ cmake ..
	$ make

Quick and most simple test:

	$ ./bin/llc -mtriple=xncm < ../xncm-test/000_retvoid.ll


Low Level Virtual Machine (LLVM)
================================

This directory and its subdirectories contain source code for the Low Level
Virtual Machine, a toolkit for the construction of highly optimized compilers,
optimizers, and runtime environments.

LLVM is open source software. You may freely distribute it under the terms of
the license agreement found in LICENSE.txt.

Please see the HTML documentation provided in docs/index.html for further
assistance with LLVM.

If you're writing a package for LLVM, see docs/Packaging.html for our
suggestions.

