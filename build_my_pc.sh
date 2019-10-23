#!/bin/bash
export KERNELDIR=~/ThunderKernel_Z176CG
cd $KERNELDIR
make clean
TOOLCHAINDIR=~/x86_64-unknown-linux-uclibc_4.9
export KBUILD_BUILD_USER="x0r3d"
export KBUILD_BUILD_HOST="L1nux1sX0R1N6"
export USE_CCACHE=1
export CCACHE_DIR=../.ccache
export FINALZIP=kernel.zip
export PATH=$PATH:$TOOLCHAINDIR/bin

make clean
make x86_64_sofia_defconfig
make  CC=x86_64-unknown-linux-uclibc-gcc -j$( nproc --all )
