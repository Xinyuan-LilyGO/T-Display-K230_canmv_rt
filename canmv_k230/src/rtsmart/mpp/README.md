# mpp

**English** | [中文](README_CN.md)


## Overview

mpp (Media Process Platform) is a suite of multimedia components developed by Canaan Creative, specifically designed for the K230 chip. It includes kernel driver libraries, user-space API libraries, and various sample programs.

## Compilation

### Preparation

After configuring the environment variables in the mpp directory with `source build_env.sh`, you can proceed to compile the mpp.

**Note: Ensure that mpp is correctly positioned within the sdk directory before compiling.**

### Detailed Compilation

- Compile all kernel layer code:

  Navigate to the directory `k230_sdk/src/big/mpp/kernel`

  Run the `make` command


- Compile the specific kernel driver library:
  
  Navigate to a specific driver directory, e.g. `k230_sdk/src/big/mpp/kernel/fft`

  Run the `make` command

- Compile all user apps API code:

  Navigate to the directory `k230_sdk/src/big/mpp/userapps/src`

  Run the `make` command


- Compile a specific user app API library:
  
  Navigate to a specific API source code directory, e.g. `k230_sdk/src/big/mpp/userapps/src/sensor`

  Run the `make` command

- Compile all user apps sample code:

  Navigate to the directory `k230_sdk/src/big/mpp/userapps/sample`

  Run the `make` command


- Compile a specific user app sample:
  
  Navigate to a specific sample directory, e.g. `k230_sdk/src/big/mpp/userapps/sample/sample_fft`

  Run the `make` command

