
[![Build Status](https://app.travis-ci.com/ericbg27/MutRoSe-Mission-Decomposer.svg?branch=main)](https://app.travis-ci.com/ericbg27/MutRoSe-Mission-Decomposer)  [![License](https://img.shields.io/badge/License-Boost%201.0-lightblue.svg)](https://www.boost.org/LICENSE_1_0.txt) [![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0) ![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.5584561.svg)


# MutRoSe-Mission-Decomposer
This is the mission decomposer for the MutRoSe (Multi-Robot systems mission Specification and Decomposition) framework. This decomposer works given: (i) a JSON Goal Model (generated in [GODA](http://pistar-goda.herokuapp.com/)), (ii) a modified HDDL specification (the original language syntax can be found in [1]), which consists of a subset of the language with the addition of new constructs, (iii) a JSON/XML configuration file and (iv) an XML world knowledge file. 

The HDDL parsing part of this software is built upon the PANDA HDDL parser, where the original PANDA HDDL parser code can be found [here](https://github.com/panda-planner-dev/pandaPIparser).

## Versions
The only working version is given in the main branch.

## Installation
To use the MutRoSe Mission Decomposer just clone this repository or download its source-code. There is an already compiled 64-bits Linux binary. To run it, simply insert the following command on a Linux terminal: 

    ./MutroseMissionDecomposer [Modified HDDL Domain File] [Goal Model JSON file] [JSON/XML configuration file]

If you want to compile it by yourself you will need to install:

 - [Boost C++ Library](https://www.boost.org/) (>= 1.74.0)
 - [GNU Bison Parser](https://www.gnu.org/software/bison/) (>= v3.7.5)
 - [Flex (Fast Lexical Analyzer Generator)](https://github.com/westes/flex) (>= v2.6.4)
 - C++17 (or greater)

When all of the dependencies are installed, one simply go to the root folder of the source-code and run:

    make -j4

If a clean build is desired, a bash script is supplied. In Linux, just run

    bash clean-objects.sh

at the root folder of the source-code and all of the object files (.o) will be removed
## Instructions and Examples

You can find instructions on how to properly use the MutRoSe Mission Decomposer and examples of mission models [here](https://github.com/ericbg27/MutRoSe-Docs).

## Maintainers
Eric Gil - [Github Profile](https://github.com/ericbg27/)

## References
[1] https://www.uni-ulm.de/fileadmin/website_uni_ulm/iui.inst.090/Publikationen/2020/Hoeller2020HDDL.pdf
