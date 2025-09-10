/* =======================================================================================================
  STCControl.cpp
  Created on: Feb 19, 2025

  ALMA sistemi s.r.l. 2024
   -------------
  Authors:

 ------------------------------------------------------------------------------
  v. 1.0 - Feb 19, 2025 - Short description
 ------------------------------------------------------------------------------
  Reference:
    R1 - Reference (document, web site, other source files, etc.)
    R2
 ------------------------------------------------------------------------------
  Functions and logics:

======================================================================================================= */
#include <iostream>

#include "STCControl.h"


int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <name>" << std::endl;
        return 1;
    }
    else
    	std::cout << "default: " << std::endl;


    std::string name = argv[1];

    return 0;
}
