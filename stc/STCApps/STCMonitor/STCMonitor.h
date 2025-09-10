/* =======================================================================================================
  main.h
  Created on: Feb 13, 2025

  ALMA sistemi s.r.l. 2024
   -------------
  Authors:
    R. Filippone
    A. Kamal Eddine

 ------------------------------------------------------------------------------
  v. 1.0 - Feb 13, 2025 - Monitor the STC controller

======================================================================================================= */

#ifndef MAIN_H_
#define MAIN_H_

void* CheckGPSData(void* p);
void* CheckResources(void* p);

void ShowGPSData(SMGNSS *ShGPSData);


#endif /* MAIN_H_ */
