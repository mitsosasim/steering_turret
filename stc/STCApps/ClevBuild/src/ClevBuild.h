/*
 * ClevBuild.h
 *
 *  Created on: Oct 10, 2024
 *      Author: alma
 */

#ifndef CLEVBUILD_H_
#define CLEVBUILD_H_

#define  CBVER 				1
#define  CBREL 				"La"
#define  HRECLEN		    128
#define  FCOUNTER		    "BCounter.txt"
#define  FHEADER		    "BCounter.h"
#define  BCROWS				9

char* getParentFromPath(char *);
char* getFileNameFromPath(char *);

int   WriteCounter(char *);
int   ReadCounter(char *);
int   WriteHeader(char *);


#endif /* CLEVBUILD_H_ */
