/* =======================================================================================================
  STCProtocol.h
  Created on: Feb 26, 2025

  ALMA sistemi s.r.l. 2024
   -------------
  Authors:
    R. Filippone
    A. Kamal Eddine

 ------------------------------------------------------------------------------
  v. 1.0 - Feb 26, 2025 - Short description
 ------------------------------------------------------------------------------
  Reference:
    R1 - Reference (document, web site, other source files, etc.)
    R2
 ------------------------------------------------------------------------------
  Functions and logics

======================================================================================================= */
#ifndef STCPROTOCOL_H_
#define STCPROTOCOL_H_

#define MAX_PAIRS 100
#define MAX_KEY_LEN 64
#define MAX_VAL_LEN 256
#define MAX_LIST_ITEMS 20
#define MAX_CMDS 12

typedef enum { TYPE_STRING, TYPE_NUMBER, TYPE_LIST } ValueType;

typedef struct {
    char key[MAX_KEY_LEN];
    ValueType type;
    char value[MAX_VAL_LEN];                 // For strings or single numbers
    char list[MAX_LIST_ITEMS][MAX_VAL_LEN];  // For arrays of numbers/strings
    int list_len;
} JsonPair;

#define NUMMIRRORS	8
#define NUMSUTS		8

#define MAXJSONLEN 2048
#define MAXJSONSYS	256
#define MAXJSONLEV	256
#define MAXJSONTUR	256
#define MAXJSONMIR	384
#define MAXJSONSUT	384
#define MAXJSONERR	256
#define MAXJSONITM   48

#define JSON_NOERR	     0	// Correct
#define JSON_UNKCLIENT	-1	// Data received from unknown client
#define JSON_PROTVERS	-2	// Protocol version mismatch
#define JSON_SYNTAX		-3	// Syntax error (including unknown label)
#define JSON_CMDDATA	-4	// Unknown command/Value out of range
#define JSON_NOTALLOWED	-5	// Command not allowed in current state
#define JSON_INSUFRES	-6	// Insufficient resources to perform the request

void JSONAck(int, char *out);
void JSONSystem(char *out);
void JSONLevel(char *out);
void JSONTurret(char *out);
void JSONMirrors(char *out);
void JSONSUTs(char *out);
void JSONMirror(int i,char *out);
void JSONSUT(int i,char *out);
void JSONErrors(char *out);
void JSONFullData(char *out);
int  JSONParse(const char *,JsonPair *);

#endif /* STCPROTOCOL_H_ */
