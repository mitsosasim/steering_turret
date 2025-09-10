/* =======================================================================================================
  STCProtocol.c
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "STCProtocol.h"

static int 		pair_count;
static JsonPair pairs[MAX_PAIRS];

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void skipspace(const char **p) {
    while (**p && (**p==' ')) (*p)++;
}
/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
int parsenumber(const char **p, char *out) {
    int i = 0;
    while ((isdigit(**p) || **p == '-' || **p == '+' || **p == '.' || **p == 'e' || **p == 'E') && i < MAX_VAL_LEN - 1) {
        out[i++] = *(*p)++;
    }
    out[i] = '\0';
    return i > 0;
}

/* -------------------------------------------------------------------------------------------------------
This parser supports a small subset of JSON:
	key-value pairs with string keys and string/integer values
 	simple flat lists of string or numeric values as values in JSON key-value pairs.
Limitations:
    No nested objects
    Arrays must contain only strings or numbers (not mixed)
    No escaped characters in strings
    Minimal whitespace tolerance

Return:
   N > 0; N is the number of read pairs/array
   JSON_SYNTAX = Syntax error
------------------------------------------------------------------------------------------------------- */
int JSONParse(const char *json,JsonPair *JP)
{
    const char *p = json;

    pair_count = 0;

    skipspace(&p);
    if (*p != '{') return(JSON_SYNTAX);
    p++; // skip '{'

    while (*p && *p != '}') {
    	skipspace(&p);
        if (*p != '"') return(JSON_SYNTAX);
        p++; // skip '"'

        // Parse key
        int i = 0;
        while (*p && *p != '"' && i < MAX_KEY_LEN - 1)
            pairs[pair_count].key[i++] = *p++;
        pairs[pair_count].key[i] = '\0';
        if (*p != '"') return(JSON_SYNTAX);
        p++; // skip '"'

        skipspace(&p);
        if (*p != ':') return(JSON_SYNTAX);
        p++; // skip ':'
        skipspace(&p);

        // Handle value or array
        if (*p == '"') {
            p++; // skip '"'
            pairs[pair_count].type = TYPE_STRING;
            i = 0;
            while (*p && *p != '"' && i < MAX_VAL_LEN - 1)
                pairs[pair_count].value[i++] = *p++;
            pairs[pair_count].value[i] = '\0';
            if (*p != '"') return(JSON_SYNTAX);
            p++; // skip '"'

        } else if (*p == '[') {
            p++; // skip '['
            pairs[pair_count].type = TYPE_LIST;
            int list_index = 0;

            while (*p && *p != ']') {
            	skipspace(&p);

                if (*p == '"') {
                    p++; // string value
                    i = 0;
                    while (*p && *p != '"' && i < MAX_VAL_LEN - 1)
                        pairs[pair_count].list[list_index][i++] = *p++;
                    pairs[pair_count].list[list_index][i] = '\0';
                    if (*p != '"') return(JSON_SYNTAX);
                    p++; // skip '"'
                } else {
                    // numeric value (int or float)
                    if (!parsenumber(&p, pairs[pair_count].list[list_index]))
                    	return(JSON_SYNTAX);
                }

                list_index++;
                if (list_index >= MAX_LIST_ITEMS) return(JSON_SYNTAX);
                skipspace(&p);
                if (*p == ',') p++;
            }

            pairs[pair_count].list_len = list_index;
            if (*p != ']') return(JSON_SYNTAX);
            p++; // skip ']'

        } else {
            // Parse number (int or float)
            pairs[pair_count].type = TYPE_NUMBER;
            if(!parsenumber(&p, pairs[pair_count].value))
            	return(JSON_SYNTAX);
        }

        pair_count++;
        if (pair_count >= MAX_PAIRS) return(JSON_SYNTAX);

        skipspace(&p);
        if (*p == ',') p++;
    }

    if(pair_count == 0) return(JSON_SYNTAX);

    memcpy(JP,pairs,sizeof(pairs));

    return(pair_count);
}

/* -------------------------------------------------------------------------------------------------------
   { header,"ack": "ackcode" }
------------------------------------------------------------------------------------------------------- */
void JSONAck(int Code, char *out)
{
	char ackdata[MAXJSONITM];

	if(Code < 0) Code *= -1;
	sprintf(ackdata,"{ \"ACK\": \"%02d\" }",Code);
	strcpy(out,ackdata);
}

/* -------------------------------------------------------------------------------------------------------
   - temperature (celsius)
   - pressure (mBar)
   - humidity (percentage)
   "temp": "XX.x","press": "XXXX","hum": "XXXX"
------------------------------------------------------------------------------------------------------- */
void JSONSystem(char *out)
{
	char part_system[MAXJSONSYS];

	memset(part_system,0,MAXJSONSYS);

	strcpy(out,part_system);
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void JSONLevel(char *out)
{
	char part_level[MAXJSONLEV];

	memset(part_level,0,MAXJSONLEV);

	strcpy(out,part_level);
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void JSONTurret(char *out)
{
	char part_turret[MAXJSONTUR];

	memset(part_turret,0,MAXJSONTUR);

	strcpy(out,part_turret);
}

/* -------------------------------------------------------------------------------------------------------
   "mirrors": [ { mirror },{ mirror } .... ]
   123456789 12345
            10
------------------------------------------------------------------------------------------------------- */
void JSONMirrors(char *out)
{
	char part_mirrors[MAXJSONMIR];
	char mirror[MAXJSONITM];
	int  i;

	memset(part_mirrors,0,MAXJSONMIR);
	memset(mirror,0,MAXJSONITM);

	strcpy(part_mirrors,"\"mirrors\": [ ");
	for(i=0;i<NUMMIRRORS;i++) {
		strcat(part_mirrors,"{ ");
		JSONMirror(i,mirror);
		strcat(part_mirrors,mirror);
		strcat(part_mirrors," }");
		if(i < (NUMMIRRORS-1))
			strcat(part_mirrors,",");
	}
	strcat(part_mirrors," ]");
	strcpy(out,part_mirrors);
}

/* -------------------------------------------------------------------------------------------------------
   "state": "fail","angle": "xxx.xx"
   123456789 123456789 123456789 123456789 12345678
            10        20        30        40
------------------------------------------------------------------------------------------------------- */
void JSONMirror(int i,char *out)
{
	char  part_mirror[MAXJSONITM];
	char  item[MAXJSONITM];
	char  state[6];
	float angle;

	memset(part_mirror,0,MAXJSONITM);

	angle = 123.45f;
	strcpy(state,"fail");

	sprintf(item,"\"state\": \"%s\"",state);
	strcpy(part_mirror,item);
	sprintf(item,"\"angle\": \"%5.2lf\"",angle);
	strcat(part_mirror,item);

	strcpy(out,part_mirror);
}

/* -------------------------------------------------------------------------------------------------------
   "suts": [ { sut },{ sut } .... ]
   123456789 1
            10
------------------------------------------------------------------------------------------------------- */
void JSONSUTs(char *out)
{
	char part_suts[MAXJSONSUT];
	char sut[MAXJSONITM];
	int  i;

	memset(part_suts,0,MAXJSONSUT);
	memset(sut,0,MAXJSONITM);

	strcpy(part_suts,"\"suts\": [ ");
	for(i=0;i<NUMSUTS;i++) {
		strcat(part_suts,"{ ");
		JSONSUT(i,sut);
		strcat(part_suts,sut);
		strcat(part_suts," }");
		if(i < (NUMSUTS-1))
			strcat(part_suts,",");
	}
	strcat(part_suts," ]");
	strcpy(out,part_suts);
}

/* -------------------------------------------------------------------------------------------------------
   "state": "fail","angle": "xxx.xx"
   123456789 123456789 123456789 123456789 12345678
            10        20        30        40
------------------------------------------------------------------------------------------------------- */
void JSONSUT(int i,char *out)
{
	char part_sut[MAXJSONITM];
	char  item[MAXJSONITM];
	char  state[6];
	float angle;

	memset(part_sut,0,MAXJSONITM);

	angle = 123.45f;
	strcpy(state,"fail");

	sprintf(item,"\"state\": \"%s\"",state);
	strcpy(part_sut,item);
	sprintf(item,"\"angle\": \"%5.2lf\"",angle);
	strcat(part_sut,item);

	strcpy(out,part_sut);
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void JSONErrors(char *out)
{
	char part_errors[MAXJSONERR];
	char error[8];
	int  i,cnt=0;

	memset(part_errors,0,MAXJSONERR);
	memset(error,0,8);

	strcpy(part_errors,"\"Errors\": [ ");
	for(i=0;i<12;i++) {
		// if some error in the error list
		// sprintf(error,"\"%d\"",error code);
		strcat(part_errors,error);
	}
	if(cnt == 0)
		strcat(part_errors,"\"none\"");

	strcat(part_errors," ]");
	strcpy(out,part_errors);
}

/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
void JSONFullData(char *out)
{
	char jsondata[MAXJSONLEN];
	char system[MAXJSONSYS],
		 level[MAXJSONLEV],
		 turret[MAXJSONTUR],
		 mirrors[MAXJSONMIR],
		 suts[MAXJSONSUT],
		 errors[MAXJSONERR];		// Take under control!

	// -------------------------------------------------------------------
	memset(jsondata,0,MAXJSONLEN);

	// -------------------------------------------------------------------
	// 2. System data
	// -------------------------------------------------------------------
	JSONSystem(system);

	// -------------------------------------------------------------------
	// 3. Leveling data
	// -------------------------------------------------------------------
	JSONLevel(level);

	// -------------------------------------------------------------------
	// 4. Turret data
	// -------------------------------------------------------------------
	JSONTurret(turret);

	// -------------------------------------------------------------------
	// 5. Mirror data
	// -------------------------------------------------------------------
	JSONMirrors(mirrors);

	// -------------------------------------------------------------------
	// 6. SUT data
	// -------------------------------------------------------------------
	JSONSUTs(suts);

	// -------------------------------------------------------------------
	// 7. List of errors
	// -------------------------------------------------------------------
	JSONErrors(errors);

	sprintf(jsondata,"{ { %s },{ %s },{ %s },{ %s },{ %s },{ %s } }",
			system,
			level,
			turret,
			mirrors,
			suts,
			errors);
	strcpy(out,jsondata);
}
