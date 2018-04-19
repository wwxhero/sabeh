/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: globals.c,v 1.1 1998/11/24 17:50:39 oahmad Exp $
 *
 * Author:       Yiannis Papelis
 * Date:         November, 1995
 *
 * Description:  All global variables used by the parser.
 *
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <setjmp.h>

jmp_buf     g_Recovery;                /* stack environ. for error recovery */
int         g_Tok;                     /* token read by lexan               */
char        *large_token_value = NULL; /* keeps values for large tokens     */
int         g_LineNum          = 1;    /* line number                       */
char        *g_LargeTokVal     = NULL; /* keeps values for large tokens     */
char        g_CurFile[512];            /* current file, for error messages  */
