/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: parse.c,v 1.8 2013/11/27 00:05:03 iowa\dheitbri Exp $
 *
 * Author:       Yiannis Papelis
 * Changes:      Omar Ahmad, Ben Wehrle
 * Date:         November, 1995
 *
 * Description:  
 *
 *  The parser for the hcsm specification file.
 *
 *  The parser is constructed using the recursive descent technique, where
 *  subroutines recognize rules and return 1 if the reduced a rule,
 *  or 0 if they did not.  In case of syntax error, the parser calls
 *  an error macro which does a long jump to the main subroutine.
 *
 *  The parser follows the BNF grammer specified in the documentation.
 *  It has embedded code to builds a THcsmParserInfo structure,
 *  most of which consists of linked lists and dynamic strings.  This
 *  structure is later processed to produce the source files.
 *
 ****************************************************************************/
#include <stdio.h>
#ifdef _PowerMAXOS
#include <unistd.h>
#endif
#include <string.h>
#include <stdlib.h>
#include <setjmp.h>

#include "parser.h"

static THcsmParserInfo   m_ParserInfo; 

/*********** prototypes for static functions ***********************/
static void hcsm_parse(void);
static int  p_header(THcsmParserInfo *pPinfo);
static int  p_attributes(TStringPair **pStrPair);
static int  p_state_machine(TStateMachine **ppSm);
static int  p_name_type_pair(TStringPair **pStrPair);
static int  p_one_trans(TTransition **ppTr);
static int  p_ident_list(TStringNode **ppStrNode);
static int  p_dial_param(TDialParam **ppDialParam);
static int  p_pairs_list(TStringPair **pStrPair);
static int  p_trans(TStateMachine *ptr);
static int  p_priv_user(TStateMachine *ptr);
static int  p_children(TStateMachine *ptr);
static int  p_inputs(TStateMachine *ptr);
static int  p_outputs(TStateMachine *ptr);
static int  p_locals(TStateMachine *ptr);
static int  p_globals(TStringPair **pStrPair);
static int  p_dials( TStateMachine* pSm );
static int  p_dialinfo( TStateMachine* pSm );
static int  p_monitors( TStateMachine* pSm );
static int  p_buttons(TStateMachine *pSm);
static int  p_connections(THcsmParserInfo *pPinfo);
static int  p_one_connection(TConnection **ppConn);
static int  PreActivityFunction( TStringPair **ppStrPair );
static int  PostActivityFunction( TStringPair **ppStrPair );
static int  create_cb(TStringPair **pStrPair);
static int  delete_cb(TStringPair **pStrPair);
static int  predicate_function( TStringPair **pStrPair );
static void BuildPredicate( THcsmParserInfo *pPinfo );
static void DumpStructures( THcsmParserInfo* pInfo );
static void UpdateRootField( THcsmParserInfo* pInfo );
static EBool IsBlank( const char *pCharArr );

/************************************************************************
 *
 * Parses header line and initial code block.  If pPinfo is NULL
 * it parses everything but doesn't copy any data to the data
 * structures
 *
 */
static int p_header(THcsmParserInfo *pPinfo)
{

	if ( g_Tok == INIT_MARKER_TOK ) {

		g_Tok = yylex();
		if ( g_Tok == VERSION_TOK ) {

			if ( pPinfo )  pPinfo->version = safe_strdup(yytext);

			g_Tok = yylex();
			if ( g_Tok == CODE_TOK ) {

				if ( pPinfo ) {
					
					pPinfo->header_code = safe_strdup(g_LargeTokVal);

				}

				g_Tok = yylex();
				return 1;  /* code block read */

			}
			else {
				
				return 1;  /* no code block */

			}

		}
		else {

			ERROR("expected version number");

		}

	}
	else {

		ERROR("expected 'SM' keyword");

	}

	return 0;

}


/************************************************************************
 *
 * Parses a comma separated list of one or more identifiers,
 * adds them to the linked list passed as an argument.
 */
static int p_ident_list( TStringNode** ppStrNode )
{
	if ( g_Tok == IDENT ) 
	{ 	
		/* got at least one node, point head to it */
		TStringNode* pHead = NULL;
		TStringNode* pNewNode;

		pHead        = NewStrNode();
		pHead->pName = safe_strdup( yytext );
		g_Tok        = yylex();

		while( 1 ) 
		{
			if( g_Tok == ',' ) 
			{
				g_Tok = yylex();
				if( g_Tok == IDENT )  
				{
					pNewNode        = NewStrNode();
					pNewNode->pName = safe_strdup( yytext );
					
					/* add to linked list, there is at least one node */
					if( pHead->pNext == NULL )  
					{
						pHead->pNext = pNewNode;
					}
					else 
					{
						TStringNode* pTrav;
						
						for( pTrav = pHead; pTrav->pNext; pTrav = pTrav->pNext )
							;
						pTrav->pNext = pNewNode;
					}

					g_Tok = yylex();

				}  /* if IDENT */
				else 
				{
					ERROR( "expected identifier" );
				}  /* if else IDENT */
			}  /* if ',' */
			else 
			{
				break;
			}  /* if else ',' */
		}  /* while */

		*ppStrNode = pHead;

		return 1;
	}  /* if IDENT */
	else 
	{
		return 0;
	}  /* if else IDENT */
}


/************************************************************************
 *
 * Parses a name type pair, creates a new node for it and returns
 * the node in the argument list.
 */
static int p_dial_param( TDialParam** ppDialParam )
{
	/*
	 * A dial info parameter list has the following format:
	 * ( DialName, Optional, ParamName, ParamType, ParamUnits, ParamDefValue, ParamComment )
	 */

	/* check for the opening parenthesis */
	if( g_Tok == '(' ) 
	{
		/* check for a identifier that represents the name */
		g_Tok = yylex();
		if( g_Tok == IDENT ) 
		{
			/* found a valid name; create a new node */
			TDialParam* pNewNode;
			pNewNode            = NewDialParam();
			pNewNode->pDialName = safe_strdup( yytext );
			pNewNode->pName     = NULL;
			pNewNode->pType     = NULL;
			pNewNode->pUnits    = NULL;
			pNewNode->pDefValue = NULL;
			pNewNode->pComment  = NULL;
			pNewNode->optional  = 0;
			pNewNode->pNext     = NULL;

			g_Tok = yylex();
			if( g_Tok == ',' ) 
			{
				/* parse the parameter's optional setting */
				g_Tok = yylex();
				if( g_Tok == POS_INT )
				{
					pNewNode->optional = atoi( safe_strdup( yytext ) );

					g_Tok = yylex();
					if( g_Tok == ',' )
					{
						/* parse the parameter's name */
						g_Tok = yylex();
						if( g_Tok == IDENT )
						{
							pNewNode->pName = safe_strdup( yytext );

							g_Tok = yylex();
							if( g_Tok == ',' )
							{
								/* parse the parameter's type */
								g_Tok = yylex();
								if( g_Tok == IDENT )
								{
									pNewNode->pType = safe_strdup( yytext );

									g_Tok = yylex();
									if( g_Tok == ',' )
									{
										/* parse the parameter's units */
										g_Tok = yylex();
										if( g_Tok == IDENT )
										{
											pNewNode->pUnits = safe_strdup( yytext );

											g_Tok = yylex();
											if( g_Tok == ',' )
											{
												/* parse the parameter's default value */
												g_Tok = yylex();
												if( g_Tok == IDENT || g_Tok == POS_INT )
												{
													pNewNode->pDefValue = safe_strdup( yytext );

													g_Tok = yylex();
													if( g_Tok == ',' )
													{
														/* parse the parameter's comment or description */
														g_Tok = yylex();
														if( g_Tok == IDENT )
														{
															pNewNode->pComment = safe_strdup( yytext );

															g_Tok = yylex();
															if( g_Tok == ')' )
															{
																*ppDialParam = pNewNode;

																g_Tok = yylex();
																return 1;
															}
															else
															{
																ERROR( "expected ) in DIAL_INFO" );
															}
														}
														else
														{
															ERROR( "expected dial parameter comment" );
														}
													}
													else
													{
														ERROR( "expected comma" );
													}
												}
												else
												{
													ERROR( "expected dial parameter default value" );
												}
											}
											else
											{
												ERROR( "expected comma" );
											}
										}
										else
										{
											ERROR( "expected dial parameter units" );
										}
									}
									else
									{
										ERROR( "expected comma" );
									}
								}
								else
								{
									ERROR( "expected dial parameter type" );
								}
							}
							else
							{
								ERROR( "expected comma" );
							}
						}
						else
						{
							ERROR( "expected dial parameter name" );
						}
					}
					else
					{
						ERROR( "expected comma" );
					}
				}
				else
				{
					ERROR( "expected dial parameter optional setting" );
				}
			}
			else
			{
				ERROR( "expected comma" );
			}
		}
		else
		{
			ERROR( "expected hcsm name" );
		}
	}
	else 
	{
		ERROR( "expected ( in DIAL_INFO" );
	}

	return 0;
}


/************************************************************************
 *
 * Parses a name type pair, creates a new node for it and returns
 * the node in the argument list.
 */
static int p_name_type_pair( TStringPair** pStrPair )
{
	/*
	 * A name-type pair has the following format:
	 * ( name, <category::>type [, description ] )
	 */
	char* tmpStr;

	/* check for the opening parenthesis */
	if( g_Tok == '(' ) 
	{
		/* check for a identifier that represents the name */
		g_Tok = yylex();
		if( g_Tok == IDENT ) 
		{
			/* found a valid name; create a new node */
			TStringPair* pNewNode;

			pNewNode        = NewStrPair();
			pNewNode->pName = safe_strdup( yytext );
			pNewNode->pDscrptn  = NULL;
			pNewNode->pCategory = NULL;
			pNewNode->pValue    = NULL;
			pNewNode->pNext     = NULL;

			/* check for the comma that separates name from type */
			g_Tok = yylex();
			if( g_Tok == ',' ) 
			{
				/* check for a identifier that represents the 
				 * category and type 
				 */
				g_Tok = yylex();
				if( g_Tok == IDENT ) 
				{
					tmpStr = safe_strdup( yytext );

					g_Tok = yylex();
					if( g_Tok == ':' ) 
					{
						/* another identifier was found, therefore the
						 * first identifier must be the category, and
						 * the second is the type 
						 */
						g_Tok = yylex();
						if ( g_Tok == IDENT ) 
						{
							/* found type information; add info to node */
							pNewNode->pValue = safe_strdup( yytext );
							pNewNode->pCategory = safe_strdup( tmpStr );

							/* Prepare to look for closing parenthesis */
							g_Tok = yylex();
						}

						else 
						{
							/* failed to find a type identifier */
							ERROR( "expected type" );
						}
					}
					else 
					{
						/* No other identifiers were found, therefore the
						 * first was the type, and there is no category. 
						 */

						/* found type information; add info to node */
						pNewNode->pValue = safe_strdup( tmpStr );
						pNewNode->pCategory = 0;
					}
					
					/* look for the closing parenthesis */
					if( g_Tok == ')' ) 
					{
						/* parsed valid name-type pair, add node to list */
						*pStrPair = pNewNode;

						g_Tok = yylex();
						return 1;
					}
					else 
					{
						/* look for the description */
						if ( g_Tok == ',' ) 
						{
							g_Tok = yylex();
							if( g_Tok != ')' )
							{
								pNewNode->pDscrptn = safe_strdup( yytext );
								g_Tok = yylex();
							}
						}
						if( g_Tok == ')' ) 
						{
							/* parsed valid name-type pair, add node to list */
							*pStrPair = pNewNode;

							g_Tok = yylex();
							return 1;
						}
						else 
						{
							ERROR( "expected )" );
						}
					}  /* if else ')' */
				}
				else 
				{
					/* failed to find a valid identifier for type */
					ERROR( "expected type" );
				}  /* if else IDENT */
			}
			else 
			{
				/* failed to find a comma that separates name from type */
				ERROR( "expected comma" );
			}  /* if else ',' */
		}
		else 
		{
			/* failed to find a valid identifier for name */
			ERROR( "expected name" );
		}  /* if else IDENT */
	}
	else 
	{
		/* failed to find an opening parenthesis */
		return 0;
	}  /* if else '(' */

	return 0;
}


/******************************************************************************
 *
 * Description:  Parses a comma separated list of one or more name and
 *   type pairs.
 *
 * Remarks:  This function expects a pointer to linked list of TStringPair
 *   nodes.  If the name and type string pair is valid, it creates a new
 *   TStringPair and inserts it at the end of the input linked list.
 *
 * Arguments:
 *   ppStrPair - A pointer to TStringPair linked list.
 *
 * Returns:
 *   A pointer to the head node of the input linked list.
 *
 */
static int p_pairs_list( TStringPair **ppStrPair )
{

	TStringPair*   pHead;           /* pointer to head of linked list        */
	TStringPair*   pNewNode;        /* pointer to new node                   */
	TStringPair*   pTrav;           /* pointer used to traverse linked list  */

	if( p_name_type_pair( &pNewNode ) ) 
	{
		pHead = pNewNode;

		while( g_Tok == ',' ) 
		{
			g_Tok = yylex();
			if( p_name_type_pair( &pNewNode ) ) 
			{
				if( pHead->pNext == NULL ) 
				{
					pHead->pNext = pNewNode;
				}
				else 
				{
					for( pTrav = pHead; pTrav->pNext; pTrav = pTrav->pNext )
						;
					pTrav->pNext = pNewNode;
				}
			}
			else 
			{
				ERROR( "expected a name-type pair" );
			}
		}

		*ppStrPair = pHead;
		return 1;
	}

	return 0;
}


/************************************************************************
 *
 * Parses a single private user declaration.  There's not much to parse
 * as the parser simply accepts the rest of the line as the declaration.
 *
 */
static int p_one_priv_user( TPrivUser** ppPriv )
{

	TPrivUser* pPriv = NewPriv();

	consume_remainder_of_line();  /* puts chars in g_LargeTokVal */

	pPriv->pDec = safe_strdup( g_LargeTokVal );
	*ppPriv     = pPriv;

	g_Tok       = yylex();

	return 1;

}


/************************************************************************
 *
 * Parses private user functions in a state machine, adds them to the 
 * linked list of the provided state machine structure.
 *
 */
static int p_priv_user( TStateMachine* pSm )
{

	TPrivUser* pPriv;

	if ( g_Tok == PRIV_DECL_TOK ) {

		if ( p_one_priv_user( &pPriv ) ) {

			AddPriv( pSm, pPriv );
			return 1;

		}
		else {	

		  ERROR( "expected private user function" );

		}

	}
	else {

		return 0;

	}

	return 0;

}


/************************************************************************
 *
 * Parses a single transition specification.  Creates new node and
 * make the argument point to it.
 *
 */
static int p_one_trans(TTransition **ppTr)
{
  TTransition    *pTr;
  TStringPair  *pStrP;
  
  if ( g_Tok == '(' ) {
    pTr    = NewTr();
    pStrP  = NewStrPair();
    g_Tok  = yylex();
    
    if ( g_Tok == IDENT ) {
      pTr->pHeadName  = safe_strdup(yytext);
      g_Tok      = yylex();
      if ( g_Tok == ',' ) {
				g_Tok = yylex();
				if ( g_Tok == IDENT ) {
					pTr->pTailName = safe_strdup(yytext);
					g_Tok     = yylex();
					if ( g_Tok == ',' ) {
						g_Tok = yylex();
						if ( g_Tok == IDENT ) {
							pTr->pPredName  = safe_strdup(yytext);
							g_Tok           = yylex();
							if ( g_Tok == ')' ) {
								g_Tok = yylex();
								if ( p_attributes(&pStrP) ) {
									pTr->pAttrs = pStrP;
								}
								*ppTr = pTr;
								return 1;
							}
							else
								ERROR("expected )");
						}
						else
							ERROR("expected transition predicate");
					}
					else
						ERROR("expected comma ','");
				}
				else
					ERROR("expected tail of transition");
      }
      else
				ERROR("expected comma ','");
    }
    else
      ERROR("expected head of transition");
  }
  else
    return 0;

  return 0;
}


/************************************************************************
 *
 * Parses transitions in a state machine, adds them to the linked
 * list of the provided state machine structure.
 *
 */
static int p_trans( TStateMachine* pSm )
{

	TTransition* pTrans;

	/* check for the transitions token */
	if ( g_Tok == TRANSITIONS_TOK ) {

		/*
		 * look for one or more transitions definitions since each line 
		 * can have more than one transition definition
		 */
		g_Tok = yylex();
		if ( p_one_trans(&pTrans) ) {

			/* first transition definition on this line is valid so add
			 * it to list
			 */
			AddTrans(pSm, pTrans);

			/* look for more transition definitions on this line */
			while ( p_one_trans(&pTrans) ) {

				AddTrans(pSm, pTrans);

			}

			return 1;

		}  /* if p_one_trans */
		else  {

			ERROR("expected transition");

		}  /* else if p_one_trans */

	}  /* if TRANSITIONS_TOK */
	else {

		return 0;

	}  /* else if TRANSITIONS_TOK */

	return 0;

}


/************************************************************************
 *
 * Parses the children entry of a state machine block, adds them to
 * the linked list of the state machine structure.
 *
 */
static int p_children(TStateMachine *pSm)
{
  TStringNode   *pStrList, *pTrav;
  
  if ( g_Tok == CHILDREN_TOK ) {
    g_Tok = yylex();
    if ( p_ident_list(&pStrList) ) {
      if ( pSm->children == NULL )
				pSm->children = pStrList;
      else
				if ( pSm->children->pNext == NULL )
					pSm->children->pNext = pStrList;
				else {
					for ( pTrav = pSm->children; pTrav->pNext; pTrav = pTrav->pNext ) 
						;
					pTrav->pNext = pStrList;
				}
      return 1;
    }
    else 
      ERROR("expected list of children");
  }
  else
    return 0;
	
  return 0;
}


/************************************************************************
 *
 * Parses the inputs of a state machine block and appends them to the
 * state machine structure.
 * 
 */
static int p_inputs(TStateMachine  *pSm)
{
  TStringPair   *pStrPair;
  TStringPair   *pTrav;
  
  if ( g_Tok == INPUTS_TOK ) {
    g_Tok = yylex();
    if ( p_pairs_list(&pStrPair) ) {
      if ( pSm->inputs == NULL )
	pSm->inputs = pStrPair;
      else 
	if ( pSm->inputs->pNext == NULL )
	  pSm->inputs->pNext = pStrPair;
	else {
	  for ( pTrav = pSm->inputs; pTrav->pNext; pTrav = pTrav->pNext )
	    ;
	  pTrav->pNext = pStrPair;
	}
      return 1;
    }
    else
      ERROR("expected list of inputs");
  }
  else
    return 0;

  return 0;
}


/************************************************************************
 *
 * Parses the outputs of a state machine block, adds them to the
 * provided state machine structure.
 *
 */
static int p_outputs(TStateMachine *pSm)
{
  TStringPair   *pStrPair;
  TStringPair   *pTrav;
  
  if ( g_Tok == OUTPUTS_TOK ) {
    g_Tok = yylex();
    if ( p_pairs_list(&pStrPair) ) {
      if ( pSm->outputs == NULL )
	pSm->outputs = pStrPair;
      else
	if ( pSm->outputs->pNext == NULL )
	  pSm->outputs->pNext = pStrPair;
	else {
	  for ( pTrav = pSm->outputs; pTrav->pNext; pTrav = pTrav->pNext )
	    ;
	  pTrav->pNext = pStrPair;
	}
      return 1;
    }
    else
      ERROR("expected list of outputs");
  }
  else
    return 0;

  return 0;
}


/************************************************************************
 *
 * Parses the local variable specification of a state machine block, adds
 * them to the provided state machine structure.
 *
 */
static int p_locals(TStateMachine *pSm)
{
  TStringPair *pStrPair;
  TStringPair   *pTrav;
  
  if ( g_Tok == LOCALS_TOK ) {
    g_Tok = yylex();
    if ( p_pairs_list(&pStrPair) ) {
      if ( pSm->locals == NULL )
	pSm->locals = pStrPair;
      else
	if ( pSm->locals->pNext == NULL )
	  pSm->locals->pNext = pStrPair;
	else {
	  for ( pTrav = pSm->locals; pTrav->pNext; pTrav = pTrav->pNext )
	    ;
	  pTrav->pNext = pStrPair;
	}
      return 1;
    }
    else
      ERROR("expected list of locals");
  }
  else
    return 0;

  return 0;
}


/************************************************************************
 *
 * Parses a global variable specification.  It appends them to
 * the provided linked list
 *
 */
static int p_globals(TStringPair **pRetVal)
{
	TStringPair   *pStrPair;
	TStringPair   *pTrav;
  
	if ( g_Tok == GLOBALS_TOK ) {
		g_Tok = yylex();
		if ( p_pairs_list(&pStrPair) ) {
			if ( (*pRetVal) == NULL )
				(*pRetVal) = pStrPair;
			else
			if ( (*pRetVal)->pNext == NULL )
				(*pRetVal)->pNext = pStrPair;
			else {
				for ( pTrav = (*pRetVal); pTrav->pNext; pTrav = pTrav->pNext )
					;
				pTrav->pNext = pStrPair;
			}
			return 1;
		}
		else
			ERROR("expected list of globals");
	}

	return 0;
}


/******************************************************************************
 *
 * Parses the dials specification of a state machine block, adds them
 * to the provided state machine structure.
 *
 */
static int p_dials( TStateMachine* pSm )
{

	TStringPair*   pStrPair;
	TStringPair*   pTrav;           /* pointer used to traverse linked list  */

	if ( g_Tok == DIALS_TOK ) {

		g_Tok = yylex();
		if ( p_pairs_list( &pStrPair ) ) {

			if ( pSm->dials == NULL ) {

				pSm->dials = pStrPair;

			}
			else {

				if ( pSm->dials->pNext == NULL ) {

					pSm->dials->pNext = pStrPair;

				}
				else {

					for ( pTrav = pSm->dials; pTrav->pNext; pTrav = pTrav->pNext )
						;
					pTrav->pNext = pStrPair;

				}

			}

			return 1;

		}
		else {

			ERROR("expected list of dials");

		}

	}
	else {

		return 0;

	}

	return 0;
}


/******************************************************************************
 *
 * Parses the dial info specification of a state machine block, adds them
 * to the provided state machine structure.
 *
 */
static int p_dialinfo( TStateMachine* pSm )
{
	TDialParam*    pDialParam;
	TDialParam*    pTrav;           /* pointer used to traverse linked list  */

	if( g_Tok == DIAL_INFO_TOK ) 
	{
		g_Tok = yylex();
		if( p_dial_param( &pDialParam ) ) 
		{
			if ( pSm->dialParams == NULL ) 
			{
				pSm->dialParams = pDialParam;
			}
			else 
			{
				if( pSm->dialParams->pNext == NULL ) 
				{
					pSm->dialParams->pNext = pDialParam;
				}
				else 
				{
					for( pTrav = pSm->dialParams; pTrav->pNext; pTrav = pTrav->pNext )
						;
					pTrav->pNext = pDialParam;
				}
			}

			return 1;
		}
		else 
		{
			ERROR( "expected dial parameters" );
		}
	}
	else 
	{
		return 0;
	}

	return 0;
}


/******************************************************************************
 *
 * Parses the monitors specification of a state machine block, adds them
 * to the provided state machine structure.
 *
 */
static int p_monitors( TStateMachine* pSm )
{

	TStringPair*   pStrPair;
	TStringPair*   pTrav;           /* pointer used to traverse linked list  */

	if ( g_Tok == MONITORS_TOK ) {

		g_Tok = yylex();
		if ( p_pairs_list( &pStrPair ) ) {

			if ( pSm->monitors == NULL ) {

				pSm->monitors = pStrPair;

			}
			else {

				if ( pSm->monitors->pNext == NULL ) {

					pSm->monitors->pNext = pStrPair;

				}
				else {

					for ( pTrav = pSm->monitors; pTrav->pNext; pTrav = pTrav->pNext )
						;
					pTrav->pNext = pStrPair;

				}

			}

			return 1;

		}
		else {

			ERROR("expected list of monitors");

		}

	}
	else {

		return 0;

	}

	return 0;
}


/************************************************************************
 *
 * Parses the buttons specification of a state machine block, adds them
 * to the provided state machine structure.
 *
 */
static int p_buttons(TStateMachine  *pSm)
{
	TStringPair*   pStrPair;
  	TStringPair*   pTrav;           /* pointer used to traverse linked list  */

	if ( g_Tok == BUTTONS_TOK ) {

		g_Tok = yylex();
		if ( p_pairs_list( &pStrPair ) ) {

			if ( pSm->buttons == NULL ) {

				pSm->buttons = pStrPair;

			}
			else {

				if ( pSm->buttons->pNext == NULL ) {

					pSm->buttons->pNext = pStrPair;

				}
				else {

					for ( pTrav = pSm->buttons; pTrav->pNext; pTrav = pTrav->pNext )
						;
					pTrav->pNext = pStrPair;

				}

			}

			return 1;

		}
		else {

			ERROR("expected list of buttons");

		}

	}
	else return 0;
	
  return 0;
}

/************************************************************************
 *
 * Parses an attribute block.  Returns the head of a linked list 
 * containing all the attributes stored in a string pair structure.
 * Items are stored in the linked list in the same way they were 
 * encountered in the file.
 *
 */
static int p_attributes(TStringPair **ppSp)
{
  TStringPair    *pHead = NULL;
  TStringPair    *pNewNode, *pTrav;
  
  if ( g_Tok == ATTR_TOK ) {
    g_Tok = yylex();
    while ( g_Tok != END_ATTR_TOK ) {
      
      pNewNode        = NewStrPair();
      pNewNode->pName  = safe_strdup(yytext);
      g_Tok      = yylex();
      if ( g_Tok == '=' ) {
	char *pCR;
	consume_remainder_of_line();  /* puts chars in g_LargeTokVal */
	pNewNode->pValue = safe_strdup(g_LargeTokVal);
	g_Tok      = yylex();
	
	/* remove carriage return */
	pCR        = strchr(pNewNode->pValue, '\n');
	if ( pCR )
	  *pCR = '\0';
      }
      else
	ERROR("expected '=' sign");
      
      /* add to the linked list */
      if ( pHead == NULL  )
	pHead = pNewNode;
      else
	if ( pHead->pNext == NULL ) {
	  pHead->pNext = pNewNode;
	}
	else {
	  for ( pTrav = pHead; pTrav->pNext; pTrav=pTrav->pNext )
	    ;
	  pTrav->pNext = pNewNode;
	}
    }
    g_Tok = yylex();
    *ppSp = pHead;
    return 1;
  }
  else
    return 0;
}


/************************************************************************
 *
 * Parses a state machine block, appends the information in the
 * pPinfo data structure.
 *
 */
static int p_state_machine( TStateMachine **ppSm )
{
  int         HadProgress;
  int         HaveAttr = 0;
  TStateMachine*      pSm;
  TStringPair*  pStrPair;

  if ( g_Tok == STATEM_TOK ) {
    g_Tok = yylex();
    pSm   = NewSm();
    
    if ( g_Tok == IDENT ) {
      pSm->name = safe_strdup(yytext);
      g_Tok     = yylex();
      if ( strlen(pSm->name) > SM_MAX_HCSM_NAME_LEN - 1 ) {
        char err[512];
				
        sprintf(err, "name '%s' is too large, max size is %d",
								pSm->name, SM_MAX_HCSM_NAME_LEN - 1);
        ERROR(err);
      }
      if ( g_Tok == '{' ) {
				g_Tok = yylex();
				
				/*
				 * loop collects info about the current state machine.  terminates
				 * when the closing brace is found
				 */
				while ( g_Tok != '}' ) {
					HadProgress = 0;
					
					if ( g_Tok == PRE_ACTIVITY_TOK ) {
						HadProgress = 1;
						g_Tok = yylex();
						if ( g_Tok == IDENT ) {
							pSm->pre_activity = safe_strdup(yytext);
							g_Tok             = yylex();
						}
						else
							ERROR("expected pre-activity function name");
					}
					
					if ( g_Tok == POST_ACTIVITY_TOK ) {
						HadProgress = 1;
						g_Tok = yylex();
						if ( g_Tok == IDENT ) {
							pSm->post_activity = safe_strdup(yytext);
							g_Tok              = yylex();
						}
						else
							ERROR("expected post-activity function name");
					}
					
					if ( g_Tok == CREATE_CB_TOK ) {
						HadProgress = 1;
						g_Tok = yylex();
						if ( g_Tok == IDENT ) {
							pSm->create_cb = safe_strdup(yytext);
							g_Tok          = yylex();
						}
						else
							ERROR("expected create_cb function name");
					}
					
					if ( g_Tok == DELETE_CB_TOK ) {
						HadProgress = 1;
						g_Tok = yylex();
						if ( g_Tok == IDENT ) {
							pSm->delete_cb = safe_strdup(yytext);
							g_Tok          = yylex();
						}
						else
							ERROR("expected delete_cb function name");
					}
					
					HadProgress |= p_trans(pSm);
					HadProgress |= p_priv_user(pSm);
					HadProgress |= p_children(pSm);
					HadProgress |= p_inputs(pSm);
					HadProgress |= p_outputs(pSm);
					HadProgress |= p_locals(pSm);
					HadProgress |= p_dials(pSm);
					HadProgress |= p_dialinfo(pSm);
					HadProgress |= p_monitors(pSm);
					HadProgress |= p_buttons(pSm);

					if ( p_attributes(&pStrPair) ) {
						if ( HaveAttr )  {
							ERROR("too many attribute blocks in a state machine");
						}
						else {
							HaveAttr = 1;
						}
					}
					if ( !HadProgress ) {
						printf( "token = %s\n", safe_strdup( yytext  ) );
						g_Tok = yylex();
						printf( "next token = %s\n", safe_strdup( yytext  ) );
						ERROR("unexpected input");
					}
				}

				g_Tok = yylex();
				*ppSm = pSm;
				return 1;
			}
      else 
				ERROR("expected opening brace '{'");
    }
    else 
      ERROR("expected state machine name");
  }
  else {
    ppSm   = NULL;  /* cause a core dump if used */
    return   0;
  }

  return 0;
}


/************************************************************************
 *
 * Parses one connection specification, creates new node to hold it
 * ands makes the argument point to it.
 *
 */
static int p_one_connection(TConnection **ppConn)
{
  TConnection   *pConn;
  
  if ( g_Tok == '(' ) {
    g_Tok = yylex();
    if ( g_Tok == IDENT ) {
      pConn            = NewConn();
      pConn->source_sm = safe_strdup(yytext);
      g_Tok            = yylex();
      if ( g_Tok == '.' ) {
				g_Tok = yylex();
				if ( g_Tok == IDENT ) {
					pConn->source_var = safe_strdup(yytext);
					g_Tok             = yylex();
					if ( g_Tok == ',' ) {
						g_Tok = yylex();
						if ( g_Tok == IDENT ) {
							pConn->dest_sm = safe_strdup(yytext);
							g_Tok          = yylex();
							if ( g_Tok == '.' ) {
								g_Tok = yylex();
								if ( g_Tok == IDENT ) {
									pConn->dest_var = safe_strdup(yytext);
									g_Tok           = yylex();
									if ( g_Tok == ')' ) {
										g_Tok   = yylex();
										*ppConn = pConn;
										return    1;
									}
									else
										ERROR("expected ')'");
								}
								else
									ERROR("expected identifier");
							}
							else
								ERROR("expected a dot '.'");
						}
						else
							ERROR("expected identifier");
					}
					else
						ERROR("expected comma ','");
				}
				else
					ERROR("expected identifier");
      }
      else
				ERROR("expected a dot '.'");
    }
    else
      ERROR("expected an identifier");
  }
  else
    return 0;
	
  return 0;
}


/************************************************************************
 *
 * Parses the connections block, appends them to the linked list
 * of the main data structure.
 *
 */
static int p_connections(THcsmParserInfo *pPinfo)
{
  TConnection       *pConn, *pHead = NULL;
  TStringPair    *pStrPair;
  
  if ( g_Tok == CONNECTIONS_TOK ) {
    g_Tok = yylex();
    if ( g_Tok == '{' ) {
      g_Tok = yylex();
      while ( p_one_connection(&pConn) ) {
				if ( p_attributes(&pStrPair) ) {
					pConn->attr = pStrPair;
				}
				AppendConnToLinkList(&pHead, pConn);
      }
      if ( g_Tok == '}' ) {
				g_Tok             = yylex();
				pPinfo->conn_list = pHead;
				return              1;
      }
      else
				ERROR("expected closing brace '}'");
    }
    else 
      ERROR("expected opening brace '{'");
  }
  else
    return 0;
	
  return 0;
}

/************************************************************************
 *
 * Parses a pre-activity function block.  It creates a new node,
 * stores the function name and text, and returns the pointer.
 *
 */
static int PreActivityFunction( TStringPair **ppStrPair )
{

	TStringPair  *pNew;

	if ( g_Tok == PRE_ACTIVITY_FUNC_TOK ) {

		g_Tok = yylex();
		if ( g_Tok == IDENT ) {

			pNew        = NewStrPair();
			pNew->pName = safe_strdup( yytext );

			g_Tok = yylex();
			if ( g_Tok == CODE_TOK ) {

				pNew->pValue = safe_strdup( g_LargeTokVal );

				if ( IsBlank( pNew->pValue ) ) {

					char err[256];

					sprintf( err, "pre-activity function (%s) code block is empty",
							 pNew->pName );
					WARNING( err );

				}

				*ppStrPair = pNew;

				g_Tok = yylex();
				return 1;

			}
			else {
				
				ERROR( "expected pre-activity func code block (line with CODE on it)" );

			}

		}
		else {

			ERROR( "expected pre-activity function name" );

		}

	}
	else {

		return 0;

	}

	return 0;

}


/************************************************************************
 *
 * Parses a post-activity function block.  It creates a new node,
 * stores the function name and text, and returns the pointer.
 *
 */
static int PostActivityFunction( TStringPair **ppStrPair )
{

	TStringPair  *pNew;

	if ( g_Tok == POST_ACTIVITY_FUNC_TOK ) {

		g_Tok = yylex();
		if ( g_Tok == IDENT ) {

			pNew        = NewStrPair();
			pNew->pName = safe_strdup( yytext );

			g_Tok = yylex();
			if ( g_Tok == CODE_TOK ) {

				pNew->pValue = safe_strdup( g_LargeTokVal );

				if ( IsBlank( pNew->pValue ) ) {

					char err[256];

					sprintf( err, "post-activity function (%s) code block is empty",
							 pNew->pName );
					WARNING( err );

				}

				*ppStrPair = pNew;

				g_Tok = yylex();
				return 1;

			}
			else {
				
				ERROR( "expected post-activity func code block (line with CODE on it)" );

			}

		}
		else {

			ERROR( "expected post-activity function name" );

		}

	}
	else {

		return 0;

	}

	return 0;

}


/************************************************************************
 *
 * Checks the input character array to see if contains only blanks
 * and end of lines.  If so, it returns true.
 *
 */
static EBool IsBlank( const char *pCharArr )
{

	char  ch;
	int   i;

	i = 0;
	ch = pCharArr[0];

	while ( ch != '\0' ) {
		
		if ( ch != '\n' && ch != ' ' ) {
		
			/* found a non-blank character */
			return eFALSE;
			
		}
				
		i++;
		ch = pCharArr[i];

	}

	return eTRUE;

}


/************************************************************************
 *
 * Parses a predicate function block, It creates a new node,
 * stores the function name and text, and returns the pointer.
 *
 */
static int predicate_function( TStringPair **ppStrPair )
{

	TStringPair  *pNew;

	if ( g_Tok == PREDICATE_FUNC_TOK ) {

		g_Tok = yylex();
		if ( g_Tok == IDENT ) {

			pNew = NewStrPair();
			pNew->pName = safe_strdup( yytext );

			g_Tok = yylex();
			if ( g_Tok == CODE_TOK ) {

				pNew->pValue = safe_strdup( g_LargeTokVal );

				if ( IsBlank( pNew->pValue ) ) {

					char err[256];

					sprintf( err, "predicate function (%s) code block is empty",
							 pNew->pName );
					ERROR( err );

				}

				*ppStrPair = pNew;

				g_Tok = yylex();
				return 1;

			}
			else {

				ERROR("expected predicate func code block (line with CODE on it)");

			}

		}
		else {

			ERROR("expected predicate function name");

		}

	}
	else {

		return 0;

	}

	return 0;

}


/************************************************************************
 *
 * Parses a create callback function block.  It creates a new node,
 * stores the function name and text, and returns the pointer.
 *
 */
static int create_cb( TStringPair **ppStrPair )
{

	TStringPair *pNew;

	if ( g_Tok == CREATE_CB_TOK ) {

		g_Tok = yylex();
		if ( g_Tok == IDENT ) {

			pNew        = NewStrPair();
			pNew->pName = safe_strdup(yytext);

			g_Tok = yylex();
			if ( g_Tok == CODE_TOK ) {

				pNew->pValue = safe_strdup(g_LargeTokVal);

				if ( IsBlank( pNew->pValue ) ) {

					char err[256];

					sprintf( err, "creation function (%s) code block is empty",
							 pNew->pName );
					WARNING( err );

				}

				*ppStrPair   = pNew;

				g_Tok = yylex();
				return 1;

			}
			else {

				ERROR("expected create CB code block (line with CODE on it)");

			}

		}
		else {

			ERROR("expected create callback function name");

		}

	}
	else {

		return 0;

	}

	return 0;

}


/************************************************************************
 *
 * Parses a delete callback function block.  It creates a new node,
 * stores the function name and text, and returns the pointer.
 *
 */
static int delete_cb( TStringPair **pStrPair )
{

	TStringPair *pNew;

	if ( g_Tok == DELETE_CB_TOK ) {

		g_Tok = yylex();
		if ( g_Tok == IDENT ) {

			pNew        = NewStrPair();
			pNew->pName = safe_strdup(yytext);

			g_Tok = yylex();
			if ( g_Tok == CODE_TOK ) {

				pNew->pValue = safe_strdup(g_LargeTokVal);

				if ( IsBlank( pNew->pValue ) ) {

					char err[256];

					sprintf( err, "deletion function (%s) code block is empty",
							 pNew->pName );
					WARNING( err );

				}

				*pStrPair = pNew;

				g_Tok = yylex();
				return 1;

			}
			else {

				ERROR("expected delete CB code block (line with CODE on it)");

			}
		}
		else {

			ERROR("expected delete callback function name");

		}

	}
	else {

		return 0;

	}

	return 0;
}


/*************************************************************************
 * 
 *  Constructs predicate information in SM children.
 *
 */
void BuildPredicate( THcsmParserInfo* pInfo )
{

	TStateMachine*      pSm;
	TTransition*    pTr;

	/* go thru all state machines */
	for ( pSm = pInfo->sm_list; pSm; pSm = pSm->next ) {

		/* go thru all transition in this state machine */
		for ( pTr = pSm->trans; pTr; pTr = pTr->pNext ) {

			/* now enter transition information into child */
			TPredicate* pPred;
			TStateMachine*  pHead;
			
			pHead = LookupSm( pInfo->sm_list, pTr->pHeadName );
			if ( pHead == NULL ) INT_ERROR( pTr->pHeadName );
			
			/* the transition originates from the head child....
			 * enter predicate information into that child
			 */
			pPred = NewPred();
			pPred->pName = pTr->pPredName;
			AddPred( pHead, pPred );
			
		}
		
	}

}


/*************************************************************************
 * 
 *  Entry point for hcsm parser
 *
 */
static void hcsm_parse(void)
{
	int              found_something;
	THcsmParserInfo* pPinfo;
	TStateMachine*   pSm;

	pPinfo = &m_ParserInfo;

	/*
	 * Header is mandatory. 
	 */
	if ( !p_header( pPinfo ) ) {

		ERROR( "can't parse header" );

	}

	/*
	 * one or more global variable declarations.  Use a dummy sm
	 * definition to collect all global variables with scope that applies
	 * to all sms.
	 */
	pPinfo->glob_list = NULL;
	while ( p_globals( &pPinfo->glob_list ) ) {

		/* do nothing */
		;

	}

	/*
	 * connections are optional
	 */
	(void) p_connections( pPinfo );

	/*
	 * any number of SM/activity/predicate/createCB/deleteCB blocks
	 */
	while ( 1 )  {

		TStringPair* pStrPair;

		found_something = 0;

		if ( p_state_machine( &pSm ) ) {

			AddSm( pPinfo, pSm );

			while ( p_state_machine( &pSm ) ) {

				AddSm( pPinfo, pSm );

			}

			found_something |= 1;
		}

		pStrPair         = NULL;
		found_something |= PreActivityFunction( &pStrPair );
		if ( pStrPair )
			AppendStrPairToLinkList( &pPinfo->pre_actv_list, pStrPair );

		pStrPair         = NULL;
		found_something |= PostActivityFunction( &pStrPair );
		if ( pStrPair )
			AppendStrPairToLinkList( &pPinfo->post_actv_list, pStrPair );

		pStrPair         = NULL;
		found_something |= predicate_function( &pStrPair );
		if ( pStrPair )
			AppendStrPairToLinkList( &pPinfo->pred_list, pStrPair );

		pStrPair         = NULL;
		found_something |= create_cb( &pStrPair );
		if ( pStrPair )
			AppendStrPairToLinkList( &pPinfo->crcb_list, pStrPair );

		pStrPair         = NULL;
		found_something |= delete_cb( &pStrPair );
		if ( pStrPair )
			AppendStrPairToLinkList( &pPinfo->decb_list, pStrPair );

		if ( g_Tok == 0 ) break;

		if ( ! found_something ) {

			char  err[256];

			sprintf( err, "expected function block or state machine, got '%s'",
					 yytext );
			ERROR( err );

		}
	}

	UpdateRootField( pPinfo );
	BuildPredicate( pPinfo );

	return;

}


/*************************************************************************
 * 
 *  Extracts the path from a file name.
 *
 */
static void ExtractPathFromFileName(
			const char* pFileNameWithPath, 
			char* pPathStr
			)
{

	char* pFileName;
	/*
	 * Break the source file name into a path and a file name.
	 *
	 * Find the last occurance of '\' and then everything before
	 * it must be the path.
	 */
	pFileName = strrchr( pFileNameWithPath, '/' );
	if ( pFileName ) {

		int numCharsToCopy = strlen( pFileNameWithPath ) - strlen( pFileName );

		/* copy the final '\' also */
		numCharsToCopy++;

		strncpy( pPathStr, pFileNameWithPath, numCharsToCopy );
		pPathStr[numCharsToCopy] = '\0';

	}
	else {

		pPathStr[0] = '\0';

	}

}


static void IncludeFile( FILE* pOutStream, char* pSourceFileName )
{
	#define LINE_STRING_SIZE 10240

	char   sourcePath[1024];     /* the path associated with source file */
	char   line[LINE_STRING_SIZE];
	char   pIncFile[256];
	char   w1[256];
	int    linenum;
	FILE*  pSourceFile;

	ExtractPathFromFileName( pSourceFileName, sourcePath );

	/*
	 * Open the source file.
	 */
	pSourceFile = (FILE *) fopen( pSourceFileName, "r" );
	if ( pSourceFile == NULL ) {

		sprintf( line, "Can't open included file '%s'", pSourceFileName );
		perror( line );
		exit( -1 );
	}

	/*
	 * Scan the rest of the input file for lines that include other
	 * SM files.  Combine all files into a single stream.
	 */
	linenum = 0;
	while ( 1 ) {

		char* pNextString;
		int   save;

		pNextString = fgets( line, LINE_STRING_SIZE, pSourceFile );
		if ( pNextString == NULL )  break;
		linenum++;

		if ( sscanf(line, "%s %s", w1, pIncFile) == 2 && 
										!strcmp(w1, "ExtFile") ) {

			char incFileWithPath[1024];

			sprintf( incFileWithPath, "%s%s", sourcePath, pIncFile );

			save = linenum+1;
			/*fprintf(pOutStream, "#line 1 %s\n", pIncFile);*/

			IncludeFile(pOutStream, incFileWithPath);

			/*fprintf(pOutStream, "#line %d %s\n", save, sourceFileName);*/

		}
		else {

			fputs( line, pOutStream );

		}

		if ( feof( pSourceFile ) )  break;

	}

	fclose( pSourceFile );

}


static void DumpStructures( THcsmParserInfo* pInfo )
{

	TStateMachine*        pSm;

	/*
	 * Go thru all the state machines.
	 */

	fprintf( stdout, "Main Code Block:\n" );
	fprintf( stdout, "%s", pInfo->header_code );
	fprintf( stdout, "%d total HCSMs:\n", CountStm( pInfo->sm_list ) );

	for ( pSm = pInfo->sm_list; pSm; pSm = pSm->next ) {

		TPredicate* pPred;

		fprintf( stdout, "  %s\n", pSm->name );

		if ( pSm->pre_activity ) {

			fprintf( stdout, "    pre-activity:  %s\n", pSm->pre_activity );

		}
		
		if ( pSm->post_activity ) {

			fprintf( stdout, "    post-activity:  %s\n", pSm->post_activity );

		}
	
		fprintf( stdout, "    %d total predicates:\n", CountPred( pSm->pred ) );

		for ( pPred = pSm->pred; pPred; pPred = pPred->pNext ) {

			fprintf( stdout, "    %s\n", pPred->pName );

		}

	}


}


static void UpdateRootField( THcsmParserInfo* pInfo )
{

	TStateMachine*      pSmNode;

	pSmNode = pInfo->sm_list;

	/*
	 * Go through the entire list of state machines.
	 */
	for ( pSmNode = pInfo->sm_list; pSmNode; pSmNode = pSmNode->next ) {

		char*    pHcsmName;
		EBool   found = eFALSE;
		TStateMachine*   pNode;

		pHcsmName = pSmNode->name;

		/*
		 * For each state machines, check to see if any other state machine
		 * contains it as a child.  If not, then the state machine is a root
		 * Hcsm.
		 */
		for ( pNode = pInfo->sm_list; pNode; pNode = pNode->next ) {

			TStringNode*  pChild;

			for ( pChild = pNode->children; pChild; pChild = pChild->pNext ) {

				if ( !strcmp( pChild->pName, pHcsmName ) ) {

					/* found the string */
					found = eTRUE;
					break;

				}

			}

			if ( found )  break;

		}

		if ( found == eFALSE ) {
		  pSmNode->isRoot = eTRUE;
		}
		else {
		  pSmNode->isRoot = eFALSE;
		}

	}

}

/*****************************************************************************
 *   Main entry point
 *****************************************************************************/
main( int argc, char **argv )
{

	char          pTempName[512];          /* name of temporary file             */
	extern FILE*  yyin;                /* main input file                    */
	FILE*         pInFile;             /* input file                         */
	FILE*         pOutFile;            /* output file                        */
	int           jmpCode;             /* return value from setjmp           */

	/*
	 * Check arguments to application.
	 */
	if( argc != 2 ) 
	{
		fprintf( stderr, "Usage: %s input_file\n", argv[0] );
		exit( -1 );
	}

	/* 
	 * Open source file, get a temporary filename and then copy the original
	 * file to the temporary file after applying the ExtFile directives.
	 */
	strcpy(pTempName, getenv("TMP"));
	strcat(pTempName, "\\");
	strcat(pTempName, tmpnam( NULL ));

	
	
	pOutFile = fopen( pTempName, "w" );
	if( pOutFile == NULL ) 
	{
		char errmsg[200];
		sprintf( errmsg, "Can't write to temp file '%s' ", pTempName ); 
		perror( errmsg );
		exit(-1);
	}

	IncludeFile( pOutFile, argv[1] );
	fclose( pOutFile );

	/*
	 * Open processed file.
	 */
	pInFile = fopen( pTempName, "r" );
	if( pInFile == NULL ) 
	{
		perror( "Can't open input file" );
		exit( -1 );
	}

	yyin  = pInFile;
	g_Tok = yylex();

	jmpCode = setjmp( g_Recovery );
	if ( jmpCode != 0 ) {

		if( pTempName )  (void) _unlink( pTempName ); 
		exit( -1 );
	}

	/*
	 * Build parser data structures 
	 */
	hcsm_parse();
	fclose( pInFile );  
	(void) _unlink( pTempName );

	/*
	 * Verify that there are no semantic errors
	 */
	VerifySemantics( &m_ParserInfo );

#if 0
	/*
	 * Dump contents of structures to screen.
	 */
	DumpStructures( &m_ParserInfo );
#endif

	/*
	 * Generate the code
	 */
	GenerateCode( &m_ParserInfo );

	puts("OK");
	return 0;

}
