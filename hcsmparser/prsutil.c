/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: prsutil.c,v 1.8 2003/06/20 15:39:44 oahmad Exp $
 *
 * Author:       Yiannis Papelis
 * Date:         November, 1995
 *
 * Description:  
 *
 *   Utility functions for the hcsm parser.  Primarily memory allocation,
 *   string and linked list manipulation.
 *
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <setjmp.h>

#include "parser.h"

/************************************************************************
 * memory and string functions that check for error returns
 */
char* safe_strdup(char* s)
{
	char* pNew = (char*) malloc( strlen( s ) + 1 );

	if( pNew == NULL ) SYS_ERROR( "memory allocation failed" );

	strcpy( pNew, s );
	return pNew;
}


/*************************************************************************
 *
 * Allocates a new state machine structure node, initializes all pointer
 * fields to NULL, and assigns identifier.  Identifiers start at 1.
 *
 */
TStateMachine* NewSm( void )
{
	TStateMachine* pSm;
	static int ident = 1;

   /* calloc sets all to NULL */
	pSm = ( TStateMachine* ) calloc( sizeof( TStateMachine ), 1 );
	if( pSm == NULL ) SYS_ERROR("memory allocation failed" );

	pSm->id = ident++;
	return pSm;
}


/*************************************************************************
 *
 * Allocates a new transition node, sets all fields to NULL and assigns
 * a positive, but not 0, identifier.
 *
 */
TTransition* NewTr( void )
{
	TTransition* pTr;
	static int count = 1;

	/* calloc sets all to NULL */
	pTr = ( TTransition* ) calloc( sizeof( TTransition ), 1 );
	if( pTr == NULL ) SYS_ERROR( "memory allocation failed" );

	pTr->id = count++;
	return pTr;
}


/*************************************************************************
 *
 * Allocates a new predicate node, sets all fields to NULL and assigns
 * a positive, but not 0, identifier.
 *
 */
TPredicate* NewPred( void )
{
	TPredicate* pPredFunc;
	static int count = 1;

	/* calloc sets all to NULL */
	pPredFunc = ( TPredicate* ) calloc( sizeof( TPredicate ), 1 );

	if( pPredFunc == NULL ) SYS_ERROR( "memory allocation failed" );

	pPredFunc->id = count++;
	return pPredFunc;
}


/*************************************************************************
 *
 * Allocates a new private user function node, sets all fields to NULL
 * and assigns a positive, but not 0, identifier.
 *
 */
TPrivUser* NewPriv( void )
{
	TPrivUser* pPriv;
	static int count = 1;

	/* calloc sets all to NULL */
	pPriv = ( TPrivUser* ) calloc( sizeof( TPrivUser ), 1 );
	if( pPriv == NULL ) SYS_ERROR( "memory allocation failed" );

	/*pPriv->id = count++;*/
	return pPriv;
}


/*************************************************************************
 *
 * Allocates a new string pair node and sets all fields to NULL.
 *
 */
TStringPair* NewStrPair( void )
{
	TStringPair* pNew;

	/* calloc clears all */
	pNew = ( TStringPair* ) calloc( sizeof( TStringPair ), 1 );
	if( pNew == NULL ) SYS_ERROR( "memory allocation failed" );

	return pNew;
}


/*************************************************************************
 *
 * Allocates a new string pair node and sets all fields to NULL.
 *
 */
TDialParam* NewDialParam( void )
{
	TDialParam* pNew;

	/* calloc clears all */
	pNew = ( TDialParam* ) calloc( sizeof( TDialParam), 1 );
	if( pNew == NULL ) SYS_ERROR( "memory allocation failed" );

	return pNew;
}


/*************************************************************************
 *
 * Counts number of nodes in a StrNode linked list
 *
 */
int   CountStrNode(TStringNode  *pNode)
{
	int    count = 0;

	for (  ; pNode; pNode = pNode->pNext )
		count++;

	return count;
}


/*************************************************************************
 *
 * Counts number of nodes in a state machine linked list
 *
 */
int   CountStm(TStateMachine  *pNode)
{
	int    count = 0;

	for (  ; pNode; pNode = pNode->next )
		count++;

	return count;
}


/*************************************************************************
 *
 * Counts number of nodes in a StrPair linked list
 *
 */
int   CountStrPair(TStringPair  *pNode)
{
	int    count = 0;

	for (  ; pNode; pNode = pNode->pNext )
		count++;

	return count;
}


/*************************************************************************
 *
 * Counts number of transition in a linked list
 *
 */
int   CountTrans(TTransition  *pNode)
{
	int    count = 0;

	for (  ; pNode; pNode = pNode->pNext )
		count++;

	return count;
}


/*************************************************************************
 *
 * Counts number of predicates in a linked list
 *
 */
int CountPred( TPredicate* pNode )
{
	int count = 0;

	for (  ; pNode; pNode = pNode->pNext )
		count++;

	return count;
}


/*************************************************************************
 *
 * Allocates a new connection structure, sets all fields to NULL, and
 * assigns a positive identifier (not 0)
 *
 */
TConnection* NewConn( void )
{
	TConnection* pNew;
	static int ident = 1;

	/* calloc clears all */
	pNew = ( TConnection* ) calloc( sizeof( TConnection ), 1 );
	if ( pNew == NULL ) SYS_ERROR("memory allocation failed");

	pNew->id = ident++;
	return pNew;
}


/*************************************************************************
 *
 * Allocates a new string pair node and sets all fields to NULL.
 *
 */
TStringNode* NewStrNode( void )
{
	TStringNode* pNew;

	/* calloc clears all */
	pNew = ( TStringNode* ) calloc( sizeof( TStringNode ), 1 );
	if ( pNew == NULL ) SYS_ERROR("memory allocation failed");

	return pNew;
}


/*************************************************************************
 *
 * Adds a connection structure to the end of a linked list.
 *
 */
void AppendConnToLinkList(TConnection **ppHead, TConnection *pItem)
{
	TConnection   *trav;

	if ( *ppHead == NULL ) {
		*ppHead = pItem;
	}
	else
	if ( (*ppHead)->next == NULL ) {
		(*ppHead)->next = pItem;
	}
	else {
		for (trav = *ppHead; trav->next; trav = trav->next)
			;
		trav->next = pItem;
	}
}


/*************************************************************************
 *
 * Adds a string pair structure to the end of a linked list.
 *
 */
void AppendStrPairToLinkList(TStringPair **ppHead, TStringPair *pItem)
{
	TStringPair   *trav;

	if ( *ppHead == NULL ) {
		*ppHead = pItem;
	}
	else
	if ( (*ppHead)->pNext == NULL ) {
		(*ppHead)->pNext = pItem;
	}
	else {
		for ( trav = *ppHead; trav->pNext; trav = trav->pNext )
			;
		trav->pNext = pItem;
	}
}


/*************************************************************************
 *
 * Adds a state machine node structure to the end of the linked list
 * in the parser info structure.
 *
 * Three cases: 
 *    empty list, just add the node,
 *    one node, add it directly,
 *    more than one node, trace the LL and append
 *
 */
void AddSm(THcsmParserInfo *pParInfo, TStateMachine *pSm)
{
	TStateMachine   *trav;    /* used to traverse the list */

	if ( pParInfo->sm_list == NULL ) {
		pParInfo->sm_list = pSm;
		return;
	}

	if ( pParInfo->sm_list->next == NULL ) {
		pParInfo->sm_list->next = pSm;
		return;
	}

	for ( trav = pParInfo->sm_list; trav->next; trav = trav->next ) 
		;
	trav->next = pSm;
}


/*************************************************************************
 *
 * Adds a transition node structure to the end of a linked list
 *
 * Three cases: 
 *    empty list, just add the node,
 *    one node, add it directly,
 *    more than one node, trace the LL and append
 *
 */
void AddTrans(TStateMachine *pSm, TTransition *pTr)
{
	TTransition   *trav;    /* used to traverse the list */

	if ( pSm->trans == NULL ) {
		pSm->trans = pTr;
		return;
	}

	if ( pSm->trans->pNext == NULL ) {
		pSm->trans->pNext = pTr;
		return;
	}

	for ( trav = pSm->trans; trav->pNext; trav = trav->pNext ) 
		;
	trav->pNext = pTr;
}



/*************************************************************************
 *
 * Adds a predicate node structure to the end of a linked list
 *
 * Three cases: 
 *    empty list, just add the node,
 *    one node, add it directly,
 *    more than one node, trace the LL and append
 *
 */
void AddPred( TStateMachine* pSm, TPredicate* pPredFunc )
{

	TPredicate* pTrav;    /* used to traverse the list */

	if ( pSm->pred == NULL ) {

		pSm->pred = pPredFunc;
		return;

	}

	if ( pSm->pred->pNext == NULL ) {

		pSm->pred->pNext = pPredFunc;
		return;

	}

	for ( pTrav = pSm->pred; pTrav->pNext; pTrav = pTrav->pNext ) {
		;
	}

	pTrav->pNext = pPredFunc;

}


/*************************************************************************
 *
 * Adds a private function node structure to the end of a linked list
 *
 * Three cases: 
 *    empty list, just add the node,
 *    one node, add it directly,
 *    more than one node, trace the LL and append
 *
 */
void AddPriv(TStateMachine *pSm, TPrivUser *pTr)
{
	TPrivUser   *trav;    /* used to traverse the list */

	if ( pSm->priv_user_func == NULL ) {
		pSm->priv_user_func = pTr;
		return;
	}

	if ( pSm->priv_user_func->pNext == NULL ) {
		pSm->priv_user_func->pNext = pTr;
		return;
	}

	for ( trav = pSm->priv_user_func; trav->pNext; trav = trav->pNext ) 
		;
	trav->pNext = pTr;
}


/*************************************************************************
 *
 *  Searches the state machine list for one with the given name
 *
 */
TStateMachine *LookupSm(TStateMachine *pHead, char *pName)
{
	for (   ; pHead; pHead = pHead->next )
		if ( !strcmp(pHead->name, pName) )
			return pHead;
	return NULL;
}



/*****************************************************************************
 *
 *  Searches the state machine list for one with the given name and
 *  returns the state machine and its order in the list.
 *
 */
int LookupSmPos( TStringNode* pHead, char* pName )
{

	int     smPositionInList;       /* position in list of matched sm       */

	smPositionInList = 0;

	/*
	 * Traverse through list of children and see which child HCSM's name
	 * matches the given name.
	 */
	for ( ; pHead; pHead = pHead->pNext ) {

		if ( !strcmp( pHead->pName, pName ) ) {

			/* return the HCSM and it's position in the list */
			return smPositionInList;

		}

		smPositionInList++;

	}

	/* no match found */
	return -1;

}



/*************************************************************************
 *
 *  Searches the predicate list for one with the given name.
 *
 */
TPredicate* LookupPred( TPredicate* pPredHead, char *pName )
{
	for ( ; pPredHead; pPredHead = pPredHead->pNext ) {

		if ( !strcmp( pPredHead->pName, pName ) )  return pPredHead;

	}

	/* didn't find a match */
	return NULL;

}



/******************************************************************************
 *
 * Description:  Searches a linked list of TStringNode nodes for duplicate
 *   names.
 *
 * Remarks:  Given a pointer to a linked list of TStringNode nodes, this 
 *   function searches that linked list for duplicate names.  It returns 
 *   NULL if it finds none, or a pointer to the duplicate name.
 *
 * Arguments:
 *   pHead - Pointer to the head of the linked list.
 *
 * Returns:
 *   The function returns NULL if it fails to find any duplicate names.
 *   Otherwise, it returns a pointer to the first duplicate name it finds.
 *
 */
char* SearchDupName( TStringNode* pHead )
{

	TStringNode*  pNode1;           /* pointer to a node on the linked list */
	TStringNode*  pNode2;           /* pointer to a node on the linked list */

	for ( pNode1 = pHead; pNode1; pNode1 = pNode1->pNext ) {

		for ( pNode2 = pHead; pNode2; pNode2 = pNode2->pNext ) {

			if ( pNode1 != pNode2 && !strcmp( pNode1->pName, pNode2->pName ) )  {

				/* found a duplicate name */
				return pNode1->pName;

			}

		}

	}

	/* no duplicate names found */
	return NULL;

}


/******************************************************************************
 *
 * Description:  Searches a linked list of TStringPair nodes for duplicate
 *   names.
 *
 * Remarks:  Given a pointer to a linked list of TStringPair nodes, this 
 *   function searches that linked list for duplicate names.  It returns 
 *   NULL if it finds none, or a pointer to the duplicate name.
 *
 * Arguments:
 *   pHead - Pointer to the head of the linked list.
 *
 * Returns:
 *   The function returns NULL if it fails to find any duplicate names.
 *   Otherwise, it returns a pointer to the first duplicate name it finds.
 *
 */
char* SearchDupPair( TStringPair* pHead )
{
	TStringPair*  pNode1;           /* pointer to a node on the linked list */
	TStringPair*  pNode2;           /* pointer to a node on the linked list */

	for ( pNode1 = pHead; pNode1; pNode1 = pNode1->pNext ) {

		for ( pNode2 = pHead; pNode2; pNode2 = pNode2->pNext ) {

			if ( pNode1 != pNode2 && 
				 !strcmp( pNode1->pName, pNode2->pName ) )  {

				/* found a duplicate name */
				return pNode1->pName;

			}

		}

	}

	/* no duplicate names found */
	return NULL;
}


/*************************************************************************
 *
 *  Searches a linked list of name pairs by name
 *
 */
TStringPair *LookupStrPair(TStringPair *pHead, char *pName)
{
	for (   ; pHead; pHead = pHead->pNext )
		if ( !strcmp(pHead->pName, pName) )
			return pHead;
	return NULL;
}


/*************************************************************************
 *
 *  Searches a linked list of strings nodes by name
 *
 */
TStringNode *LookupStrNode(TStringNode *pHead, char *pName)
{
	for (   ; pHead; pHead = pHead->pNext )
		if ( !strcmp(pHead->pName, pName) )
			return pHead;
	return NULL;
}

/*************************************************************************
 *
 *  Discards a linked list
 *
 */
void DiscardStrPairList(TStringPair  *pHead)
{
	TStringPair   *tem;

	if ( pHead == NULL )
		return;

	if ( pHead->pNext == NULL ) {
		if ( pHead->pName )
			free(pHead->pName);
		if ( pHead->pValue )
			free(pHead->pValue);
	}

	do {
		tem = pHead->pNext;

		if ( pHead->pName )
			free(pHead->pName);
		if ( pHead->pValue )
			free(pHead->pValue);
		free(pHead);

		pHead = tem;

	} while ( pHead );

}


/*************************************************************************
 *
 *  Assigns identifiers to a linked list of nodes. Returns lowest identifier
 *  that was not assigned to any node.
 *
 */
int AssignIdsToStrPairList(TStringPair  *pHead, int  start)
{

	for ( ; pHead; pHead = pHead->pNext ) {

		pHead->id = start++;

	}

	return start;

}

/*********************************************************************
 * 
 * add entry to cache list 
 * 
 */

void addToCache(TStringPair **cache, TStringPair *obj)
{
  TStringPair *pObj;

  pObj = (TStringPair *)malloc(sizeof(TStringPair));

  pObj->id = obj->id;
  pObj->pName = obj->pName;

  /* tack on to front of cache list */
  pObj->pNext = *cache;

  /* assign cache to be obj */
  *cache = pObj;
}


/*********************************************************************
 * 
 * Maintain cache of button and dial (name,id) pairs to insure that
 * the ids are unique 
 * 
 */

int checkCache(TStringPair **cache, TStringPair *obj)
{
  TStringPair *pCache, *pObj;
  EBool found = eFALSE;
  int return_value;

  if (*cache == NULL) {

    /* create new object */
    pObj = (TStringPair *)malloc(sizeof(TStringPair));
    pObj->id = obj->id;
    pObj->pName = obj->pName;
    *cache = pObj;
    pObj->pNext = NULL;

    return_value = 0;
  }
  else {

    pCache = *cache;
    while ((pCache != NULL) && (found != eTRUE)) {
      
      if (!strcmp(pCache->pName, obj->pName)) {
	/* found it... return the id */
	return_value = pCache->id;
	found = eTRUE;
      }
      else {
	/* advance pointer */
	pCache = pCache->pNext;
      }
    }
    
    if (found != eTRUE) {
      /* add object to cache */
      addToCache(cache, obj);
      return_value = 0;
    }
  }

  return(return_value);
}


/*************************************************************************
 *
 *  Assigns unique identifiers to a linked list of nodes.  Returns the lowest
 *  identifier that was not assigned to any node.
 *
 */
int AssignUniqueIdsToStrNodeList(TStringNode  *pHead, int  start)
{
  static TStringPair *buttonCache = NULL;
  TStringPair tempHolder;
  int returnedId;

  for ( ; pHead; pHead = pHead->pNext ) {

    /* assign temp id first */
    pHead->id = start;

    /* temporary holder so I can still use other cache */
    tempHolder.id = pHead->id;
    tempHolder.pName = pHead->pName;
    returnedId = checkCache(&buttonCache, &tempHolder);

    if (returnedId == 0)
      start++;
    else
      pHead->id = returnedId;
    
  }
  return start;
}

/*************************************************************************
 *
 *  Assigns unique identifiers to a linked list of nodes. Returns lowest 
 *  identifier that was not assigned to any node.  For use with the 
 *  assigning of ids to buttons, dials, and monitors.
 *
 *  if type equals 0 - Dial
 *                 1 - Input Parameters
 *                 2 - Output Parameter
 *                 3 - Local Vars
 *                 4 - Global Vars
 *                 5 - Monitors
 */

int AssignUniqueIdsToStrPairList(TStringPair *pHead, int start, int type)
{
  static TStringPair *dialCache = NULL;
  static TStringPair *monitorCache = NULL;
  static TStringPair *IparCache = NULL;
  static TStringPair *OparCache = NULL;
  static TStringPair *LvarCache = NULL;
  static TStringPair *GvarCache = NULL;
  static TStringPair *buttonCache = NULL;
  int returnedId;

  /* I hope this code doesn't get read until I can clean it...
     maybe around Dec! :o  - Pete */

  for ( ; pHead; pHead = pHead->pNext ) {

    /* assign temp id first */
    pHead->id = start;

    if (type == 0) 
      returnedId = checkCache(&dialCache, pHead);
    else if (type == 1)
      returnedId = checkCache(&IparCache, pHead);
    else if (type == 2)
      returnedId = checkCache(&OparCache, pHead);
    else if (type == 3)
      returnedId = checkCache(&LvarCache, pHead);
    else if (type == 4)
      returnedId = checkCache(&GvarCache, pHead);
    else if (type == 5)
      returnedId = checkCache(&monitorCache, pHead);
    else if (type == 6)
      returnedId = checkCache(&buttonCache, pHead);
    else
      fprintf(stderr, "Internal error: type incorrect for Assigning Ids\n");

    if (returnedId == 0)
      start++;
    else
      pHead->id = returnedId;
    
  }

  return start;
}

/*************************************************************************
 *
 *  Assigns identifiers to a linked list of nodes.  Returns the lowest
 *  identifier that was not assigned to any node.
 *
 */
int  AssignIdsToStrNodeList(TStringNode  *pHead, int  start)
{
	for ( ; pHead; pHead = pHead->pNext ) {
		pHead->id = start++;
	}
	return start;
}


/*************************************************************************
 *
 *  Assigns identifiers to a linked list of transitions.  Returns the 
 *  lowest  identifier that was not assigned to any node.
 *
 */
int  AssignIdsToTransList(TTransition  *pHead, int  start)
{
	for ( ; pHead; pHead = pHead->pNext ) {
		pHead->id = start++;
	}
	return start;
}


/***************************************************************************/
void dump_ds(THcsmParserInfo *pPinfo)
{
	TStateMachine      *pSm;
	TTransition   *pTr;
	TConnection    *pConn;

	printf("Version string:  '%s'\n", pPinfo->version);
	if ( pPinfo->header_code ) {
		printf("Header code\n-----------------\n%s\n--------------------\n",
				pPinfo->header_code);
	}
	else
		printf("No header code.\n");

	for ( pSm = pPinfo->sm_list; pSm; pSm = pSm->next ) {
		printf("*********** State machine '%s'\n", pSm->name);

		if ( pSm->pre_activity )
			printf("Pre-Activity func:   '%s'\n", pSm->pre_activity);
		if ( pSm->post_activity )
			printf("Post-Activity func:   '%s'\n", pSm->post_activity);
		if ( pSm->create_cb )
			printf("Creation func:   '%s'\n", pSm->create_cb);
		if ( pSm->delete_cb )
			printf("Delete func  :   '%s'\n", pSm->delete_cb);

		if ( pSm->trans ) { 
			printf("  Transitions: \n");
			for ( pTr=pSm->trans; pTr; pTr = pTr->pNext ) {
				printf("    From '%s' to '%s', func '%s'.\n", pTr->pHeadName,
								pTr->pTailName, pTr->pPredName);
				if ( pTr->pAttrs ) {
					TStringPair   *p;
	
					printf("        Attributes\n");
					for (p=pTr->pAttrs; p; p = p->pNext) {
						printf("           %s = '%s'\n", p->pName, p->pValue);
					}
				}
				else
					printf("       No attributes.\n");
			}
		}

		if ( pSm->children ) { 
			TStringNode   *pStr;

			printf("  Children: ");
			for ( pStr=pSm->children; pStr; pStr = pStr->pNext ) {
				printf("'%s' ", pStr->pName);
			}
			printf("\n");
		}

		if ( pSm->buttons ) {
			TStringPair   *pStr;

			printf("  Buttons: ");
			for ( pStr=pSm->buttons; pStr; pStr = pStr->pNext ) {
				printf("'%s' ", pStr->pName);
			}
			printf("\n");
		}

		if ( pSm->inputs ) {
			TStringPair  *pPair;

			printf("  Inputs: \n");
			for (pPair=pSm->inputs; pPair; pPair = pPair->pNext ) {
				printf("    '%s'  '%s'\n", pPair->pName, pPair->pValue);
			}
			printf("\n");
		}

		if ( pSm->outputs ) {
			TStringPair  *pPair;

			printf("  Outputs: \n");
			for (pPair=pSm->outputs; pPair; pPair = pPair->pNext ) {
				printf("    '%s'  '%s'\n", pPair->pName, pPair->pValue);
			}
			printf("\n");
		}

		if ( pSm->locals ) {
			TStringPair  *pPair;

			printf("  Locals: \n");
			for (pPair=pSm->locals; pPair; pPair = pPair->pNext ) {
				printf("    '%s'  '%s'\n", pPair->pName, pPair->pValue);
			}
			printf("\n");
		}

		if ( pSm->dials ) {
			TStringPair  *pPair;

			printf("  Dials: \n");
			for (pPair=pSm->dials; pPair; pPair = pPair->pNext ) {
				printf("    '%s'  '%s'\n", pPair->pName, pPair->pValue);
			}
			printf("\n");
		}

		if ( pSm->monitors ) {
			TStringPair  *pPair;

			printf("  Monitors: \n");
			for (pPair=pSm->monitors; pPair; pPair = pPair->pNext ) {
				printf("    '%s'  '%s'\n", pPair->pName, pPair->pValue);
			}
			printf("\n");
		}
	}

	if ( pPinfo->conn_list ) {
		printf("Connections:\n");
		for (pConn = pPinfo->conn_list; pConn; pConn = pConn->next) {
			printf("  %s.%s  ->  %s.%s\n", pConn->source_sm, pConn->source_var,
							pConn->dest_sm, pConn->dest_var);
			if ( pConn->attr ) {
				TStringPair  *pPair;

				printf("    ATTR\n");
				for (pPair = pConn->attr; pPair; pPair = pPair->pNext) {
					printf("      %s = '%s'\n", pPair->pName, pPair->pValue);
				}
				printf("\n");
			}
		}
	}

	if ( pPinfo->pre_actv_list ) {
		TStringPair    *pS;

		printf("Pre-Activity Functions:\n");
		for (pS=pPinfo->pre_actv_list; pS; pS = pS->pNext) {
			printf(" ---->%s<----\n", pS->pName);
			printf("%s", pS->pValue);
			printf("--------------\n");
		}
	}

	if ( pPinfo->post_actv_list ) {
		TStringPair    *pS;

		printf("Post-Activity Functions:\n");
		for (pS=pPinfo->post_actv_list; pS; pS = pS->pNext) {
			printf(" ---->%s<----\n", pS->pName);
			printf("%s", pS->pValue);
			printf("--------------\n");
		}
	}

	if ( pPinfo->pred_list ) {
		TStringPair    *pS;

		printf("Predicate Functions:\n");
		for (pS=pPinfo->pred_list; pS; pS = pS->pNext) {
			printf(" ---->%s<----\n", pS->pName);
			printf("%s", pS->pValue);
			printf("--------------\n");
		}
	}

	if ( pPinfo->crcb_list ) {
		TStringPair    *pS;

		printf("Creation Callback Functions:\n");
		for (pS=pPinfo->crcb_list; pS; pS = pS->pNext) {
			printf(" ---->%s<----\n", pS->pName);
			printf("%s", pS->pValue);
			printf("--------------\n");
		}
	}

	if ( pPinfo->decb_list ) {
		TStringPair    *pS;

		printf("Delete Callback Functions:\n");
		for (pS=pPinfo->decb_list; pS; pS = pS->pNext) {
			printf(" ---->%s<----\n", pS->pName);
			printf("%s", pS->pValue);
			printf("--------------\n");
		}
	}
}
