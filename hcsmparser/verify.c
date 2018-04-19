/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: verify.c,v 1.2 2003/06/20 15:39:44 oahmad Exp $
 *
 * Author:       Yiannis Papelis
 * Date:         November, 1995
 *
 * Description:
 *
 *  This module contains all the subroutines that verify the logical
 *  consistency of the data in an hcsm configuration file.
 *  The main subroutine accepts the parser data structure and then
 *  calls a variety of other routines to perform various consistency
 *  checks.
 *
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <setjmp.h>
#include <string.h>

#include "parser.h"

#define  NONE -1

/*************************************************************************
 *
 *  These subroutines detect cycles in the hcsm graph.  
 *
 *  The first one does a recursive postorder traversal of the graph, 
 *  searching for marked nodes.  It returns 1 if it finds a marked 
 *  node, 0 otherwise.
 *
 *  The second, traverses all hcsm nodes.  Ftor each state machine 
 *  node, it first marks it and then does a postorder traversal of the 
 *  subtree whose root is the current node using the first subroutine.
 *  If during the traversal the marked node is encountered then there
 *  is a cycle, otherwise there is no cycle.
 *
 */
static void Visit(
	TStateMachine             *pNode,         /* the node to visit                */
	THcsmParserInfo  *pPinfo,        /* the parser data structure        */
	int               parent)         /* which parent visits, can be NONE */
{
	TStateMachine      *pSm;     /* ptr to child state machine node       */
	TStringNode  *pChld;   /* ptr to child node, used to lookup pSm */
	EBool      found;   /* boolean                               */
	int         i;

/*
 * search list of visitors for current parent
 */
	found = eFALSE;
	for ( i = 0; i<pNode->vinfo->nVisitors; i++ ) {
		if ( pNode->vinfo->Visitors[i] == parent ) {
			found = eTRUE;
			break;
		}
	}

	if ( found )
		return;

/*
 * Add the parent to the visitors list
 */
	pNode->vinfo->Visitors = (int *)realloc(pNode->vinfo->Visitors, 
						++(pNode->vinfo->nVisitors) * sizeof(int));
	if ( pNode->vinfo->Visitors == NULL )
		SYS_ERROR("memory allocation failed");
	pNode->vinfo->Visitors[pNode->vinfo->nVisitors - 1] = parent;

/*
 * If the node has less than 2 visitors, then visit its children
 */
	if ( pNode->vinfo->nVisitors < 2 ) {
		for ( pChld = pNode->children; pChld; pChld = pChld->pNext ) {

			pSm = LookupSm(pPinfo->sm_list, pChld->pName);
			if ( pSm == NULL ) {
				char buf[200];

				sprintf(buf, "file %s:%d", __FILE__, __LINE__);
				INT_ERROR(buf);
			}

			Visit(pSm, pPinfo, pNode->id);
		}
	}
}

 
static void DetectCycles(THcsmParserInfo *pPinf)
{
	TStateMachine     *pSm, *pSm2;    /* utility pointers      */
	char      buf[200];       /* scratch string buffer */


/*
 * allocate memory for the visitor structures
 */
	for ( pSm = pPinf->sm_list; pSm; pSm = pSm->next ) {
		pSm->vinfo = (TVisitor *)calloc(sizeof(TVisitor), 1);
		if ( pSm->vinfo == NULL ) 
			SYS_ERROR("memory allocation failed");
	}

/*
 * For each node, perform a traversal of their subtree.  If the
 * root node is visited more than once, then it belongs to the cycle
 */
	for ( pSm = pPinf->sm_list; pSm; pSm = pSm->next ) {

		/* 
		 * Initialize the number of visitors from each node.  
		 * Leave the allocated
		 * memory there, it will automatically be reused as needed.
		 */
		for ( pSm2 = pPinf->sm_list;  pSm2; pSm2 = pSm2->next )
			pSm2->vinfo->nVisitors = 0;

		Visit(pSm, pPinf, NONE);
		if ( pSm->vinfo->nVisitors > 1 ) {
			sprintf(buf, "state machine '%s' is part of a cycle", pSm->name);
			SEM_ERROR(buf);
		}

	}

/*
 * Now that all is done, we can free memory used only for cycle detection
 */
	for ( pSm = pPinf->sm_list; pSm; pSm = pSm->next ) {
		if ( pSm->vinfo->Visitors )
			free(pSm->vinfo->Visitors);
		free(pSm->vinfo);
	}

}


/*************************************************************************
 *
 *  Dispatches other subroutines or locally performs consistency checks.
 *
 */
void VerifySemantics( THcsmParserInfo *pPinf )
{
	char        err_buf[512];
	TStateMachine       *pSm1, *pSm2;
	TStringNode   *pChld;
	TConnection      *pConn;
	TTransition     *pTrans;

/*
 * Detect duplicate names of state machines
 */
	for ( pSm1 = pPinf->sm_list; pSm1; pSm1 = pSm1->next ) {
		for ( pSm2 = pPinf->sm_list; pSm2; pSm2 = pSm2->next ) {
			if ( pSm1 != pSm2 && !strcmp(pSm1->name, pSm2->name) ) {
				sprintf(err_buf,"name '%s' used for multiple state machines",
					pSm1->name);
				SEM_ERROR(err_buf);
			}
		}
	}


/*
 * Detect duplicate names for children, buttons, inputs, outputs, 
 * locals, globals, dials or monitors
 */
	for ( pSm1 = pPinf->sm_list; pSm1; pSm1 = pSm1->next ) {
		char  *which;

		/* Children */
		if ( which = SearchDupName(pSm1->children) ) {
			sprintf(err_buf,"multiple use of child '%s' in state machine '%s'",
					which, pSm1->name);
			SEM_ERROR(err_buf);
		}

		/* Buttons */
		if ( which = SearchDupPair(pSm1->buttons) ) {
			sprintf(err_buf,"duplicate button definition of '%s' in '%s'",
					which, pSm1->name);
			SEM_ERROR(err_buf);
		}

		/* Inputs */
		if ( which = SearchDupPair(pSm1->inputs) ) {
			sprintf(err_buf,"duplicate definition of input var '%s' in '%s'",
					which, pSm1->name);
			SEM_ERROR(err_buf);
		}

		/* Outputs */
		if ( which = SearchDupPair(pSm1->outputs) ) {
			sprintf(err_buf,"duplicate definition of output var '%s' in '%s'",
					which, pSm1->name);
			SEM_ERROR(err_buf);
		}

		/* Locals */
		if ( which = SearchDupPair(pSm1->locals) ) {
			sprintf(err_buf,"duplicate definition of local var '%s' in '%s'",
					which, pSm1->name);
			SEM_ERROR(err_buf);
		}

		/* Dials */
		if ( which = SearchDupPair(pSm1->dials) ) {
			sprintf(err_buf,"duplicate definition of dial '%s' in '%s'",
					which, pSm1->name);
			SEM_ERROR(err_buf);
		}

		/* Monitors */
		if ( which = SearchDupPair(pSm1->monitors) ) {
			sprintf(err_buf,"duplicate definition of monitor '%s' in '%s'",
					which, pSm1->name);
			SEM_ERROR(err_buf);
		}
	}

/*
 * Verify that all cross-references originating from state machines
 * are valid.  
 */
	for ( pSm1 = pPinf->sm_list; pSm1; pSm1 = pSm1->next ) {

		/* make sure all children are valid state machines */
		for ( pChld = pSm1->children; pChld; pChld = pChld->pNext ) {
			if ( LookupSm(pPinf->sm_list, pChld->pName) == NULL ) {
				sprintf(err_buf, 
"sm '%s' references unknown child '%s'", pSm1->name, pChld->pName);
				SEM_ERROR(err_buf);
			}
		}

		/* make  sure all cross referencess in all transitions exist */
		for ( pTrans = pSm1->trans; pTrans; pTrans = pTrans->pNext ) {
			if ( LookupSm(pPinf->sm_list, pTrans->pHeadName) == NULL ) {
				sprintf(err_buf,
"unknown sm '%s' referenced in transitions of '%s'", pTrans->pHeadName, pSm1->name);
				SEM_ERROR(err_buf);
			}
			if ( LookupSm(pPinf->sm_list, pTrans->pTailName) == NULL ) {
				sprintf(err_buf,
"unknown sm '%s' referenced in transitions of '%s'", pTrans->pTailName, pSm1->name);
				SEM_ERROR(err_buf);
			}
			if ( LookupStrPair(pPinf->pred_list, pTrans->pPredName) == NULL ) {
				sprintf(err_buf,
"unknown predicate '%s' referenced in transitions of '%s'", pTrans->pPredName,
									pSm1->name);
				SEM_ERROR(err_buf);
			}
		}

		/* ensure the activity/create_cb/delete/cb funcs are there*/
		if ( pSm1->pre_activity ) {
			if ( LookupStrPair(pPinf->pre_actv_list, pSm1->pre_activity) == NULL ) {
				sprintf(err_buf, "unknown pre-activity function '%s' in sm '%s'",
									pSm1->pre_activity, pSm1->name);
				SEM_ERROR(err_buf);
			}
		}
		if ( pSm1->post_activity ) {
			if ( LookupStrPair(pPinf->post_actv_list, pSm1->post_activity) == NULL ) {
				sprintf(err_buf, "unknown post-activity function '%s' in sm '%s'",
									pSm1->post_activity, pSm1->name);
				SEM_ERROR(err_buf);
			}
		}
		if ( pSm1->create_cb ) {
			if ( LookupStrPair(pPinf->crcb_list, pSm1->create_cb) == NULL ) {
				sprintf(err_buf, "unknown creation callback '%s' in sm '%s'",
									pSm1->create_cb, pSm1->name);
				SEM_ERROR(err_buf);
			}
		}
		if ( pSm1->delete_cb ) {
			if ( LookupStrPair(pPinf->decb_list, pSm1->delete_cb) == NULL ) {
				sprintf(err_buf, "unknown deletion callback '%s' in sm '%s'",
									pSm1->delete_cb, pSm1->name);
				SEM_ERROR(err_buf);
			}
		}
	}


/* 
 * Verify that all references coming out of connections are valid.
 * The outer loop iterates on connection.
 * For each connection we first ensure that the source sm exists, and
 * then verify that the corresponding variable is part of the sm.
 * The same for the destination sm.
 */
	for ( pConn = pPinf->conn_list; pConn; pConn = pConn->next ) {
		int  IsValidVar;   /* Boolean */

		/* check references to source state machines */
		if ( ( pSm1 = LookupSm(pPinf->sm_list, pConn->source_sm)) == NULL )  {
			sprintf(err_buf, "unknown sm '%s' referenced in connections",
							pConn->source_sm);
			SEM_ERROR(err_buf);
		}
		IsValidVar = (LookupStrPair(pSm1->inputs, pConn->source_var)!=NULL)  |
				 	 (LookupStrPair(pSm1->outputs, pConn->source_var)!=NULL) |
				 	 (LookupStrPair(pSm1->locals, pConn->source_var)!=NULL);
		if ( !IsValidVar ) {
			sprintf(err_buf, 
					"unknown variable '%s.%s' referenced in connections",
					pSm1->name, pConn->source_var);
			SEM_ERROR(err_buf);
		}

		if ( ( pSm1 = LookupSm(pPinf->sm_list, pConn->dest_sm)) == NULL )  {
			sprintf(err_buf, "unknown sm '%s' referenced in connections",
							pConn->dest_sm);
			SEM_ERROR(err_buf);
		}
		IsValidVar = (LookupStrPair(pSm1->inputs, pConn->dest_var)!=NULL) |
				 	 (LookupStrPair(pSm1->outputs, pConn->dest_var)!=NULL) |
				 	 (LookupStrPair(pSm1->locals, pConn->dest_var)!=NULL);
		if ( !IsValidVar ) {
			sprintf(err_buf, 
					"unknown variable '%s.%s' referenced in connections",
					pSm1->name, pConn->source_var);
			SEM_ERROR(err_buf);
		}
	}


/*
 * Verify the structure of the parent child graph, by checking that
 * all references in the transitions of a state machine are its children
 */
	for ( pSm1 = pPinf->sm_list; pSm1; pSm1 = pSm1->next ) {
		for ( pTrans = pSm1->trans; pTrans; pTrans = pTrans->pNext ) {
			if ( LookupStrNode(pSm1->children, pTrans->pHeadName) == NULL ) {
				sprintf(err_buf, 
						"invalid transition, '%s' is not a child of '%s'",
						pTrans->pHeadName, pSm1->name);
			}
			if ( LookupStrNode(pSm1->children, pTrans->pTailName) == NULL ) {
				sprintf(err_buf, 
						"invalid transition, '%s' is not a child of '%s'",
						pTrans->pTailName, pSm1->name);
			}
		}
	}

/*
 * Verify that there are no cycles in the graph
 */
	DetectCycles(pPinf);
}
