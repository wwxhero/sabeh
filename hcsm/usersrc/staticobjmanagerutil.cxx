/***************************************************************************** *
 * (C) Copyright 2001 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: staticobjmanagerutil.cxx,v 1.18 2018/09/07 14:38:23 IOWA\dheitbri Exp $
 *
 * Author:  Matt Schikore
 *
 * Date:    January, 2001
 *
 * Description:  Contains code for the static object manager HCSM.
 *
 ****************************************************************************/

#include <cvedpub.h>
#include <sobjmngrparseblock.h>
#include <sol2.h>
#include <hcsmcollection.h>

#include <transmat.h>

#include "staticobjmanagerutil.h"

using namespace CVED;

void
StaticObjManInitialSetup( 
			const CSobjMngrParseBlock* cpSnoBlock,
			CCved& cved,
			CHcsmCollection& rootCollection,
			CHcsm* pStaticObjMan 
			)
{
	CSnoBlock::cTChildIterator pItr;
	for( pItr = cpSnoBlock->BeginChild(); pItr != cpSnoBlock->EndChild(); pItr++ )
	{
		CStaticObjParseBlock rB( *pItr );

		//
		// If this is a LRI static object then set its option (if needed) and 
		// skip to the next object.
		//
		bool isLriStaticObj = !rB.GetIsNewObj();
		if( isLriStaticObj ) 
		{
			int cvedId;
			if( cved.GetObj( rB.GetName(), cvedId ) ) 
			{
				cved.SetObjOption( cvedId, rB.GetOption() );
			}
		   	continue;
		}

		const CSolObj* pSolObj = cved.GetSol().GetObj( rB.GetSolName() );
		if( NULL == pSolObj ) 
		{
			fprintf( 
				stderr, 
				"Sabeh:StaticObjManInitialSetup: Warning: Unknown SOL name '%s'\n",
				rB.GetSolName().c_str()
				);
			continue;
		}

		cvTObjAttr  attr;
		memset( &attr, 0, sizeof(attr) );
		attr.solId = pSolObj->GetId();
		attr.hcsmId = 0;
		attr.xSize = pSolObj->GetLength();
		attr.ySize = pSolObj->GetWidth();
		attr.zSize = pSolObj->GetHeight();
		attr.colorIndex = rB.GetColorIndex();

		cvEObjType type = cvString2ObjType( pSolObj->GetCategoryName().c_str() );

		cvTObjState state;	
		memset( &state, 0, sizeof(state) );

		const CSolObjComposite* pComposite = dynamic_cast<const CSolObjComposite*>( pSolObj );
		if (pComposite) {
			// set the explicit object's composite object state in CVED
			vector<CStaticObjParseBlock::TCompositePiece> refs = rB.GetCompositePieces();

			state.compositeSignState.numChildren = refs.size();
			int index = 0;
			vector<CStaticObjParseBlock::TCompositePiece>::const_iterator i;
			for( i = refs.begin(); i != refs.end(); i++ ) 
			{
				state.compositeSignState.childrenOpts[index] = i->option;
				index++;
			}
		}

		state.anyState.vel = 0;	

		CPoint3D pt3D = rB.GetPosition();
		state.anyState.position.x = pt3D.m_x;
		state.anyState.position.y = pt3D.m_y;
		state.anyState.position.z = pt3D.m_z;

		double yaw = rB.GetOrientation();
		state.anyState.tangent.i = cos( yaw );
		state.anyState.tangent.j = sin( yaw );
		state.anyState.tangent.k = 0.0;
		state.anyState.lateral.i = state.anyState.tangent.j;
		state.anyState.lateral.j = 0.0 - state.anyState.tangent.i;
		state.anyState.lateral.k = 0.0;

		state.anyState.audioState = rB.GetAudioState();
		state.anyState.visualState = rB.GetVisualState();

		vector<CPoint2D>    pts;
        CPoint2D            pt2D;
        pt2D.m_x = 0 - attr.xSize/2;
        pt2D.m_y = 0 - attr.ySize/2;
        pts.push_back( pt2D );
        pt2D.m_x = 0 - attr.xSize/2;
        pt2D.m_y = 0 + attr.ySize/2;
        pts.push_back( pt2D );
        pt2D.m_x = 0 + attr.xSize/2;
        pt2D.m_y = 0 + attr.ySize/2;
        pts.push_back( pt2D );
        pt2D.m_x = 0 + attr.xSize/2;
        pt2D.m_y = 0 - attr.ySize/2;
        pts.push_back( pt2D );

        CTransMat rotMatrix;
        rotMatrix.RotZ( yaw );
        vector<CPoint2D>::iterator tempIdx;
        for( tempIdx = pts.begin(); tempIdx != pts.end(); tempIdx++ )
		{
            *tempIdx = rotMatrix.Apply( *tempIdx );
            tempIdx->m_x += state.anyState.position.x;
            tempIdx->m_y += state.anyState.position.y;
		} 
		CBoundingBox bb( pts );
		state.anyState.boundBox[0].x = bb.GetMinX();
		state.anyState.boundBox[0].y = bb.GetMinY();
		state.anyState.boundBox[1].x = bb.GetMaxX();
		state.anyState.boundBox[1].y = bb.GetMaxY();

		//
		// Create the static obj and if it's an explicit object then set its
		// option if provided.
		//
		int cvedId = cved.CreateStaticObj( rB.GetName(), type, attr, state );
		bool staticObjError = cvedId < 0;
		if( staticObjError )
		{
			fprintf( 
				stderr, 
				"StaticObjManagerUtil: unable to create static object named '%s'\n", 
				rB.GetName().c_str()
				);
			continue;
		}
		else if( rB.GetOption() != 0 ) 
		{
			// set the explicit static object's option state in CVED
			cved.SetObjOption( cvedId, rB.GetOption() );
		}
	}
	
	// now get all of the CVED static objects and set their HCSM id to me
	CCved::TIntVec vec;
	cved.GetAllStaticObjs( vec );
	CCved::TIntVec::iterator itr;
	for( itr = vec.begin(); itr != vec.end(); itr++ ) 
	{
		CObj obj( cved, *itr );
		obj.SetHcsmId( rootCollection.GetHcsmId( pStaticObjMan ) );
	}
}  // end of InitialSetup


