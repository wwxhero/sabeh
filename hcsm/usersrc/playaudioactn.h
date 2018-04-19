/////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by NADS & Simulation Center, The University of
//     Iowa.  All rights reserved.
//
// Version:     $Id: playaudioactn.h,v 1.8 2013/05/08 15:31:03 IOWA\vhorosewski Exp $
// Author(s):   Jillian Vogel
// Date:        October, 1999
//
// Description: The definition of CPlayAudioActn, a subclass of CAction.  
//	This class causes the indicated objects to play the indicated audios.
//
//	This file should not be directly included.  Include action.h instead.
//
/////////////////////////////////////////////////////////////////////////////

#ifndef _PLAY_AUDIO_ACTN_H_
#define _PLAY_AUDIO_ACTN_H_

#include "hcsmspec.h"
#include "action.h"

class CHcsmCollection;

class CPlayAudioActn : public CAction
{
public:
	CPlayAudioActn( const CActionParseBlock* cpBlock, CHcsmCollection* );
	CPlayAudioActn( const CPlayAudioActn& cRhs );
	~CPlayAudioActn();
	CPlayAudioActn& operator=( const CPlayAudioActn& cRhs );
	virtual bool IsFinished() { return m_isFinished; };

	void Execute( const set<CCandidate>* cpInstigators = 0 );
	inline const char* GetName() const { return "PlayAudio"; };

protected:
	CHcsmCollection* m_pHC;
	string           m_audio;

	int              m_FrameCnt;
	bool			 m_isFinished;
	bool             m_started;
};

#endif	// _PLAY_AUDIO_ACTN_H_

