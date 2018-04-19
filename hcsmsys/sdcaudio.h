//////////////////////////////////////////////////////////////////////////////
//
// (C) Copyright 1998 by National Advanced Driving Simulator and
// Simulation Center, the University of Iowa and The University
// of Iowa. All rights reserved.
//
// Version:      $Id: sdcaudio.h,v 1.1 2002/10/07 03:42:27 Yiannis Papelis Exp $
//
// Author(s):    
//
// Date:         October, 2002
// Description:  Interface file for SDC audio playback
//
//////////////////////////////////////////////////////////////////////////////

// NOTE: file assumes it is included after cved header files

bool InitAudio();
void InitAudioForRun(const CCved *);
void AudioExec();
void TermAudioForRun();
void TermAudio();
