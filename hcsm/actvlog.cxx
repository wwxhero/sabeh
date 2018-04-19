/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version: $Id: actvlog.cxx,v 1.5 2016/10/28 20:53:08 IOWA\dheitbri Exp $
 *
 * Author:  Yiannis Papelis
 *
 * Date:    May, 2003
 *
 * Description:  The implementation file for the classes that implement
 *   the activity log mechanism.
 *
 ****************************************************************************/

#include "actvlog.h"

// Provide consistency in error messages
static const char s_Pre[] = "Activity Log";


/////////////////////////////////////////////////////////////////////////////
//
// This function initializes the specific instance.  All events are lost
// and memory is freed
//
void
CActvLog::Init()
{
	m_nodes.clear();
	m_curNode.m_numItems     = 0;
	m_curNode.m_nextFreeByte = 0;
	m_nextIter               = 0;
	m_verbose                = false;
}


/////////////////////////////////////////////////////////////////////////////
//
// Stores all logged events in a file whose name is specified as the sole
// argument.  
//
// Returns:
// True if everything was ok or false otherwise.  In case of errors, error
// messages are printed to stderr.
//
bool
CActvLog::Store( const string& cFileName )
{
	FILE* pF = fopen(cFileName.c_str(), "wb");
	if ( pF == 0 ) {
		fprintf(
			stderr, 
			"%s: Can't open file %s to store output: %s\n",
			s_Pre, 
			cFileName.c_str(), 
			_sys_errlist[errno]
			);

		return false;
	}

	EActvLogType magic = eMAGIC;
	fwrite(&magic, 1, sizeof(magic), pF);
	fwrite(g_ActvLogVersion, 1, sizeof(g_ActvLogVersion), pF);

	list<ActvNode>::iterator p;
	for (p=m_nodes.begin(); p!= m_nodes.end(); p++) {
		StoreNode(pF, *p);
	}
	StoreNode(pF, m_curNode);

	fwrite(&magic, 1, sizeof(magic), pF);
	fclose(pF);

	return true;
}


/////////////////////////////////////////////////////////////////////////////
//
// Loads all events from file whose name is specified as the sole
// argument.  Once loading is completed, events can be accessed with the
// Get and GetNext() functions.
//
// Returns:
// True if everything was ok or false otherwise.  In case of errors, error
// messages are printed to stderr.
//
bool
CActvLog::Load(
	const string& cFileName,	// name of file
	bool          ignVersion	// if true, ignore version incompatibilities
	)	
{
	list<ActvNode>::iterator   p;
	miniHeader  hdr;
	CRootEvent* pEv;

	Init();

	FILE *outtest;
	outtest = fopen("TestOutput.txt", "w");

	FILE* pF = fopen(cFileName.c_str(), "rb");
	if ( pF == 0 ) {
		fprintf(
			stderr, 
			"%s: Can't open file %s to load events: %s\n",
			s_Pre, 
			cFileName.c_str(), 
			_sys_errlist[errno]
			);
		return false;
	}

	EActvLogType magic;
	if ( fread(&magic, sizeof(magic), 1, pF) != 1 ) {
		fprintf(stderr, "%s: Load failed: cannot read magic number.\n", s_Pre);
		fclose(pF);
		return false;
	}

	if ( magic != eMAGIC ) {
		fprintf(stderr, "%s: Load failed: file is not an activity log.\n", s_Pre);
		fclose(pF);
		return false;
	}

	char fileVer[sizeof(g_ActvLogVersion)];

	if ( fread(fileVer, sizeof(g_ActvLogVersion), 1, pF) != 1 ) {
		fprintf(stderr, "%s: Load failed: cannot read file version.\n", s_Pre);
		fclose(pF);
		return false;
	}

	if ( !ignVersion && strcmp(fileVer, g_ActvLogVersion) ) {
		fprintf(stderr, "%s: Load failed: incompatible versions "
			"(file=%s, sw=%s).\n", s_Pre, fileVer, g_ActvLogVersion);
		fclose(pF);
		return false;
	}

	while ( 1 ) {
		size_t nRead;

		if ( (nRead = fread(&hdr, 1, sizeof(hdr), pF)) != sizeof(hdr) ) {
			if ( nRead == sizeof(EActvLogType) && hdr.type == eMAGIC ) {
				break;
			}
			fprintf(stderr, "%s: Load failed: premature end of file.\n", s_Pre);
			fclose(pF);
			return false;
		}

		pEv = CRootEvent::CreateEventByType(hdr.type);
		if ( pEv == 0 ) {
			fprintf(stderr, "%s: Load failed: unexpected event type.\n", s_Pre);
			fclose(pF);
			return false;
		}

		char* pB = new char[pEv->GetSize()];
		fread(pB, pEv->GetSize(), 1, pF);
		pEv->CopyData(pB);
		delete[] pB;

		fprintf(outtest, "%d\n", hdr.frame);
		Add(hdr.frame, pEv);
		delete pEv;
	}

	fclose(outtest);
	fclose(pF);
	return true;
}


/////////////////////////////////////////////////////////////////////////////
//
// Returns the number of events in the log.
//
int  
CActvLog::GetCount( void ) const
{
	int count = 0;

	list<ActvNode>::const_iterator   p;

	for (p=m_nodes.begin(); p!= m_nodes.end(); p++) {
		count += p->m_numItems;
	}
	count += m_curNode.m_numItems;

	return count;
}


/////////////////////////////////////////////////////////////////////////////
//
// Sets the verbose flag according to value.  When the verbose flag is on, 
// each logged event is printed on stdout as well as stored internally.
//
void
CActvLog::SetVerbose( bool value )
{
	m_verbose = value; 
}

/////////////////////////////////////////////////////////////////////////////
//
// Adds an event to the log.
//
void
CActvLog::Add( int frame, const CRootEvent* cpItem )
{
	miniHeader header;

	int itemSize = cpItem->GetSize() ;

	header.type  = cpItem->GetId();
	header.frame = frame;
	header.len   = itemSize;

	if ( m_curNode.m_nextFreeByte + itemSize + sizeof(miniHeader) <= ACTIV_NODE_DATA_SIZE ) {	// fits
		memcpy(m_curNode.m_data + m_curNode.m_nextFreeByte, &header, sizeof(header));
		memcpy(m_curNode.m_data + m_curNode.m_nextFreeByte + sizeof(header),
					cpItem->GetData(), itemSize);
		m_curNode.m_nextFreeByte += itemSize + sizeof(header);
		m_curNode.m_numItems++;
	}
	else {	// full, add to the list and start a new one
		m_nodes.push_back(m_curNode);

		memcpy(m_curNode.m_data, &header, sizeof(header));
		memcpy(m_curNode.m_data + sizeof(header), cpItem->GetData(), itemSize);
		m_curNode.m_numItems     = 1;
		m_curNode.m_nextFreeByte = itemSize + sizeof(header);
	}

	if ( m_verbose )  cpItem->Print(frame);
}


/////////////////////////////////////////////////////////////////////////////
//
// This function returns the data associated with a logged event.  
// The event is identified by its cardinal number within the log.  The
// data, its type and the frame during which it was logged is returned.
//
// The function also establishes the "current" event, which is used by
// the GetNext() function to determine which event to return.
//
// Args:
//   which - the cardinal number of the event to return; can be between
//           0 and the GetCount() - 1
//   buf   - a buffer onto which the data for the event will be stored.
//           Make sure the buffer is large enough to accommodate the
//           largest event type.  Use the MAX_ACTIVITY_LOG_SIZE macro
//           to dimension the buffer, as it is guaranteed to be larger
//           than the largest event.
//
//   type  - the type of the event
//   frame - the frame during which the event was logged
//
// Returns:
//   True if the event was found else false.
//
bool 
CActvLog::Get(
	int            which, 
	char           buf[MAX_ACTIVITY_LOG_SIZE], 
	EActvLogType&  type, 
	int&           frame
	)
{
	list<ActvNode>::iterator  p;
	int                       first;
	bool                      gotit = false;

	first = 0;
	for( p = m_nodes.begin(); p != m_nodes.end(); p++ ) 
	{
		if( which >= first && which < first + p->m_numItems ) 
		{
			gotit = true;
			GetFromNode( *p, which, first, buf, type, frame );
			break;
		}

		first += p->m_numItems;
	}

	if( !gotit ) 
	{
		gotit = GetFromNode( m_curNode, which, first, buf, type, frame );
	}

	if( gotit ) m_nextIter = which+1;
	return gotit;
}


/////////////////////////////////////////////////////////////////////////////
//
// This function returns the data associated with a logged event.  
// The event is the one immediately after the value specified in the most
// recent Get() function.  If the Get() function hasn't been called, 
// event 0 is returned.  Starting with 0, this function can be called
// multiple times to return all the events in the log.
//
// The arguments and return value are identical to the Get() function.
//
bool 
CActvLog::GetNext(
	char           buf[MAX_ACTIVITY_LOG_SIZE], 
	EActvLogType&  type, 
	int&           frame
	)
{
	return Get( m_nextIter++, buf, type, frame );
}


/////////////////////////////////////////////////////////////////////////////
//
// Internal support function that reads a specific event from a node
//
//
bool
CActvLog::GetFromNode(
		ActvNode &node,							// the node to read from 
		int which,								// which event to ready
		int firstInNode,						// first event in node
		char buf[MAX_ACTIVITY_LOG_SIZE],		// where to copy the data
		EActvLogType& type,						// what type it is
		int&          frame
		)					// when it was logged
{
	int ofs  = 0;
	int skip = which - firstInNode;

	if( skip < 0 || skip >= node.m_numItems )  return false;

	miniHeader* pHdr;
	int i;
	for( i = 0; i < skip; i++ ) 
	{
		pHdr = (miniHeader *)&node.m_data[ofs];
		ofs += sizeof(miniHeader) + pHdr->len;
	}
	pHdr = (miniHeader *)&node.m_data[ofs];
	ofs += sizeof(miniHeader);
	memcpy( buf, &node.m_data[ofs], pHdr->len );
	type  = pHdr->type;
	frame = pHdr->frame;

	return true;
}


/////////////////////////////////////////////////////////////////////////////
//
// Internal support function what writes all event in a node to a file
//
//
bool
CActvLog::StoreNode(
		FILE*      pF,		// where to store the data; assume file is open
		ActvNode&  node		// the node to store
		)
{
	int          ofs = 0;
	miniHeader*  pH;

	int i;
	for( i = 0; i < node.m_numItems; i++ ) 
	{
		pH = (miniHeader*) ( &node.m_data[ofs] );
		fwrite( pH, 1, sizeof(miniHeader), pF );
		fwrite( &node.m_data[ofs + sizeof(miniHeader)], 1, pH->len, pF );
		ofs += sizeof(miniHeader)+pH->len;
	}

	return true;
}


#if 0
int main(int argc, char* argv[])
{
	CActvLog  log;
	CEventTestEvent  e1;
	CEventTestEvent  e2;

	int count;
	for (count=0; count<20; count++) {
		e1.m_data.p1 = 1+count;
		e1.m_data.p2 = 1+count;
		e1.m_data.p3[0] = 0;	e1.m_data.p3[1] = 1;
		e1.m_data.p3[2] = 2;	e1.m_data.p3[3] = 3;
		e1.m_data.p4 = count;
		log.Add(0, &e1);

		e2.m_data.p1 = 100*count;
		e2.m_data.p4 = 100.0f*count;
		log.Add(1, &e2);
	}

	printf("Added %d event 1 and %d event 1\n", count, count);

	char name[] = "c:\\tempHcsmLog.log";
//	log.Store(name);

	CActvLog log2;

	if ( !log2.Load(name) ) {
		fprintf(stderr, "**** Load failed\n");
		exit(-1);
	}

	char buf[MAX_ACTIVITY_LOG_SIZE];
	EActvLogType type;
	CRootEvent *pE;
	int frm;

	if ( log2.Get(0, buf, type, frm) ) {

		pE = CRootEvent::CreateEventByType(type);
		pE->CopyData(buf);
//		pE->Print(frm);
		delete pE;

		while ( log2.GetNext(buf, type, frm) ) {
			pE = CRootEvent::CreateEventByType(type);
			pE->CopyData(buf);
//			pE->Print(frm);
			delete pE;
		}
	}

	printf("----Reverse: (%d items)\n", log2.GetCount());
	for (int i = log2.GetCount()-1; i>=0; i--) {
		log2.Get(i, buf, type, frm);
		pE = CRootEvent::CreateEventByType(type);
		pE->CopyData(buf);
//		pE->Print(frm);
		delete pE;

	}
	return 0;
}

#endif
