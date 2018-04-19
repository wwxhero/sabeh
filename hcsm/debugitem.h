/*****************************************************************************
 *
 * (C) Copyright 1999 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: debugitem.h,v 1.16 2014/04/23 19:22:54 IOWA\dheitbri Exp $
 *
 * Author(s):    Yiannis Papelis
 *
 * Date:         October, 1999
 *
 * Description:  Interface & implementation for the debug item class.
 *
 ****************************************************************************/

#ifndef __DEBUG_ITEM_H_
#define __DEBUG_ITEM_H_

#include <string>
using namespace std;

#define DEBUG_TEXT_LEN   32
#define GR_DEBUG_TEXT_LEN   16

/////////////////////////////////////////////////////////////////////////////
/// This class represents a debug item or record.  It can be either
/// a string or a graphic command.  The Hcsms can log such messages
/// which can later be viewed by client programs.
///
/// Even though the class looks a natural for a hierarcy (i.e., derive
/// a separate class for text and various graphics), it is kept as
/// a flat class to increase performance and make coding simpler, especially
/// when it is inserted/extracted from queues.
///\ingroup HCSM
/////////////////////////////////////////////////////////////////////////////
class CHcsmDebugItem {
public:
	CHcsmDebugItem() : m_type(eNONE) {};
	~CHcsmDebugItem() {};

	CHcsmDebugItem(const CHcsmDebugItem &);
	const CHcsmDebugItem &operator=(const CHcsmDebugItem &);

	struct TItem {
		int  nParts;			// for messages taking more than one structure
		long frame;
		int  hcsmId;
		int  level;
		int  isItText;			// if set, it's text, else graphic
		union {
			struct TText {
				char  c[DEBUG_TEXT_LEN];
			} text;
			struct TGraphic {
				union {
					struct { // used for shapes
						double p1, p2, p3, p4;
						char  fill;
					} shape;
					struct { // used for text
						char c[GR_DEBUG_TEXT_LEN];
						double x, y;
						char align;
					} text;
				};
				char  type;		// 0 = line, 1 = rect, 2 = circle
				short color;
			} graphic;
		}    data;
	};

	void store(vector<TItem> &) const;
	void load(const vector<TItem> &);

//
// The debug level indicates a threshold below which messages are 
// not logged.  When the debug level of the HCSM is ROUTINE, 
// messages of any level are logged.  When the level is Normal, only
// normal and urgent messages are logged.  When the level is Urgent,
// only urgent messages are logged.
	enum ELevel {
			eDEBUG_ROUTINE,
			eDEBUG_NORMAL,
			eDEBUG_URGENT
	};

	// An enumeration for where to align graphical text
	enum EAlign {
			eTOP_LEFT,
			eTOP_RIGHT,
			eBOTTOM_LEFT,
			eBOTTOM_RIGHT
	};

	// A simple color enumeration to be used in the graphics commands
	enum EColor { 
			eWHITE, 
			eBLACK, 
			eRED,
			eGREEN,
			eBLUE,
			eYELLOW,
			eMAGENTA
	};

	// what it is
	enum EType {
		eNONE,
		eTEXT,
		eLINE,
		eRECT,
		eCIRCLE,
		eGRAPHIC_TEXT,
		eTRIGGER_FIRE
	};

public:
	void SetLine(double, double, double, double, EColor, bool);
	void SetCircle(double, double, double, EColor, bool);
	void SetRect(double, double, double, double, EColor, bool);
	void SetText(const string &);
	void SetTriggerFire(int hcsmId);
	void SetGraphicalText(const string &, double, double, EColor, bool);
	void SetFrame(long f) { m_frame = f; };
	void SetHcsmId(int i) { m_hcsmId = i; };
	void SetLevel(ELevel l) { m_level = l; };

	void GetLine(double &, double &, double &, double &, EColor &, bool &) const;
	void GetCircle(double &, double &, double &, EColor &, bool &) const;
	void GetRect(double &, double &, double &, double &, EColor &, bool &) const;
	void GetGraphicalText(string &, double &, double &, EColor &, bool &) const;
	void GetText(string &) const;
	EType GetType(void) const { return m_type; };
	long GetFrame(void) const { return m_frame; };
	int  GetHcsmId(void) const { return m_hcsmId; };
	ELevel GetLevel(void) const { return m_level; };
	inline char GetLevelChar(void) const;

private:
	long     m_frame;					// which frame the item was logged
	int      m_hcsmId;					// which Hcsm logged the item

	EType    m_type;					// what it is
	ELevel   m_level;					// the event's level

	double    m_x1, m_y1, m_x2, m_y2;	// endpoints for line/rect
	double    m_radius;					// circle radius
	EColor   m_color;					// color, for graphics
	bool     m_filled;					// if graphics primitives are "filled"
	string   m_str;						// string
};



/////////////////////////////////////////////////////////////////////////////
//
// The assignment operator performs a "selective" copy, carying only
// things that matter.
//
//
inline const CHcsmDebugItem &
CHcsmDebugItem::operator=(const CHcsmDebugItem &rhs)
{
	if ( this != &rhs ) {
		switch ( rhs.m_type ) {
			case eNONE:			// no reason to copy anything
				m_type = rhs.m_type;
				break;

			case eTEXT:
				m_type    = rhs.m_type;
				m_level   = rhs.m_level;
				m_frame   = rhs.m_frame;
				m_hcsmId  = rhs.m_hcsmId;
				m_str     = rhs.m_str;
				break;

			case eLINE:
				m_type    = rhs.m_type;
				m_level   = rhs.m_level;
				m_frame   = rhs.m_frame;
				m_hcsmId  = rhs.m_hcsmId;
				m_x1      = rhs.m_x1;
				m_y1      = rhs.m_y1;
				m_x2      = rhs.m_x2;
				m_y2      = rhs.m_y2;
				m_color   = rhs.m_color;
				break;

			case eRECT:
				m_type    = rhs.m_type;
				m_level   = rhs.m_level;
				m_frame   = rhs.m_frame;
				m_hcsmId  = rhs.m_hcsmId;
				m_x1      = rhs.m_x1;
				m_y1      = rhs.m_y1;
				m_x2      = rhs.m_x2;
				m_y2      = rhs.m_y2;
				m_color   = rhs.m_color;
				m_filled  = rhs.m_filled;
				break;

			case eCIRCLE:
				m_type    = rhs.m_type;
				m_level   = rhs.m_level;
				m_frame   = rhs.m_frame;
				m_hcsmId  = rhs.m_hcsmId;
				m_x1      = rhs.m_x1;
				m_y1      = rhs.m_y1;
				m_radius  = rhs.m_radius;
				m_color   = rhs.m_color;
				m_filled  = rhs.m_filled;
				break;

			case eGRAPHIC_TEXT:
				m_type    = rhs.m_type;
				m_level   = rhs.m_level;
				m_frame   = rhs.m_frame;
				m_hcsmId  = rhs.m_hcsmId;
				m_x1      = rhs.m_x1;
				m_y1      = rhs.m_y1;
				m_str     = rhs.m_str;
				m_color   = rhs.m_color;
				m_filled  = rhs.m_filled;
				break;

			case eTRIGGER_FIRE:
				m_type = rhs.m_type;
				m_level   = rhs.m_level;
				m_frame   = rhs.m_frame;
				m_hcsmId = rhs.m_hcsmId;
				m_x1      = rhs.m_x1;
				m_y1      = rhs.m_y1;
				break;
		}
	}

	return *this;
}


/////////////////////////////////////////////////////////////////////////////
//
// Description: store the debug item into one or more TItem structures
//
// Remarks:
// Only strings can overflow and would need to be stored in multiple items
//
inline void 
CHcsmDebugItem::store(vector<TItem> &Items) const
{
	Items.clear();

	if ( m_type == eTEXT ) {
		int quant = DEBUG_TEXT_LEN-1;

		if ( (int)m_str.size()+1 > quant ) {
			int nParts;
			nParts = 1 + (int)(m_str.size() + 1 ) / quant;

			TItem item = { 0 };
			item.frame    = m_frame;
			item.hcsmId   = m_hcsmId;
			item.isItText = 1;
			item.level = (int)m_level;

			int i;
			for (i=0; i<nParts; i++) {
				TItem   it;

				it = item;
				if ( i == 0 ) it.nParts = nParts;
				memcpy(it.data.text.c, m_str.data() + i * quant, quant);
				Items.push_back(it);
			}
		}
		else {
			TItem item    = { 0 };
			item.nParts   = 1;
			item.frame    = m_frame;
			item.hcsmId   = m_hcsmId;
			item.isItText = 1;
			item.level    = (int)m_level;
			strcpy_s(item.data.text.c, m_str.c_str());

			Items.push_back(item);
		}
	}
	else {
		TItem item = { 0 };

		item.nParts   = 1;
		item.frame    = m_frame;
		item.hcsmId   = m_hcsmId;
		item.isItText = 0;
		item.level    = (int)m_level;
		if (m_type != eGRAPHIC_TEXT) {
			//  The item is not graphical text
			item.data.graphic.shape.p1 = m_x1;
			item.data.graphic.shape.p2 = m_y1;
			item.data.graphic.shape.p3 = m_x2;
			item.data.graphic.shape.fill  = (int)m_filled;
			if ( m_type == eCIRCLE )
				item.data.graphic.shape.p4 = m_radius;
			else
				item.data.graphic.shape.p4 = m_y2;
		} else {
			//  The item is graphical text
			item.data.graphic.text.x = m_x1;
			item.data.graphic.text.y = m_y1;
			item.data.graphic.text.align = m_filled;

			int len = (int)m_str.size();
			if (len > GR_DEBUG_TEXT_LEN - 1) len = GR_DEBUG_TEXT_LEN - 1;
			memcpy(item.data.graphic.text.c, m_str.data(), len);
		}

		item.data.graphic.color = (int)m_color;
		item.data.graphic.type = m_type;

		Items.push_back(item);
	}
}


inline void 
CHcsmDebugItem::load(const vector<TItem> &Items)
{
	vector<TItem>::const_iterator pI;

	if ( Items.empty() ) {
		m_type = eNONE;
		return;
	}

	pI = Items.begin();
	if ( pI->isItText ) {
		m_frame  = pI->frame;
		m_hcsmId = pI->hcsmId;
		m_type   = eTEXT;
		m_level  = (ELevel)pI->level;
		m_str = "";

		if ( Items.begin()->nParts > 1 ) {
			for (pI = Items.begin(); pI != Items.end(); pI ++) {
				string s(pI->data.text.c);

				m_str += s;
			}
		}
		else {
			m_str = pI->data.text.c;
		}
	}
	else {
		m_frame  = pI->frame;
		m_hcsmId = pI->hcsmId;
		m_level  = (ELevel)pI->level;
		m_type   = (EType)pI->data.graphic.type;
		m_color   = (EColor)pI->data.graphic.color;

		if (m_type != eGRAPHIC_TEXT) {
			m_x1      = pI->data.graphic.shape.p1;
			m_y1      = pI->data.graphic.shape.p2;
			m_x2      = pI->data.graphic.shape.p3;
			m_radius  = m_y2 = pI->data.graphic.shape.p4;
			m_filled  = pI->data.graphic.shape.fill ? true : false;
		} else {
			m_str     = pI->data.graphic.text.c;
			m_x1      = pI->data.graphic.text.x;
			m_y1      = pI->data.graphic.text.y;
			m_filled  = pI->data.graphic.text.align ? true : false;
		}
	}
}


inline
CHcsmDebugItem::CHcsmDebugItem(const CHcsmDebugItem &copy)
{
	*this = copy;
}


/////////////////////////////////////////////////////////////////////////////
//
// Description:
// Remarks:
// Returns: Nothing
// Arguments:
//
inline void 
CHcsmDebugItem::SetLine(double x1, double y1, double x2, double y2, EColor c, bool arrow)
{
	m_type    = eLINE;
	m_x1      = x1;
	m_y1      = y1;
	m_x2      = x2;
	m_y2      = y2;
	m_color   = c;
	m_filled  = arrow; // for a line, 'filled' means it's an arrow
}


/////////////////////////////////////////////////////////////////////////////
//
// Description:
// Remarks:
// Returns: Nothing
// Arguments:
//
inline void
CHcsmDebugItem::SetGraphicalText(const string& str, double x, double y, EColor c, bool align)
{
	m_type   = eGRAPHIC_TEXT;
	m_x1     = x;
	m_y1     = y;
	m_color  = c;
	m_str    = str;
	m_filled = align;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description:
// Remarks:
// Returns: Nothing
// Arguments:
//
inline void 
CHcsmDebugItem::SetCircle(double x, double y, double r, EColor c, bool f)
{
	m_type    = eCIRCLE;
	m_x1      = x;
	m_y1      = y;
	m_radius  = r;
	m_color   = c;
	m_filled  = f;
}


/////////////////////////////////////////////////////////////////////////////
//
// Description:
// Remarks:
// Returns: Nothing
// Arguments:
//
inline void 
CHcsmDebugItem::SetRect(double x1, double y1, double x2, double y2, 
						EColor c, bool b)
{
	m_type    = eRECT;
	m_x1      = x1;
	m_y1      = y1;
	m_x2      = x2;
	m_y2      = y2;
	m_color   = c;
	m_filled  = b;
}


/////////////////////////////////////////////////////////////////////////////
//
// Description:
// Remarks:
// Returns: Nothing
// Arguments:
//
inline void 
CHcsmDebugItem::SetText(const string &str)
{
	m_type  = eTEXT;
	m_str   = str;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description:
// Remarks:
// Returns: Nothing
// Arguments:
//
inline void
CHcsmDebugItem::SetTriggerFire(int hcsmId)
{
	m_type = eTRIGGER_FIRE;
	m_hcsmId = hcsmId;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description:
// Remarks:
// Returns: Nothing
// Arguments:
//
inline void 
CHcsmDebugItem::GetLine(double &x1, double &y1, double &x2, 
						double &y2, EColor &c, bool &b) const
{
	x1  = m_x1;
	y1  = m_y1;
	x2  = m_x2;
	y2  = m_y2;
	c   = m_color;
	b   = m_filled;
}


/////////////////////////////////////////////////////////////////////////////
//
// Description:
// Remarks:
// Returns: Nothing
// Arguments:
//
inline void 
CHcsmDebugItem::GetCircle(double &xc, double &yc, double &rad, 
						  EColor &c, bool &f) const
{
	xc  = m_x1;
	yc  = m_y1;
	rad = m_radius;
	c   = m_color;
	f   = m_filled;
}


/////////////////////////////////////////////////////////////////////////////
//
// Description:
// Remarks:
// Returns: Nothing
// Arguments:
//
inline void 
CHcsmDebugItem::GetRect(double &x1, double & y1, double &x2, double &y2, 
						EColor &c, bool &f) const
{
	x1 = m_x1;
	y1 = m_y1;
	x2 = m_x2;
	y2 = m_y2;
	c  = m_color;
	f  = m_filled;
}


/////////////////////////////////////////////////////////////////////////////
//
// Description:
// Remarks:
// Returns: Nothing
// Arguments:
//
inline void
CHcsmDebugItem::GetGraphicalText(string& str, double& x, double& y,
		EColor& c, bool& align) const
{
	x     = m_x1;
	y     = m_y1;
	c     = m_color;
	str   = m_str;
	align = m_filled;
}

/////////////////////////////////////////////////////////////////////////////
//
// Description:
// Remarks:
// Returns: Nothing
// Arguments:
//
inline void 
CHcsmDebugItem::GetText(string &str) const
{
	str = m_str;
}


inline char 
CHcsmDebugItem::GetLevelChar(void) const
{
	switch ( m_level ) {
		case eDEBUG_NORMAL : return 'N';
		case eDEBUG_ROUTINE : return 'R';
		case eDEBUG_URGENT : return 'U';
		default : return '?';
	}
}


#endif // __DEBUG_ITEM_H_
