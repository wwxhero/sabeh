/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: parser.h,v 1.19 2011/10/18 15:22:25 iowa\dheitbri Exp $
 *
 * Author:       Yiannis Papelis
 * Date:         November, 1995
 *
 * Description:  The header file for the hcsm parser.
 *
 ****************************************************************************/
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

/*  Symbols for the lexical analyzer */
#define IDENT                  1
#define POS_INT                2
#define INIT_MARKER_TOK        3
#define STATEM_TOK             4
#define PRE_ACTIVITY_TOK       5
#define POST_ACTIVITY_TOK      6
#define CREATE_CB_TOK          7
#define DELETE_CB_TOK          8
#define PRIV_DECL_TOK          9
#define TRANSITIONS_TOK        10
#define CHILDREN_TOK           11
#define INPUTS_TOK             12
#define OUTPUTS_TOK            13
#define LOCALS_TOK             14
#define GLOBALS_TOK            15
#define BUTTONS_TOK            16
#define DIALS_TOK              17
#define MONITORS_TOK           18
#define CONNECTIONS_TOK        19
#define PRE_ACTIVITY_FUNC_TOK  20
#define POST_ACTIVITY_FUNC_TOK 21
#define CODE_TOK               22
#define PREDICATE_FUNC_TOK     23
#define ATTR_TOK               24
#define END_ATTR_TOK           25
#define VERSION_TOK            26
#define DIAL_INFO_TOK          27

/*** the following #define reflects a value in sm.h */
#define SM_MAX_HCSM_NAME_LEN  64

#define ERROR(__msg) \
  fprintf(stderr,"Syntax error in %s, line %d.\n  Err msg: %s.\n",g_CurFile,\
	g_LineNum, __msg), longjmp(g_Recovery, 1)

#define SEM_ERROR(msg) \
  fprintf(stderr, "Semantic error: %s.\n", msg), longjmp(g_Recovery, 2)

#define SYS_ERROR(msg) \
  fprintf(stderr, "System error: %s.\n", msg), \
  longjmp(g_Recovery, 3)

#define INT_ERROR(msg) \
  fprintf(stderr, "Internal error @ %s:%d %s.\n", __FILE__, __LINE__, msg), \
  longjmp(g_Recovery, 3)

#define WARNING(__msg) \
  fprintf(stderr,"Warning on %s, line %d.\n  Warning msg: %s.\n",g_CurFile,\
	g_LineNum, __msg)

#ifdef _WIN32
extern char*   yytext;
#else
extern char    yytext[];
#endif

extern int     yyleng;

extern int     g_LineNum;
extern int     g_Tok;
extern char*   g_LargeTokVal;
extern char    g_CurFile[512];
extern jmp_buf g_Recovery;

typedef enum { eTRUE= 1, eFALSE = 0 } EBool;

/******************************************************************************
 *
 * Configuration variables - can be overriden at the compile line.
 *
 */
#ifndef SM_TARGET_DIR
//#define SM_TARGET_DIR "genhcsm\\"   ////.NET
#define SM_TARGET_DIR ""
#endif

#ifndef SM_TEMPLATE_TO_CLASS_FILE
#define SM_TEMPLATE_TO_CLASS_FILE "templatetoclass.cxx"
#endif

#ifndef SM_CLASS_HEADER_FILE   
#define SM_CLASS_HEADER_FILE "genhcsm.h"
#endif

#ifndef SM_CLASS_IMPLEMENT_FILE   
#define SM_CLASS_IMPLEMENT_FILE "genhcsm.cxx"
#endif

#ifndef SM_STORAGE_HEADER_FILE   
#define SM_STORAGE_HEADER_FILE "genstorage.h"
#endif

#ifndef SM_STORAGE_IMPLEMENT_FILE   
#define SM_STORAGE_IMPLEMENT_FILE "genstorage.cxx"
#endif

#ifndef SM_COMMUNICATE_HEADER_FILE   
#define SM_COMMUNICATE_HEADER_FILE "gencommunicate.h"
#endif

#ifndef SM_COMMUNICATE_IMPLEMENT_FILE   
#define SM_COMMUNICATE_IMPLEMENT_FILE "gencommunicate.cxx"
#endif

#ifndef SM_GLOBAL_HEADER_FILE   
#define SM_GLOBAL_HEADER_FILE "genhcsmglobal.h"
#endif

#ifndef SM_HCSM_DIAL_MONITOR_INL_FILE
#define SM_HCSM_DIAL_MONITOR_INL_FILE "hcsmdialmonitor.inl"
#endif

#ifndef SM_GEN_BUTTON_DIAL_HEADER_FILE
#define SM_GEN_BUTTON_DIAL_HEADER_FILE "genbuttondialinfo.h"
#endif

#ifndef SM_GEN_BUTTON_DIAL_IMPLEMENT_FILE
#define SM_GEN_BUTTON_DIAL_IMPLEMENT_FILE "genbuttondialinfo.cxx"
#endif

/******************************************************************************
 *
 * Parser data structures.
 *
 */

/*
 * This structure holds a pair of strings which represent a name and a
 * related type/value.
 */
typedef struct TStringPair {
	int             id;        /* positive identifier, CAN'T BE 0           */
	char*           pName;     /* name of string pair                       */
	char*           pValue;    /* holds related type/value					*/
	char*           pCategory; /* holds optional type category				*/
	char*			pDscrptn;  /* hold an optional description 				*/
	struct TStringPair* pNext; /* ptr to next node in list, NULL if none    */
} TStringPair;

typedef struct TDialParam {
	char*           pDialName; /* dial's name                               */
	char*           pName;     /* parameter's name                          */
	char*           pType;     /* parameter's type                          */
	char*           pUnits;    /* parameter's units                         */
	char*           pDefValue; /* parameter's default value                 */
	char*           pComment;  /* parameter's description                   */
	int             optional;  /* 0-->false, 1-->true                       */
	struct TDialParam* pNext;  /* ptr to next node in list, NULL if none    */
} TDialParam;

/*
 * This structure holds information about transitions between a state
 * machine's children.  This structure represents a linked list where
 * each node holds information about one transition.
 */
typedef struct TTransition {
	int             id;        /* positive identifier, CAN'T BE 0           */
	char*           pHeadName; /* name of head state machine                */
	char*           pTailName; /* name of tail state machine                */
	char*           pPredName; /* name of predicate function                */
	TStringPair*    pAttrs;    /* linked list of attributes, NULL if none   */
	struct TTransition* pNext; /* ptr to next node in list, NULL if none    */
} TTransition;

/*
 * This structure holds information about predicate functions that are 
 * attached to transitions which originate from a state machine.  This 
 * structure represents a linked list where each node holds information
 * about one predicate function.
 */
typedef struct TPredicate {
	int             id;        /* identification number of the pred. func.  */
	char*           pName;     /* name of predicate function                */
	struct TPredicate* pNext;  /* ptr to next node in list, NULL if none    */
} TPredicate;

/*
 * This structure represents a linked list where each node holds information
 * about one private user function.
 */
typedef struct TPrivUser {
	/*int             id;*/        /* id number of the priv. user func.tion    */
	/*char*           pName;*/     /* name of priv. user function in SM file   */
	char*           pDec;      /* declaration of priv. user function       */
	/*char*           pType;*/     /* return type of priv. user function       */
	struct TPrivUser* pNext;   /* ptr to next node in list, NULL if none   */
} TPrivUser;

/*
 * This structure represents a linked list where each node holds a string.
 */
typedef struct TStringNode {
	int             id;        /* positive identifier, CAN'T BE 0           */
	char*           pName;     /* dynamic string                            */
	char*			pDscrptn;  /* hold an optional description 				*/
	struct TStringNode* pNext; /* ptr to next item in list, NULL if none    */
} TStringNode;


/*
 * This structure is an aid to detecting cycles in the graph.  It
 * keeps track of visitors to a node during traversals.
 */
typedef struct TVisitor  {
	int             nVisitors; /* how many visitors in 'visitors' array     */
	int*            Visitors;  /* ids of visitors, dynamic memory and size  */
} TVisitor;

/*
 * Holds information about all state machines.
 */
typedef struct TStateMachine {
	int             id;        /* a pos. int. identifier, *** can't be 0    */
	char*           name;      /* sm name                                   */
	EBool           isRoot;    /* identifies sm as a root sm                */
	char*           pre_activity;/* name of actv func, NULL if none         */
	char*           post_activity;/* name of actv func, NULL if none        */
	char*           create_cb; /* name of create callback, NULL if none     */
	char*           delete_cb; /* name of delete callback, NULL if none     */
	TTransition*    trans;     /* trans. linked list, same order as in file */
	TPredicate*     pred;      /* pred linked list, same order as in file   */
	TPrivUser*		priv_user_func; /* priv link list, same order as file   */
	TStringNode*    children;  /* linked list of children, NULL if none     */
	TStringPair*    buttons;   /* linked list of buttons, NULL if none      */
	TStringPair*    inputs;    /* linked list of inputs, NULL if none       */
	TStringPair*    outputs;   /* linked list of outputs, NULL if none      */
	TStringPair*    locals;    /* linked list of locals, NULL if none       */
	TStringPair*    dials;     /* linked list of dials, NULL if none        */
	TDialParam*     dialParams;/* linked list of dial params, NULL if none  */
	TStringPair*    monitors;  /* linked list of monitors, NULL if none     */
	struct TStateMachine* next;/* for linked lists, NULL terminates         */

	/* The following field(s) are strictly for detecting cycles.  No
	 * assumptions about their contents can be made at any other time
	 */
	TVisitor*       vinfo;     /* list of visitors - for detecting cycles   */
} TStateMachine;


/*
 * Holds information about all connections.
 */
typedef struct TConnection {
	int             id;        /* positive identifier, CAN'T BE 0           */
	char*           source_sm; /* name of source state machine              */
	char*           source_var;/* name of source variable                   */
	char*           dest_sm;   /* name of destination state machine         */
	char*           dest_var;  /* name of destination variable              */
	TStringPair*    attr;      /* LL of attributes associated with connec.  */
	struct TConnection* next;  /* for linked lists, NULL terminates         */
} TConnection;

/* 
 * This is the main data structure that holds all parser information.
 * It uses dynamic memory.
 */
typedef struct THcsmParserInfo {
	char*           version;   /* string with version number                */
	char*           header_code;/* code block from header                   */
	TStateMachine*  sm_list;   /* LL of sm, same order as file              */
	TConnection*    conn_list; /* LL of connections, same order as file     */
	TStringPair*    pre_actv_list; /* LL of actv funcs, same order as file  */
	TStringPair*    post_actv_list; /* LL of actv funcs, same order as file */
	TStringPair*    pred_list; /* LL of pred functions, same order as file  */
	TStringPair*    crcb_list; /* LL of create callbask, same order as file */
	TStringPair*    decb_list; /* LL of delete callback, same order as file */
	TStringPair*    glob_list; /* LL of global vars                         */
} THcsmParserInfo;


/******************************************************************************
 *
 * Function prototypes.
 *
 */
void           dump_ds( THcsmParserInfo* ptr );
void           consume_remainder_of_line( void );
char*          safe_strdup( char* pStr );
TStateMachine* NewSm( void );
TTransition*   NewTr( void );
TPredicate*    NewPred( void );
TPrivUser*     NewPriv( void );
TStringPair*   NewStrPair( void );
TDialParam*    NewDialParam( void );
TStringNode*   NewStrNode( void );
TConnection*   NewConn( void );

char*          SearchDupName( TStringNode* pHead );
char*          SearchDupPair( TStringPair* pHead );

TStateMachine* LookupSm( TStateMachine* pHead, char* pName );
int            LookupSmPos( TStringNode* pHead, char* pName );
TStringPair*   LookupStrPair( TStringPair* pHead, char* pName );
TStringNode*   LookupStrNode( TStringNode* pHead, char* pName );
TPredicate*    LookupPred( TPredicate* pPredHead, char* pName );

void           DiscardStrPairList( TStringPair *pHead );

int            CountStm(TStateMachine  *head);
int            CountTrans(TTransition  *head);
int            CountPred( TPredicate* pNode );
int            CountStrNode(TStringNode *head);
int            CountStrPair(TStringPair *head);

void           AppendConnToLinkList(TConnection **ppHead, TConnection *pItem);
void           AppendStrPairToLinkList(TStringPair **ppHead, TStringPair *pItem);
void           AppendStrNodeToLinkList(TStringNode **ppHead, TStringNode *pItem);
void           AddSm(THcsmParserInfo* pPinfo, TStateMachine* pSm);
void           AddTrans(TStateMachine* pSm, TTransition* pTr);
void           AddPred(TStateMachine* pSm, TPredicate* pPredFunc);
void           AddPriv(TStateMachine *pSm, TPrivUser *pTr);

#ifdef __cplusplus
extern "C" {
#endif
int         yylex(void);
#ifdef __cplusplus
}
#endif

void           VerifySemantics( THcsmParserInfo* pPinfo );

int            AssignIdsToStrPairList(TStringPair *head, int start);
int            AssignUniqueIdsToStrPairList(TStringPair  *head, int start, int type);
int            AssignIdsToStrNodeList(TStringNode *head, int start);
int            AssignUniqueIdsToStrNodeList(TStringNode  *head, int start);
int            AssignIdsToTransList(TTransition  *head, int start);

void           GenerateCode( THcsmParserInfo* pPinfo );
