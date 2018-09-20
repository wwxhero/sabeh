/*****************************************************************************
 *
 * (C) Copyright 1998 by National Advanced Driving Simulator and
 * Simulation Center, the University of Iowa and The University
 * of Iowa. All rights reserved.
 *
 * Version:      $Id: workerthread.h,v 1.2 2018/05/18 22:03:48 IOWA\dheitbri Exp $
 *
 * Author(s):    Omar Ahmad
 *
 * Date:         May, 2018
 *
 * Description:  Interface for the Worker thread class.
 *
 ****************************************************************************/
#include <string>
#pragma once;
using namespace std;

#include "hcsmobject.h"

///////////////////////////////////////////////////////////////////////////////
///\brief
///   baseclass for handling threading
///\par use
///    The threading is started with - Start(), thread is stoped with Stop()
///    The ThreadBase class will run in a loop and call back to threadLoop 
///    every frame. This class is designed to handle the basics of starting 
///    and stoping threads. 
//////////////////////////////////////////////////////////////////////////////
class CThreadImpl;
class CThreadBase{
    friend class CThreadImpl;
public:
    CThreadBase();
    virtual bool Start();
    virtual bool Stop();
    ~CThreadBase();
protected:
    virtual bool ThreadLoop() = 0;
    void Lock();
    void Unlock();
private:
    static void theadMain(CThreadBase *);
    void ConcreteMain();
    std::shared_ptr<CThreadImpl> m_threadImpl;
    volatile bool m_exit;
    volatile bool m_running;
};
class CByteBuffer;
class CBuffer{
public:
    CBuffer(int i):_id(i){};
    virtual CBuffer* clone() = 0;
    virtual CByteBuffer* AsByteBuffer(){return nullptr;};
    virtual void clear(){};
protected:
    int _id;
};
class CByteBuffer: public CBuffer{
public:
    CByteBuffer(int id):CBuffer(id){};
    virtual CBuffer* clone() override;
    virtual CByteBuffer* AsByteBuffer(){return this;};
    std::vector<char>& operator()();
    std::vector<char>& Get();
protected:
    std::vector<char> _data;
};
class CDoubleBufferThread: public CThreadBase{
public:
    typedef std::shared_ptr<CBuffer> TBuffRef;
    bool ThreadLoop() override;
    virtual void BufferLoop();
protected:
    TBuffRef _buffA;
    TBuffRef _buffB;
    TBuffRef _buffC;
    int cnt;
};
////////////////////////////////////////////////////
///\brief For handling read-writer
class CLockImpl;
class CAccumulatorBufferThread: public CThreadBase{
public:
    CAccumulatorBufferThread(){}
    typedef std::shared_ptr<CBuffer> TBuffRef;
    typedef std::vector<TBuffRef> TBufferList;
    bool ThreadLoop() override;
    virtual void BufferLoop(TBuffRef) = 0;
    void GetCurrentBuffer(TBufferList&);
protected:
    typedef std::shared_ptr<CLockImpl> TLockRef;
    TLockRef _lockRImpl;
    TBuffRef _template;
    TBufferList _accumlator;
    int cnt;
};