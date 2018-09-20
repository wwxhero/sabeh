#include "workerthread.h"


#ifdef _WIN32
#include <process.h>
class CThreadImpl{
public:
    CThreadImpl(CThreadBase *target): _target(target),handle(0){
        InitializeCriticalSectionAndSpinCount(&_criticalSection,4000); 
    }
    static void ThreadFunct(void *thr){
        auto base = (CThreadImpl*)(thr);
        std::shared_ptr<CThreadImpl> self;
        self = base->_target->m_threadImpl;
        base->_target->theadMain(base->_target);
        base->handle=0;
    }
    bool Start(){
        handle = ::_beginthread(ThreadFunct,0,this);
        if (handle > 0)
            return true;
        return false;
    }
    ~CThreadImpl(){
        if (_target && _target->m_threadImpl.use_count()>1){
            _endthreadex(handle);    
        }
        DeleteCriticalSection(&_criticalSection);
        
    }
    void Lock(){
        EnterCriticalSection(&_criticalSection);
    }
    void Unlock(){
        LeaveCriticalSection(&_criticalSection);
    }
    CRITICAL_SECTION _criticalSection; 
    CThreadBase* _target;
    uintptr_t handle;
};

#endif
CThreadBase::CThreadBase():m_exit(false),m_running(false){
    m_threadImpl = std::unique_ptr<CThreadImpl>(new CThreadImpl(this));
}
bool CThreadBase::Start(){
    m_exit = false;
    return m_threadImpl->Start();
}
bool CThreadBase::Stop(){
    m_exit = true;
    int cnt = 0;
    while(m_running && cnt < 200){
        cnt++;
        Sleep(0);
    }
    return true;
}
void CThreadBase::Lock(){
    if (m_threadImpl.get())
        m_threadImpl->Lock();
}
void CThreadBase::Unlock(){
    if (m_threadImpl.get())
        m_threadImpl->Unlock();
}
CThreadBase::~CThreadBase(){
	int cnt = 0;
	while (m_exit && m_running) {
		if (cnt > 200) break;
		Sleep(0);
        cnt++;
	}
    m_threadImpl->_target = nullptr;
    m_threadImpl.reset();
    
}


void CThreadBase::theadMain(CThreadBase * myInst){
    myInst->ConcreteMain();
}
void CThreadBase::ConcreteMain(){
    m_running = true;
    while (true){
        ThreadLoop();
        if (m_exit){
            break;
        }
    }
    m_running = false;
}

CBuffer* CByteBuffer::clone(){
    CByteBuffer* ret = new CByteBuffer(_id);
    ret->_data = _data;
    return ret;
}
std::vector<char>& CByteBuffer::operator()(){
    return _data;
}

std::vector<char>& CByteBuffer::Get(){
    return _data;    
}
bool CAccumulatorBufferThread::ThreadLoop(){
    TBuffRef element  = TBuffRef(_template->clone());
    BufferLoop(element);
    Lock();{
        _accumlator.push_back(element);
    }Unlock();
    return true;
}
void CAccumulatorBufferThread::GetCurrentBuffer(TBufferList& input){
    Lock();{
        input = std::move(_accumlator);
    }Unlock();
}