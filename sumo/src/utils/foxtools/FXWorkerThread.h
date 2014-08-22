/****************************************************************************/
/// @file    FXWorkerThread.h
/// @author  Michael Behrisch
/// @date    2014-07-13
/// @version $Id$
///
//
/****************************************************************************/
// SUMO, Simulation of Urban MObility; see http://sumo-sim.org/
// Copyright (C) 2004-2014 DLR (http://www.dlr.de/) and contributors
/****************************************************************************/
//
//   This file is part of SUMO.
//   SUMO is free software: you can redistribute it and/or modify
//   it under the terms of the GNU General Public License as published by
//   the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
/****************************************************************************/

#ifndef FXWorkerThread_h
#define FXWorkerThread_h


// ===========================================================================
// included modules
// ===========================================================================
#ifdef _MSC_VER
#include <windows_config.h>
#else
#include <config.h>
#endif

#include <list>
#include <vector>
#include <fx.h>
#include <FXThread.h>
#include <utils/common/RandHelper.h>


class FXWorkerThread : public FXThread {

public:
    class Task {
    public:
        virtual void run(FXWorkerThread* context) = 0;
        void setIndex(const int newIndex) {
            myIndex = newIndex;
        }
    private:
        int myIndex;
    };

    class Pool {
    public:
        Pool(int numThreads=0) : myRunningIndex(0), myNumFinished(0) {
            while (numThreads > 0) {
                new FXWorkerThread(*this);
                numThreads--;
            }
        }
        
        virtual ~Pool() {
            for (std::vector<FXWorkerThread*>::iterator it = myWorkers.begin(); it != myWorkers.end(); ++it) {
                delete *it;
            }
        }

        void addWorker(FXWorkerThread* const w) {
            if (myWorkers.empty()) {
                std::cout << "created pool at " << SysUtils::getCurrentMillis() << std::endl;
            }
            myWorkers.push_back(w);
        }
        
        void add(Task* const t) {
            t->setIndex(myRunningIndex++);
            RandHelper::getRandomFrom(myWorkers)->add(t);
        }
        
        void addFinished(Task* const t) {
            myMutex.lock();
            myNumFinished++;
            myFinishedTasks.push_back(t);
            myCondition.signal();
            myMutex.unlock();
        }

        Task* popFinished() {
            Task* result = 0;
            myMutex.lock();
            if (!myFinishedTasks.empty()) {
                result = myFinishedTasks.front();
                myFinishedTasks.pop_front();
            }
            myMutex.unlock();
            return result;
        }

        void waitAllAndClear() {
            myMutex.lock();
            while (myNumFinished < myRunningIndex) {
                myCondition.wait(myMutex);
            }
            if (myRunningIndex > 0) {
                std::cout << "finished waiting for " << myRunningIndex << " tasks at " << SysUtils::getCurrentMillis() << std::endl;
            }
            for (std::list<Task*>::iterator it = myFinishedTasks.begin(); it != myFinishedTasks.end(); ++it) {
                delete *it;
            }
            myFinishedTasks.clear();
            myRunningIndex = 0;
            myNumFinished = 0;
            myMutex.unlock();
        }
        
        int getPending() const {
            return myRunningIndex - myNumFinished;
        }
        
        int size() const {
            return (int)myWorkers.size();
        }
        
        void lock() {
            myPoolMutex.lock();
        }

        void unlock() {
            myPoolMutex.unlock();
        }

    private:
        std::vector<FXWorkerThread*> myWorkers;
        FXMutex myMutex;
        FXMutex myPoolMutex;
        FXCondition myCondition;
        std::list<Task*> myFinishedTasks;
        int myRunningIndex;
        int myNumFinished;
    };

public:
    FXWorkerThread(Pool& pool): FXThread(), myPool(pool), myStopped(false), myCounter(0) {
        pool.addWorker(this);
        start();
    }

    virtual ~FXWorkerThread() {
        stop();
    }

    void add(Task* t) {
        myMutex.lock();
        myTasks.push_back(t);
        myCondition.signal();
        myMutex.unlock();
    }

    FXint run() {
        while (!myStopped) {
            myMutex.lock();
            while (!myStopped && myTasks.empty()) {
                myCondition.wait(myMutex);
            }
            if (myStopped) {
                myMutex.unlock();
                break;
            }
            Task* t = myTasks.front();
            myTasks.pop_front();
            myMutex.unlock();
            t->run(this);
            myCounter++;
            if (myCounter % 1000 == 0) std::cout << (int)this << " ran " << myCounter << " tasks " << std::endl;
            myPool.addFinished(t);
        }
        std::cout << "ran " << myCounter << " tasks " << std::endl;
        return 0;
    }
    
    void stop() {
        myMutex.lock();
        myStopped = true;
        myCondition.signal();
        myMutex.unlock();
        join();
    }

    void poolLock() {
        myPool.lock();
    }

    void poolUnlock() {
        myPool.unlock();
    }

private:
    Pool& myPool;
    FXMutex myMutex;
    FXCondition myCondition;
    std::list<Task*> myTasks;
    bool myStopped;
    int myCounter;
};


#endif
