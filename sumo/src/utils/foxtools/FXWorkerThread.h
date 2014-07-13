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
        virtual void run() = 0;
    private:
        int myIndex;
    };

    class Pool {
    public:
        Pool(int numThreads) {
            while (numThreads > 0) {
                myWorkers.push_back(new FXWorkerThread(*this));
                numThreads--;
            }
        }
        
        void add(Task* t) {
            RandHelper::getRandomFrom(myWorkers)->add(t);
        }
        
        void addFinished(Task* t) {
            myMutex.lock();
            myFinishedTasks.push_back(t);
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

    private:
        std::vector<FXWorkerThread*> myWorkers;
        FXMutex myMutex;
        std::list<Task*> myFinishedTasks;
        int myRunningIndex;
    };

public:
    FXWorkerThread(Pool& pool): FXThread(), myPool(pool), myStopped(false) {
        start();
    }

    virtual ~FXWorkerThread() {
        stop();
    }

    void add(Task* t) {
        myMutex.lock();
        myTasks.push_back(t);
        myMutex.unlock();
        myCondition.signal();
    }

    FXint run() {
        while (!myStopped) {
            myMutex.lock();
            while (myTasks.empty()) {
                myCondition.wait(myMutex);
            }
            if (myStopped) {
                myMutex.unlock();
                break;
            }
            Task* t = myTasks.front();
            myTasks.pop_front();
            myMutex.unlock();
            t->run();
            myPool.addFinished(t);
        }
        return 0;
    }
    
    void stop() {
        myStopped = true;
        myCondition.signal();
    }

private:
    Pool& myPool;
    FXMutex myMutex;
    FXCondition myCondition;
    std::list<Task*> myTasks;
    bool myStopped;
};


#endif
