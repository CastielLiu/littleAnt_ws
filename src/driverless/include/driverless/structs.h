#ifndef STRUCTS_H_
#define STRUCTS_H_

#include <atomic>
#include <boost/thread/locks.hpp>    
#include <boost/thread/shared_mutex.hpp>    
#include <mutex>

typedef boost::shared_mutex SharedMutex;
typedef boost::unique_lock<SharedMutex> WriteLock;
typedef boost::shared_lock<SharedMutex> ReadLock;




#endif
