

#include <cassert>
#include <atomic>
#include <vector>
#include <thread>
#include <iostream>
#include <memory>
#include <mutex>
#include <future>
#include <chrono>

#include "os.hpp"

class NoLock
{
public:
	ALWAYS_INLINE void lock()
	{
	}
	ALWAYS_INLINE void unlock()
	{
	}
private:
};

class MutexLock
{
public:
	ALWAYS_INLINE void lock()
	{
		m.lock();
	}
	ALWAYS_INLINE void unlock()
	{
		m.unlock();
	}
private:
	std::mutex m;
};

class SpinLockTASnomod
{
public:
	ALWAYS_INLINE void lock()
	{
		while (locked.exchange(true) == true)
			CpuRelax();
	}
	ALWAYS_INLINE void unlock()
	{
		locked.store(false);
	}
private:
	alignas(CACHELINE_SIZE) std::atomic_bool locked = { false };
};

class SpinLockTAS
{
public:
	ALWAYS_INLINE void lock()
	{
		while (locked.exchange(true, std::memory_order_acquire) == true)
			CpuRelax();
	}
	ALWAYS_INLINE void unlock()
	{
		locked.store(false, std::memory_order_release);
	}
private:
	alignas(CACHELINE_SIZE) std::atomic_bool locked = { false };
};

class SpinLockTTAS
{
public:
	ALWAYS_INLINE void lock()
	{
		do {
			while (locked.load(std::memory_order_relaxed))
				CpuRelax();
		} while (locked.exchange(true, std::memory_order_acquire) == true);
	}
	ALWAYS_INLINE void unlock()
	{
		locked.store(false, std::memory_order_release);
	}
private:
	alignas(CACHELINE_SIZE) std::atomic_bool locked = { false };
};

class SpinLockAndersonFalseSharing
{
public:
	SpinLockAndersonFalseSharing(size_t maxThreads = std::thread::hardware_concurrency()) :
		LockedFlags(maxThreads)
	{
		for (auto &flag : LockedFlags)
			flag = true;

		LockedFlags[0] = false;
	}

	ALWAYS_INLINE void lock()
	{
		const size_t index = NextFreeIdx.fetch_add(1) % LockedFlags.size();
		auto &flag = LockedFlags[index];

		if (index == 0)
			NextFreeIdx -= LockedFlags.size();

		while (flag)
			CpuRelax();

		flag = true;
	}

	ALWAYS_INLINE void unlock()
	{
		const size_t idx = NextServingIdx.fetch_add(1);

		if (idx >= LockedFlags.size())
			NextServingIdx -= LockedFlags.size();

		LockedFlags[idx%LockedFlags.size()] = false;
	}

private:
	using PaddedFlag = std::atomic_bool;

	std::vector<PaddedFlag> LockedFlags;
	std::atomic_size_t      NextFreeIdx = { 0 };
	std::atomic_size_t      NextServingIdx = { 1 };
};
class SpinLockAnderson
{
public:
	SpinLockAnderson(size_t maxThreads = std::thread::hardware_concurrency()) :
		LockedFlags(maxThreads)
	{
		for (auto &flag : LockedFlags)
			flag.first = true;

		LockedFlags[0].first = false;
	}

	ALWAYS_INLINE void lock()
	{
		const size_t index = NextFreeIdx.fetch_add(1) % LockedFlags.size();
		auto &flag = LockedFlags[index].first;

		if (index == 0)
			NextFreeIdx -= LockedFlags.size();

		while (flag.load(std::memory_order_acquire))
			CpuRelax();

		flag.store(true);
	}

	ALWAYS_INLINE void unlock()
	{
		const size_t idx = NextServingIdx.fetch_add(1);

		if (idx >= LockedFlags.size())
			NextServingIdx -= LockedFlags.size();

		LockedFlags[idx%LockedFlags.size()].first.store(false, std::memory_order_release);
	}

private:
	using PaddedFlag = std::pair<std::atomic_bool,
		uint8_t[CACHELINE_SIZE - sizeof(std::atomic_bool)]>;
	static_assert(sizeof(PaddedFlag) == CACHELINE_SIZE, "");

	alignas(CACHELINE_SIZE) std::vector<PaddedFlag> LockedFlags;
	alignas(CACHELINE_SIZE) std::atomic_size_t      NextFreeIdx = { 0 };
	alignas(CACHELINE_SIZE) std::atomic_size_t      NextServingIdx = { 1 };
};

class AndersonSpinLock
{
public:
	AndersonSpinLock(size_t maxThreads = std::thread::hardware_concurrency()) :
		LockedFlags(maxThreads)
	{
		for (auto &flag : LockedFlags)
			flag.first = true;

		LockedFlags[0].first = false;
	}

	ALWAYS_INLINE void lock()
	{
		const size_t index = NextFreeIdx.fetch_add(1) % LockedFlags.size();
		auto &flag = LockedFlags[index].first;

		// Ensure overflow never happens
		if (index == 0)
			NextFreeIdx -= LockedFlags.size();

		while (flag)
			CpuRelax();

		flag = true;
	}

	ALWAYS_INLINE void unlock()
	{
		const size_t idx = NextServingIdx.fetch_add(1);
		LockedFlags[idx%LockedFlags.size()].first = false;
	}

private:
	using PaddedFlag = std::pair<std::atomic_bool, uint8_t[CACHELINE_SIZE - sizeof(std::atomic_bool)]>;
	static_assert(sizeof(PaddedFlag) == CACHELINE_SIZE, "");

	alignas(CACHELINE_SIZE) std::vector<PaddedFlag> LockedFlags;
	alignas(CACHELINE_SIZE) std::atomic_size_t      NextFreeIdx = { 0 };
	alignas(CACHELINE_SIZE) std::atomic_size_t      NextServingIdx = { 1 };
};

class Node {
public:
	std::atomic_bool* link;
	Node() {
		link = new std::atomic_bool;
	}
	~Node() {
		delete link;
		link = nullptr;
	}

};
class NodeA {
public:
	std::atomic_bool* link;
	NodeA() {
		link = nullptr;
	}
	~NodeA() {
		//delete link; // not my ptr
		link = nullptr;
	}

};
static thread_local Node myNode;
static thread_local NodeA myPred;
class SpinLockCLH
{
public:
	SpinLockCLH()
	{
		tail = reinterpret_cast<uintptr_t>(new bool);
		*(reinterpret_cast<bool*>((uintptr_t)tail)) = false;
		//std::cout << "T:" << reinterpret_cast<bool*>((uintptr_t)tail) << std::endl;
	}
	ALWAYS_INLINE void lock()
	{
		*myNode.link = true;
		myPred.link= reinterpret_cast<std::atomic_bool*>(
			tail.exchange(reinterpret_cast<uintptr_t>(myNode.link))
			);
		while (*myPred.link)
			CpuRelax();
		//std::cout << "lock:" << myNode.link <<" "<< myPred.link << std::endl;
	}

	ALWAYS_INLINE void unlock()
	{
		//std::cout << "unlock:" << myNode.link << " " << myPred.link << std::endl;
		*myNode.link = false;
		myNode.link = myPred.link;
	}

private:

	alignas(CACHELINE_SIZE)std::atomic_uintptr_t tail;
	

};

class McsLock
{
public:
    struct QNode
    {
        std::atomic<QNode *> Next = {nullptr};
        std::atomic_bool     Locked = {false};
    };

public:
    ALWAYS_INLINE void lock(QNode &node)
    {
        node.Next = nullptr;
        node.Locked = true;

        QNode *oldTail = Tail.exchange(&node);

        if (oldTail != nullptr)
        {
            oldTail->Next = &node;

            while (node.Locked == true)
                CpuRelax();
        }
    }

    ALWAYS_INLINE void unlock(QNode &node)
    {
        if (node.Next.load() == nullptr)
        {
            QNode *tailWasMe = &node;
            if (Tail.compare_exchange_strong(tailWasMe, nullptr))
                return;
            
            while (node.Next.load() == nullptr)
                CpuRelax();
        }

        node.Next.load()->Locked = false;
    }

private:
    std::atomic<QNode *> Tail = {nullptr};
};


SpinLockCLH lclh;

template<typename LockType>
struct DATA {
	int c1, c2,count,thnum,iters;
	LockType *lo;
	std::atomic_int *ai;
};

volatile int Gai;

template<typename LockType>
void test1(DATA<LockType>* dt) {

	LockType &lo=*dt->lo;
	std::atomic_int *all_ai=dt->ai;

	//BindThisThreadToCore(dt->thnum);

	//(*all_ai)++;
	//while ((*all_ai) < dt->count)
	//	CpuRelax();

	for (int i = 0; i < dt->iters; i++) {
		lo.lock();

		for (int r = 0; r < 16; r++)
			Gai++;

		lo.unlock();
	}
}

template<typename LockType>
int test(int runs, int iters, int threads) {
	LockType lock;
	std::atomic_int *ai = new(std::atomic_int);
	*ai = 0;

	std::thread th[4];

	auto startTime = std::chrono::high_resolution_clock::now();
	Gai = 0;
	for (int i = 0; i < threads; i++) {
		DATA<LockType>* dt = new DATA<LockType>;
		dt->lo = &lock;
		dt->c1 = 16;
		dt->c2 = 0;
		dt->ai = ai;
		dt->count = threads;
		dt->thnum = i;
		dt->iters = iters/threads;
		th[i] = std::thread(test1<LockType>, dt);
	}
	for (int i = 0; i < threads; i++) {
		th[i].join();
	}
	auto endTime = std::chrono::high_resolution_clock::now();
	//std::cout << Gai << std::endl;

	std::chrono::milliseconds time = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

	return time.count();
}
/*

void testMM1(DATA<McsLock>* dt) {

	McsLock &lo = *dt->lo;
	std::atomic_int *all_ai = dt->ai;

	BindThisThreadToCore(dt->thnum);

	McsLock::QNode node;
	//(*all_ai)++;
	//while ((*all_ai) < dt->count)
	//	CpuRelax();

	volatile std::atomic_size_t cnt = { 0 };
	volatile std::atomic_int ai;
	for (int i = 0; i < dt->iters; i++) {
		lo.lock(node);

		for (size_t l = 0; l<16; l++)
			cnt++;

		lo.unlock(node);

	}
}

int testMM(int runs,int iters, int threads) {
	McsLock lock;
	std::atomic_int *ai = new(std::atomic_int);
	*ai = 0;

	std::thread th[4];

	auto startTime = std::chrono::high_resolution_clock::now();

	for (int i = 0; i < threads; i++) {
		DATA<McsLock>* dt = new DATA<McsLock>;
		dt->lo = &lock;
		dt->c1 = 10;
		dt->c2 = 0;
		dt->ai = ai;
		dt->count = threads;
		dt->thnum = i;
		dt->iters = iters/threads;
		th[i] = std::thread(testMM1, dt);
	}
	for (int i = 0; i < threads; i++) {
		th[i].join();
	}
	auto endTime = std::chrono::high_resolution_clock::now();
	std::chrono::milliseconds time = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

	return time.count();
}
*/

template<typename LockType>
int test(size_t numRuns, size_t numItersPerRun, size_t numThreads)
{
	std::vector<std::chrono::milliseconds> runs(numRuns);
	const size_t numItersPerThread = numItersPerRun / numThreads;
	volatile std::atomic_size_t cnt = { 0 };
	int res = 0;

	for (size_t i = 0; i<numRuns; i++)
	{
		std::vector<std::future<void>> futures(numThreads);
		LockType lock;
		std::atomic_size_t numThreadsReady = { 0 };
		const auto startTime = std::chrono::high_resolution_clock::now();

		for (size_t j = 0; j<numThreads; j++)
		{
			futures[j] = std::async(std::launch::async, [&, j]()
			{
				//BindThisThreadToCore(j);

				// Wait until all threads are ready
				//numThreadsReady++;
				//while (numThreadsReady < numThreads)
				//    CpuRelax();

				// 
				for (size_t k = 0; k<numItersPerThread; k++)
				{
					//McsLock::QNode node;
					//lock.Enter(node);
					lock.lock();
					for (size_t l = 0; l<16; l++)
						cnt++;
					//lock.Leave(node);
					lock.unlock();
				}
			});
		}

		for (auto &f : futures)
			f.wait();

		const auto endTime = std::chrono::high_resolution_clock::now();
		res += (std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime)).count();
	}

	return res;
}



int main() {

	std::thread th[10];



	int ti = 0;;
	for (int tcount = 1; tcount <= 4; tcount++) {
		/**/
		ti = 0;
		ti += test<SpinLockTASnomod>(1,10,tcount);

		std::cout << ti << std::endl;
		//std::cout << "TASnom:" << ti << std::endl;

		ti = 0;
		ti += test<SpinLockTAS>(1,10,tcount);
		std::cout << ti << std::endl;
		//std::cout << "TAS:" << ti << std::endl;

		ti = 0;
		ti += test<SpinLockTTAS>(1,10,tcount);
		std::cout << ti << std::endl;
		//std::cout << "TTAS:" << ti << std::endl;

		ti = 0;
			ti += test<SpinLockAndersonFalseSharing>(1,10,tcount);
		std::cout << ti << std::endl;
		//std::cout << "AndersonFalseSharing:" << ti << std::endl;

		ti = 0;
			ti += test<SpinLockAnderson>(1,10,tcount);
		std::cout << ti << std::endl;
		//std::cout << "Anderson:" << ti << std::endl;

		ti = 0;
			ti += test<MutexLock>(1,10,tcount);
		std::cout << ti << std::endl;
		//std::cout << "Mutex:" << ti << std::endl;

		ti = 0;
			ti += test<SpinLockCLH>(1,10,tcount);
		std::cout << ti << std::endl;
		//std::cout << "CLH:" << ti << std::endl;

		ti = 0;
		for (int i = 0; i< 1; i++) {
			//ti += test<NoLock>(10,10000,tcount);
		}
		std::cout << ti << std::endl;
		//std::cout << "CLH:" << ti << std::endl;
		/*/
		ti = 0;
		for (int i = 0; i < 10; i++) {
			ti += testMM(10,10000,tcount);
		}
		std::cout << ti << std::endl;
		/**/
	}



	
	/*
	ti = 0;
	for (int i = 0; i < 10; i++) {
		ti += test<AndersonSpinLock>(1000, 4);
	}
	std::cout << "AndersonSpinLock:" << ti << std::endl;*/

	for (int i = 0; i < 3; i++) {
		//th[i] = std::thread(foo);
	}
	for (int i = 0; i < 3; i++) {
		//th[i].join();
	}
	system("pause");

}
