#ifndef OS_HPP
#define OS_HPP

#include <cassert>
#include <thread>
#include <chrono>
#include <random>

#define WIN     0
#define UNIX    1
#define OS      WIN

#if (OS == WIN)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>

#define ALWAYS_INLINE   __forceinline
#elif (OS == UNIX)
#include <pthread.h>

#define ALWAYS_INLINE   inline __attribute__((__always_inline__))
#endif


ALWAYS_INLINE static void CpuRelax()
{
#if (OS == WIN)
	//_mm_pause();
#elif (OS == UNIX)
	//asm("pause");
#endif
}


ALWAYS_INLINE void BindThisThreadToCore(size_t threadIdx)
{
#if (OS == WIN)
	assert(threadIdx < 64);
	const auto thisThread = GetCurrentThread();
	const auto res0 = SetThreadAffinityMask(thisThread, 1ULL << threadIdx);
	assert(res0 != 0);
	const auto res1 = SetThreadPriority(thisThread, THREAD_PRIORITY_TIME_CRITICAL);
	assert(res1);
#elif (OS == UNIX)
	pthread_t thisThread = pthread_self();
	cpu_set_t cpuSet;
	CPU_ZERO(&cpuSet);
	CPU_SET(threadIdx, &cpuSet);
	const auto res = pthread_setaffinity_np(thisThread, sizeof(cpuSet), &cpuSet);
	if (res != 0)
	{
		std::cout << res << std::endl;
		std::cout << threadIdx << std::endl;
	}
	assert(res == 0);
#endif
}

#define CACHELINE_SIZE 64

#endif