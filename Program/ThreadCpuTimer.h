#ifndef THREAD_CPU_TIMER_H
#define THREAD_CPU_TIMER_H

#include <stdexcept>
#include <string>

#ifdef _WIN32
#include <windows.h>
#else
#include <cerrno>
#include <cstring>
#include <time.h>
#endif

class ThreadCpuTimer
{
public:
#ifdef _WIN32
	using TimePoint = unsigned long long; // 100ns ticks (kernel + user)
#else
	using TimePoint = timespec;
#endif

	static TimePoint now()
	{
#ifdef _WIN32
		FILETIME creationTime;
		FILETIME exitTime;
		FILETIME kernelTime;
		FILETIME userTime;
		if (!GetThreadTimes(GetCurrentThread(), &creationTime, &exitTime, &kernelTime, &userTime))
			throw std::runtime_error("GetThreadTimes failed");
		return fileTimeToTicks(kernelTime) + fileTimeToTicks(userTime);
#else
		TimePoint ts;
		if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) != 0)
			throw std::runtime_error(std::string("clock_gettime(CLOCK_THREAD_CPUTIME_ID) failed: ") + std::strerror(errno));
		return ts;
#endif
	}

	static double elapsedSeconds(const TimePoint& startTime)
	{
#ifdef _WIN32
		const TimePoint current = now();
		return static_cast<double>(current - startTime) * 1e-7;
#else
		const TimePoint current = now();
		const time_t sec = current.tv_sec - startTime.tv_sec;
		const long nsec = current.tv_nsec - startTime.tv_nsec;
		return static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9;
#endif
	}

private:
#ifdef _WIN32
	static unsigned long long fileTimeToTicks(const FILETIME& fileTime)
	{
		ULARGE_INTEGER value;
		value.LowPart = fileTime.dwLowDateTime;
		value.HighPart = fileTime.dwHighDateTime;
		return value.QuadPart;
	}
#endif
};

#endif
