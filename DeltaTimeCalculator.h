#pragma once
#include <chrono>

class DeltaTimeCalculator
{
public:
	DeltaTimeCalculator()
	{
	}
	void startTimer()
	{
		start = std::chrono::high_resolution_clock::now();
	}

	uint32_t getDurationInMs()
	{
		end = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

		start = std::chrono::high_resolution_clock::now();
		
		totalTimeMs += static_cast<uint32_t>(duration.count());

		return static_cast<uint32_t>(duration.count());
	}

	uint32_t getTotalTimeMs() const 
	{
		return totalTimeMs;
	}
private:
	std::chrono::steady_clock::time_point start;
	std::chrono::steady_clock::time_point end;

	uint32_t totalTimeMs{ 0 };
};