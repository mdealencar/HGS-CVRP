#include "Logger.h"

#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>

namespace {
std::mutex g_logMutex;
std::ostream *g_stream = &std::cout;
std::unique_ptr<std::ofstream> g_ownedFile;
}

std::ostream &hgs_log_stream()
{
	std::lock_guard<std::mutex> lock(g_logMutex);
	return *g_stream;
}

extern "C" void hgs_set_output_stdout()
{
	std::lock_guard<std::mutex> lock(g_logMutex);
	g_ownedFile.reset();
	g_stream = &std::cout;
}

extern "C" void hgs_set_output_stderr()
{
	std::lock_guard<std::mutex> lock(g_logMutex);
	g_ownedFile.reset();
	g_stream = &std::cerr;
}

extern "C" int hgs_set_output_file(const char *filePath)
{
	if (filePath == nullptr) return 1;

	std::lock_guard<std::mutex> lock(g_logMutex);
	auto file = std::make_unique<std::ofstream>(filePath, std::ios::out | std::ios::trunc);
	if (!file->is_open()) return 1;
	g_stream = file.get();
	g_ownedFile = std::move(file);
	return 0;
}
