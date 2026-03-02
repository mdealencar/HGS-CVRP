#ifndef LOGGER_H
#define LOGGER_H

#include <ostream>

std::ostream &hgs_log_stream();

extern "C" {
void hgs_set_output_stdout();
void hgs_set_output_stderr();
int hgs_set_output_file(const char *filePath);
}

#endif
