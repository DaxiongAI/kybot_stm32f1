#ifndef __LOG_H__
#define __LOG_H__

#include <stdbool.h>

// level(uppercase) - print string
#define LOG_LEVEL_FUN(F) \
F(DEBUG, DEBUG) \
F(INFO, INFO) \
F(WARN, WARN) \
F(ERROR, ERROR) \
F(FATAL, FATAL)	\
F(AT, AT)	\
F(TERM, TERM)	

enum log_level_e{
#define DEFINE_ENUM(a, b) LOG_LEVEL_##a,
LOG_LEVEL_FUN(DEFINE_ENUM)
#undef DEFINE_ENUM
LOG_LEVEL_NUM          // add 1
};

// #define LOG_LEVEL_DEFAULT      (LOG_LEVEL_WARN)

void log_prefix(bool fixed_content, int the_file_id, const char *func, int line, int level, const char *prefix, const char *fmt, ...);
void qlog(bool fixed_content, int the_file_id, const char *func, int line, int level, const char *fmt, ...);
void qlog_no_prefix(const char *fmt, ...);
uint8_t *get_time(void);

const char *log_level_to_str(int level);

extern uint8_t log_level;
extern uint8_t enable_log_color;

#define log_no_fixed(...)   qlog(false, 0, NULL, 0, 0, __VA_ARGS__)
#define log_level(_level, ...)   qlog(true, this_file_id, __func__, __LINE__, (_level), __VA_ARGS__)

#define log_info(...)   log_level(LOG_LEVEL_INFO, __VA_ARGS__)
#define log_debug(...)  log_level(LOG_LEVEL_DEBUG, __VA_ARGS__)
#define log_warn(...)   log_level(LOG_LEVEL_WARN, __VA_ARGS__)
#define log_error(...)  log_level(LOG_LEVEL_ERROR, __VA_ARGS__)
#define log_fatal(...)  log_level(LOG_LEVEL_FATAL, __VA_ARGS__)
#define log_at(...)   	log_level(LOG_LEVEL_AT, __VA_ARGS__)
#define log_term(...)   qlog(false, this_file_id, __func__, __LINE__, LOG_LEVEL_TERM, __VA_ARGS__)

#define log_none(...)   qlog_no_prefix(__VA_ARGS__)
#endif
