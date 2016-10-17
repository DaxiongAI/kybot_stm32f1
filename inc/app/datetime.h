#ifndef __DATETIME_H__
#define __DATETIME_H__

#include "string.h"

char *return_year(char *D);
char *return_month(char *D);
char *return_day(char *D);
char *return_date(char *D, char *spl);

char *return_hour(char *T);
char *return_minute(char *T);
char *return_second(char *T);
char *return_time(char *T);
char *return_datetime(char *D,char *T, char *date_spl, char *datetime_spl);

#endif
