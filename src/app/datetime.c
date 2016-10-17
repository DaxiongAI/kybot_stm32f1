//{{{ include
#include <stdlib.h>
#include "datetime.h"
//}}}

//{{{ define
const char *months1[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug","Sep", "Oct", "Nov", "Dec"};
const char *months2[] = {"01","02","03","04","05","06","07","08","09","10","11","12"};

char date[12];
char year[5];
char month[3];
char day[3];
char time[10];
char hour[3];
char minute[3];
char second[3];
char datetime[20];
//}}}

//{{{ year month day
char *return_year(char *D)
{
        unsigned char i = 0;
        char *p = NULL;
        char Datestr[12];
        strcpy(Datestr, D);
        for(i = 0; i < 5; i++)
                year[i] = '\0';
        // month
        p = (char *)strtok((char *)Datestr, " ");
        // day
        p = (char *)strtok(NULL, " ");
        // year
        p = (char *)strtok(NULL, " ");
        strcat(year, p);
        return year;
}
char *return_month(char *D)
{
        unsigned char i = 0;
        char Datestr[12];
        strcpy(Datestr, D);
        for(i = 0; i < 3; i++)
                month[i] = '\0';
        for(i = 0; 12 > i; i++) {
                if(0 == strncmp(months1[i], Datestr, 3)) {
                        strcat(month, months2[i]);
                        break;
                }
        }
        return month;
}
char *return_day(char *D)
{
        unsigned char i = 0;
        char *p = NULL;
        char Datestr[12];
        char day_tmp[3];
        strcpy(Datestr, D);
        for(i = 0; i < 3; i++)
                day[i] = '\0';
        // month
        p = (char *)strtok((char *)Datestr, " ");
        // day
        p = (char *)strtok(NULL, " ");
        if(strlen(p) == 1){
                day_tmp[0] = '\0';
                day_tmp[1] = p[0];
                strcat(day, day_tmp);
        }
        else
                strcat(day, p);
        return day;
}
//}}}

//{{{ date
char *return_date(char *D, char *spl)
{
        unsigned char i = 0;
        for(i = 0; i < 12; i++)
                date[i] = '\0';
        strcat(date, return_year(D));
        if(spl != NULL)
                strcat(date, spl);
        strcat(date, return_month(D));
        if(spl != NULL)
                strcat(date, spl);
        strcat(date, return_day(D));
        return date;
}
//}}}

//{{{ hour minute second
char *return_hour(char *T)
{
        unsigned char i = 0;
        char Timestr[8];
        strcpy(Timestr, T);
        for(i = 0; i < 3; i++)
                hour[i] = '\0';
        strncat(hour, Timestr, 2);
        return hour;
}
char *return_minute(char *T)
{
        unsigned char i = 0;
        char Timestr[8];
        strcpy(Timestr,T);
        for(i = 0; i < 3; i++)
                minute[i] = '\0';
        strncat(minute, Timestr + 3, 2);
        return minute;
}

char *return_second(char *T)
{
        unsigned char i = 0;
        char Timestr[8];
        strcpy(Timestr, T);
        for(i = 0; i < 3; i++)
                second[i] = '\0';
        strncat(second, Timestr + 6, 2);
        return second;
}
//}}}

//{{{ time
char *return_time(char *T)
{
        unsigned char i = 0;
        char Timestr[8];
        strcpy(Timestr,T);
        for(i=0;i<10;i++)
                time[i] = '\0';
        strncat(time,Timestr,8);
        return time;
}

char *return_datetime(char *D, char *T, char *date_spl, char *datetime_spl)
{
        unsigned char i = 0;
        for(i=0;i<20;i++)
                datetime[i] = '\0';
        strcat(datetime, return_date(D, date_spl));
        if(datetime_spl != NULL)
                strcat(datetime, datetime_spl);
        strcat(datetime, return_time(T));
        return datetime;
}
//}}}
