#ifndef __FILE_ID_H__
#define __FILE_ID_H__



#define FILE_ID_F(f)     file_id_##f, 
typedef enum{
#include "all_file_list.h"
        file_id_num
}file_id_t;
#undef FILE_ID_F

#define FILE_ID_NAME_DEFINE(_f)         const char file_id_name_##_f[] = #_f
#define this_file_id_name_define()
#define file_id_name_var(_f)            file_id_name_##_f

// extern const char file_id_name_xxx;
#define FILE_ID_F(_f)            extern const char file_id_name_##_f[];
#include "all_file_list.h"
#undef FILE_ID_F

extern const char *g_file_name[];
#define file_id_name(_f)                g_file_name[_f]
#define this_file_name()                file_id_name(this_file_id)

#endif
