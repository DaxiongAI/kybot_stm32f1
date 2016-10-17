//{{{ include
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "file_id.h"

#define this_file_id   file_id_file_id
//}}}

//{{{ define
#define FILE_ID_F(_f)            FILE_ID_NAME_DEFINE(_f);
#include "all_file_list.h"
#undef FILE_ID_F



#define FILE_ID_F(_f)            file_id_name_##_f,
const char *g_file_name[] = {
#include "all_file_list.h"
};
#undef FILE_ID_F
//}}}
