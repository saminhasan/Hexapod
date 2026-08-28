#include "Hexapod_dev_capi_host.h"
static Hexapod_dev_host_DataMapInfo_T root;
static int initialized = 0;
__declspec( dllexport ) rtwCAPI_ModelMappingInfo *getRootMappingInfo()
{
    if (initialized == 0) {
        initialized = 1;
        Hexapod_dev_host_InitializeDataMapInfo(&(root), "Hexapod_dev");
    }
    return &root.mmi;
}

extern "C" {
rtwCAPI_ModelMappingInfo *mexFunction() {return(getRootMappingInfo());}
}
