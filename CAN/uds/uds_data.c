//
// Created by kuro on 2018/11/29.
//

#include "data_uds.h"

static UdsInfo g_udsInfo;

UdsInfo* data_usdinfo_get_usdinfo()
{
    return &g_udsInfo;
}