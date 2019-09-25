/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the
 *       distribution.
 *    3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "cmd_util.h"
#include "cjson/cJSON.h"

static cJSON * g_object;

static enum cmd_status cmd_json_parse_exec(char *cmd)
{
    char *out = NULL;
    cJSON *json = NULL;
	char *text = cmd;

	if (g_object) {
		cJSON_Delete(g_object);
		g_object = NULL;
	}

    json = cJSON_Parse(text);
    if (!json) {
        CMD_ERR("[%s]\n", cJSON_GetErrorPtr());
		return CMD_STATUS_FAIL;
    } else {
		g_object = json;
      	out = cJSON_Print(json);
        //cJSON_Delete(json);
		if(!out){
			CMD_ERR("malloc error\n");
			return CMD_STATUS_FAIL;
		}
     	CMD_DBG("%s\n", out);
        cmd_free(out);
    }

	return CMD_STATUS_OK;
}

static enum cmd_status cmd_json_getvalue_exec(char *cmd)
{
	int cnt;
	//char *out = NULL;
	cJSON *json = NULL;
	char string[50];

	cnt = cmd_sscanf(cmd, "s=%s", string);
	if (cnt != 1) {
		CMD_ERR("invalid argument %s\n", cmd);
		return CMD_STATUS_FAIL;
	}

	if (!g_object) {
		CMD_ERR("invalid g_object %s\n", cmd);
		return CMD_STATUS_FAIL;
	}
	json = cJSON_GetObjectItem(g_object, string);
	if (!json) {
		CMD_ERR("get err %s\n", cmd);
		return CMD_STATUS_FAIL;
	}

	CMD_DBG("json->type: %d\n", json->type);

	switch (json->type) {
		case cJSON_String:
			CMD_DBG("value: %s\n", json->valuestring);
			break;
		case cJSON_Number:
			CMD_DBG("value: %d\n", json->valueint);
			break;
		case cJSON_Array:
		case cJSON_Object:
			{
				char *out = cJSON_Print(json);
				if(!out){
					CMD_ERR("malloc error\n");
					return CMD_STATUS_FAIL;
				}
				CMD_DBG("value: %s\n", out);
				cmd_free(out);
				break;
			}
		default:
			CMD_ERR("invalid type %s\n", cmd);
			return CMD_STATUS_FAIL;
	}

	return CMD_STATUS_OK;
}

int cjson_test(void);
static enum cmd_status cmd_json_test_exec(char *cmd)
{
	cjson_test();

	return CMD_STATUS_OK;
}

static const struct cmd_data g_json_cmds[] = {
	{ "parse",		cmd_json_parse_exec },
	{ "getValue",	cmd_json_getvalue_exec },
	{ "test",		cmd_json_test_exec },
};

enum cmd_status cmd_json_exec(char *cmd)
{
	return cmd_exec(cmd, g_json_cmds, cmd_nitems(g_json_cmds));
}
