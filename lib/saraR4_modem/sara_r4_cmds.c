#include <zephyr.h>
#include <shell/shell.h>
#include "sara_r4_cmds.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(SARA_R4_CMDS, LOG_LEVEL_DBG);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_saraR4,
                               SHELL_CMD(send, NULL, "Send AT Command",
                                         sarar4_send_AT_command),
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(saraR4, &sub_saraR4, "Sara-R4 Modem Command Set", NULL);

static int sarar4_send_AT_command(const struct shell *shell, size_t argc, char **argv)
{

    if (argc != 2)
    {
        LOG_ERR("Invalid Arguments (Expected 1 AT Command)");
        sarar4_send_command_usage();
        return -1;
    }

    if (argv[1][0] != 'A' && argv[1][1] != 'T')
    {
        LOG_ERR("Invalid command, must use AT+XX");
        return -1;
    }
    //Basic command parsing done

        LOG_DBG("Hey: %s", log_strdup(argv[1]));
    return 0;
}

void sarar4_send_command_usage(void)
{
    LOG_INF("Usage: saraR4 send AT+XX\n");
}