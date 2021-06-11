#ifndef SARA_R4_CMDS_H
#define SARA_R4_CMDS_H

static int sarar4_send_AT_command(const struct shell *shell, size_t argc, char **argv);

void sarar4_send_command_usage(void);

#endif