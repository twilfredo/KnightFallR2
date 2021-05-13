#ifndef MCP9808_H
#define MCP9808_H

#define MCP_DEV_ID 0x04
#define MCP_DEV_ADDR 0x18
#define MCP_DEV_ID_ADDR 0x07
#define MCP_AMB_TEMP_ADDR 0x05 //MSB - LSB

int mcp9808_verify(void);

float mcp9808_read_ambient_temp(void);

#endif
