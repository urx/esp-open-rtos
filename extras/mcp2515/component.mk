# Component makefile for extras/mcp2515

# expected anyone using this driver includes it as 'mcp2515/mcp2515.h'
INC_DIRS += $(mcp2515_ROOT)..

# args for passing into compile rule generation
mcp2515_SRC_DIR = $(mcp2515_ROOT)

$(eval $(call component_compile_rules,mcp2515))