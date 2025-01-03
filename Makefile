NRF_DEV_NUM := 52840
LINKER_SCRIPT:= nrf52840_xxaa.ld
OUTPUT_DIR := build

# Searches for every .c and .S file
SRCS = system_nrf52.c printf.c putchar.c $(wildcard *.S)
# Prepends OUTPUT_DIR and appends .o to every entry ind SRCS
OBJS := $(SRCS:%=$(OUTPUT_DIR)/%.o)

TASKFILES = $(wildcard task*.c)
TASKS = $(TASKFILES:.c=)

# Include folders common to all targets
INC_DIRS += \
  ./ \
  ./Include \
  ./CMSIS_5/CMSIS/Core/Include

# Prepends -I to every INC_DIRS entry
INCLUDES = $(INC_DIRS:%=-I%)


OPT = -O3 -g3

CFLAGS = ${INCLUDES}
CFLAGS += $(OPT)
CFLAGS += -DNRF${NRF_DEV_NUM}_XXAA
CFLAGS += -DARM_MATH_CM4
CFLAGS += -DFLOAT_ABI_HARD
CFLAGS += -Wall
CFLAGS += -fno-builtin
CFLAGS += -mthumb
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mabi=aapcs
CFLAGS += -mfloat-abi=hard
CFLAGS += -mfpu=fpv4-sp-d16
CFLAGS += -fsingle-precision-constant
CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections
CFLAGS += -Wa,-adhlns="$@.lst"

CPPFLAGS = ${CFLAGS} -fno-exceptions

ASMFLAGS += -g3
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DFLOAT_ABI_HARD
ASMFLAGS += -DNRF${NRF_DEV_NUM}_XXAA

LDFLAGS += $(OPT)
LDFLAGS += -T$(LINKER_SCRIPT)
LDFLAGS += -mthumb -mabi=aapcs
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker dump unused sections
LDFLAGS += -Wl,--gc-sections,-Map=${OUTPUT_DIR}/build.map
# use newlib in nano version and system call stubs
LDFLAGS += --specs=nosys.specs

ARFLAGS = -rcs

.PHONY: all flash flash% recover

default:
	@echo "You need to specify a task!"


.PRECIOUS: ${OUTPUT_DIR}/build%.hex

task%: ${OUTPUT_DIR}/build%.hex
	@echo "Build successful!"

all: ${TASKS}
	@echo "All build successful!"

recover:
	nrfjprog -f NRF52 --recover

flash:
	@echo "You need to specify a task (e.g. 'make flash2' for task 2)."

flash%: task%
	nrfjprog -f NRF52 --program ${OUTPUT_DIR}/build$*.hex --chiperase --verify
	nrfjprog -f NRF52 --pinresetenable
	nrfjprog -f NRF52 --reset

${OUTPUT_DIR}/%.c.o: ./%.c
	@echo "CC $<"
	arm-none-eabi-gcc ${CFLAGS} -c $< -o $@

${OUTPUT_DIR}/%.S.o: ./%.S
	@echo "CC $<"
	arm-none-eabi-gcc ${CFLAGS} -c $< -o $@

${OUTPUT_DIR}/%.cpp.o: ./%.cpp
	@echo "CC $<"
	arm-none-eabi-c++ ${CPPFLAGS} -c $< -o $@

${OUTPUT_DIR}/build%.elf: $(OBJS) ${OUTPUT_DIR}/task%.c.o
	arm-none-eabi-c++ ${LDFLAGS} $(OBJS) ${OUTPUT_DIR}/task$*.c.o -o $@ ${LIB_FILES}
	@arm-none-eabi-size $@

${OUTPUT_DIR}/build%.hex: ${OUTPUT_DIR}/build%.elf
	@echo "Preparing $@"
	@arm-none-eabi-objcopy -O ihex $< $@
