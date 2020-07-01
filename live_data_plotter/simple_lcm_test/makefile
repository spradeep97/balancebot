CFLAGS = -ggdb3 -std=c11 -Wall -Wunused-parameter -Wstrict-prototypes -Werror -Wextra -Wshadow
CFLAGS += -fsanitize=signed-integer-overflow
CFLAGS += -Wno-sign-compare -Wno-unused-parameter -Wno-unused-variable
CFLAGS += -fsanitize=address -fsanitize=undefined

message:message.c l2g_t.c
	gcc -o $@ $^ $(CFLAGS) -lm -lpthread -O3 -I$(HOME)/local/include -L$(HOME)/local/lib -llcm
