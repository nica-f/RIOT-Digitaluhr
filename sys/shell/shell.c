/*
 * Copyright (C) 2009, 2020 Freie Universität Berlin
 * Copyright (C) 2013, INRIA.
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     sys_shell
 * @{
 *
 * @file
 * @brief       Implementation of a very simple command interpreter.
 *              For each command (i.e. "echo"), a handler can be specified.
 *              If the first word of a user-entered command line matches the
 *              name of a handler, the handler will be called with the whole
 *              command line as parameter.
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      René Kijewski <rene.kijewski@fu-berlin.de>
 * @author      Juan Carrano <j.carrano@fu-berlin.de>
 * @author      Hendrik van Essen <hendrik.ve@fu-berlin.de>
 *
 * @}
 */

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>

#include "shell.h"
#include "shell_commands.h"

#define ETX '\x03'  /** ASCII "End-of-Text", or ctrl-C */
#define BS  '\x08'  /** ASCII "Backspace" */
#define DEL '\x7f'  /** ASCII "Delete" */

#ifdef MODULE_SHELL_COMMANDS
    #define MORE_COMMANDS _shell_command_list
#else
    #define MORE_COMMANDS
#endif /* MODULE_SHELL_COMMANDS */

#ifdef MODULE_NEWLIB
    #define flush_if_needed() fflush(stdout)
#else
    #define flush_if_needed()
#endif /* MODULE_NEWLIB */

#ifndef SHELL_NO_ECHO
    #define ECHO_ON 1
#else
    #define ECHO_ON 0
#endif /* SHELL_NO_ECHO */

#ifndef SHELL_NO_PROMPT
    #define PROMPT_ON 1
#else
    #define PROMPT_ON 0
#endif /* SHELL_NO_PROMPT */

#define SQUOTE '\''
#define DQUOTE '"'
#define ESCAPECHAR '\\'
#define BLANK ' '

enum PARSE_STATE {
    PARSE_SPACE,
    PARSE_UNQUOTED,
    PARSE_SINGLEQUOTE,
    PARSE_DOUBLEQUOTE,
    PARSE_ESCAPE_MASK,
    PARSE_UNQUOTED_ESC,
    PARSE_SINGLEQUOTE_ESC,
    PARSE_DOUBLEQUOTE_ESC,
};

static enum PARSE_STATE escape_toggle(enum PARSE_STATE s)
{
    return s ^ PARSE_ESCAPE_MASK;
}

static shell_command_handler_t find_handler(const shell_command_t *command_list, char *command)
{
    const shell_command_t *command_lists[] = {
        command_list,
        MORE_COMMANDS
    };

    /* iterating over command_lists */
    for (unsigned int i = 0; i < ARRAY_SIZE(command_lists); i++) {

        const shell_command_t *entry;

        if ((entry = command_lists[i])) {
            /* iterating over commands in command_lists entry */
            while (entry->name != NULL) {
                if (strcmp(entry->name, command) == 0) {
                    return entry->handler;
                }
                else {
                    entry++;
                }
            }
        }
    }

    return NULL;
}

static void print_help(const shell_command_t *command_list)
{
    printf("%-20s %s\n", "Command", "Description");
    puts("---------------------------------------");

    const shell_command_t *command_lists[] = {
        command_list,
        MORE_COMMANDS
    };

    /* iterating over command_lists */
    for (unsigned int i = 0; i < ARRAY_SIZE(command_lists); i++) {

        const shell_command_t *entry;

        if ((entry = command_lists[i])) {
            /* iterating over commands in command_lists entry */
            while (entry->name != NULL) {
                printf("%-20s %s\n", entry->name, entry->desc);
                entry++;
            }
        }
    }
}

/**
 * Break input line into words, create argv and call the command handler.
 *
 * Words are broken up at spaces. A backslash escaped the character that comes
 * after (meaning if it is taken literally and if it is a space it does not break
 * the word). Spaces can also be protected by quoting with double or single
 * quotes.
 *
 State diagram for the tokenizer:
```
           ┌───[\]────┐   ┌─────["]────┐   ┌───[']─────┐  ┌───[\]────┐
           ↓          │   ↓            │   │           ↓  │          ↓
  ┏━━━━━━━━━━┓      ┏━┷━━━━━┓        ┏━┷━━━┷━┓       ┏━━━━┷━━┓     ┏━━━━━━━━━━┓
  ┃DQUOTE ESC┃      ┃DQUOTE ┠───["]─>┃SPACE  ┃<─[']──┨SQUOTE ┃     ┃SQUOTE ESC┃
  ┗━━━━━━━━┯━┛      ┗━━━━━━┯┛        ┗┯━━━━┯━┛       ┗━┯━━━━━┛     ┗━━━┯━━━━━━┛
           │         ↑     │          │    │           │     ↑(store)  │
           │  (store)│     │   ┌─[\]──┘    └──[*]────┐ │     │         │
           └──[*]──▶┴◀[*]┘   │                     │ └[*]▶┴◀──[*]──┘
                               ↓     ┏━━━━━━━┓       ↓
                               ├◀[\]┨NOQUOTE┃◀─────┼◀─┐
                               │     ┗━━━━━┯━┛(store)↑   │
                               │           │         │   │
                               │           └─[*]─────┘   │
                               │     ┏━━━━━━━━━━━┓       │
                               └───▶┃NOQUOTE ESC┠──[*]──┘
                                     ┗━━━━━━━━━━━┛
```
 */
static void handle_input_line(const shell_command_t *command_list, char *line)
{
    static const char *INCORRECT_QUOTING = "shell: incorrect quoting";

    /* first we need to calculate the number of arguments */
    int argc = 0;
    char *readpos = line;
    char *writepos = readpos;
    enum PARSE_STATE pstate = PARSE_SPACE;

    while (*readpos != '\0') {
        switch (pstate) {
            case PARSE_SPACE:
                if (*readpos != BLANK) {
                    argc++;
                }
                if (*readpos == SQUOTE) {
                    pstate = PARSE_SINGLEQUOTE;
                }
                else if (*readpos == DQUOTE) {
                    pstate = PARSE_DOUBLEQUOTE;
                }
                else if (*readpos == ESCAPECHAR) {
                    pstate = PARSE_UNQUOTED_ESC;
                }
                else if (*readpos != BLANK) {
                    pstate = PARSE_UNQUOTED;
                    break;
                }
                goto parse_end;
                
            case PARSE_UNQUOTED:
                if (*readpos == BLANK) {
                    pstate = PARSE_SPACE;
                    *writepos++ = '\0';
                    goto parse_end;
                }
                else if (*readpos == ESCAPECHAR) {
                    pstate = escape_toggle(pstate);
                    goto parse_end;
                }
                break;

            case PARSE_SINGLEQUOTE:
                if (*readpos == SQUOTE)  {
                    pstate = PARSE_SPACE;
                    *writepos++ = '\0';
                    goto parse_end;
                }
                else if (*readpos == ESCAPECHAR) {
                    pstate = escape_toggle(pstate);
                    goto parse_end;
                }
                break;

            case PARSE_DOUBLEQUOTE:
                if (*readpos == DQUOTE) {
                    pstate = PARSE_SPACE;
                    *writepos++ = '\0';
                    goto parse_end;
                }
                else if (*readpos == ESCAPECHAR) {
                    pstate = escape_toggle(pstate);
                    goto parse_end;
                }
                break;

            default: /* QUOTED state */
                pstate = escape_toggle(pstate);
                break;
        }
        *writepos++ = *readpos;
        parse_end:
        readpos++;
    }
    *writepos = '\0';

    if (pstate != PARSE_SPACE && pstate != PARSE_UNQUOTED) {
        puts(INCORRECT_QUOTING);
        return;
    }

    if (argc == 0) {
        return;
    }

    /* then we fill the argv array */
    int collected;
    char *argv[argc];

    readpos = line;
    for (collected = 0; collected < argc; collected++) {
        argv[collected] = readpos;
        readpos += strlen(readpos) + 1;
    }

    /* then we call the appropriate handler */
    shell_command_handler_t handler = find_handler(command_list, argv[0]);
    if (handler != NULL) {
        handler(argc, argv);
    }
    else {
        if (strcmp("help", argv[0]) == 0) {
            print_help(command_list);
        }
        else {
            printf("shell: command not found: %s\n", argv[0]);
        }
    }
}

static inline void print_prompt(void)
{
    if (PROMPT_ON) {
        putchar('>');
        putchar(' ');
    }

    flush_if_needed();
}

static inline void echo_char(char c)
{
    if (ECHO_ON) {
        putchar(c);
    }
}

static inline void white_tape(void)
{
    if (ECHO_ON) {
        putchar('\b');
        putchar(' ');
        putchar('\b');
    }
}

static inline void new_line(void)
{
    if (ECHO_ON) {
        putchar('\r');
        putchar('\n');
    }
}

/**
 * @brief   Read a single line from standard input into a buffer.
 *
 * In addition to copying characters, this routine echoes the line back to
 * stdout and also supports primitive line editing.
 *
 * If the input line is too long, the input will still be consumed until the end
 * to prevent the next line from containing garbage.
 *
 * We allow Unix (\n), DOS (\r\n), and Mac linebreaks (\r).
 * QEMU transmits only a single '\r' == 13 on hitting enter ("-serial stdio").
 * DOS newlines are handled like hitting enter twice.
 *
 * @param   buf     Buffer where the input will be placed.
 * @param   size    Size of the buffer. The maximum line length will be one less
 *                  than size, to accommodate for the null terminator.
 *                  The minimum buffer size is 1.
 *
 * @return  length of the read line, excluding the terminator, if reading was
 *          successful.
 * @return  EOF, if the end of the input stream was reached.
 * @return  -ENOBUFS if the buffer size was exceeded.
 */
static int readline(char *buf, size_t size)
{
    int curr_pos = 0;
    bool length_exceeded = false;

    assert((size_t) size > 0);

    while (1) {
        assert((size_t) curr_pos < size);

        int c = getchar();

        switch (c) {

            case EOF:
                return EOF;

            case ETX:
                /* Ctrl-C cancels the current line. */
                curr_pos = 0;
                length_exceeded = false;
                /* fall-thru */
            case '\r':
                /* fall-thru */
            case '\n':
                buf[curr_pos] = '\0';

                new_line();

                return (length_exceeded) ? -ENOBUFS : curr_pos;

            /* check for backspace: */
            case BS:    /* 0x08 (BS) for most terminals */
                /* fall-thru */
            case DEL:   /* 0x7f (DEL) when using QEMU */
                if (curr_pos > 0) {
                    curr_pos--;
                    if ((size_t) curr_pos < size) {
                        buf[curr_pos] = '\0';
                        length_exceeded = false;
                    }
                    white_tape();
                }
                break;

            default:
                /* Always consume characters, but do not not always store them */
                if ((size_t) curr_pos < size - 1) {
                    buf[curr_pos++] = c;
                }
                else {
                    length_exceeded = true;
                }
                echo_char(c);
                break;
        }

        flush_if_needed();
    }
}

void shell_run_once(const shell_command_t *shell_commands,
                    char *line_buf, int len)
{
    print_prompt();

    while (1) {
        int res = readline(line_buf, len);

        switch (res) {

            case EOF:
                return;

            case -ENOBUFS:
                puts("shell: maximum line length exceeded");
                break;

            default:
                handle_input_line(shell_commands, line_buf);
                break;
        }

        print_prompt();
    }
}
