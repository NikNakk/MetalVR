// Force-included into every OpenHMD source this library compiles, which were written for Linux and are used
// unchanged. What differs on macOS is put right here:
//
// - qsort_r: glibc's takes the comparison's argument last and passes it last; macOS's takes it before the
//   comparison and passes it first. The same call compiles on both and sorts with garbage on the wrong one.
// - thread stacks: glibc gives a thread 8 MB, macOS 512 KB, and the tracker keeps large arrays on the stack.
// - stdio.h, which some of the sources use without including.
//
// And one thing that is not a difference: the tracker reports through printf, some of it once for every LED of
// every camera frame. What it prints goes, a line at a time, to whoever the Rift driver names, with the lines
// that repeat at frame rate left out (ohmd_compat.c).
#pragma once

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

void metalvr_qsort_r_gnu(void *base, size_t count, size_t size, int (*compare)(const void *, const void *, void *),
                         void *arg);
int metalvr_pthread_create_large(pthread_t *thread, const pthread_attr_t *attr, void *(*start)(void *), void *arg);

int metalvr_ohmd_printf(const char *format, ...) __attribute__((format(printf, 1, 2)));
int metalvr_ohmd_puts(const char *text);

#define printf metalvr_ohmd_printf
#define puts metalvr_ohmd_puts
#define qsort_r metalvr_qsort_r_gnu
#define pthread_create metalvr_pthread_create_large
