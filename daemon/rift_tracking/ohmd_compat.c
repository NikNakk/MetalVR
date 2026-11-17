// See ohmd_compat.h.
#include "ohmd_compat.h"

#undef printf
#undef puts
#undef qsort_r
#undef pthread_create

#include <stdarg.h>
#include <string.h>

#define LARGE_STACK (8u << 20)

struct gnu_compare {
  int (*compare)(const void *, const void *, void *);
  void *arg;
};

static int bsd_compare(void *thunk, const void *a, const void *b) {
  const struct gnu_compare *gnu = thunk;
  return gnu->compare(a, b, gnu->arg);
}

void metalvr_qsort_r_gnu(void *base, size_t count, size_t size, int (*compare)(const void *, const void *, void *),
                         void *arg) {
  struct gnu_compare gnu = {compare, arg};
  qsort_r(base, count, size, &gnu, bsd_compare);
}

int metalvr_pthread_create_large(pthread_t *thread, const pthread_attr_t *attr, void *(*start)(void *), void *arg) {
  pthread_attr_t large;
  (void)attr; // OpenHMD passes none
  if (pthread_attr_init(&large) != 0)
    return pthread_create(thread, NULL, start, arg);
  pthread_attr_setstacksize(&large, LARGE_STACK);
  const int ret = pthread_create(thread, &large, start, arg);
  pthread_attr_destroy(&large);
  return ret;
}

/* ---- what the tracker prints ---- */

static void to_stdout(const char *line) {
  fputs(line, stdout);
  fputc('\n', stdout);
}

static void (*log_sink)(const char *line) = to_stdout;

void metalvr_ohmd_set_log(void (*sink)(const char *line)) { log_sink = sink ? sink : to_stdout; }

/* Printed for every LED or candidate pose of a frame: of no use at fifty frames a second */
static int every_frame(const char *line) {
  static const char *const starts[] = {"mismatched LED id", "Error pose candidate", "[II] No blob for device",
                                       "[II] Forcing orientation observation"};
  for (size_t i = 0; i < sizeof(starts) / sizeof(starts[0]); i++)
    if (strncmp(line, starts[i], strlen(starts[i])) == 0)
      return 1;
  return 0;
}

/* A thread's line so far: the tracker builds its lines from several calls */
static _Thread_local char pending[1024];
static _Thread_local size_t pending_length;

static void emit(const char *text) {
  for (const char *c = text; *c; c++) {
    if (*c != '\n') {
      if (pending_length + 1 < sizeof(pending))
        pending[pending_length++] = *c;
      continue;
    }
    pending[pending_length] = 0;
    pending_length = 0;
    if (pending[0] && !every_frame(pending))
      log_sink(pending);
  }
}

int metalvr_ohmd_printf(const char *format, ...) {
  char text[1024];
  va_list args;
  va_start(args, format);
  const int n = vsnprintf(text, sizeof(text), format, args);
  va_end(args);
  emit(text);
  return n;
}

int metalvr_ohmd_puts(const char *text) {
  emit(text);
  emit("\n");
  return 0;
}
