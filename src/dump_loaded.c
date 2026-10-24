#include <ctype.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "detect_disc.h"
#include "disc_dump.h"
#include "disc_serial.h"
#include "dump_loaded.h"
#include "notify.h"
#include "redump.h"

#if defined(__ORBIS__) && defined(__PROSPERO__)
#error "Define only one of __ORBIS__ or __PROSPERO__"
#elif defined(__ORBIS__)
#include <ps4/klog.h>
#elif defined(__PROSPERO__)
#include <ps5/klog.h>
#else
#error "Build for PS4 (__ORBIS__) or PS5 (__PROSPERO__)"
#endif

// Defined in disc_dump.c. Why the last dump failed, or an empty string.
const char *dump_disc_error(void);

// Also from disc_dump.c. Unreadable CD sectors the last dump wrote as zeros, with the first LBA, last LBA and number of runs.
uint32_t dump_disc_bad_sectors(uint32_t *first, uint32_t *last, uint32_t *runs);

// The cue this payload writes fits well under this. A larger sheet is left unchanged.
#define CUE_REWRITE_MAX 32768u

// Saturn images go to the PS5 RetroArch title's content folder for Saturn, which RetroArch sees as /app0/content/saturn.
// Every other disc stays in /data.
#define DUMP_DIR_DEFAULT "/data"
#define DUMP_DIR_SATURN "/data/homebrew/PPSA99169/content/saturn"

// Where the current dump's files are read and written. dump_loaded() sets it before anything else.
static const char *dump_dir = DUMP_DIR_DEFAULT;

struct dump_log {
  const char *label;
  unsigned last_pct;
  uint64_t last_mark;
  uint64_t done;
  uint64_t start_ms;
  uint64_t sample_ms;
  uint64_t sample_done;
};

static int serial_filename(const char *serial) {
  const unsigned char *p;

  if (!serial || serial[0] == '\0') {
    return 0;
  }
  for (p = (const unsigned char *)serial; *p != '\0'; p++) {
    if (!isalnum(*p) && *p != '-' && *p != '_') {
      return 0;
    }
  }
  return 1;
}

// GENERIC_PS1 and the other unknown-title ids are not a Redump serial.
static int generic_serial(const char *serial) {
  return serial && strncmp(serial, "GENERIC_", 8) == 0;
}

// The game title in the Saturn header (112 bytes at 0x60 of LBA 0, space padded) as a file name: "<title> (<serial>)".
// The serial is kept in the name because the discs of one game can share a title.
static int saturn_title_name(char *dst, size_t dst_len, const char *serial) {
  uint8_t header[2048];
  char title[113];
  size_t n = 0;
  size_t i;
  int space = 1;

  if (!disc_saturn_header(header)) {
    return 0;
  }
  for (i = 0; i < 112; i++) {
    unsigned char c = header[0x60 + i];

    // Anything that is not plain ASCII, or that a file name cannot hold, becomes a space.
    if (c < 0x20 || c > 0x7E || strchr("/\\\":*?<>|", c) != NULL) {
      c = ' ';
    }
    if (c == ' ') {
      if (!space) {
        title[n++] = ' ';
      }
      space = 1;
    } else {
      title[n++] = (char)c;
      space = 0;
    }
  }
  while (n > 0 && (title[n - 1] == ' ' || title[n - 1] == '.')) {
    n--;
  }
  title[n] = '\0';
  if (n == 0) {
    return 0;
  }
  return snprintf(dst, dst_len, "%s (%s)", title, serial) < (int)dst_len;
}

// Redump stems may contain spaces and parentheses. Reject a name that cannot be one path component.
static int dump_name_ok(const char *name, size_t path_len) {
  const unsigned char *p;
  size_t n = 0;

  if (!name || name[0] == '\0' || name[0] == '.') {
    return 0;
  }
  for (p = (const unsigned char *)name; *p != '\0'; p++) {
    if (*p < 0x20 || *p == '/' || *p == '\\' || *p == '"') {
      return 0;
    }
    n++;
  }
  // dump_dir, "/", name, ".cue" and the terminating NUL.
  return strlen(dump_dir) + n + 6 <= path_len;
}

static uint64_t mono_ms(void) {
  struct timespec ts;

  if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
    return 0;
  }
  return (uint64_t)ts.tv_sec * 1000ull + (uint64_t)ts.tv_nsec / 1000000ull;
}

static void format_span(char *dst, size_t n, uint64_t ms) {
  unsigned sec = (unsigned)(ms / 1000ull);
  unsigned min = sec / 60u;
  unsigned hr = min / 60u;

  sec %= 60u;
  min %= 60u;
  if (hr > 0) {
    snprintf(dst, n, "%u:%02u:%02u", hr, min, sec);
  } else {
    snprintf(dst, n, "%u:%02u", min, sec);
  }
}

static void format_bytes(char *dst, size_t n, uint64_t bytes) {
  const uint64_t mib = 1024ull * 1024ull;
  const uint64_t gib = mib * 1024ull;

  if (bytes >= gib) {
    uint64_t tenths = (bytes * 10ull) / gib;
    snprintf(dst, n, "%u.%u GiB", (unsigned)(tenths / 10ull), (unsigned)(tenths % 10ull));
  } else if (bytes >= mib) {
    uint64_t tenths = (bytes * 10ull) / mib;
    snprintf(dst, n, "%u.%u MiB", (unsigned)(tenths / 10ull), (unsigned)(tenths % 10ull));
  } else if (bytes >= 1024ull) {
    snprintf(dst, n, "%u KiB", (unsigned)(bytes / 1024ull));
  } else {
    snprintf(dst, n, "%u B", (unsigned)bytes);
  }
}

static void format_speed(char *dst, size_t n, uint64_t bytes, uint64_t ms) {
  uint64_t per_sec;
  uint64_t tenths;

  if (ms == 0) {
    ms = 1;
  }
  per_sec = (bytes * 1000ull) / ms;
  tenths = (per_sec * 10ull) / (1024ull * 1024ull);
  snprintf(dst, n, "%u.%u MiB/s", (unsigned)(tenths / 10ull), (unsigned)(tenths % 10ull));
}

static void log_dump_progress(struct dump_log *log, uint64_t done, const char *mark) {
  char span[16];
  char speed[24];
  uint64_t now;
  uint64_t elapsed;
  uint64_t dt;
  uint64_t delta;

  now = mono_ms();
  elapsed = now >= log->start_ms ? now - log->start_ms : 0;
  dt = now >= log->sample_ms ? now - log->sample_ms : 0;
  delta = done >= log->sample_done ? done - log->sample_done : 0;
  format_span(span, sizeof(span), elapsed);
  format_speed(speed, sizeof(speed), delta, dt);
  log->sample_ms = now;
  log->sample_done = done;
  klog_printf("[Dump] %s %s %s %s\n", log->label, mark, span, speed);
}

static void on_dump_progress(uint64_t done, uint64_t total, void *user) {
  struct dump_log *log = user;
  char mark[32];

  log->done = done;
  if (total > 0) {
    unsigned pct = (unsigned)((done * 100ull) / total);

    if (pct > 100u) {
      pct = 100u;
    }
    if (pct == log->last_pct) {
      return;
    }
    log->last_pct = pct;
    snprintf(mark, sizeof(mark), "%u%%", pct);
    log_dump_progress(log, done, mark);
    return;
  }
  if (done < log->last_mark + ((uint64_t)64 << 20)) {
    return;
  }
  log->last_mark = done;
  snprintf(mark, sizeof(mark), "%u MiB", (unsigned)(done / (1024u * 1024u)));
  log_dump_progress(log, done, mark);
}

static void log_dump_result(const struct dump_log *log, int ok) {
  char span[16];
  char size[24];
  char speed[24];
  uint64_t now;
  uint64_t elapsed;

  now = mono_ms();
  elapsed = now >= log->start_ms ? now - log->start_ms : 0;
  format_span(span, sizeof(span), elapsed);
  format_bytes(size, sizeof(size), log->done);
  format_speed(speed, sizeof(speed), log->done, elapsed);
  klog_printf("[Dump] %s %s %s in %s, average %s\n", log->label, ok ? "done" : "failed", size, span, speed);
}

static int write_bytes(const char *path, const void *data, size_t n) {
  FILE *fp = fopen(path, "wb");
  size_t wrote;

  if (!fp) {
    return -1;
  }
  wrote = fwrite(data, 1, n, fp);
  if (fclose(fp) != 0 || wrote != n) {
    return -1;
  }
  return 0;
}

// The cue's first line names the bin. A renamed dump has to keep that name in step with the file.
static int rewrite_cue_bin(const char *cue_path, const char *bin_name) {
  FILE *fp;
  char *old;
  char *neu;
  char extra;
  char *quote;
  char *nl;
  size_t n;
  size_t tail;
  int head;
  int rc = -1;

  old = malloc(CUE_REWRITE_MAX);
  neu = malloc(CUE_REWRITE_MAX);
  if (!old || !neu) {
    free(old);
    free(neu);
    return -1;
  }
  fp = fopen(cue_path, "rb");
  if (!fp) {
    goto out;
  }
  n = fread(old, 1, CUE_REWRITE_MAX - 1, fp);
  if (n == 0 || ferror(fp) || fread(&extra, 1, 1, fp) == 1) {
    fclose(fp);
    goto out;
  }
  fclose(fp);
  old[n] = '\0';
  if (n < 6 || memcmp(old, "FILE \"", 6) != 0) {
    goto out;
  }
  quote = strchr(old + 6, '"');
  nl = strchr(old, '\n');
  if (!quote || !nl || quote > nl) {
    goto out;
  }
  head = snprintf(neu, CUE_REWRITE_MAX, "FILE \"%s\" BINARY\n", bin_name);
  if (head < 0 || (size_t)head >= CUE_REWRITE_MAX) {
    goto out;
  }
  tail = n - (size_t)(nl + 1 - old);
  if ((size_t)head + tail >= CUE_REWRITE_MAX) {
    goto out;
  }
  memcpy(neu + head, nl + 1, tail);
  if (write_bytes(cue_path, neu, (size_t)head + tail) != 0) {
    write_bytes(cue_path, old, n);
    goto out;
  }
  rc = 0;
out:
  free(old);
  free(neu);
  return rc;
}

// mkdir -p. A directory that already exists is not an error.
static int ensure_dir(const char *path) {
  char tmp[512];
  size_t n = strlen(path);
  size_t i;

  if (n == 0 || n >= sizeof(tmp)) {
    return -1;
  }
  memcpy(tmp, path, n + 1);
  for (i = 1; i <= n; i++) {
    if (tmp[i] == '/' || tmp[i] == '\0') {
      char saved = tmp[i];

      tmp[i] = '\0';
      if (mkdir(tmp, 0777) == 0) {
        // Made here. Open it to other users on the console, so the emulator that reads the images can list it.
        (void)chmod(tmp, 0777);
      } else {
        struct stat st;

        if (stat(tmp, &st) != 0 || !S_ISDIR(st.st_mode)) {
          return -1;
        }
      }
      tmp[i] = saved;
    }
  }
  return 0;
}

static int data_file(char *dst, size_t dst_len, const char *stem, const char *ext) {
  int n = snprintf(dst, dst_len, "%s/%s%s", dump_dir, stem, ext);

  if (n < 0 || (size_t)n >= dst_len) {
    return -1;
  }
  return 0;
}

// size 0 accepts any non-empty regular file. A positive size must match.
static int file_is_image(const char *path, uint64_t size) {
  struct stat st;

  if (stat(path, &st) != 0 || !S_ISREG(st.st_mode) || st.st_size <= 0) {
    return 0;
  }
  if (size == 0) {
    return 1;
  }
  return (uint64_t)st.st_size == size;
}

// A finished CD dump is the bin plus its cue. DVD and BD are the iso.
// A shorter bin or iso is a partial dump and is not kept.
static int stem_dumped(const char *stem, int cd, uint64_t size) {
  char path[512];

  if (!dump_name_ok(stem, sizeof(path))) {
    return 0;
  }
  if (data_file(path, sizeof(path), stem, cd ? ".bin" : ".iso") != 0 || !file_is_image(path, size)) {
    return 0;
  }
  if (!cd) {
    return 1;
  }
  if (data_file(path, sizeof(path), stem, ".cue") != 0) {
    return 0;
  }
  return file_is_image(path, 0);
}

static void log_already(int cd, const char *stem) {
  klog_printf("[Dump] %s/%s.%s already dumped\n", dump_dir, stem, cd ? "bin" : "iso");
  notify("Already dumped: %.120s", stem);
}

struct dumped_name {
  int cd;
  uint64_t size;
  char stem[512];
};

static int visit_dumped(const char *name, void *user) {
  struct dumped_name *hit = user;
  size_t n;

  if (!stem_dumped(name, hit->cd, hit->size)) {
    return 0;
  }
  n = strlen(name);
  if (n + 1 > sizeof(hit->stem)) {
    return 0;
  }
  memcpy(hit->stem, name, n + 1);
  return 1;
}

static int rename_dump_files(int cd, const char *from_stem, const char *to_stem) {
  char from[512];
  char to[512];
  char bin_from[512];
  char bin_to[512];
  char bin_name[520];
  int n;

  if (strcmp(from_stem, to_stem) == 0) {
    return 0;
  }
  if (!cd) {
    if (data_file(from, sizeof(from), from_stem, ".iso") != 0 || data_file(to, sizeof(to), to_stem, ".iso") != 0) {
      return -1;
    }
    return rename(from, to);
  }
  if (data_file(from, sizeof(from), from_stem, ".cue") != 0 || data_file(to, sizeof(to), to_stem, ".cue") != 0 ||
      data_file(bin_from, sizeof(bin_from), from_stem, ".bin") != 0 || data_file(bin_to, sizeof(bin_to), to_stem, ".bin") != 0) {
    return -1;
  }
  n = snprintf(bin_name, sizeof(bin_name), "%s.bin", to_stem);
  if (n < 0 || (size_t)n >= sizeof(bin_name) || rewrite_cue_bin(from, bin_name) != 0) {
    return -1;
  }
  if (rename(bin_from, bin_to) != 0) {
    snprintf(bin_name, sizeof(bin_name), "%s.bin", from_stem);
    rewrite_cue_bin(from, bin_name);
    return -1;
  }
  if (rename(from, to) != 0) {
    rename(bin_to, bin_from);
    snprintf(bin_name, sizeof(bin_name), "%s.bin", from_stem);
    rewrite_cue_bin(from, bin_name);
    return -1;
  }
  return 0;
}

static void log_dat_result(int cd, int keep_name, const char *serial, const char *name, const char *image_path, const struct disc_hash *hash) {
  char matched[512];
  const char *final_name = name;
  const char *ext = cd ? "bin" : "iso";

  matched[0] = '\0';
  if (redump_verify(serial, image_path, hash, cd ? 0 : 1, matched, sizeof(matched)) && dump_name_ok(matched, sizeof(matched))) {
    if (strcmp(matched, name) != 0 && rename_dump_files(cd, name, matched) == 0) {
      final_name = matched;
    }
    klog_printf("[Redump] %s/%s.%s verified\n", dump_dir, final_name, ext);
    return;
  }
  // An image that does not match Redump goes back to the serial, except where the game name is wanted (Saturn).
  if (!keep_name && strcmp(name, serial) != 0 && rename_dump_files(cd, name, serial) == 0) {
    final_name = serial;
  }
  klog_printf("[Redump] %s/%s.%s not in Redump dat\n", dump_dir, final_name, ext);
}

static void dump_log_reset(struct dump_log *log, const char *label) {
  log->label = label;
  log->last_pct = 0;
  log->last_mark = 0;
  log->done = 0;
  log->start_ms = mono_ms();
  log->sample_ms = log->start_ms;
  log->sample_done = 0;
}

// The screen messages for the end of a dump. The name is the one the files were written under.
static void notify_dump_result(int ok, const char *serial, const char *name) {
  uint32_t bad_first = 0;
  uint32_t bad_last = 0;
  uint32_t bad_runs = 0;
  uint32_t bad = ok ? dump_disc_bad_sectors(&bad_first, &bad_last, &bad_runs) : 0;

  if (!ok) {
    const char *why = dump_disc_error();

    klog_printf("[Dump] failed: %s\n", why[0] != '\0' ? why : "no reason recorded");
    if (why[0] != '\0') {
      notify("Dump failed: %s\n%.180s", serial, why);
    } else {
      notify("Dump failed: %s", serial);
    }
  } else if (bad > 0) {
    klog_printf("[Dump] %u unreadable sectors in %u ranges (LBA %u to %u) were written as zeros\n", (unsigned)bad, (unsigned)bad_runs, (unsigned)bad_first, (unsigned)bad_last);
    notify("Dump complete with %u unreadable sectors: %s\n%u ranges, LBA %u to %u, written as zeros", (unsigned)bad, serial, (unsigned)bad_runs, (unsigned)bad_first, (unsigned)bad_last);
  } else if (strcmp(name, serial) == 0) {
    notify("Dump complete: %s", serial);
  } else {
    notify("Dump complete: %s\n%.120s", serial, name);
  }
}

void dump_loaded(enum disc_type type, const char *serial) {
  char image[512];
  char stem[512];
  const char *name = serial;
  struct dump_log log;
  struct disc_hash hash;
  struct dumped_name hit;
  uint64_t image_bytes = 0;
  int cd = type == DISC_PS1_CD_ROM || type == DISC_PS2_CD_ROM || type == DISC_SS_CD_ROM;
  int ps3 = type == DISC_PS3_BD_ROM || type == DISC_PS3_DVD_ROM;
  int rc;

  if (!cd && type != DISC_PS2_DVD_ROM && !ps3) {
    return;
  }
  dump_dir = type == DISC_SS_CD_ROM ? DUMP_DIR_SATURN : DUMP_DIR_DEFAULT;
  if (!serial_filename(serial)) {
    klog_printf("[Dump] unusable title id\n");
    notify("Cannot dump: unusable title id");
    return;
  }
  // GENERIC_* is not a Redump serial. The dump starts under that name, and a checksum match can rename it.
  if (!generic_serial(serial)) {
    redump_name(serial, stem, sizeof(stem));
    if (dump_name_ok(stem, sizeof(image))) {
      name = stem;
    }
  }
  // A Saturn disc that Redump has no single name for is named from its own header.
  if (type == DISC_SS_CD_ROM && name == serial && saturn_title_name(stem, sizeof(stem), serial) && dump_name_ok(stem, sizeof(image))) {
    name = stem;
  }
  if (data_file(image, sizeof(image), name, cd ? ".bin" : ".iso") != 0) {
    return;
  }

  // The drive size tells a finished image from a partial one, and one pressing
  // of a shared serial from another. An unknown size still protects a file
  // this dump would truncate.
  if (disc_image_bytes(type, &image_bytes) != 0) {
    image_bytes = 0;
  }
  if (stem_dumped(name, cd, image_bytes)) {
    log_already(cd, name);
    return;
  }
  if (strcmp(name, serial) != 0 && stem_dumped(serial, cd, image_bytes)) {
    log_already(cd, serial);
    return;
  }
  hit.cd = cd;
  hit.size = image_bytes;
  hit.stem[0] = '\0';
  if (image_bytes > 0 && !generic_serial(serial) && redump_each_name(serial, visit_dumped, &hit)) {
    log_already(cd, hit.stem);
    return;
  }
  if (image_bytes > 0 && generic_serial(serial)) {
    char sole[512];

    redump_name_for_size(image_bytes, cd ? 0 : 1, sole, sizeof(sole));
    if (stem_dumped(sole, cd, image_bytes)) {
      log_already(cd, sole);
      return;
    }
  }

  if (strcmp(name, serial) == 0) {
    notify("Dumping disc %s", serial);
  } else {
    notify("Dumping disc %s\n%.120s", serial, name);
  }
  if (ensure_dir(dump_dir) != 0) {
    klog_printf("[Dump] could not create %s\n", dump_dir);
    notify("Cannot create %s", dump_dir);
    return;
  }

  memset(&hash, '\0', sizeof(hash));
  dump_log_reset(&log, image);
  if (cd) {
    klog_printf("[Dump] %s/%s.bin %s/%s.cue\n", dump_dir, name, dump_dir, name);
    rc = dump_disc(type, image, &hash, on_dump_progress, &log);
  } else if (!ps3) {
    klog_printf("[Dump] %s\n", image);
    rc = dump_disc(type, image, &hash, on_dump_progress, &log);
  } else {
    klog_printf("[Dump] %s\n", image);
    // IRD creation is commented out until the disc key can be read.
    // The drive has no disc-key ioctl, so Data1 and Data2 would stay zero.
    // struct disc_ird *ird = NULL;
    // char ird_path[512];
    // int n;
    // rc = dump_ps3(image, &ird, &hash, on_dump_progress, &log);
    rc = dump_ps3(image, NULL, &hash, on_dump_progress, &log);
    log_dump_result(&log, rc == 0);
    notify_dump_result(rc == 0, serial, name);
    if (rc == 0) {
      log_dat_result(cd, type == DISC_SS_CD_ROM, serial, name, image, &hash);
    }
    // if (rc == 0 && ird) {
    //   n = snprintf(ird_path, sizeof(ird_path), "/data/%s.ird", name);
    //   if (n >= 0 && (size_t)n < sizeof(ird_path)) {
    //     klog_printf("[Dump] %s disc key unavailable\n", serial);
    //     klog_printf("[Dump] %s\n", ird_path);
    //     if (disc_ird_write(ird, ird_path, on_dump_progress, &log) != 0) {
    //       unlink(ird_path);
    //     }
    //   }
    //   disc_ird_free(ird);
    // }
    return;
  }
  log_dump_result(&log, rc == 0);
  notify_dump_result(rc == 0, serial, name);
  if (rc == 0) {
    log_dat_result(cd, type == DISC_SS_CD_ROM, serial, name, image, &hash);
  }
}
