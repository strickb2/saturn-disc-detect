#ifndef NOTIFY_H_
#define NOTIFY_H_

// Shows a notification on the console's screen, the same toast the system uses for its own messages.
void notify(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

#endif
