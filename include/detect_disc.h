#ifndef DETECT_DISC_H_
#define DETECT_DISC_H_

#include <stdint.h>

#include "disc_type.h"

struct disc_pass {
  int fd;
  uint32_t path;
  uint32_t target;
  uint32_t lun;
};

enum disc_type detect_disc(void);

// Device node of the CD unit detect_disc() last settled on, such as "/dev/cd0".
const char *disc_dev_path(void);

// CAM status of the last command sent, for error reports.
uint32_t disc_cam_status(void);

// The 2048 user bytes of LBA 0, read when detect_disc() identified a Saturn disc. 0 if there are none.
int disc_saturn_header(uint8_t out[2048]);

int disc_pass_open(struct disc_pass *pass);
void disc_pass_close(struct disc_pass *pass);
int disc_pass_scsi(struct disc_pass *pass, const uint8_t *cdb, uint8_t cdb_len, void *data, uint32_t data_len, uint32_t timeout_ms, uint32_t *sense_out);
void disc_request_read_speed(uint32_t end_lba);

#endif
