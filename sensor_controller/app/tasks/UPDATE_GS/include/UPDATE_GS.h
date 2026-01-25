#ifndef UPDATE_GS_H
#define UPDATE_GS_H

#define TXD_PIN 34
#define RXD_PIN 35

void UPDATE_GS(void *arg);
int send_data(const char* logName, const char* data);
void init(void);

#endif // UPDATE_GS_H

