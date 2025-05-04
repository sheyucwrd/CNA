#ifndef SR_H
#define SR_H


extern void A_init(void);
extern void B_init(void);
extern void A_input(struct pkt);
extern void B_input(struct pkt);
extern void A_output(struct msg);
extern void A_timerinterrupt(void);
#define BIDIRECTIONAL 0

/* optional */
extern void B_output(struct msg);
extern void B_timerinterrupt(void);

#endif