#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"

/* ******************************************************************
   Go Back N protocol.  Adapted from J.F.Kurose
   ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.2

   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).

   Modifications:
   - removed bidirectional GBN code and other code not used by prac.
   - fixed C style to adhere to current programming style
   - added GBN implementation
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet
                          MUST BE SET TO 6 when submitting assignment */
#define SEQSPACE 7      /* the min sequence space for GBN must be at least windowsize + 1 */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */


static bool acked[SEQSPACE];             /* A 端：标记哪些 seqnum 已被 ACK */
static struct pkt recv_buffer[SEQSPACE]; /* B 端：缓存乱序到达且在窗口内的包 */
static bool received[SEQSPACE];  

/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver
   the simulator will overwrite part of your packet with 'z's.  It will not overwrite your
   original checksum.  This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  int i;

  checksum = packet.seqnum;
  checksum += packet.acknum;
  for ( i=0; i<20; i++ )
    checksum += (int)(packet.payload[i]);

  return checksum;
}

bool IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return (false);
  else
    return (true);
}


/********* Sender (A) variables and functions ************/

static struct pkt buffer[WINDOWSIZE];  /* array for storing packets waiting for ACK */
static int windowfirst;    /* array indexes of the first/last packet awaiting ACK */
static int windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message) {
    int i;
    struct pkt sendpkt;

    /* 如果窗口未满，和原来一样，但加入 acked 标记逻辑 */
    if (windowcount < WINDOWSIZE) {
        if (TRACE > 1)
            printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

        /* 构造 packet */
        sendpkt.seqnum = A_nextseqnum;
        sendpkt.acknum = NOTINUSE;
        for (i = 0; i < 20; i++)
            sendpkt.payload[i] = message.data[i];
        sendpkt.checksum = ComputeChecksum(sendpkt);

        /* 缓存并标记未确认 */
        buffer[A_nextseqnum] = sendpkt;
        acked[A_nextseqnum]  = false;
        windowcount++;

        /* 发送与调试输出不变 */
        if (TRACE > 0)
            printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
        tolayer3(A, sendpkt);

        /* 如果这是第一个未 ack 的包，启动定时器 */
        if (windowcount == 1)
            starttimer(A, RTT);

        /* 更新下一个 seqnum */
        A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;
    }
    else {
        if (TRACE > 0)
            printf("----A: New message arrives, send window is full\n");
        window_full++;
    }
}



/* called from layer 3, when a packet arrives for layer 4
   In this practical this will always be an ACK as B never sends data.
*/
void A_input(struct pkt packet) {
    int seqfirst, seqlast, ackn;

    if (IsCorrupted(packet)) {
        if (TRACE > 0)
            printf("----A: corrupted ACK is received, do nothing!\n");
        return;
    }
    if (TRACE > 0)
        printf("----A: uncorrupted ACK %d is received\n", packet.acknum);

    ackn = packet.acknum;
    /* 计算当前窗口中最老／最新包的 seqnum */
    seqfirst = (A_nextseqnum - windowcount + SEQSPACE) % SEQSPACE;
    seqlast  = (seqfirst + windowcount - 1) % SEQSPACE;

    /* 判断 ACK 是否在窗口内，且之前未确认 */
    if (windowcount > 0
     && ((seqfirst <= seqlast
            && ackn >= seqfirst && ackn <= seqlast)
       || (seqfirst > seqlast
            && (ackn >= seqfirst || ackn <= seqlast)))
     && !acked[ackn])
    {
        if (TRACE > 0)
            printf("----A: ACK %d is not a duplicate\n", ackn);

        /* 标记该包已 ACK */
        acked[ackn] = true;

        /* 累积确认：滑出所有已 ACK 包 */
        while (windowcount > 0 && acked[seqfirst]) {
            /* 保留 acked[seqfirst]=true 以便后续跳过 */
            seqfirst = (seqfirst + 1) % SEQSPACE;
            windowcount--;
        }

        /* 重新管理定时器 */
        stoptimer(A);
        if (windowcount > 0)
            starttimer(A, RTT);
    }
    else {
        if (TRACE > 0)
            printf("----A: duplicate ACK received, do nothing!\n");
    }
}

/* called when A's timer goes off */
void A_timerinterrupt(void) {
    int idx, seqfirst;

    if (TRACE > 0)
        printf("----A: time out, resend packets!\n");

    /* 找到最老那一个未确认包的 seqnum */
    seqfirst = (A_nextseqnum - windowcount + SEQSPACE) % SEQSPACE;

    /* 仅重传这一个包 */
    idx = seqfirst;
    if (TRACE > 0)
        printf("---A: resending packet %d\n", buffer[idx].seqnum);
    tolayer3(A, buffer[idx]);

    /* 重启定时器 */
    starttimer(A, RTT);
}



/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void) {
    int i;
    /* 保留原来对 windowfirst/windowcount/A_nextseqnum 的初始化 */
    A_nextseqnum = 0;
    windowfirst   = 0;
    windowcount   = 0;
    /* 新增：把所有位置初始化为“已确认”，方便滑动时跳过 */
    for (i = 0; i < SEQSPACE; i++)
        acked[i] = true;
}



/********* Receiver (B)  variables and procedures ************/

static int expectedseqnum; /* the sequence number expected next by the receiver */
static int B_nextseqnum;   /* the sequence number for the next packets sent by B */


/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet) {
    struct pkt sendpkt;
    int i, rel;

    /* 判断是否损坏 */
    if (!IsCorrupted(packet)) {
        /* 在窗口内且未缓存时，缓存 */
        rel = (packet.seqnum - expectedseqnum + SEQSPACE) % SEQSPACE;
        if (rel < WINDOWSIZE && !received[packet.seqnum]) {
            recv_buffer[packet.seqnum] = packet;
            received[packet.seqnum]     = true;
        }

        /* 对每个到达包立即 ACK（累积逻辑已去除） */
        sendpkt.seqnum = B_nextseqnum;
        sendpkt.acknum = packet.seqnum;
        B_nextseqnum   = (B_nextseqnum + 1) % 2;
    }
    else {
        /* 损坏则重发对上一个按序收到的包的 ACK */
        sendpkt.seqnum = B_nextseqnum;
        sendpkt.acknum = (expectedseqnum == 0
                          ? SEQSPACE - 1
                          : expectedseqnum - 1);
        B_nextseqnum = (B_nextseqnum + 1) % 2;
    }

    /* 填充无用字段并发送 ACK */
    for (i = 0; i < 20; i++)
        sendpkt.payload[i] = '0';
    sendpkt.checksum = ComputeChecksum(sendpkt);
    tolayer3(B, sendpkt);

    /* 按序交付所有已经缓存且连续的包 */
    while (received[expectedseqnum]) {
        tolayer5(B, recv_buffer[expectedseqnum].payload);
        received[expectedseqnum] = false;
        expectedseqnum           = (expectedseqnum + 1) % SEQSPACE;
    }
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void) {
    int i;
    expectedseqnum = 0;
    B_nextseqnum   = 1;
    for (i = 0; i < SEQSPACE; i++)
        received[i] = false;
}

/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

/* Note that with simplex transfer from a-to-B, there is no B_output() */
void B_output(struct msg message)
{
}

/* called when B's timer goes off */
void B_timerinterrupt(void)
{
}
