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
#define SEQSPACE 12      /* the min sequence space for GBN must be at least windowsize + 1 */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */


typedef struct {
    struct pkt packet;
    bool      sent;
    bool      acked;    
    
} PacketStatus;
static PacketStatus buffer[SEQSPACE]; 
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
static int windowbase;  /*left side number*/              
static int A_nextseqnum;               /* the next sequence number to be used by the sender */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;

  /* if not blocked waiting on ACK */
  if ( (A_nextseqnum + SEQSPACE - windowbase) % SEQSPACE < WINDOWSIZE) {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for ( i=0; i<20 ; i++ )
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt);

    /* put packet in window buffer */
    /* windowlast will always be 0 for alternating bit; but not for GoBackN */
    buffer[A_nextseqnum].packet = sendpkt;
    buffer[A_nextseqnum].sent   = true;
    buffer[A_nextseqnum].acked  = false;
    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3 (A, sendpkt);

    /* 如果这是窗口基序号对应的包，启动定时器 */
    if (sendpkt.seqnum == windowbase) {
      starttimer(A, RTT);
    }

    /* get next sequence number, wrap back to 0 */
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;
  }
  /* if blocked,  window is full */
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
  int ack = packet.acknum;
  int offset = (ack + SEQSPACE - windowbase) % SEQSPACE;
  int outstanding;

  /* if received ACK is not corrupted */
  if (!IsCorrupted(packet)) {
      if (TRACE > 0)
          printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
      total_ACKs_received++;

      /* check if new ACK or duplicate */
      if (offset < WINDOWSIZE && buffer[ack].sent && !buffer[ack].acked) {
          buffer[ack].acked = true;
          if (TRACE > 0)
              printf("----A: ACK %d is not a duplicate\n", packet.acknum);
          new_ACKs++;

          /* slide windowbase */
          while (buffer[windowbase].acked) {
              buffer[windowbase].sent  = false;
              buffer[windowbase].acked = false;
              windowbase = (windowbase + 1) % SEQSPACE;
          }
      }
      else {
          if (TRACE > 0)
              printf("----A: duplicate ACK received, do nothing!\n");
      }

      /* start timer again if there are still more unacked packets */
      outstanding = (A_nextseqnum + SEQSPACE - windowbase) % SEQSPACE;
      if (outstanding > 0) {
          stoptimer(A);
          starttimer(A, RTT);
      } else {
          stoptimer(A);
      }
  }
  else {
      if (TRACE > 0)
          printf("----A: corrupted ACK is received, do nothing!\n");
  }
}



/* called when A's timer goes off */
void A_timerinterrupt(void) {
    int i;
    int outstanding;

    /* 超时提示，保留原样 */
    if (TRACE > 0)
        printf("----A: time out,resend packets!\n");

    /* 计算当前窗口还有多少未 ACK 的包 */
    outstanding = (A_nextseqnum + SEQSPACE - windowbase) % SEQSPACE;

    /* 重传窗口里所有未 ACK 包（GBN/Stop-&-Wait 版本）*/
    for (i = 0; i < outstanding; i++) {
        int idx = (windowbase + i) % SEQSPACE;
        if (TRACE > 0)
            printf("---A: resending packet %d\n",
                   buffer[idx].packet.seqnum);
        tolayer3(A, buffer[idx].packet);
        packets_resent++;
    }

    starttimer(A, RTT);
  }




/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{

  /* initialise A's window, buffer and sequence number */
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  windowbase   = 0;


}


/********* Receiver (B)  variables and procedures ************/

   /* the sequence number for the next packets sent by B */
static bool       received[SEQSPACE] = {false};
static struct pkt recv_buffer[SEQSPACE];
static int        expectedseqnum;
static int        B_nextseqnum;

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet) {
    struct pkt sendpkt;
    int i;
    int offset;            
    
    /* 1) 先构造 sendpkt 的固定字段 */
    sendpkt.seqnum = B_nextseqnum;
    B_nextseqnum   = (B_nextseqnum + 1) % 2;
    for (i = 0; i < 20; i++)
        sendpkt.payload[i] = '0';

    /* 2) 校验＆统计 */
    if (IsCorrupted(packet)) {
        if (TRACE > 0)
            printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
        sendpkt.acknum = (expectedseqnum + SEQSPACE - 1) % SEQSPACE;
    }
    else {
        packets_received++;

        /* now it’s safe to assign offset, because the declaration was up top */
        offset = (packet.seqnum + SEQSPACE - expectedseqnum) % SEQSPACE;
        if (offset < WINDOWSIZE && !received[packet.seqnum]) {
            received[packet.seqnum]     = true;
            recv_buffer[packet.seqnum] = packet;
        }

        while (received[expectedseqnum]) {
            if (TRACE > 0)
                printf("----B: packet %d is correctly received, send ACK!\n",
                       expectedseqnum);

            tolayer5(B, recv_buffer[expectedseqnum].payload);
            received[expectedseqnum] = false;
            expectedseqnum = (expectedseqnum + 1) % SEQSPACE;
        }

        sendpkt.acknum = packet.seqnum;
    }

    /* 3) 发送 ACK */
    sendpkt.checksum = ComputeChecksum(sendpkt);
    tolayer3(B, sendpkt);
}



/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{

  expectedseqnum = 0;
  B_nextseqnum = 1;

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
