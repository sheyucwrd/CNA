
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"

#define RTT  16.0
#define WINDOWSIZE 6
#define SEQSPACE 12
#define NOTINUSE (-1)

static bool acked[SEQSPACE];
static struct pkt recv_buffer[SEQSPACE];
static bool received[SEQSPACE];

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

static struct pkt buffer[SEQSPACE];
static int windowfirst;
static int windowcount;
static int A_nextseqnum;

void A_output(struct msg message) {
    struct pkt sendpkt;
    int i;

    if ( windowcount < WINDOWSIZE) {
        if (TRACE > 1)
            printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

        sendpkt.seqnum = A_nextseqnum;
        sendpkt.acknum = NOTINUSE;
        for ( i=0; i<20 ; i++ )
            sendpkt.payload[i] = message.data[i];
        sendpkt.checksum = ComputeChecksum(sendpkt);

        buffer[A_nextseqnum] = sendpkt;
        acked[A_nextseqnum]  = false;
        windowcount++;

        if (TRACE > 0)
            printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
        tolayer3 (A, sendpkt);

        if (windowcount == 1)
            starttimer(A,RTT);

        A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;
    }
    else {
        if (TRACE > 0)
            printf("----A: New message arrives, send window is full\n");
        window_full++;
    }
}

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
    seqfirst = (A_nextseqnum - windowcount + SEQSPACE) % SEQSPACE;
    seqlast  = (seqfirst + windowcount - 1) % SEQSPACE;

    if (windowcount > 0 &&
        ((seqfirst <= seqlast && ackn >= seqfirst && ackn <= seqlast) ||
         (seqfirst > seqlast && (ackn >= seqfirst || ackn <= seqlast))) &&
        !acked[ackn]) {

        if (TRACE > 0)
            printf("----A: ACK %d is not a duplicate\n", ackn);
        new_ACKs++;

        acked[ackn] = true;

        while (windowcount > 0 && acked[seqfirst]) {
            seqfirst = (seqfirst + 1) % SEQSPACE;
            windowcount--;
        }

        stoptimer(A);
        if (windowcount > 0)
            starttimer(A, RTT);
    } else {
        if (TRACE > 0)
            printf ("----A: duplicate ACK received, do nothing!\n");
    }
}

void A_timerinterrupt(void) {
    int seqfirst;
    if (TRACE > 0)
        printf("----A: time out,resend packets!\n");

    seqfirst = (A_nextseqnum - windowcount + SEQSPACE) % SEQSPACE;
    tolayer3(A, buffer[seqfirst]);
    packets_resent++;

    starttimer(A,RTT);
}

void A_init(void) {
    int i;
    A_nextseqnum = 0;
    windowfirst = 0;
    windowcount = 0;
    for (i = 0; i < SEQSPACE; i++)
        acked[i] = true;
}

/********* Receiver (B) ************/

static int expectedseqnum;
static int B_nextseqnum;

void B_input(struct pkt packet) {
    struct pkt sendpkt;
    int i, rel;

    if (!IsCorrupted(packet)) {
        rel = (packet.seqnum - expectedseqnum + SEQSPACE) % SEQSPACE;
        if (rel < WINDOWSIZE && !received[packet.seqnum]) {
            recv_buffer[packet.seqnum] = packet;
            received[packet.seqnum]     = true;
        }

        sendpkt.seqnum = B_nextseqnum;
        sendpkt.acknum = packet.seqnum;
        B_nextseqnum   = (B_nextseqnum + 1) % 2;
    } else {
        if (TRACE > 0)
            printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
        sendpkt.seqnum = B_nextseqnum;
        sendpkt.acknum = (expectedseqnum == 0 ? SEQSPACE - 1 : expectedseqnum - 1);
        B_nextseqnum = (B_nextseqnum + 1) % 2;
    }

    for ( i=0; i<20 ; i++ )
        sendpkt.payload[i] = '0';
    sendpkt.checksum = ComputeChecksum(sendpkt);
    tolayer3 (B, sendpkt);

    while (received[expectedseqnum]) {
        if (TRACE > 0)
            printf("----B: packet %d is correctly received, send ACK!\n", expectedseqnum);
        packets_received++;
        tolayer5(B, recv_buffer[expectedseqnum].payload);
        received[expectedseqnum] = false;
        expectedseqnum = (expectedseqnum + 1) % SEQSPACE;
    }
}

void B_init(void) {
    int i;
    expectedseqnum = 0;
    B_nextseqnum = 1;
    for (i = 0; i < SEQSPACE; i++)
        received[i] = false;
}

void B_output(struct msg message) {}
void B_timerinterrupt(void) {}
