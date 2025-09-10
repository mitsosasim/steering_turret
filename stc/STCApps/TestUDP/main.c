/* =======================================================================================================
  main.c
  Created on: Aug 26, 2025

  ALMA sistemi s.r.l. 2024
   -------------
  Authors:
    R. Filippone
    A. Kamal Eddine

 ------------------------------------------------------------------------------
  v. 1.0 - Aug 26, 2025 - Short description
 ------------------------------------------------------------------------------
  Reference:
    R1 - Reference (document, web site, other source files, etc.)
    R2
 ------------------------------------------------------------------------------
  Functions and logics

======================================================================================================= */


/* -------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------- */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>

#define DEST_IP "192.168.1.241"
#define DEST_PORT 9914
#define MESSAGE "test"

int main() {
    int sockfd;
    struct sockaddr_in dest_addr;

    // Create UDP socket
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        perror("socket creation failed");
        return 1;
    }

    // Setup destination address
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(DEST_PORT);
    if (inet_pton(AF_INET, DEST_IP, &dest_addr.sin_addr) <= 0) {
        perror("Invalid address or address not supported");
        close(sockfd);
        return 1;
    }

    // Send the message
    do {
		ssize_t sent_bytes = sendto(sockfd, MESSAGE, strlen(MESSAGE), 0,
									(struct sockaddr *)&dest_addr, sizeof(dest_addr));
		if (sent_bytes < 0) {
			perror("sendto failed");
			close(sockfd);
			return 1;
		}

		printf("Sent message: \"%s\" to %s:%d\n", MESSAGE, DEST_IP, DEST_PORT);
		sleep(1);
    }while(1);

    // Clean up
    close(sockfd);
    return 0;
}

