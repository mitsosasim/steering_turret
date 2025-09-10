#include <gpiod.h>
#include <stdio.h>
#include <unistd.h>

/*
Pin	Chip Line
			3	1	20
			5	1	21
			7	4	24
			8	4	20
			10	4	19
			11	1	22
			12	3	24
GPIO1_C2	13	1	18   <= GPIO(bank 1)_(offset C)(number 2)
GPIO1_A1	15	1	1    <= GPIO(bank 1)_(offset 0)(number 1)
			16	1	4
			18	4	21
			19	1	8
			21	1	7
			22	4	25
			23	1	9
			24	1	10
			26	1	13
			27	1	11
			28	1	12
			29	4	27
			31	4	28
			32	3	28
			33	3	29
			35	3	26
			36	3	30
			37	3	25
			38	3	27
			40	3	31

On Linux, using the new /dev/gpiochip API, the _n_ in GPIO_n___XX_ appears to correlate to the number of the /dev/gpiochip_n_,
and the _XX_ to the definition RK_P_XX_ of lines in include/dt-bindings/pinctrl/rockchip.h of the Linux kernel source.
Having these named in the dts would be nice.
You can use libgpiod to drive them, and test them with the included tools (gpioinfo, gpioset, …)
For example, gpioset 4 25=1 (run as root) would turn pin 22 on. Do beware that poking the wrong GPIO pin can lock up your system.
The conversion table at right is also available as a C header file.
 */

#define GPIO_CHIP 		"/dev/gpiochip1"	// Might be gpiochip 0/1 depending on the board
#define GPIO_NAME 		"GPIO1_A1"

unsigned int bank = 0,
			 offset = 0,
			 number = 0;

int GetGPIOata(char *);

int main(int argc, char **argv)
{
	struct gpiod_chip *chip;
    struct gpiod_line *line;
    int i=0,ret;
    unsigned int LineId;

	/* --------------------------------------------------------------------------------------
	   Test the gpiod library
	--------------------------------------------------------------------------------------
	chip = gpiod_chip_open(GPIO_CHIP);
	if(!chip) {
		perror("gpiod_chip_open");
		return 1;
	}
	printf("Opened GPIO chip successfully\n");
	gpiod_chip_close(chip);
	return 0;*/


    /*
    The formula for converting GPIO<bank>_<letter><number> to a GPIO number is:
    GPIO number = [(bank × 32) + (letter_offset × 8) + number]%32  (modulo 32)
    Letter offset: A = 0, B = 1, C = 2, D = 3
    GPIO1_C2 = (1 × 32) + (2 × 8) + 2 = 32 + 16 + 2 = 50
    */
    LineId = GetGPIOata(GPIO_NAME);

    /* --------------------------------------------------------------------------------------
	   Open the GPIO chip
	-------------------------------------------------------------------------------------- */
    printf("Open chip:\n");
    chip = gpiod_chip_open(GPIO_CHIP);
    if (!chip) {
        perror("failed");
        return 1;
    }

    // Get the GPIO line
    printf("Get line:\n");
    line = gpiod_chip_get_line(chip,LineId);
    if (!line) {
        perror("Get line failed");
        gpiod_chip_close(chip);
        return 1;
    }

    // Request the line as output, initial value LOW
    printf("Request as output:\n");
    ret = gpiod_line_request_output(line,"gpio_test", 0);
    if (ret < 0) {
        perror("Request line as output failed");
        gpiod_chip_close(chip);
        return 1;
    }

    // Set GPIO LOW
    printf("GPIO set LOW\n");
    gpiod_line_set_value(line,0);

	while(++i < 30) {

		// Set GPIO HIGH
		gpiod_line_set_value(line,1);
		printf("GPIO set HIGH\n");

	    sleep(1);

	    // Set GPIO LOW
	    gpiod_line_set_value(line,0);
	    printf("GPIO set LOW\n");

	    sleep(1);
	}

	// Release line and close chip
	gpiod_line_release(line);
	gpiod_chip_close(chip);

	return 0;
}


/* --------------------------------------------------------------------------------------
	The formula for converting GPIO<bank>_<letter><number> to a GPIO number is:
	GPIO number = [(bank × 32) + (letter_offset × 8) + number]%32  (modulo 32)
	Letter offset: A = 0, B = 1, C = 2, D = 3
	GPIO1_C2 = (1 × 32) + (2 × 8) + 2 = 32 + 16 + 2 = 50
-------------------------------------------------------------------------------------- */
int GetGPIOata(char *p)
{
	int i=0,id,Line;
	char ch;

	while(p[i] != '\0') {
		ch = p[i];
		switch(i) {
		case 4:
			bank = ch - 48;
			break;
		case 6:
			offset = ch - 65;
			break;
		case 7:
			number = ch -48;
			break;
		}
		++i;
	}
	id = (bank * 32) + (offset * 8) + number;
	Line = id%32;
    printf("%s => pin: %d => line: %d\n",GPIO_NAME,id,Line);
    printf("    ^ ^^\n");
    printf("    | |+- number: %d\n",number);
    printf("    | +-- offset: %d\n",offset);
    printf("    +---- bank:   %d\n",bank);
    return(Line);
}

