// HcsmLogControl.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <hcsmcollection.h>

int               g_SizeInKbytes = 16*1024;
CHcsmMemoryLog    g_HcsmLog;
TMemBlock*        g_pBlock;

static void
Usage(void)
{
	fprintf(stderr, "Usage:\n  "
		"HcsmLogControl  [-create sizeInKbytes] [-dump fname]\n"
		"  -create    Create the region so scnvif can log debug info\n"
		"  -dump      Ascii dump of existing binary file\n");
}


int main(int argc, char* argv[])
{

	if ( argc < 2 ) {
		Usage();
		exit(-1);
	}

	if ( !strcmp(argv[1], "-create")) {
		if ( argc > 2 ) 
			g_SizeInKbytes = atoi(argv[2]);

		printf("Creating a %d MB shared memory region ...\n", 
			g_SizeInKbytes/1024);

		g_pBlock = g_HcsmLog.CreateMem(g_SizeInKbytes);
		if ( g_pBlock == 0 ) {
			printf("Memory block creation failed.\n");
			exit(-1);
		}

		while ( 1 ) {
			int n, choice;

			printf(
				"\n"
				" 0 - Clear log\n"
				" 1 - Ascii dump to file\n"
				" 2 - Binary dump to file\n"
				" 3 - Print info\n"
				"99 - Quit\n");
			n = scanf("%d", &choice);
			if ( n == 0 ) continue;

			if ( choice == 0 ) {
				g_HcsmLog.ClearLog();
			}
			else if ( choice == 1 ) {
				char fname[128];

				printf("Enter file name for ascii dump: ");
				scanf("%s", fname);
				g_HcsmLog.DumpToFile(fname);
			}
			else if ( choice == 2 ) {
				char fname[128];

				printf("Enter file name for binary dump: ");
				scanf("%s", fname);
				g_HcsmLog.DumpToFile(fname, true);
			}
			else if ( choice == 3 ) {
				printf("Capacity:  %d slots\n", g_pBlock->m_NumSlots);
				printf("Events  :  %d\n", g_pBlock->m_Next);
			}
			else {
				break;
			}
		}
	}
	else if ( !strcmp(argv[1], "-dump")) {
		if ( argc <3 ) { 
			Usage();
			exit(-1);
		}

		FILE *pF = fopen(argv[2], "rb");
		if ( pF == 0 ) {
			perror("Can't open file");
			exit(-1);
		}
		TMemBlock  block;
		TMemLog    log;

		fread(&block, sizeof(block), 1, pF);
		log = block.m_Data[0];

		printf("  Time      Frame  HcsmId   Tag       Message\n");
		printf(" --------------------------------------------------------\n");
		while ( !feof(pF) ) {
			printf("%8ud  %5d  %4d     %4d    %s\n",
				log.m_TimeStamp, log.m_Frame, log.m_HcsmId, log.m_Tag, log.m_Msg);

			fread(&log, sizeof(log), 1, pF);
		}

		fclose(pF);
	}
	else {
		Usage();
		exit(-1);
	}


	printf("Bye bye ...\n");
	return 0;
}
