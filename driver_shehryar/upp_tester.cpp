#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <string.h>

int main(int argc, char **argv )
{
	int count = 0;
	const int BytesToRead = 8192*2; //200MB
	int	bytesRead  = -1;
	int bytesWritten = -1 ;
	int upp_fd,out_fd;
	char readBuff[BytesToRead] = {0,};
 	int k = 0;
	
	
	readBuff[BytesToRead]={'\0'};
	upp_fd = open( "/dev/upp", O_RDWR);
	out_fd = open( "output_upp.264", O_RDWR|O_CREAT);
	for(count = 0; count<100; count++){
	
	if (upp_fd == -1) 
	{
		fprintf( stderr , "OPEN failed [%d] - UPP\n" , errno );
		if (errno == 2) 
		{
			fprintf( stderr , "NOTE! Check that /dev/upp actually exists, and has the right permissions\n" );
		}
		return errno;
	}
	if (out_fd == -1) 
	{
		fprintf( stderr , "OPEN /output_upp failed [%d] - Out.dat\n" , errno );
	}
	
	//fprintf( stderr , "UPP TESTER Version 1.0.17\n\n" ); 
	
	for (k=0;k<64;k++)
	{	bytesRead    = read( upp_fd, readBuff, 16384);
		
	
		fprintf( stderr , "READ  [%d] bytes out of [%d] bytes\n" , bytesRead    , sizeof(readBuff) );

		bytesWritten = write( out_fd, readBuff, bytesRead );
		fprintf( stderr , "WROTE [%d] bytes out of [%d] bytes\n" , bytesWritten , sizeof(readBuff) );
	}	
 }	
	close(upp_fd);
	close(out_fd);
	
		
	
	
	return 0;
}
