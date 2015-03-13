/* Virtual Tape Library */ 

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>

#define MAGIC_NUMBER "VTAPBMS"
#define BLOCK_SIZE 512
#define PADDING_CHAR 0x1F

#define MAGICNO_OFFSET 0
#define LABEL_OFFSET 10
#define DESCR_OFFSET 31
#define SIZE_OFFSET 132
#define CREAT_DATE_OFFSET 143

#define BOB "2"

#define GET_REC_SIZE 13
#define WRITE_HEADER 20
#define VALIDATE 21
#define REWIND 22
#define NON_REWIND 23
#define SHOW_BACKUPS 24

char vtape[20]="/dev/rmt/0node";	// symbolic link to "/devices/pseudo/lyr@1:node" which is a 'rewind' version of the tape
char media_header[BLOCK_SIZE];		//Buffer to hold all header information. 
char magic_number[8]="VTAPBMS";		//Magic number for the tape.
char label[20];				//Label for the tape.		
char description[100];			//User description of the tape.	
char tape_size[10];			//Size of the tape.	
char creation_date[30];			//Last modification date.

struct file_table			//File Table
{
	char file_name[20];		//File name.
	off_t start_address;		//Offset of start of file.
	int no_of_records;		//Number of records in the file.
	char backup_date[10];		//Creation date of back-up file.
};

/*#define write_media_header 0
#define validate 1
#define print_header 2
#define backup 3
#define restore 4
#define nfsf 5
#define nbsf 6*/

int initialize(char size_of_tape[]);
int validate();
int insert();
int print_header();
int backup(int argc,char* argv[]);
int restore(int argc,char* argv[]);
int show_records(int argc,char* argv[]);
int show_backups();
int fsf(char no_backups_move[5]);
int bsf(char no_backups_move[5]);

/*
 * initialize
 *
 * Parameters:
 *	- size_of_tape:	Size of the media in bytes.
 *
 * Tape Structure:
 * 	- magic_number:	Magic number for the tape.
 *	- label:		Label for the tape.		
 *	- desc:		User description of the tape.
 *	- size_of_tape:	Size of the media in bytes. 
 *	- creation_date:	Tape creation date
 *
 * Return Value:
 *	- 0 on success
 *	- exits with error number in case of failure
 *
 * This function will write all the header information on the virtual tape media created. 
 * 
 */
int initialize(char size_of_tape[5])
{
	int fd,num,i, ret;
	time_t now = time(NULL);
   	struct tm *t = localtime(&now);

	printf("\nEnter the tape label: ");
	gets(label);
	
	printf("\nEnter the description of the backup: ");
	gets(description);
	
	
   	strftime( creation_date, sizeof(creation_date), "%x %X", t );	
	
	//strftime(last_mod_date, sizeof(last_mod_date), "%x %X", t );
	
	for(i=0;i<BLOCK_SIZE;i++)
		{	media_header[i]=PADDING_CHAR;
			//file_index_table[i]=PADDING_CHAR;
		}
	
	for(i=0;i<7;i++)
		media_header[i+MAGICNO_OFFSET]=magic_number[i];
	for(i=0;i<20 && label[i]!='\0' ;i++)
		media_header[i+LABEL_OFFSET]=label[i];
	for(i=0;i<100 && description[i]!='\0' ;i++)
		media_header[i+DESCR_OFFSET]=description[i];
	for(i=0;i<4 && size_of_tape[i]!='\0';i++)
		media_header[i+SIZE_OFFSET]=size_of_tape[i];		
	for(i=0;i<20 && creation_date[i]!='\0' ;i++)
		media_header[i+CREAT_DATE_OFFSET]=creation_date[i];
	
	fd=open(vtape,O_RDWR,666);
	ret=ioctl(fd,WRITE_HEADER);
	
	if(ret==-1)
	{
		printf("Error issuing WRITE_HEADER ioctl to tape media!! Exiting!!\n\n");
		close(fd);
		exit(1);
	}

	num=write(fd,media_header,512);

	if(num==-1)
	{
		printf("Error writing to tape media!! Exiting!!\n\n");
		close(fd);
		exit(1);
	}

	close(fd);

	return 0;
} 	

/*
 * validate
 *
 * Parameters:
 *	- takes no paramteres
 *
 * Return Value:
 *	- 0 on success
 *	- exits with error number in case of failure
 *
 * This function checks whether the media to be inserted is a valid tape media. 
 * It compares the magic number of the media to the magic number of the Virtual Tape(VTAPBMS).
 *
 */
int validate()
{
	int fd,ret,len, i;
	char magic_no[9];
	char buf[512];

	fd=open(vtape,O_RDONLY,666);

	if(fd == -1)
	{
		printf("Device not found!! Exiting!!\n");
		exit(1);
	}

	ret=ioctl(fd,VALIDATE);

	if(ret!=0)
	{
		printf("111Unable to find tape media!! Exiting!!\n");
		exit(1);
	}

	//ret=read(fd,magic_no,(strlen(magic_number)+1));	

	ret=read(fd,buf,BLOCK_SIZE);
        //printf("%s",buf);
	for(i=0;i<8;i++)
	{
		magic_no[i]=buf[i];
	}
    	//printf("magic no:%s",magic_no);
	/*if(ret!=0)
	{
		printf("Unable to read media!! Exiting!!\n");
		exit(1);
	}*/
        //printf("%s",magic_no);

	if(strncmp(magic_no,"VTAPBMS",7))
	{
		printf("222Unable to find tape media. Exiting!!\n");
		exit(1);
	}
	else	
		printf("Valid media can be inserted \n");
	
	close(fd);

	return 0;
}	

/*
 * insert
 *
 * Parameters:
 *	- takes no paramteres
 *
 * Return Value:
 *	- 0 on success
 *	- exits with error number in case of failure
 *
 * This function asks the user whether he wants to use the tape in rewind or non-rewind mode 
 * and sets the device path accordingly.
 * It also validates the tape.
 *
 */
int insert()
{
	int ret,choice,fd;

	printf("Valid tape found!!\n\nDo you want the tape to be inserted in REWIND-0 or NON-REWIND-1 mode? Enter your choice [0/1]:");

	scanf("%d",&choice);

	ret=0;
	
	if(choice)
	{
		strcpy(vtape,"/dev/rmt/0rnode");
		ret=validate();

		if(ret!=0)
		{
			printf("Unable to find tape media!! Exiting!!\n");
			exit(1);
		}

		fd=open(vtape,O_RDONLY,666);
		ret=ioctl(fd,NON_REWIND);
		close(fd);
	}

	else
	{
		ret=validate();

		if(ret!=0)
		{
			printf("Unable to find tape media!! Exiting!!\n");
			exit(1);
		}
	
		fd=open(vtape,O_RDONLY,666);
		ret=ioctl(fd,REWIND);
		close(fd);

	}

	printf("Your Device Node is: %s\n",vtape);
	return ret;
}

/*
 * print_header
 *	
 * Parameters:
 *	- takes no paramteres
 *
 * Return Value:
 *	- 0 on success
 *	- exits with error number in case of failure
 * 
 * This function checks whether the media to be inserted is a valid tape media by calling the validate() 
 * function. It prints the contents of the media header in the following format.
 *
 * 	- MAGIC NUMBER:			Magic number for the tape.
 *	- TAPE LABEL:			Label for the tape.		
 *	- TAPE DESCRIPTION:		User description of the tape.
 *	- SIZE OF TAPE:			Size of the media in bytes. 
 *	- TAPE CREATION DATE:		Tape creation date
 *
 */
int print_header()
{
	int fd, len, i, choice, ret;
	off_t off;
	char mediaheader[512], size_of_tape[4];	
	
	validate();	

	fd=open(vtape,O_RDONLY,0666);
	
	if(fd == -1)
	{
		printf("Device not found!! Exiting!!\n");
		exit(1);
	}
	
	ret=ioctl(fd,VALIDATE);

	if(ret!=0)
	{
		printf("Unable to find tape media!! Exiting!!\n");
		exit(1);
	}

	ret=read(fd,mediaheader,512);
	
	/*if(ret!=0)
	{
		printf("Unable to read tape media!! Exiting!!\n");
		exit(1);
	}*/ 

	for(i=0;i<7 && mediaheader[i]!=PADDING_CHAR ;i++)
		magic_number[i]=mediaheader[i+MAGICNO_OFFSET];
	magic_number[i]='\0';		
	
	for(i=0;i<20 && mediaheader[i+LABEL_OFFSET]!=PADDING_CHAR ;i++)
		label[i]=mediaheader[i+LABEL_OFFSET];
	label[i]='\0';	

	for(i=0;i<100 && mediaheader[i+DESCR_OFFSET]!=PADDING_CHAR ;i++)
		description[i]=mediaheader[i+DESCR_OFFSET];
	description[i]='\0';

	for(i=0;i<4 && mediaheader[i+SIZE_OFFSET]!=PADDING_CHAR;i++)
		size_of_tape[i]=mediaheader[i+SIZE_OFFSET];
	size_of_tape[i]='\0';

	for(i=0;i<20 && mediaheader[i+CREAT_DATE_OFFSET]!=PADDING_CHAR;i++)
		creation_date[i]=mediaheader[i+CREAT_DATE_OFFSET];
	creation_date[i]='\0';

	while(1)
	{
		printf("THE CONTENTS OF THE TAPE MEDIA HEADER ARE:\n");
		printf("1:Magic Number\n2:Description\n3:Tape Label\n4:Size of the tape\n5.:Tape Creation Date\n0:All fields\n");
		printf("Enter ur choice [0-5]:\n");

		scanf("%d",&choice);

		switch(choice)
		{
			case 1: 	printf("MAGIC NUMBER: %s\n",magic_number);
				break;
			case 2:	printf("TAPE LABEL: %s\n",label);
				break;
			case 3:	printf("DESCRIPTION ABOUT THE TAPE:\n %s\n",description);
				break;
			case 4:	printf("SIZE OF THE TAPE: %s\n",size_of_tape);
				break;
			case 5: 	printf("TAPE CREATION DATE: %s\n",creation_date);
				break;
			case 0:
			default:
				printf("MAGIC NUMBER: %s\n",magic_number);
				printf("TAPE LABEL: %s\n",label);
				printf("DESCRIPTION ABOUT THE TAPE:\n %s\n",description);
				printf("SIZE OF THE TAPE: %s\n",size_of_tape);
				printf("TAPE CREATION DATE: %s\n",creation_date);
				exit(0);
		}
 	}
	close(fd);
	
	return 0;

}

/*
 * backup
 *
 * Parameter:
 * 	- command line paramters of the main() function
 *
 * Return Value:
 *	- 0 on success
 *	- exits with error number in case of failure
 * 
 * This function backs up the file specified by file_src in the tape.
 *
 */
int backup(int argc,char* argv[])
{
	int fd, ret, i;
	char cmd[100]="tar -cvf ";

	strcat(cmd,vtape);

	strcat(cmd," ");

	fd=open(vtape,O_RDWR);

	if(fd == -1)
	{
		printf("Device not found!! Exiting!!\n");
		exit(1);
	}
	
	for(i=3;i<argc;i++)
	{
		strcat(cmd,argv[i]);
		strcat(cmd," ");
	}
	printf("%s\n",cmd);
	ret=system( cmd );	
	printf("ret value from system command: %d\n",ret);

	if(ret != 0)
	{
		close(fd);
		exit(1);
	}

	close(fd);

	return ret;
}

/*
 * restore
 *
 * Parameter:
 * 	- command line paramters of the main() function
 *
 * Return Value:
 *	- 0 on success
 *	- exits with error number in case of failure
 * 
 * This function restores the file which has been archived in the tape and which needs to 
 * be restored by copying it's contents into the filepath specified by file_dest.
 *
 */
int restore(int argc,char* argv[])
{
	int fd, ret,i;
	char cmd[100]="tar -xvf ";

	strcat(cmd,vtape);

	strcat(cmd," ");
	
	fd=open(vtape,O_RDWR);

	if(fd == -1)
	{
		printf("Device not found!! Exiting!!\n");
		exit(1);
	}
	
	for(i=3; i<argc; i++)
	{
	strcat(cmd,argv[i]);
	strcat(cmd," ");
	}

	printf("%s\n\n",cmd);	

	ret=system(cmd);	
	
	if(ret != 0)
	{
		printf("Tape Restore Error!! Exiting!!\n");
		close(fd);
		exit(1);
	}

	close(fd);

	return ret;
}	

/*
 * show_records
 *
 * Parameter:
 * 	- command line paramters of the main() function
 *
 * Return Value:
 *	- 0 on success
 *	- exits with error number in case of failure
 * 
 * This function displays all the records of the current backup to which the read pointer is pointing to.
 */
int show_records(int argc,char* argv[])
{
	int fd, ret, i;
	char cmd[100]="tar -tvf ";

	strcat(cmd,vtape);

	strcat(cmd," ");
	
	fd=open(vtape,O_RDWR);

	if(fd == -1)
	{
		printf("Device not found!! Exiting!!\n");
		exit(1);
	}
	
	for(i=4; i<argc; i++)
	{
	strcat(cmd,argv[i]);
	strcat(cmd," ");
	}

	ret=system(cmd);	
	
	if(ret != 0)
	{
		printf("Tape: Show Records Error!! Exiting!!\n");
		close(fd);
		exit(1);
	}

	close(fd);

	return ret;
}

/*
 * show_backups
 *	
 * Parameters:
 *	- takes no paramteres
 *
 * Return Value:
 *	- 0 on success
 *	- exits with error number in case of failure
 * 
 * This function displays the details of all the backups in the tape.
 *
 */
int show_backups()
{
	int fd, ret;
	
	fd=open(vtape,O_RDONLY,666);
	
	ret=ioctl(fd,SHOW_BACKUPS);

	if(ret != 0)
	{
		printf("Tape: Show Backups Error!! Exiting!!\n");
		close(fd);
		exit(1);
	}

	close(fd);

	return ret;
}

/*
 * fsf
 *
 * Parameter:
 *	- no_backups_move:	Forwards the read head by the number of backups 
 *			specified in no_backups_move.
 *
 * Return Value:
 *	- 0 on success
 *	- exits with error number in case of failure
 *  
 * This function moves the read head forward by no_backups_move number of backups.
 *
 */
int fsf(char no_backups_move[5])
{
	int fd, ret;
	char cmd[30]="mt -f ";

	strcat(cmd,vtape);
	strcat(cmd," ");
	strcat(cmd,"fsf");
	strcat(cmd," ");
	strcat(cmd,no_backups_move);
        printf("%s",cmd);
	fd=open(vtape,O_RDWR);

	if(fd == -1)
	{
		printf("Device not found!! Exiting!!\n");
		exit(1);
	}

	ret=system(cmd);

	if(ret !=0)
	{
		printf("FSF failed!! Exiting!! \n");
		close(fd);
		exit(1);
	}

	close(fd);
	
	return ret;
}	

/*
 * bsf
 *
 * Parameter:
 * 	- no_backups_move:	Rewinds the read head by the number of backups 
 *			specified in no_backups_move.
 *
 * Return Value:
 *	- 0 on success
 *	- exits with error number in case of failure
 * 
 * This function moves the read head backwards by no_backups_move number of backups.
 *
 */
int bsf(char no_backups_move[5])
{
	int fd, ret;
	char cmd[30]="mt -f ";

	strcat(cmd,vtape);
	strcat(cmd," ");
	strcat(cmd,"bsf");
	strcat(cmd," ");
	strcat(cmd,no_backups_move);

	fd=open(vtape,O_RDWR);

	if(fd == -1)
	{
		printf("Device not found!! Exiting!!\n");
		exit(1);
	}

	ret=system(cmd);

	if(ret !=0)
	{
		printf("BSF failed!! Exiting!! \n");
		close(fd);
		exit(1);
	}

	close(fd);
	
	return ret;
}	

int get_record_size()
{
	int fd,ret;
	
	fd=open(vtape,O_RDWR,0666);

	if(ret==-1)
	{
		printf("Unable to open device %s !! Exitting!!\n",vtape);
		exit(1);
	}

	ret=ioctl(fd,GET_REC_SIZE);

	if(ret)
	{
		printf("Unable to get record size!! Exitting!!\n",vtape);
		close(fd);
		exit(1);
	}

	close(fd);
	return 0;
}

int main(int argc,char *argv[])
{
	int ret;

	//initialize(argv[1]);

	if(!strcmp(argv[1],"initialize"))
	{	
		ret=initialize(argv[2]);
	}

	else if(!strcmp(argv[1],"insert"))
	{
		ret=insert();
	}
	
	else if(!strcmp(argv[1],"backup"))
	{
		ret=backup(argc,argv);
	}

	else if(!strcmp(argv[1],"restore"))
	{
		ret=restore(argc,argv);
	}

	else if(!strcmp(argv[1],"show_records"))
	{
		ret=show_records(argc,argv);
	}

	else if(!strcmp(argv[1],"show_backups"))
	{
		ret=show_backups();
	}

	else if(!strcmp(argv[1],"fsf"))
	{
		ret=fsf(argv[2]);
	}

	else if(!strcmp(argv[1],"bsf"))
	{
		ret=bsf(argv[2]);
	}

	else if(!strcmp(argv[1],"print_header"))
	{
		ret=print_header();
	}
	
	else if(!strcmp(argv[1],"get_record_size"))
	{
		ret=get_record_size();
	}

	return 0;
}
