#include <unistd.h>   
#include <stdint.h>
#include <stdio.h>   
#include <termios.h>   
#include <fcntl.h>   
#include <sys/select.h>
#include <sys/time.h>
#include <string.h>   
#include <mysql/mysql.h>
#include <time.h>   
#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <termios.h>


//为了保证用户输入的波特率是个正确的值，所以需要这两个数组验证，对于设置波特率时候，前面要加个B   
int speed_arr[] = { B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300,  
    B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300, };  
  
int name_arr[] = {115200, 57600, 38400, 19200, 9600, 4800, 2400, 1200, 300,  
    115200, 57600, 38400, 19200, 9600, 4800, 2400, 1200, 300, };  


#define MAX_TIME 85
#define DHT11PIN 7 
#define ATTEMPTS 5                 //retry 5 times when no response

int dht11_val[5]={0,0,0,0,0};


// the following programm is really tough for now,
// but ,God knows, if spending more time , I can totally handle it!
int open_port( int fd, int comport,char *compath)
{
    //char *dev[] = { "/dev/ttyS0","/dev/ttyhS1","/dev/ttyS2"};
    //long vdisable;

    //serail comprot 1
    if(comport)
    {
      fd = open(compath, O_RDWR | O_NOCTTY | O_NDELAY);
      if(-1 == fd){ 
             perror("Can't open serial port! \n ");
             return (-1);
            }
    }
    if(fcntl(fd,F_SETFL, 0) < 0)
    printf("Fcntl failed!\n ");
    else
      printf("Fcntl = %d \n ",fcntl(fd, F_SETFL, 0));

    if(isatty(STDIN_FILENO) == 0 )
    printf("Standard input is not a termina device \n");
    else
     printf("isatty successed! \n");
     printf("fd = %d \n",fd);
    return fd;
}

//setup serial comport parameter; fd, 9600, 8, 'n',1
int set_opt( int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio, oldtio;
    if ( tcgetattr( fd, &oldtio) != 0 )
    {
        perror("Setup Serial 1 ");
        return -1;
    }

    bzero(&newtio, sizeof(newtio)); //Initiate the termios struct

    newtio.c_cflag |= CLOCAL | CREAD;

    newtio.c_cflag &= ~CSIZE;

    switch(nBits)
    {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
    }

    switch(nEvent)
    {
        case 'O':
        //the next two lines is to setup the odd-even testing bit
        newtio.c_cflag |= PARENB;
        //newtio.c_cflag |= PARODD;

        //newtio.c_iflag |= (INPCK | ISTRIP );
        newtio.c_iflag |= (INPCK);
        break;

	/*
        case 'E':
        //the next two lines is to setup the odd-even testing bit
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PAREVE;
        newtio.c_iflag |= (INPCK | ISTRIP );
        break;
	*/

        //no testing bit 
        case 'N':   
        newtio.c_cflag &= ~PARENB;
        break;
    }

    //the following lines is to setup the bits rate
    switch(nSpeed)
    {
        case 2400:
        cfsetispeed( &newtio,B2400);
        cfsetospeed( &newtio,B2400);
        break;

        case 4800:
        cfsetispeed( &newtio,B4800);
        cfsetospeed( &newtio,B4800);
        break;

        case 9600:
              cfsetispeed( &newtio,B9600);
              cfsetospeed( &newtio,B9600);
        break;
 
        case 115200:
              cfsetispeed( &newtio,B115200);
              cfsetospeed( &newtio,B115200);
        break;

        default :
              cfsetispeed( &newtio,B9600);
              cfsetospeed( &newtio,B9600);
        break;
    }

    // setup the stop bit 
    if( nStop == 1 )
       newtio.c_cflag &= ~CSTOPB;
    else if( nStop == 2 )
       newtio.c_cflag |= CSTOPB;

    newtio.c_cc[VTIME] = 0;  //èù?canonical ê?°?oè??aêó?á???a?êó??o?‰a?????àü‰π?‰∏?á?í‰∏∫???‰Ω?
    newtio.c_cc[VMIN] = 0;   //èù?canonical ê?°?oè??aá??êú??∞è?≠óá¨?ê?∞?o?MIN‰∏a???êò??°?á§∫?éΩêa°??≥readá??êú??∞è?≠ó??éê?∞ ?o?
    tcflush(fd, TCIFLUSH);

    if( (tcsetattr( fd, TCSANOW, &newtio)) != 0 )
    {
        perror("Com set failed! \n");
        return -1;
    }
    printf("Comport setup accomplish! \n");
    return 0;
}

  
/*----------------------------------------------------------------------------- 
  函数名:      set_speed 
  参数:        int fd ,int speed 
  返回值:      void 
  描述:        设置fd表述符的串口波特率 
 *-----------------------------------------------------------------------------*/  
void set_speed(int fd ,int speed)  
{  
    struct termios opt;  
    int i;  
    int status;  
  
    tcgetattr(fd,&opt);  
    for(i = 0;i < sizeof(speed_arr)/sizeof(int);i++)  
    {  
        if(speed == name_arr[i])                        //找到标准的波特率与用户一致   
        {  
            tcflush(fd,TCIOFLUSH);                      //清除IO输入和输出缓存   
            cfsetispeed(&opt,speed_arr[i]);         //设置串口输入波特率   
            cfsetospeed(&opt,speed_arr[i]);         //设置串口输出波特率   
  
            status = tcsetattr(fd,TCSANOW,&opt);    //将属性设置到opt的数据结构中，并且立即生效   
            if(status != 0)  
                perror("tcsetattr fd:");                //设置失败   
            return ;  
        }  
        tcflush(fd,TCIOFLUSH);                          //每次清除IO缓存   
    }  
}  
/*----------------------------------------------------------------------------- 
  函数名:      set_parity 
  参数:        int fd 
  返回值:      int 
  描述:        设置fd表述符的奇偶校验 
 *-----------------------------------------------------------------------------*/  
int set_parity(int fd)  
{  
    struct termios opt;  
  
    if(tcgetattr(fd,&opt) != 0)                 //或许原先的配置信息   
    {  
        perror("Get opt in parity error:");  
        return -1;  
    }  
  
    /*通过设置opt数据结构，来配置相关功能，以下为八个数据位，不使能奇偶校验*/  
    opt.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP  
                | INLCR | IGNCR | ICRNL | IXON);  
    opt.c_oflag &= ~OPOST;  
    opt.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);  
    opt.c_cflag &= ~(CSIZE | PARENB);  
    opt.c_cflag |= CS8;  

    /*
        switch(nEvent)
    {
        case 'O':
        //the next two lines is to setup the odd-even testing bit
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP );
        break;

        //no testing bit 
        case 'N':   
        newtio.c_cflag &= ~PARENB;
        break;
    }
    */
  
    tcflush(fd,TCIFLUSH);                           //清空输入缓存   
  
    if(tcsetattr(fd,TCSANOW,&opt) != 0)  
    {  
        perror("set attr parity error:");  
        return -1;  
    }  
  
    return 0;  
}  
/*----------------------------------------------------------------------------- 
  函数名:      serial_init 
  参数:        char *dev_path,int speed,int is_block 
  返回值:      初始化成功返回打开的文件描述符 
  描述:        串口初始化，根据串口文件路径名，串口的速度，和串口是否阻塞,block为1表示阻塞 
 *-----------------------------------------------------------------------------*/  
int serial_init(char *dev_path,int speed,int is_block)  
{  
    int fd;  
    int flag;  
  
    flag = 0;  
    flag |= O_RDWR;                     //设置为可读写的串口属性文件   
    if(is_block == 0)  
        flag |=O_NONBLOCK;              //若为0则表示以非阻塞方式打开   
  
    fd = open(dev_path,flag);               //打开设备文件   
    if(fd < 0)  
    {  
        perror("Open device file err:");  
        close(fd);  
        return -1;  
    }  
  
    /*打开设备文件后，下面开始设置波特率*/  
    set_speed(fd,speed);                //考虑到波特率可能被单独设置，所以独立成函数   
  
    /*设置奇偶校验*/  
    if(set_parity(fd) != 0)  
    {  
        perror("set parity error:");  
        close(fd);                      //一定要关闭文件，否则文件一直为打开状态   
        return -1;  
    }  
  
    return fd;  
}  
 
int serial_send(int fd,char *str,unsigned int len)  
{  
    int ret;  
  
    if(len > strlen(str))                    //判断长度是否超过str的最大长度   
        len = strlen(str);  
  
    ret = write(fd,str,len);  
    if(ret < 0)  
    {  
        perror("serial send err:");  
        return -1;  
    }  
  
    return ret;  
}  
  
/*----------------------------------------------------------------------------- 
  函数名:      serial_read 
  参数:        int fd,char *str,unsigned int len,unsigned int timeout 
  返回值:      在规定的时间内读取数据，超时则退出，超时时间为ms级别 
  描述:        向fd描述符的串口接收数据，长度为len，存入str，timeout 为超时时间 
 *-----------------------------------------------------------------------------*/  
int serial_read(int fd, char *str, unsigned int len, unsigned int timeout)  
{  
    fd_set rfds;  
    struct timeval tv;  
    int ret;                                //每次读的结果   
    int sret;                               //select监控结果   
    int readlen = 0;                        //实际读到的字节数   
    char * ptr;  
  
    ptr = str;                          //读指针，每次移动，因为实际读出的长度和传入参数可能存在差异   
  
    FD_ZERO(&rfds);                     //清除文件描述符集合   
    FD_SET(fd,&rfds);                   //将fd加入fds文件描述符，以待下面用select方法监听   
  
    /*传入的timeout是ms级别的单位，这里需要转换为struct timeval 结构的*/  
    tv.tv_sec  = timeout / 1000;  
    tv.tv_usec = (timeout%1000)*1000;  
  
    /*防止读数据长度超过缓冲区*/  
    //if(sizeof(&str) < len)   
    //  len = sizeof(str);   
  
  
    /*开始读*/  
    while(readlen < len)  
    {  
        sret = select(fd+1,&rfds,NULL,NULL,&tv);        //检测串口是否可读   
  
        if(sret == -1)                              //检测失败   
        {  
            perror("select:");  
            break;  
        }  
        else if(sret > 0)                        //检测成功可读   
        {  
            ret = read(fd,ptr,1);  
            if(ret < 0)  
            {  
                perror("read err:");  
                break;  
            }  
            else if(ret == 0)  
                break;  
  
            readlen += ret;                             //更新读的长度   
            ptr     += ret;                             //更新读的位置   
        }  
        else                                                    //超时   
        {  
            //printf("timeout!\n");  
            break;  
        }  
    }  
  
    return readlen;  
} 

int dht11_read_val(){
    uint8_t lststate=HIGH;         //last state
    uint8_t counter=0;
    uint8_t j=0,i;
    for(i=0;i<5;i++)
        dht11_val[i]=0;
         
    //host send start signal    
    pinMode(DHT11PIN,OUTPUT);      //set pin to output 
    digitalWrite(DHT11PIN,LOW);    //set to low at least 18ms 
    delay(18);
    digitalWrite(DHT11PIN,HIGH);   //set to high 20-40us
    delayMicroseconds(40);
     
    //start recieve dht response
    pinMode(DHT11PIN,INPUT);       //set pin to input
    for(i=0;i<MAX_TIME;i++)         
    {
        counter=0;
        while(digitalRead(DHT11PIN)==lststate){     //read pin state to see if dht responsed. if dht always high for 255 + 1 times, break this while circle
            counter++;
            delayMicroseconds(1);
            if(counter==255)
                break;
        }
        lststate=digitalRead(DHT11PIN);             //read current state and store as last state. 
        if(counter==255)                            //if dht always high for 255 + 1 times, break this for circle
            break;
        // top 3 transistions are ignored, maybe aim to wait for dht finish response signal
        if((i>=4)&&(i%2==0)){
            dht11_val[j/8]<<=1;                     //write 1 bit to 0 by moving left (auto add 0)
            if(counter>16)                          //long mean 1
                dht11_val[j/8]|=1;                  //write 1 bit to 1 
            j++;
        }
    }
    // verify checksum and print the verified data
    if((j>=40)&&(dht11_val[4]==((dht11_val[0]+dht11_val[1]+dht11_val[2]+dht11_val[3])& 0xFF))){
        printf("RH:%d,TEMP:%d\n",dht11_val[0],dht11_val[2]);
        return 1;
    }
    else
        return 0;
}

int main(int argc,char* argv)
{


	int fd_HM;  
	int round;  
	int NumBytes;
	unsigned char buf[1024];  
	int checkval;
	int id = 325;
	int incre = 0;
    char buf_rec[1024];

	long fileup = 1;

    int attempts=ATTEMPTS;
    int write_byte = 0;
    char Data[50];
    int fd_ZB; // fd for zigbee module
    time_t t;
    char *currentTime;
    int zigbeeId = 1;

    memset(Data,0,sizeof(Data));

    if(wiringPiSetup()==-1)
        exit(1);
    while(attempts){                        //you have 5 times to retry
        int success = dht11_read_val();     //get result including printing out
        if (success) {                      //if get result, quit program; if not, retry 5 times then quit
            break;
        }
        attempts--;
        delay(2500);
    }

	unsigned char test[20] = {0xfe,0xfe,0xfe,0xfe,0x68,0x20,0x11,0x06,0x29,0x17,0x26,0x32,0x00,0x01,0x03,0x1f,0x90,0x00,0xea,0x16};

	//unsigned char test[20] = {0xfe,0xfe,0xfe,0xfe,
          //                  0x68,0x20,0x43,0x05,0x35,0x17,0x26,0x32,0x00,0x01,0x03,0x1f,0x90,0x00,0x27,0x16};


 	//unsigned char test[28] = {0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,
          //                  0x68,0x20,0x79,0x49,0x13,0x14,0x00,0x11,0x11,0x01,0x03,0x1f,0x90,0x12,0x58,0x16}; 

	fd_HM = open_port(fd_HM,1,"/dev/ttyUSB0");

	if((checkval = set_opt(fd_HM,2400,8,'O',1)) < 0)
	{
		perror("Set_opt failed!\n");
	}
	
    fd_ZB =  serial_init("/dev/ttyUSB1",9600,1);  
    if(fd_ZB < 0)  
    {  
        perror("serial init err:");  
        return -1;  
    }  


	MYSQL my_connection;
        int res;
	char  mysqlInsert[100];
	 memset(mysqlInsert,0,sizeof(mysqlInsert));
	 mysql_init(&my_connection);
        if(!mysql_real_connect(&my_connection,"139.129.28.105","mrbs","1234","HeatMeter",0,NULL,0))
                {
                        printf("Mysql Connection Failed\n");

                }
	

	while(1){

            serial_read(fd_ZB,buf_rec,10,3000);

			NumBytes = write(fd_HM,test,28);
			for(round = 0;round < 8;round++){
			printf("%02x\n",test[round]);
		}
	//printf("Send %d bytes to Plc\n",NumBytes);
	NumBytes = serial_read(fd_HM,buf,1024,500);
	//sleep(1);
	printf("Read %d bytes from Plc:\n",NumBytes);
	for(round = 0;round < NumBytes ;round++){
		printf("%02x\t",buf[round]);

	}
	printf("\n");
	int x2e = 0;
	while((buf[x2e] != 0x2e)){
		++x2e;
	}
	
    if(dht11_read_val()){
            time(&t);
            currentTime = ctime(&t);    
            printf("%s",currentTime);
            sprintf(Data,"%d Hum:%d%sTemp:%d%sCT:%s",zigbeeId,dht11_val[0]," ",dht11_val[2]," ",currentTime);   
            printf("%s\n",Data);
                    serial_send(fd_ZB,Data,50);
            //write(fdLocalRestore,Data,strlen(Data));
            printf("Write %d byte:%s",(int)write_byte,Data);
        }


	
	//printf("Found 0x2e: %d\n",x2e);

	//current cool quantity(kWh)
	int cu_cool_fri_low = (buf[21] % 16) * 10000000;
	int cu_cool_fri_ten = (buf[21] / 16) * 100000000;
	int cu_cool_sec_low = (buf[20] % 16) * 100000;
	int cu_cool_sec_ten = (buf[20] / 16) * 1000000;
	int cu_cool_thr_low = (buf[19] % 16) * 1000;
	int cu_cool_thr_ten = (buf[19] / 16) * 10000;
	int cu_cool_four_low = (buf[18] % 16) * 10;
	int cu_cool_four_ten = (buf[18] / 16) * 100;
	int result = cu_cool_fri_low + cu_cool_fri_ten + cu_cool_sec_low +cu_cool_sec_ten + cu_cool_thr_low + cu_cool_thr_ten +cu_cool_four_low + cu_cool_four_ten;
	double coolquan = (double)(result /1000.00);
	printf("current cool quantity:%.2lf kWh\n",coolquan);

	//current heat quantity(kWh)
	int cu_heat_fri_low = (buf[26] % 16) * 10000000;
        int cu_heat_fri_ten = (buf[26] / 16) * 100000000;
        int cu_heat_sec_low = (buf[25] % 16) * 100000;
        int cu_heat_sec_ten = (buf[25] / 16) * 1000000;
        int cu_heat_thr_low = (buf[24] % 16) * 1000;
        int cu_heat_thr_ten = (buf[24] / 16) * 10000;
        int cu_heat_four_low = (buf[23] % 16) * 10;
        int cu_heat_four_ten = (buf[23] / 16) * 100;
	int result2 = cu_heat_fri_low + cu_heat_fri_ten + cu_heat_sec_low + cu_heat_sec_ten + cu_heat_thr_low + cu_heat_thr_ten +cu_heat_four_low + cu_heat_four_ten;
	double heatquan = (double)(result2 / 1000.000);
	printf("current heat quantity:%.4lf kWh\n",heatquan);

	//heat power(W)
	int heat_fri_low = (buf[31] % 16) * 10000000;
        int heat_fri_ten = (buf[31] / 16) * 100000000;
        int heat_sec_low = (buf[30] % 16) * 100000;
        int heat_sec_ten = (buf[30] / 16) * 1000000;
        int heat_thr_low = (buf[29] % 16) * 1000;
        int heat_thr_ten = (buf[29] / 16) * 10000;
        int heat_four_low = (buf[28] % 16) * 10;
        int heat_four_ten = (buf[28] / 16) * 100;
	int result3 = heat_fri_low + heat_fri_ten + heat_sec_low + heat_sec_ten + heat_thr_low + heat_thr_ten + heat_four_low + heat_four_ten;

	double heatpower = (double)(result3 / 1000.00);
	printf("heat power:%.2lf KW\n",heatpower);

	//instant fluid rate(m3/h)
	int rate_fri_low = (buf[36] % 16) * 10000000;
        int rate_fri_ten = (buf[36] / 16) * 100000000;
        int rate_sec_low = (buf[35] % 16) * 100000;
        int rate_sec_ten = (buf[35] / 16) * 1000000;
        int rate_thr_low = (buf[34] % 16) * 1000;
        int rate_thr_ten = (buf[34] / 16) * 10000;
        int rate_four_low = (buf[33] % 16) * 10;
        int rate_four_ten = (buf[33] / 16) * 100;
	int result4 = rate_fri_low + rate_fri_ten + rate_sec_low + rate_sec_ten + rate_thr_low + rate_thr_ten + rate_four_low + rate_four_ten ;

	double instrate = (double)(result4 / 100000.000);
	printf("instant fluid :%.4lf m3/h\n",instrate);
		
		

	//accum fluid (m3)
	int accum_fri_low = (buf[41] % 16) * 10000000;
        int accum_fri_ten = (buf[41] / 16) * 100000000;
        int accum_sec_low = (buf[40] % 16) * 100000;
        int accum_sec_ten = (buf[40] / 16) * 1000000;
        int accum_thr_low = (buf[39] % 16) * 1000;
        int accum_thr_ten = (buf[39] / 16) * 10000;
        int accum_four_low = (buf[38] % 16) * 10;
        int accum_four_ten = (buf[38] / 16) * 100;
	int result5 = accum_fri_low + accum_fri_ten + accum_sec_low + accum_sec_ten + accum_thr_low + accum_thr_ten + accum_four_low + accum_four_ten;
	double accuflui = (double)(result5 / 1000.000);
	printf("accum fluid:%.2lf m3\n",accuflui);


	int supply_temp_point_low = buf[43] % 16;
	int supply_temp_point_ten = buf[43] / 16; 
	double supply_temp_point = (double)(supply_temp_point_low + supply_temp_point_ten*10) / 100;
	//printf("supply_temp_point_low:%d\n",supply_temp_point_low);
	//printf("supply_temp_point_ten:%d\n",supply_temp_point_ten);	
	//printf("supply_temp_point:%.2lf\n",supply_temp_point);

	int supply_temp_int_low = buf[44] % 16;
	int supply_temp_int_ten = buf[44] / 16;
	double supply_temp_result = (double)supply_temp_int_ten * 10 + (double)supply_temp_int_low + supply_temp_point;
	//printf("%02x\n",buf[52]);

	//printf("%02x\n",buf[53]);
	printf("Supply temperature:%.2lf C\n",supply_temp_result);

	int return_temp_point_low = buf[46] % 16;
        int return_temp_point_ten = buf[46] / 16;
        double return_temp_point = (double)(return_temp_point_low + return_temp_point_ten * 10) / 100;

        int return_temp_int_low = buf[47] % 16;
        int return_temp_int_ten = buf[47] / 16;
        double return_temp_result = (double)(return_temp_int_ten * 10 + return_temp_int_low + return_temp_point);
	//printf("%02x\n",buf[55]);
	printf("Return temperature:%.2lf C\n",return_temp_result);
        //printf("%02x\n",buf[56]);
	sprintf(mysqlInsert, "insert into HeatMeter (id,coquan,heatquan,heatpower,instaflu,accumflu,supptemp,returtemp,Tm) values(%d %s %.2lf %s %.4lf %s %.2lf %s %.4lf  %s %.2lf %s %.2lf %s %.2lf %s %s %s", id,",",coolquan,",",heatquan,",",heatpower,",",instrate,",",accuflui,",",supply_temp_result,",",return_temp_result,",","now())","\n");
	printf("%s\n",mysqlInsert);
    if((buf_rec[0] == 'H') && (buf_rec[1] == 'M')){
        serial_send(fd_ZB,mysqlInsert,200);
    }
	
	//remove("/home/App/query.txt");


	  int  readfile =  open("/home/App/query.txt",O_WRONLY|O_TRUNC|O_CREAT,S_IRWXU);

	++fileup;
	write(readfile,mysqlInsert,strlen(mysqlInsert));

	close(readfile);
	bzero(mysqlInsert,180);
	// const char *sql = (const char *)mysqlInsert;	
	//res = mysql_query(&my_connection,(const char *)sql);

                        if(!res){
                                 printf("Inserted %lu rows\n",(unsigned long)mysql_affected_rows(&my_connection));
					printf("Until now %d data have been collected\n",++incre);
                        //      printf("run 7\n");
                        }else{
                              fprintf(stderr,"Insert error %d : %s \n",mysql_errno(&my_connection),mysql_error(&my_connection));
                        //printf("run 8\n");
                        }




		sleep(3);
	}

	close(fd_HM);  

	return 0;
}
