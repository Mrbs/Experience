#include <unistd.h>   
#include <stdint.h>
#include <stdio.h>   
#include <termios.h>   
#include <fcntl.h>   
#include <sys/select.h>
#include <sys/time.h>
#include <string.h>   
//#include <mysql/mysql.h>
#include <time.h>   
#include <wiringPi.h>

//为了保证用户输入的波特率是个正确的值，所以需要这两个数组验证，对于设置波特率时候，前面要加个B   
int speed_arr[] = { B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300,  
    B115200, B57600, B38400, B19200, B9600, B4800, B2400, B1200, B300, };  
  
int name_arr[] = {115200, 57600, 38400, 19200, 9600, 4800, 2400, 1200, 300,  
    115200, 57600, 38400, 19200, 9600, 4800, 2400, 1200, 300, };  

int SupplyValve    = 1;
int ReturnValve    = 4;

double LastInstrate;
double CommonePoint;
double Ret_ValveTest;
double Tolerance;


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

            break;  
        }  
    }  
  
    return readlen;  
} 

void SpeculativeDecision(double FR,char *Flag,int *Counter,int *Peak,double *LastInstrate,double *SafeTolerance)
{

    printf("Instant Fluid rate:%0.4lf!\n",FR);  
    Commone_Point(FR,Flag,Counter,Peak,&CommonePoint);
    //double Tolerance        = 0.025;


    if(CommonePoint){
        ReturnValveTest(FR,CommonePoint,*SafeTolerance,LastInstrate);
    }

}

void Commone_Point(double FR,char *Flag,int *Counter,int *Peak,double * retSD)
{
    double foundCommonePoint;

    int FR_Order    = (FR * 10000) -4000;
    int Flag_byte   = FR_Order / 8;
    int Flag_bit    = FR_Order % 8;
    *(Flag + Flag_byte) = 1 << Flag_bit;
    ++*(Counter + FR_Order);

    if(Counter[FR_Order] > *Peak){
        *Peak = Counter[FR_Order];
        foundCommonePoint = (double)(0.4000 + (double)FR_Order / 10000);
        printf("The most occurence:%d times,and value is: %0.4lf\n",
                Counter[FR_Order],foundCommonePoint);
    }

    if((*Peak) < 200) {
        printf("Deactivated! Cause Peak(%d) < 100\n",*Peak);
        *retSD = 0.0000;
    } 
   if((*Peak) > 200) {
        printf("Activated\n");
        *retSD = foundCommonePoint;
        printf("Peak Fluid rate:%0.4lf!\n",*retSD);  
    }   
}

void ReturnValveTest(double Point_Test,double Commone_FR,double TestTolerane,double * retSD)
{
    if(((Point_Test - Commone_FR) > (Commone_FR * TestTolerane))){
        printf("In ReturnValveTest Function\n");
        pinMode(ReturnValve,OUTPUT);
        digitalWrite(ReturnValve,Low);

        printf("Close Return Valve\n");

        printf("Suspeciour Error Fluid rate:%0.4lf!\n",Point_Test);  
        sleep(40);
        //Opps = 1;
        *retSD = Point_Test;
    }

}

int main(int argc,char* argv)
{


	int fd_Plc;  
	int round;  
	int NumBytes;
	unsigned char buf[1024];  
	int checkval;
	int id = 1;
	int incre = 0;
	int counter = 0;
        double Warning = 0.0010;

 	unsigned char test[28] = {0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,0xfe,
                                0x68,0x20,0x79,0x49,0x13,0x14,0x00,0x11,0x11,0x01,0x03,
                                0x1f,0x90,0x12,0x58,0x16}; 

	fd_Plc = open_port(fd_Plc,1,"/dev/ttyUSB1");

	if((checkval = set_opt(fd_Plc,2400,8,'O',1)) < 0)
	{
		perror("Set_opt failed!\n");
	}

	
	if(wiringPiSetup() == -1)
		return 0;
	puts("Setup Done!");

	int pinStateSupplyValve = digitalRead(SupplyValve);
    int pinStateReturnValve = digitalRead(ReturnValve);
	
	pinMode(SupplyValve,OUTPUT);
	pinMode(ReturnValve,OUTPUT);
	digitalWrite(SupplyValve,HIGH);
	digitalWrite(SupplyValve,HIGH);


	

    int res;
	char  mysqlInsert[100];
	 memset(mysqlInsert,0,sizeof(mysqlInsert));

     /*
        The following variables for Speculative Decision
     */
    char    SD_Flag[125];
    int     SD_Counter[1000];
    memset(SD_Counter,0,4000);
    int     SD_Peak = 0;
    int increm = 0;
    Tolerance = 0.025



	while(1){
			NumBytes = write(fd_Plc,test,28);
			for(round = 0;round < 8;round++){
			printf("%02x\n",test[round]);
		}
	//printf("Send %d bytes to Plc\n",NumBytes);
	NumBytes = serial_read(fd_Plc,buf,1024,500);
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
	
	//printf("Found 0x2e: %d\n",x2e);

	//current cool quantity(kWh)
	int cu_cool_fri_low = (buf[30] % 16) * 10000000;
	int cu_cool_fri_ten = (buf[30] / 16) * 100000000;
	int cu_cool_sec_low = (buf[29] % 16) * 100000;
	int cu_cool_sec_ten = (buf[29] / 16) * 1000000;
	int cu_cool_thr_low = (buf[28] % 16) * 1000;
	int cu_cool_thr_ten = (buf[28] / 16) * 10000;
	int cu_cool_four_low = (buf[27] % 16) * 10;
	int cu_cool_four_ten = (buf[27] / 16) * 100;
	int result = cu_cool_fri_low + cu_cool_fri_ten + 
                    cu_cool_sec_low +cu_cool_sec_ten + cu_cool_thr_low + 
                    cu_cool_thr_ten +cu_cool_four_low + cu_cool_four_ten;
	double coolquan = (double)(result /1000.00);
	printf("current cool quantity:%.2lf kWh\n",coolquan);

	//current heat quantity(kWh)
	int cu_heat_fri_low = (buf[35] % 16) * 10000000;
        int cu_heat_fri_ten = (buf[35] / 16) * 100000000;
        int cu_heat_sec_low = (buf[34] % 16) * 100000;
        int cu_heat_sec_ten = (buf[34] / 16) * 1000000;
        int cu_heat_thr_low = (buf[33] % 16) * 1000;
        int cu_heat_thr_ten = (buf[33] / 16) * 10000;
        int cu_heat_four_low = (buf[32] % 16) * 10;
        int cu_heat_four_ten = (buf[32] / 16) * 100;
	int result2 = cu_heat_fri_low + cu_heat_fri_ten + cu_heat_sec_low + 
                    cu_heat_sec_ten + cu_heat_thr_low + cu_heat_thr_ten +cu_heat_four_low + 
                    cu_heat_four_ten;
	double heatquan = (double)(result2 / 1000.000);
	printf("current heat quantity:%.4lf kWh\n",heatquan);

	//heat power(W)
	int heat_fri_low = (buf[40] % 16) * 10000000;
        int heat_fri_ten = (buf[40] / 16) * 100000000;
        int heat_sec_low = (buf[39] % 16) * 100000;
        int heat_sec_ten = (buf[39] / 16) * 1000000;
        int heat_thr_low = (buf[38] % 16) * 1000;
        int heat_thr_ten = (buf[38] / 16) * 10000;
        int heat_four_low = (buf[37] % 16) * 10;
        int heat_four_ten = (buf[37] / 16) * 100;
	int result3 = heat_fri_low + heat_fri_ten + heat_sec_low + heat_sec_ten + 
                    heat_thr_low + heat_thr_ten + heat_four_low + heat_four_ten;

	double heatpower = (double)(result3 / 1000.00);
	printf("heat power:%.2lf KW\n",heatpower);

	//instant fluid rate(m3/h)
	int rate_fri_low = (buf[45] % 16) * 10000000;
        int rate_fri_ten = (buf[45] / 16) * 100000000;
        int rate_sec_low = (buf[44] % 16) * 100000;
        int rate_sec_ten = (buf[44] / 16) * 1000000;
        int rate_thr_low = (buf[43] % 16) * 1000;
        int rate_thr_ten = (buf[43] / 16) * 10000;
        int rate_four_low = (buf[42] % 16) * 10;
        int rate_four_ten = (buf[42] / 16) * 100;
	int result4 = rate_fri_low + rate_fri_ten + rate_sec_low + 
                    rate_sec_ten + rate_thr_low + rate_thr_ten + rate_four_low + rate_four_ten ;

	double instrate = (double)(result4 / 100000.000);
	printf("instant fluid :%.4lf m3/h\n",instrate);
		
		

	//accum fluid (m3)
	int accum_fri_low = (buf[50] % 16) * 10000000;
        int accum_fri_ten = (buf[50] / 16) * 100000000;
        int accum_sec_low = (buf[49] % 16) * 100000;
        int accum_sec_ten = (buf[49] / 16) * 1000000;
        int accum_thr_low = (buf[48] % 16) * 1000;
        int accum_thr_ten = (buf[48] / 16) * 10000;
        int accum_four_low = (buf[47] % 16) * 10;
        int accum_four_ten = (buf[47] / 16) * 100;
	int result5 = accum_fri_low + accum_fri_ten + accum_sec_low + accum_sec_ten + 
                        accum_thr_low + accum_thr_ten + accum_four_low + accum_four_ten;
	double accuflui = (double)(result5 / 1000.000);
	printf("accum fluid:%.2lf m3\n",accuflui);


	int supply_temp_point_low = buf[52] % 16;
	int supply_temp_point_ten = buf[52] / 16; 
	double supply_temp_point = (double)(supply_temp_point_low + supply_temp_point_ten*10) / 100;
	//printf("supply_temp_point_low:%d\n",supply_temp_point_low);
	//printf("supply_temp_point_ten:%d\n",supply_temp_point_ten);	
	//printf("supply_temp_point:%.2lf\n",supply_temp_point);

	int supply_temp_int_low = buf[53] % 16;
	int supply_temp_int_ten = buf[53] / 16;
	double supply_temp_result = (double)supply_temp_int_ten * 10 + 
                                (double)supply_temp_int_low + supply_temp_point;
	//printf("%02x\n",buf[52]);

	//printf("%02x\n",buf[53]);
	printf("Supply temperature:%.2lf C\n",supply_temp_result);

	int return_temp_point_low = buf[55] % 16;
        int return_temp_point_ten = buf[55] / 16;
        double return_temp_point = (double)(supply_temp_point_low + 
                                    supply_temp_point_ten * 10) / 100;

        int return_temp_int_low = buf[56] % 16;
        int return_temp_int_ten = buf[56] / 16;
        double return_temp_result = (double)(return_temp_int_ten * 10 + 
                                    return_temp_int_low + return_temp_point);
	//printf("%02x\n",buf[55]);
	printf("Return temperature:%.2lf C\n",return_temp_result);
        //printf("%02x\n",buf[56]);
	/*
	sprintf(mysqlInsert, "insert into HeatMeter (id,coquan,heatquan,heatpower,
            instaflu,accumflu,supptemp,returtemp,Tm) 
            values(%d %s %.2lf %s %.4lf %s %.2lf %s %.4lf  %s %.2lf %s %.2lf %s %.2lf %s %s", id,",",
            coolquan,",",heatquan,",",heatpower,",",instrate,",",accuflui,",",supply_temp_result,",",
	 res = mysql_query(&my_connection,mysqlInsert);

                        if(!res){

                                    (unsigned long)mysql_affected_rows(&my_connection));
					printf("Until now %d data have been collected\n",++incre);
                        //      printf("run 7\n");
                        }else{
                              fprintf(stderr,"Insert error %d : %s \n",
                              mysql_errno(&my_connection),mysql_error(&my_connection));

                        }
	*/
                                 printf("Now run %d times\n",++increm);
    int RetSD;
    LastInstrate = 0.0000;

        printf("Last Instant before SpeculativeDecisoin fluid %0.4lf\n",LastInstrate);
    if(instrate > 0.4000){
        SpeculativeDecision(instrate,SD_Flag,SD_Counter,&SD_Peak,&LastInstrate,&Tolerance);
    }

    if((LastInstrate) && (instrate == 0.0000)) {
        pinMode(ReturnValve,OUTPUT);
        digitalWrite(ReturnValve,HIGH);
        printf("Open Return Valve\n");


        printf("Instant fluid after speculativeDecisoin %0.4lf\n",LastInstrate);
        printf("Expanding tolerance should be by 0.005\n");
        Tolerance += 0.005;
    }

    if((LastInstrate) && (instrate > 0.1000)){
        pinMode(SupplyValve,OUTPUT);
        digitalWrite(SupplyValve,LOW);
        printf("Close Supply Valve \n");
        printf("Instant fluid after speculativeDecisoin %0.4lf\n",LastInstrate);
        printf("Catchya\n");
        printf("Reducing tolerance should be by 0.005\n");
        Tolerance -= 0.005;
    }
	//	sleep(2);
	}

	close(fd_Plc);  

	return 0;
}
