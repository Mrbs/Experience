#include <stdio.h>
unsigned int CRC16_1(unsigned char *buf, unsigned int length)
{
    unsigned int i;
    unsigned int j;
    unsigned int c;
    unsigned int crc = 0xFFFF;   //设置crc寄存器为0xffff
    for (i=0; i<length; i++)
    {
        c = *(buf+i) & 0x00FF;
        crc^=c;
        for (j=0; j<8; j++)
        {
             if (crc & 0x0001)
             {
                crc >>= 1;
                crc ^= 0xA001;
             }
             else
             { 
                crc >>= 1;
             }
        }
   }
    return(crc);
}

unsigned char test[6] ={0x02,0x03,0x00,0x00,0x00,0x00};
unsigned char RTU_CMD[8];
unsigned char len = 6;

void main( void )
{
	int j;
	unsigned int resl;
	unsigned char *reverse;

		
	resl = CRC16_1(test,6);

	reverse =	(unsigned char *)(&resl);
	printf("0x%x\n",resl);


	printf("0x%x\n",*(reverse));
	printf("0x%x\n",*(reverse + 1));

	for(j=0;j<6;j++){
		RTU_CMD[j] = test[j];
	}
	RTU_CMD[6] = *(reverse);
	RTU_CMD[7] = *(reverse + 1);
	
	for(j=0;j<8;j++){
		printf("0x%x\t",RTU_CMD[j]);
	}
	printf("\n");
}
