#include <stdio.h>
#include <stdlib.h>
#include <mysql/mysql.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <wiringPi.h>

#define DURATION        825000000
#define VAL_DESC        2 
#define VAL_INCR        3
long    counter;
float   Punishment      = 0.0700;

int main (void)
{
        char *begin = "\n------------------Begin----------------------";
        printf("%s\n",begin);
	char *Quot_RagingBUll = "Whether he is sinner,I do not know. All I know is once I was blind, now I can see.\n";
	printf("%s",Quot_RagingBUll);

        char ID[16];
        char IFR[16];
        char HP[16];
        char IFR_RT[16];

        int     dID;
        float   dIFR;
        float   dHP;
        float   dIFR_RT;

        float   Coor_Rate = 0.0000;
        long    leap = 0;

        bzero(ID,16);
        bzero(IFR,16);
        bzero(HP,16);
        bzero(IFR_RT,16);

        MYSQL           *conn;
        MYSQL_RES       *res;
        MYSQL_ROW       row;

        char            *server         = "139.129.28.105";
        char            *user           = "mrbs";
        char            *passwd         = "1234";
        char            *database       = "HeatMeter";


        conn    =       mysql_init(NULL);

        if(!mysql_real_connect(conn,server,user,passwd,database,0,NULL,CLIENT_MULTI_RESULTS)) {
                fprintf(stderr,"%s\n",mysql_error(conn));
                exit(1);
        }

        char            *query_critia = "select * from critia1 order by tm desc limit 0,1 ";
        char            *query_instaflu = "select instaflu  from HeatMeter where id = 1 order by Tm desc limit 0,1";


        if(wiringPiSetup() == -1)
                exit(1);

        puts("Setup Done!");

        int pinState = digitalRead(VAL_DESC);
            pinState = digitalRead(VAL_INCR);


        while(1){
                if(mysql_query(conn,query_critia)){
                        fprintf(stderr,"%s\n",mysql_error);
                        exit(1);
                }

                res = mysql_use_result(conn);
                printf("\n");

                while((row = mysql_fetch_row(res)) != NULL){

                        printf("Id: %s \n",row[0]);
                        strcpy(ID,row[0]);
                        printf("Specified Instant Fluid Rate: %s \n",row[1]);
                        strcpy(IFR,row[1]);
                        printf("Specified Heat Power: %s \n",row[2]);
                        strcpy(HP,row[2]);
                }

                printf("ID:%s\n",ID);
                printf("FIR:%s\n",IFR);
                printf("HP:%s\n",HP);

                dID     = atoi(ID);
                dIFR    = atof(IFR);
                dHP     = atof(HP);

                printf("dID:%d\n",dID);
                printf("dIFR:%.4lf\n",dIFR);
                printf("dHP:%.4lf\n",dHP);
                sleep(3);

                if(mysql_query(conn,query_instaflu)){
                fprintf(stderr,"%s\n",mysql_error);
                exit(1);
                }

                res = mysql_use_result(conn);
                printf("\n");

                while((row = mysql_fetch_row(res)) != NULL){
                        printf("Realtime instant fluid rate: %s \n",row[0]);
                        strcpy(IFR_RT,row[0]);
                }

                dIFR_RT = atof(IFR_RT);
                printf("IFR_RT:%.4lf\n",dIFR_RT);
                sleep(3);

                Coor_Rate = (dIFR_RT - dIFR) / dIFR ;

                if(Coor_Rate > 0.0800){
                        pinMode(VAL_DESC,OUTPUT);
                        digitalWrite(VAL_DESC,HIGH);

                        leap = DURATION * (Coor_Rate - Punishment);

                        for(counter = 0;counter < leap;counter++)
                        {
                                //Do nothing just looping
                        }
                        digitalWrite(VAL_DESC,LOW);
                        sleep(300);
                }
		if(Coor_Rate < -0.0500){
			pinMode(VAL_INCR,OUTPUT);
			digitalWrite(VAL_INCR,HIGH);	
			leap = DURATION * 0.03;
			for(counter = 0;counter < leap;counter++)
			{
				// Do nothing just looping
			}
			digitalWrite(VAL_INCR,LOW);
			sleep(240);
		}
        }
        mysql_free_result(res);

        mysql_close(conn);

        return 0;
}
