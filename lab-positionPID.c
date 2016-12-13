#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "mraa/pwm.h"
#include "mraa/gpio.h"
#include "cJSON.h"

#include <fcntl.h>
#include <curl/curl.h>

#include <sys/ipc.h>
#include <sys/shm.h>
#include <error.h>
#include <ctype.h>
#include <limits.h>

#include <net/if.h>

#include <signal.h>

#define Resolution 1000
#define Timeout 1
#define SHAREBUFFSIZE 40*1024

static volatile int counter = 0;
static volatile int counter1 = 0;
static volatile int dir_ct = 0;
static volatile int Dir = 1;
static volatile int flag1 = 0;
static volatile int flag2 = 0;
static  float n = 0.0;
static  const float fullspeed = 60.0;

struct __pid{
	float SetSpeed;			//设定转速值
	float ActualSpeed;		//实际转速值
	float err;				//定义偏差值
	float err_last;			//上一个偏差值
	float Kp,Ki,Kd;			//比例，积分，微分系数
	float voltage;			//控制电机的电压值
	float integral;			//积分值
}pid;

void PID_init()				//PID初始化（位置式PID）
{
	printf("PID_init begin...\n");
	pid.SetSpeed = 0.0;
	pid.ActualSpeed = 0.0;
	pid.err = 0.0;
	pid.err_last = 0.0;
	pid.Kp = 0.4;
	pid.Ki = 0.4;
	pid.Kd = 0.0;
	pid.integral = 0.0;
	printf("PID_init end...\n");
}

float PID_realize(float speed)	//PID实现
{
	pid.SetSpeed = speed;
	pid.ActualSpeed = n;
	pid.err = pid.SetSpeed - pid.ActualSpeed;
	pid.integral += pid.err;
	pid.voltage = pid.Kp*pid.err + pid.Ki*pid.integral + pid.Kd*(pid.err - pid.err_last);
	pid.err_last = pid.err;
	return pid.voltage;
}

/*float PID_realize(float speed)	//PID实现
{
	pid.SetSpeed = speed;
	pid.ActualSpeed = n;
	pid.err = pid.SetSpeed - pid.ActualSpeed;
	pid.integral += pid.err;
	pid.voltage = pid.Kp*pid.err + pid.Ki*pid.integral + pid.Kd*(pid.err - pid.err_last);
	pid.err_last = pid.err;
	pid.ActualSpeed = pid.voltage * 1.0;
	return pid.ActualSpeed;
}	*/


void parseJson(char * text, char result[3][10])	{										//JSON解析函数
	cJSON * root = cJSON_Parse ( text ) ;
	if ( ! root ) {		//捕捉异常
		printf ( "Error before: [%s]\n" , cJSON_GetErrorPtr ( ) ) ;
		return ;
	}
	char * out = cJSON_Print ( root ) ;																//捕捉字符串
	//printf ( "text:\n%s\n\n" , out ) ;																//打印字符串
	char* P = cJSON_GetObjectItem ( root , "P" ) -> valuestring;			//解析P
	//printf ( "The value of P is : %s\n" , P ) ;												//打印P对应的值
	char* I = cJSON_GetObjectItem ( root , "I" ) ->valuestring;  			//解析I
	//printf ( "The value of I is : %s\n" , I ) ;												//打印I对应的值
	char* D = cJSON_GetObjectItem ( root , "D" ) ->valuestring;  			//解析D
	//printf ( "The value of D is : %s\n" , D ) ;												//打印D对应的值
	strcpy(result[0], P);
	strcpy(result[1], I); 
	strcpy(result[2], D);  
	cJSON_Delete ( root ) ;        																		//删除JSON结构体
	return ;	
}

static size_t read_callback(void *ptr, size_t size, size_t nmemb, void *stream)
{
	size_t retcode;
	curl_off_t nread;
	retcode = fread(ptr, size, nmemb, stream);
	nread = (curl_off_t)retcode;
	fprintf(stderr, "*** We read %d" CURL_FORMAT_CURL_OFF_T" bytes from file\n", nread);
	return retcode;
}

char * getLocaltime(void)
{
	char *wday[]={"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
	char *standard_time;
	standard_time = (char *)malloc(100);
	time_t timep;
	struct tm *p;
	time(&timep);
	p = localtime(&timep);  /*取得当地时间 */
	printf("%d%d%d\n",(1900+p->tm_year),(1+p->tm_mon),p->tm_mday);
	sprintf(standard_time,"%d-%d-%d%s%d-%d-%d",(1900+p->tm_year),(1+p->tm_mon),p->tm_mday,wday[p->tm_wday],p->tm_hour,p->tm_min,p->tm_sec);
	printf("%s\n",standard_time);
	return standard_time;
}

void uploadFile(char* file_name)
{
	CURL *curl;
	CURLcode res;
	FILE * hd_src;
	struct stat file_info;
 
	char file[20]; 			//此处用char*有Bug，用数组杠杠的
	strcpy(file,file_name);
	printf("File name is: %s.\n",file);
	char *url;

	url = "http://192.168.1.156/put-bin.php";
	//strcpy(url,phpURL);
	stat(file, &file_info);      //得到本地文件的大小

	/*得到一个指向相同文件的文件指针，也可以用file *再调用一次fopen()实现，这里仅仅是个例子*/
	hd_src = fopen(file, "rb");        

	curl_global_init(CURL_GLOBAL_ALL);    //在windows中，会将winsock初始化

	curl = curl_easy_init();        //得到一个curl句柄
	if(curl) {
		curl_easy_setopt(curl, CURLOPT_READFUNCTION, read_callback);    //使用自己的回调函数

		curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);     //使能上传

		curl_easy_setopt(curl, CURLOPT_PUT, 1L);       //设定HTTP PUT请求方式

		curl_easy_setopt(curl, CURLOPT_URL, url);       //确定目的URL，URL必须包含文件名，而不仅仅是路径名

		curl_easy_setopt(curl, CURLOPT_READDATA, hd_src);  //选择上传的文件
		/* 提供上传文件大小信息；为了保证正确的文件大小，必须将其转换为curl_off_t格式 */ 
		curl_easy_setopt(curl, CURLOPT_INFILESIZE_LARGE,(curl_off_t)file_info.st_size);

		res = curl_easy_perform(curl);   //开始上传操作！

		if(res != CURLE_OK)       //检察上传过程是否出错
			fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
 
		curl_easy_cleanup(curl);   //记得使用完以后清除curl句柄
	}
	
	fclose(hd_src);             //关闭本地文件

	curl_global_cleanup();
}

void uploadStat(char* device_stat)
{
	CURL *curl;
	CURLcode res;
	FILE * hd_src;
	struct stat file_info;
 
	char *file = "device_stat.txt";
	char *url;

	hd_src = fopen(file,"w+");
	fprintf(hd_src,"%s",device_stat);
	fclose(hd_src);

	url = "http://192.168.1.156/put-stat.php";
	//strcpy(url,phpURL);
	stat(file, &file_info);      //得到本地文件的大小

	/*得到一个指向相同文件的文件指针，也可以用file *再调用一次fopen()实现，这里仅仅是个例子*/
	hd_src = fopen(file, "rb");        

	curl_global_init(CURL_GLOBAL_ALL);    //在windows中，会将winsock初始化

	curl = curl_easy_init();        //得到一个curl句柄
	if(curl) {
		curl_easy_setopt(curl, CURLOPT_READFUNCTION, read_callback);    //使用自己的回调函数
 
		curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);     //使能上传
 
		curl_easy_setopt(curl, CURLOPT_PUT, 1L);       //设定HTTP PUT请求方式
 
		curl_easy_setopt(curl, CURLOPT_URL, url);       //确定目的URL，URL必须包含文件名，而不仅仅是路径名
 
		curl_easy_setopt(curl, CURLOPT_READDATA, hd_src);  //选择上传的文件
		/* 提供上传文件大小信息；为了保证正确的文件大小，必须将其转换为curl_off_t格式 */ 
		curl_easy_setopt(curl, CURLOPT_INFILESIZE_LARGE,(curl_off_t)file_info.st_size);
 
		res = curl_easy_perform(curl);   //开始上传操作！
	
		if(res != CURLE_OK)       //检察上传过程是否出错
			fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res));
 
		curl_easy_cleanup(curl);   //记得使用完以后清除curl句柄
	}
	
	fclose(hd_src);             //关闭本地文件

	curl_global_cleanup();
}

void my_handler(void)
{
	uploadStat("0");
	exit(1);
}

void *thread_fuction(void *arg)
{
    printf("新线程正在运行...\n");

	mraa_init();
	mraa_pwm_context pwm1;
	mraa_gpio_context IN1,IN2;
   // mraa_result_t r1 = MRAA_SUCCESS, r2 = MRAA_SUCCESS;
    pwm1 = mraa_pwm_init(3);
    IN1 = mraa_gpio_init(2);
    IN2 = mraa_gpio_init(4);
	
	mraa_gpio_dir(IN1, MRAA_GPIO_OUT);
    mraa_gpio_dir(IN2, MRAA_GPIO_OUT);
	
	mraa_gpio_write(IN1, 1);
    mraa_gpio_write(IN2, 1);
    
    mraa_pwm_period_us(pwm1, 200);		//设置PWM周期（us为单位）
    mraa_pwm_enable(pwm1, 1);
	printf("新线程端口设置完毕！\n");
	printf("开始执行PID控制！\n");
	PID_init();
	mraa_gpio_write(IN1, 1);
	mraa_gpio_write(IN2, 0);
	float value = 0.0;

	int i;
	i = 0;

/*	while(i<1000){
		float speed = PID_realize(40.0);
		n = speed;
		printf("The PID-adjusted speed is %f\n.",speed);
		i++;
		sleep(1);
	}		*/	
	
	float sum = 0;
	float ave = 0;
	float set_speed = 40.0;

	for(i=0;i<1000;i++)			//PID控制测试代码
	{
		//if(counter > 1500) 
				//counter /= 2;

		float speed = PID_realize(set_speed);
		//printf("The actual speed is %f.\n",n);
//		if(speed < 0) speed = -speed;
		printf("The adjusted speed is %f.\n",speed);
		//if(speed < 40.0) speed = 0.4615 * speed + 21.217;
		//else if(speed > 40.0) speed = 0.8394 * speed + 5.7438;
		//value = fabs(speed) / fullspeed;
		//if(value < 0.6) value = 0.6;
		//speed = 0.0077*speed*speed + 0.4825*speed - 0.87;

/*		if(speed > 66.0) speed = 66.0;
		else if(speed > 63.5) speed = 2.4*speed - 92.4;
		else if(speed > 44) speed = 1.2722*speed - 21.091;
		else if(speed > 12) speed = 0.9625*speed - 7.3637;
		else if(speed > 0) speed = 0.4145*speed - 0.125;		

		if(speed > 60.0) speed = 60.0;
		else if(speed > 59) speed = 5*speed - 240;
		else if(speed > 30.5) speed = 1.015*speed - 5.3113;
		else if(speed > 27.0) speed = 1.6667*speed - 25.833;
		else if(speed > 26.0) speed = 6.4286*speed - 157.5;
		else speed = 10;																																			

		if(speed > 64.21) speed = 60.0;
		else if(speed > 62.46) speed = 2.8751*speed - 123.46;
		else if(speed > 46.03) speed = 0.9227*speed - 2.9689;
		else if(speed > 29.03) speed = 0.8804*speed - 0.5448;
		else if(speed > 12.5) speed = 0.9127*speed - 1.3677;
		else if(speed > 2.0) speed = 0.9453*speed - 1.8855;
		else speed = 0.0;																				*/

		if(speed > 64.0) speed = 60.0;
		else if(speed > 58.28) speed = 0.8394*speed + 5.8899;
		else if(speed > 39.56) speed = 0.7954*speed + 6.6154;
		else if(speed > 20.87) speed = 1.0619*speed - 4.2124;
		else if(speed > 9.07) speed = 1.2688*speed - 6.6922;
		else if(speed > 4.27) speed = 1.0868*speed - 4.9579;
		else speed = 0.0;	

		value = speed/60.0;
		//if(i <= 20) mraa_pwm_write(pwm1,35.0/60.0);
		//else       
				mraa_pwm_write(pwm1,value);

		counter = 0;
		dir_ct = 0;
		sleep(1);

		counter /= 1.5;
		n = (float)(60*counter )/ ((float) (Resolution*Timeout));
		//sum += n;
		//ave = sum/(i+1);
		printf("编码器轴的转速为: %.2f r/min.   counter值为: %d \n",n,counter);
    //printf("编码器轴的转速为: %.2f r/min.   平均转速值为: %.2f r/min.  counter值为: %d \n",n,ave,counter);
		
	}


/*	while (1) {
        mraa_gpio_write(IN1, 0);
        mraa_gpio_write(IN2, 1);
		sleep(2);

		value = 0.0f;
        for(i=0;i<100;i++)
        {
            value = value + 0.01f;
            mraa_pwm_write(pwm1, value);
            usleep(40000);
        }
		value = 1.0f;
        mraa_pwm_write(pwm1, value);
        sleep(4);
    }  */
    
	mraa_pwm_write(pwm1, 0);
    mraa_gpio_write(IN1, 0);
    sleep(2);
    
	mraa_gpio_close(IN1);
    mraa_gpio_close(IN2);
    pthread_exit("线程执行完毕!\n");
}

void
interrupt_A1(void* args)
{
/*	if(flag1 >= 4){
		flag1 = 0;	++counter; dir_ct++; 
	}
	if(flag2 >= 4){
		flag2 = 0;	++counter; dir_ct--;
	}
	if(flag1 < 4 && mraa_gpio_read(args)==1) {
		++flag1;
	}
	if(flag2 < 4 && mraa_gpio_read(args)==0) {
		++flag2;
	}
	//printf("counter is: %d\n",counter);		*/
    ++counter;
/*    if( mraa_gpio_read(args)==1)
        ++dir_ct;
    else if( mraa_gpio_read(args)==0) 
        --dir_ct;												*/
}
/*
void
interrupt_B1(void* args)
{
	if(flag1 >= 4){
		flag1 = 0;	++counter; dir_ct++; 
	}
	if(flag2 >= 4){
		flag2 = 0;	++counter; dir_ct--;
	}
	if(flag1 < 4 && mraa_gpio_read(args)==0) {
		++flag1;
	}
	if(flag2 < 4 && mraa_gpio_read(args)==1) {
		++flag2;
	}
}

void
interrupt_A2(void* args)
{
	if(flag1 >= 4){
		flag1 = 0;	++counter; dir_ct++; 
	}
	if(flag2 >= 4){
		flag2 = 0;	++counter; dir_ct--;
	}
	if(flag1 < 4 && mraa_gpio_read(args)==0) {
		++flag1;
	}
	if(flag2 < 4 && mraa_gpio_read(args)==1) {
		++flag2;
	}
}

void
interrupt_B2(void* args)
{
	if(flag1 >= 4){
		flag1 = 0;	++counter; dir_ct++; 
	}
	if(flag2 >= 4){
		flag2 = 0;	++counter; dir_ct--;
	}
	if(flag1 < 4 && mraa_gpio_read(args)==1) {
		++flag1;
	}
	if(flag2 < 4 && mraa_gpio_read(args)==0) {
		++flag2;
	}
}		*/

int main(int argc,char *argv[])
{
	int shmid ;
	char *shmaddr ;
	struct shmid_ds buf ;
	struct sigaction sigIntHandler;
  
	sigIntHandler.sa_handler = my_handler;  
	sigemptyset(&sigIntHandler.sa_mask);  
	sigIntHandler.sa_flags = 0;  
  
	sigaction(SIGINT, &sigIntHandler, NULL);

	shmid = shmget(IPC_PRIVATE, SHAREBUFFSIZE, IPC_CREAT|0600 ) ;
	if ( shmid < 0 )
	{
		perror("get shm  ipc_id error") ;
		return -1 ;
	}

	int res;               //用于保存创建线程的返回值
	pthread_t a_thread;    //用于保存线程标识符等信息
	void *thread_result;    //用于保存线程结束时的返回值

	mraa_init();
    
	pid_t pid;
    //! [Interesting]
    
	mraa_gpio_context A,B;
	A = mraa_gpio_init(8);
	B = mraa_gpio_init(7);
    
	mraa_gpio_dir(A, MRAA_GPIO_IN);
	mraa_gpio_dir(B, MRAA_GPIO_IN);

	gpio_edge_t edge1 = MRAA_GPIO_EDGE_FALLING;		//设定边沿触发方式
	gpio_edge_t edge2 = MRAA_GPIO_EDGE_RISING;

	mraa_gpio_isr(A, edge2, &interrupt_A1, B);		//设定边沿触发中断处理子函数入口
/*	mraa_gpio_isr(B, edge1, &interrupt_B1, A);
	mraa_gpio_isr(A, edge2, &interrupt_A2, B);
	mraa_gpio_isr(B, edge2, &interrupt_B2, A);		*/
	printf("端口设置完毕！\n");
	
	uploadStat("1"); 		//设备状态置为1（被占用）

	int server_sockfd,client_sockfd;
	int server_len,client_len;int opt=SO_REUSEADDR;
	struct sockaddr_in server_address;
	struct sockaddr_in client_address;
	server_sockfd=socket(PF_INET,SOCK_STREAM,IPPROTO_TCP);
	setsockopt(server_sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
	server_address.sin_family=AF_INET;
	server_address.sin_addr.s_addr=inet_addr("192.168.1.248");
	server_address.sin_port=htons(9090);
	server_len=sizeof(server_address);
	bind(server_sockfd,(struct sockaddr*) &server_address,server_len);
	listen(server_sockfd,5);

	res = pthread_create(&a_thread, NULL, thread_fuction, NULL);    //创建线程
	if( res != 0)
	{
		perror("线程创建失败");
		exit(EXIT_FAILURE);
	}

	char ch[50],zh[600];
	int i,j,len;
	const float PI=3.141593;

	if((pid=fork())>0)		//实时通信进程
	{
		shmaddr = (char *) shmat(shmid, NULL, 0 ) ;
		if ( (int)shmaddr == -1 )
		{
			perror("shmat addr error") ;
			return -1 ;
		}
		float *t=(float *)shmaddr;
		while(1)
		{
			char PID[3][10];
			memset(PID, 0, sizeof(PID));
			printf("服务器等待消息\n");
			client_len = sizeof(client_address);
			client_sockfd=accept(server_sockfd,(struct sockaddr*) &client_address,(socklen_t *__restrict) &client_len);				 
				
			read(client_sockfd, &ch, 45);
			parseJson(ch, PID);

			//printf("The value of P from Android is: %s.\n",PID[0]);
			//printf("The value of I from Android is: %s.\n",PID[1]);
			//printf("The value of D from Android is: %s.\n",PID[2]);

			//printf("从客户端接收的字符是:%c....\n",ch[0]);

/*			if(ch[0]=='E')			//收到结束字符即退出实验进程
			{	
				printf("已接收到结束字符！\n");
				close(client_sockfd);
				uploadStat("0");		//设备状态置0
				kill(pid,SIGKILL);		//关闭子进程
				exit(0);				//主进程退出
			}		*/
			write(client_sockfd, &n, sizeof(float));	
			//close(pipefd[0]); write(pipefd[1], ch, strlen(ch));   //管道向子进程写入数据
			//printf("The value is %f,%f,%f\n",t[0],t[1],t[2]);
			//send(client_sockfd,shmaddr,sizeof(float),0);
			close(client_sockfd);
			//system("uptime");
			//wait(NULL);
		}
		shmdt( shmaddr );
	}

	else if (pid==0) 	
	{
		pid_t pid_cld;
		shmaddr = (char *)shmat( shmid, NULL, 0 ) ;
		if ( (int)shmaddr == -1 )
		{
			perror("shmat addr error") ;
			return -1 ;
		}

		int pipefd[2];						//管道描述符
		char num_buf[5];					//管道读取缓冲区
		if(pipe(pipefd) == -1){
			perror("创建管道出错！");
			exit(1);
		}

		float *s=(float *)shmaddr;
		int fileflag = O_CREAT|O_RDWR;
		int fd;
		int i;
		int flag = 0;
		if((pid_cld=fork())>0){			//获取转速和转向信号进程
		while(1)
		{
			//close(pipefd[1]); read(pipefd[0], &aio_choose, sizeof(char));
			fd = open("lab-temp.bin",fileflag,0644);	//建立临时文件
printf("counter1 is:%d\n",counter);
			for(i=0;i<60;i++)							//一次上传1min的数据
			{
				sleep(Timeout);
/*       			if(dir_ct >= 0)
           			Dir = 1;
        		else 
            		Dir = -1;
        		n = 60*counter*Dir / (((float) Resolution)*Timeout);
				//if(n>=30.0 || n<=-30.0)  n = 0.0;
        		printf("编码器轴的转速为: %.2f r/min.   dir_ct值为: %d \n",n,dir_ct);
        		counter = 0;
        		dir_ct = 0;		*/
				s[0] = n;							//子进程数据更新到共享内存
				write( fd, &n, sizeof(float));			//数据写入文件	
			}
			close(fd);
			//uploadFile("temp.bin");
			sprintf(num_buf,"%d",flag);
			write(pipefd[1],num_buf,strlen(num_buf));		//循环的次数写入管道
			if(flag) 
				flag = 0;
			else
				flag = 1;
		}
		shmdt( shmaddr );
		exit(0);
	}
	else if(pid_cld == 0){				//历史数据保存进程
		int len;
		char serial[5] = "1",temp[5] = "1";
		do
		{
			//close(pipefd[1]);
			len = read(pipefd[0],num_buf,sizeof(num_buf));			//从管道读出flag
			num_buf[len] = '\0';
			strcpy(serial, num_buf);
			printf("三级子进程第一次读出的次数是%s.\n",num_buf);
			sleep(1);
		}while(strcmp(serial,temp) == 0);
		while(1)
		{
			while( strcmp(serial,temp) == 0){
				//close(pipefd[1]);
				len = read(pipefd[0],num_buf,sizeof(num_buf));			//从管道读出flag
				num_buf[len] = '\0';
				strcpy(serial, num_buf);
				printf("三级子进程读出的次数是%s.\n",num_buf);
				sleep(1);
			}
			strcpy(temp, serial);
			printf("开始上传数据！\n");
			uploadFile("lab-temp.bin");				//上传实验文件
			sleep(1);				
		}				
	}
	}
}
