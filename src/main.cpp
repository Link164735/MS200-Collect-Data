#include "ord_lidar_driver.h"
#include "filters/FullScanFilter.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <csignal>
#include <unistd.h> // for delay function on POSIX systems
#include <open3d/Open3D.h>
#include <fstream>
#include <ctime>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <termio.h>
#include <time.h>
#include <pthread.h>

using namespace std;
using namespace ordlidar;
using namespace open3d;

int read_data(int fd, void *buf, int len);
int write_data(int fd, void *buf, int len);
int setup_port(int fd, int baud, int databits, int parity, int stopbits);
void print_usage(char *program_name);

pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t data_ready = PTHREAD_COND_INITIALIZER;
int data_available = 0;

int flag = 0;
int flag_B = 0;
int flag_A = 0;

// Global variables
volatile sig_atomic_t running = 1;

std::string GetCurrentDateTimeAsString() {
    std::time_t now = std::time(nullptr);
    std::tm localTime = *std::localtime(&now);
    
    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &localTime); // Format: YYYYMMDD_HHMMSS
    
    return buffer;
}

// Point struct to hold lidar data
struct Point {
    int index;
    float distance;
    float angle;
    Point(int _index, float _distance, float _angle) : index(_index), distance(_distance), angle(_angle) {}
};

// Signal handler for SIGINT
static void sig_handle(int signo) {
    printf("Program exit, [%s,%s] Received SIGNAL %d\n", __FILE__, __func__, signo);
    running = 0;
}

void *read_thread(void *arg) {
    int fd = *(int *)arg;
    char buffer[1]; // 存储读取的数据
    char result[204800] = ""; // 存储完整的 JSON 数据，增大缓冲区以避免截断

    while (1) {
        int bytes_read = read_data(fd, buffer, sizeof(buffer));
        if (bytes_read > 0) {
            //printf("Read Thread: Read %d bytes: %s\n", bytes_read, buffer);
            cout << buffer[0] << endl;
            if(buffer[0] == '6'){
		cout<<"收到信号B"<<endl;
            	flag_B = 1;
            }
            else{
            	flag_B = 0;
            }
	    if(buffer[0] == '5'){
	    	cout<<"收到信号A"<<endl;
		flag = 1;
		printf("收到信号，结束读取线程.\n");
		break;
	    }
	    
        } 
	if (bytes_read == 0) {
            printf("No more data available, closing thread.\n");
            break; // 设备关闭，退出循环
        }
        if (bytes_read < -1) {
            // 处理读取错误，打印错误信息
            perror("Read error");
            break; // 退出循环
        }
    }
    
    pthread_exit(NULL);
}

void *write_thread(void *arg) {
    int fd = *(int *)arg;
	//char input[102400]; // 存储用户输入的数据
	//int input = 0;

    while (1) {
    
        //printf("Enter data to write (or 'q' to quit): ");
        //fgets(input, sizeof(input), stdin);

        //if (strcmp(input, "q\n") == 0 || strcmp(input, "Q\n") == 0) {
            // 用户输入 'q' 或 'Q'，退出循环
           // break;
        //}
        char input = 'B';
        if (flag_B == 1){
            input = 'A';
            flag = 0;
        }
        int len = 1;
        //cout<<input<<endl;
        int bytes_written = write_data(fd, &input, len);
        if (bytes_written > 0) {
            //cout << bytes_written <<endl;
            //printf("Write Thread: Wrote %d bytes: %s\n", bytes_written, input);
        }
        if (flag == 1){
            printf("收到信号，结束写入线程.\n");
            break;
        }
    }
    
    pthread_exit(NULL);
}

int main(int argc, char *argv[]) {
    //(void)argc; (void)argv;
    int fd;
    int baud;
    int len;
    //int count;
    int i;
    int databits;
    int stopbits;
    int parity;
    //波特率、数据位、校验位和停止位校准
    if (argc != 6) {
        print_usage(argv[0]);
        return 1;
    }
    //波特率、数据位、校验位和停止位校准
    //atoi函数将命令行参数转换成整数
    baud = atoi(argv[2]);
    if ((baud < 0) || (baud > 4000000)) {//波特率921600
        fprintf(stderr, "Invalid baudrate!\n");
        return 1;
    }
 
    databits = atoi(argv[3]);
    if ((databits < 5) || (databits > 8)) {//数据位
        fprintf(stderr, "Invalid databits!\n");
        return 1;
    }
 
    parity = atoi(argv[4]);
    if ((parity < 0) || (parity > 2)) {//校验位
        fprintf(stderr, "Invalid parity!\n");
        return 1;
    }
 
    stopbits = atoi(argv[5]);
    if ((stopbits < 1) || (stopbits > 2)) {//停止位
        fprintf(stderr, "Invalid stopbits!\n");
        return 1;
    }
    
    //通过 #include <fcntl.h> 头文件引入的,用于打开文件或设备
    //input 要打开的文件或设备的路径名 打开文件的选项标志 设置文件的访问权限 output 文件描述符
    fd = open(argv[1], O_RDWR, 0);
    if (fd < 0) {//如果文件描述符小于 0，表示打开文件失败
        fprintf(stderr, "open <%s> error %s\n", argv[1], strerror(errno));
        return 1;
    }
    //cout<<"11"<<endl;
 
    if (setup_port(fd, baud, databits, parity, stopbits)) {//调用 setup_port 函数来配置打开的串口
        //fprintf(stderr, "setup_port error %s\n", strerror(errno));
        close(fd);
        return 1;
    }
    //cout<<"22"<<endl;
    
    pthread_t read_tid, write_tid;
    
    int ret;
    // 创建读取线程
    ret = pthread_create(&read_tid, NULL, read_thread, &fd);
    if (ret != 0) {
        fprintf(stderr, "Failed to create read thread\n");
        return 1;
    }

    // 创建写入线程
    ret = pthread_create(&write_tid, NULL, write_thread, &fd);
    //cout<<ret<<endl;
    if (ret != 0) {
        fprintf(stderr, "Failed to create write thread\n");
        return 1;
    }
    

    // 等待读取线程和写入线程结束
    pthread_join(read_tid, NULL);
    pthread_join(write_tid, NULL);
    
    while (flag != 1){
    	printf("未检测到电机信号!\n");
    	std::this_thread::sleep_for(std::chrono::seconds(3));
    }
    signal(SIGINT, sig_handle);

    uint8_t type = ORADAR_TYPE_SERIAL;
    int model = ORADAR_MS200;

    OrdlidarDriver device(type, model);
    full_scan_data_st scan_data;

    // Set serial port name based on platform
    #if defined(_WIN32)
    string port_name("com18");
    #else
    string port_name("/dev/ttyACM0");
    #endif

    int serialBaudrate = 230400;
    bool is_logging = true;
    bool ret2 = false;
    long long count = 0;

    device.SetSerialPort(port_name, serialBaudrate);

    // Connect to the lidar device
    while (running) {
        if (device.Connect()) {
            printf("Lidar device connected successfully.\n");
            break;
        } else {
            printf("Failed to connect to lidar device on %s.\n", port_name.c_str());
            sleep(1); // Example delay
        }
    }

    // Initialize the full scan filter
    FullScanFilter filter;
    FilterPara para;
    para.filter_type = FullScanFilter::FS_Intensity;

    // Initialize Open3D window
    visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("Open3D Point Cloud Visualization", 800, 600);
    float v = 2.8;
    float dt = 1.0;
    float z_init = 275.0;
    // Main loop
    // && visualizer.GetWindow()->Running()
    auto view_control = visualizer.GetViewControl();
    std::string timestamp = GetCurrentDateTimeAsString();
    std::string filename = "point_cloud_" + timestamp + ".txt";
    while (running) {
        ret2 = device.GrabFullScanBlocking(scan_data, 1000);
        if (ret2) {
            printf("Count = %lld, Valid points: %d\n", ++count, scan_data.vailtidy_point_num);
            if (is_logging) {
                // Create point cloud
                auto point_cloud = make_shared<geometry::PointCloud>();

                // Populate points from scan data
                for (int i = 0; i < scan_data.vailtidy_point_num; i++) {
                    float angle = scan_data.data[i].angle;
                    if((angle >= 0.0 && angle <= 30.0) || (angle >= 330.0 && angle <= 360.0)){
                    double angle_radians = angle * M_PI / 180.0;
                    float x = scan_data.data[i].distance * sin(angle_radians);
                    float y = v * dt * count;
                    float z = z_init -scan_data.data[i].distance * cos(angle_radians); // Initial z value, adjust as needed
                    //cout<<"angle: "<<angle<<", x: "<<x<<", y: "<<y<<", z: "<<z<<endl;
                    point_cloud->points_.push_back(Eigen::Vector3d(x, y, z));
                    }
                }
                // Save point cloud to a txt file
                ofstream outFile(filename, ios::out | ios::app);
                if (outFile.is_open()) {
                    for (auto& point : point_cloud->points_) {
                        outFile << point.x() << " " << point.y() << " " << point.z() << endl;
                    }
                    outFile.close();
                    //cout << "Point cloud saved to point_cloud.txt" << endl;
                } else {
                    cerr << "Error: Unable to open file for writing." << endl;
                }

                // Update or add point cloud to visualization
                //visualizer.UpdateGeometry();
                visualizer.PollEvents();
                visualizer.AddGeometry(point_cloud);
                
                view_control.Rotate(M_PI / 4, M_PI / 4); // 绕x,y轴旋转45度
                visualizer.UpdateGeometry();
                visualizer.PollEvents();
                
                if(count > 180){
                   printf("over");
                   break;
                }

                
            }
        } else {
            printf("Error: Failed to get full scan data.\n");
        }
    }

    // Clean up lidar device connection
    device.Disconnect();

    return 0;
}

static int baudflag_arr[] = {
    B3500000, B4000000,
    B3000000, B2500000, B2000000,  B1500000, B1152000, B1000000,	
    B921600, B460800, B230400,  B115200,  B57600,   B38400,
    B19200,  B9600,   B4800,    B2400,    B1800,    B1200,
    B600,    B300,    B150,     B110,     B75,      B50
};
static int speed_arr[] = {
    3500000, 4000000,
    3000000, 2500000, 2000000, 1500000, 1152000, 1000000,
    921600,  460800,  230400,  115200,  57600,   38400,
    19200,   9600,    4800,    2400,    1800,    1200,
    600,     300,     150,     110,     75,      50
};

int speed_to_flag(int speed)
{
    int i;
 
    for (i = 0;  i < sizeof(speed_arr)/sizeof(int);  i++) {
        if (speed == speed_arr[i]) {
            return baudflag_arr[i];
        }
    }
 
    fprintf(stderr, "Unsupported baudrate, use 9600 instead!\n");
    return B9600;
}

static struct termio oterm_attr;


//来配置串口参数
//根据传入的参数设置串口的波特率、数据位、校验位和停止位等参数
//input 文件描述符 波特率 数据位 校验位 停止位 output 返回 0 表示配置成功，-1 表示配置失败
int setup_port(int fd, int baud, int databits, int parity, int stopbits)
{
    struct termio term_attr;//声明了一个 struct termio 类型的结构体变量 term_attr，用于存储串口的属性
 
    //获取当前串口的属性，并将其存储在 term_attr 结构体中。如果获取失败，则函数返回 -1
    //input 文件描述符 交互协议 可变参数arg output 执行成功时返回 0，失败则返回 -1 
    //全局变量 errorno 值，在用户空间使用 ioctl 时，可以出错判断以及处理
    //TCGETA 在 <termio.h> 头文件中被定义 用于向操作系统请求获取当前终端设备的设置
    if (ioctl(fd, TCGETA, &term_attr) < 0) {//printf("ioctl: %s\n", strerror(errno));
        return -1;
    }
 
    
    memcpy(&oterm_attr, &term_attr, sizeof(struct termio));
 
    term_attr.c_iflag &= ~(INLCR | IGNCR | ICRNL | ISTRIP);
    term_attr.c_oflag &= ~(OPOST | ONLCR | OCRNL);
    term_attr.c_lflag &= ~(ISIG | ECHO | ICANON | NOFLSH);
    term_attr.c_cflag &= ~CBAUD;
    term_attr.c_cflag |= CREAD | speed_to_flag(baud);
 
    
    term_attr.c_cflag &= ~(CSIZE);
    switch (databits) {
        case 5:
            term_attr.c_cflag |= CS5;
            break;
 
        case 6:
            term_attr.c_cflag |= CS6;
            break;
 
        case 7:
            term_attr.c_cflag |= CS7;
            break;
 
        case 8:
        default:
            term_attr.c_cflag |= CS8;
            break;
    }
 
    
    switch (parity) {
        case 1:  
            term_attr.c_cflag |= (PARENB | PARODD);
            break;
 
        case 2:  
            term_attr.c_cflag |= PARENB;
            term_attr.c_cflag &= ~(PARODD);
            break;
 
        case 0:  
        default:
            term_attr.c_cflag &= ~(PARENB);
            break;
    }
 
 
    
    switch (stopbits) {
        case 2:  
            term_attr.c_cflag |= CSTOPB;
            break;
 
        case 1:  
        default:
            term_attr.c_cflag &= ~CSTOPB;
            break;
    }
 
    term_attr.c_cc[VMIN] = 1;
    term_attr.c_cc[VTIME] = 0;
 
    if (ioctl(fd, TCSETAW, &term_attr) < 0) {
        return -1;
    }
 
    if (ioctl(fd, TCFLSH, 2) < 0) {
        return -1;
    }
 
    return 0;
}
 
 
int read_data(int fd, void *buf, int len)//一次读取1024字节数据
{
    int count;
    int ret;
 
    ret = 0;
    count = 0;
    
    //while (len > 0) {
    //读取数据 #include <unistd.h>头文件引入
    //input 文件描述符，指向要读取数据的对象 指向存储读取数据的缓冲区的指针 要读取的字节数
    //output 如果成功则返回实际读取的字节数，如果到达文件末尾返回 0，如果出现错误返回 -1，并设置全局变量 errno 来指示失败的原因
    ret = read(fd, (char*)buf + count, len); //count字节的偏移量
    
    if (ret < 1) {
        fprintf(stderr, "Read error %s\n", strerror(errno));
        //break;
    }
 
    count += ret;
    len = len - ret;
 
    //}
 
    *((char*)buf + count) = 0;
    return count;
}
 
 
int write_data(int fd, void *buf, int len)
{
    int count;
    int ret;
 
    ret = 0;
    count = 0;
 
    while (len > 0) {
 
        ret = write(fd, (char*)buf + count, len);
        if (ret < 1) {
            fprintf(stderr, "Write error %s\n", strerror(errno));
            break;
        }
 
        count += ret;
        len = len - ret;
    }
 
    return count;
}

void print_usage(char *program_name)
{
    fprintf(stderr,
            "*************************************\n"
            "  A Simple Serial Port Test Utility\n"
            "*************************************\n\n"
            "Usage:\n  %s <device> <baud> <databits> <parity> <stopbits> \n"
            "       databits: 5, 6, 7, 8\n"
            "       parity: 0(None), 1(Odd), 2(Even)\n"
            "       stopbits: 1, 2\n"
            "Example:\n  %s /dev/ttyS4 115200 8 0 1\n\n",
            program_name, program_name
           );
}

