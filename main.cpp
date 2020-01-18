/*
    Data Logger implementation in K64F.
    Inputs:
        x6 Analog Inputs;
        x2 Digital (Frequency) Inputs;
        x1 Internal Accelerometer (FXOS8700);
        x1 External Accelerometer and Gyroscope (LSM6DS3);
    In this set, it is designed for a 200Hz sample rate.
    All the data are saved periodically (every 0.25s) to a folder in the SD card.
    To read the data, use the file "read_struct.c" in the folder results.
    
   Implemented by Diego "sid" Hamilton(Electronics Coordinator 2018) and adapted by Einstein "Hashtag" Gustavo(Electronics Coordinator 2019) 
   at Mangue Baja Team, UFPE.
*/

#include "mbed.h"
#include "LSM6DS3.h"
#include "FXOS8700CQ.h"
#include "SDFileSystem.h"

#define SERIAL_BAUD 1000000                     // Board baud rate
#define BUFFER_SIZE 4000                        // Acquisition buffer
#define SAVE_WHEN 50                           // Number of packets to save (fail safe)
#define SAMPLE_FREQ 200                      // Frequency in Hz

/* Debug */
PwmOut signal_wave(D7);                         // Debug wave to test frequency channels
/* I/O */
Serial pc(USBTX, USBRX, SERIAL_BAUD);           // Debug purposes
SDFileSystem sd(PTE3, PTE1, PTE2, PTE4, "sd");  // MOSI, MISO, SCK, CS
DigitalOut warning(LED1);                         // When device is ready, led is permanently OFF
DigitalOut logging(LED2);                         // When data is beign acquired, led is ON
InterruptIn start(D2, PullUp);                  // Press button to start/stop acquisition
InterruptIn freq_chan1(D0, PullUp);             // Frequency channel 1
InterruptIn freq_chan2(D1, PullUp);             // Frequency channel 2
AnalogIn pot0(A0),
         pot1(A1),
         pot2(A2),
         pot3(A3),
         pot4(A4),
         pot5(A5);
LSM6DS3 LSM6DS3(PTE25,PTE24);                   // External accelerometer and gyroscope
FXOS8700CQ fxos(PTE25,PTE24);                   // Internal accelerometer (and magnetometer, not used)

/* Data structure */
typedef struct
{
    int16_t acclsmx;
    int16_t acclsmy;
    int16_t acclsmz;
    int16_t anglsmx;
    int16_t anglsmy;
    int16_t anglsmz;
    int16_t accfxox;
    int16_t accfxoy;
    int16_t accfxoz;
    uint16_t analog0;
    uint16_t analog1;
    uint16_t analog2;
    uint16_t analog3;
    uint16_t analog4;
    uint16_t analog5;
    uint16_t pulses_chan1;
    uint16_t pulses_chan2;
    uint32_t time_stamp;
} packet_t;

Timer t;                                        // Device timer
Ticker acq;                                     // Acquisition timer interrupt source
CircularBuffer<packet_t, BUFFER_SIZE> buffer;   // Acquisition buffer
int buffer_counter = 0;                         // Packet currently in buffer
bool running = false;                           // Device status
//bool running = true;                          // adaptation
uint16_t pulse_counter1 = 0,
         pulse_counter2 = 0;                    // Frequency counter variables
uint16_t acc_addr = 0;                          /* LSM6DS3 address, if not connected
                                                   address is 0 and data is not stored */

void sampleISR();                               // Data acquisition ISR
uint32_t count_files_in_sd(const char *fsrc);   // Compute number of files in SD
void freq_channel1_ISR();                       // Frequency counter ISR, channel 1
void freq_channel2_ISR();                       // Frequency counter ISR, channel 2
void toggle_logging();                          // Start button ISR

int main()
{
    logging = 0;                                // logging led OFF
    int num_parts = 0,                          // Number of parts already saved
        num_files = 0,                          // Number of files in SD
        svd_pck = 0,                            // Number of saved packets (in current part)
        t1,t2,t3;
    char name_dir[12];                          // Name of current folder (new RUN)
    char name_file[20];                         // Name of current file (partX)
    FILE* fp;                                   
    packet_t temp;
    signal_wave.period_us(50);
    signal_wave.write(0.5f);
    
    /* Initialize accelerometers */
    acc_addr = LSM6DS3.begin(LSM6DS3.G_SCALE_245DPS, LSM6DS3.A_SCALE_2G, \
                             LSM6DS3.G_ODR_208, LSM6DS3.A_ODR_208);
    fxos.init();

    /* Wait for SD ready */
    while(sd.disk_status())
    {
        sd.disk_initialize();
        warning = 0;
        wait(1.0);
        warning = 1;
        wait(1.0);
    }
    
    num_files = count_files_in_sd("/sd");
    sprintf(name_dir, "%s%d", "/sd/RUN", num_files + 1);
    
    start.fall(toggle_logging);                 // Attach start button ISR
    while(!running)                             // Wait button press
        warning = 1;                            // For some reason if this line is empty the code doesn't run

    /* Create RUN directory */
    mkdir(name_dir, 0777);
    warning = 0;
    //sprintf(name_file, "%s%s%d", name_dir, "/part", num_parts++);
    sprintf(name_file, "%s%s%d", name_dir, "/part", num_parts+1);
    fp = fopen(name_file, "a");                 // Creates first data file
    t.start();                                  // Start device timer
    freq_chan1.fall(freq_channel1_ISR);
    freq_chan2.fall(freq_channel2_ISR);
    acq.attach(&sampleISR, 1.0/SAMPLE_FREQ);    // Start data acquisition
    logging = 1;                                // logging led ON
        
    while(running)
    {

        if(buffer.full())
        {
            fclose(fp);
            warning = 1;                        // Turn warning led ON if buffer gets full (abnormal situation)
            pc.putc('X');                       // Debug message
        }
        else if(!buffer.empty())
        {   
            //t1 = t.read_ms();
            pc.putc('G');                       // Debug message
            /* Remove packet from buffer and writes it to file */
            buffer.pop(temp);                   
            buffer_counter--;
            fwrite((void *)&temp, sizeof(packet_t), 1, fp);
            svd_pck++;
            /* Create new data file */
            if(svd_pck == SAVE_WHEN)
            {   
                
                //fclose(fp);
                //sprintf(name_file, "%s%s%d", name_dir, "/part", num_parts++);
                //t2 = t.read_ms();
                //fp = fopen(name_file, "w");
                //printf("t2=%d\r\n",(t.read_ms()-t2));
                //pc.printf("%d\r\n", buffer_counter);  // Debug message
                svd_pck = 0;
            }
            //printf("t1=%d\r\n",(t.read_ms()-t1));
        }
        /* Software debounce for start button */
        if((t.read_ms() > 10) && (t.read_ms() < 200))
            start.fall(toggle_logging);
    }
    
    /* Reset device if start button is pressed while logging */
    fclose(fp);
    logging = 0;
    NVIC_SystemReset();
    return 0;
}

void sampleISR()
{
    static uint16_t last_acq = t.read_ms();     // Time of last acquisition
    packet_t acq_pck;                           // Current data packet
    Data fxos_acc;
    fxos_acc = fxos.get_values();               // Read FXOS8700 data
    /* Store LSM6DS3 data if it's connected */
    if (acc_addr != 0)
    {
        LSM6DS3.readAccel();                    // Read Accelerometer data
        LSM6DS3.readGyro();                     // Read Gyroscope data
        
        acq_pck.acclsmx = LSM6DS3.ax_raw;
        acq_pck.acclsmy = LSM6DS3.ay_raw;   
        acq_pck.acclsmz = LSM6DS3.az_raw;
        acq_pck.anglsmx = LSM6DS3.gx_raw;
        acq_pck.anglsmy = LSM6DS3.gy_raw;
        acq_pck.anglsmz = LSM6DS3.gz_raw;
    }
    else
    {
        acq_pck.acclsmx = 0;
        acq_pck.acclsmy = 0;   
        acq_pck.acclsmz = 0;
        acq_pck.anglsmx = 0;
        acq_pck.anglsmy = 0;
        acq_pck.anglsmz = 0;
    }
    acq_pck.accfxox = fxos_acc.ax;
    acq_pck.accfxoy = fxos_acc.ay;
    acq_pck.accfxoz = fxos_acc.az;
    acq_pck.analog0 = pot0.read_u16();          // Read analog sensor 0            
    acq_pck.analog1 = pot1.read_u16();          // Read analog sensor 1
    acq_pck.analog2 = pot2.read_u16();          // Read analog sensor 2
    acq_pck.analog3 = pot3.read_u16();          // Read analog sensor 3
    acq_pck.analog4 = pot4.read_u16();          // Read analog sensor 4
    acq_pck.analog5 = pot5.read_u16();          // Read analog sensor 5
    acq_pck.pulses_chan1 = pulse_counter1;      // Store frequence channel 1
    acq_pck.pulses_chan2 = pulse_counter2;      // Store frequence channel 2
    acq_pck.time_stamp = t.read_ms();           // Timestamp of data acquistion
    
    pulse_counter1= 0;
    pulse_counter2= 0;
    buffer.push(acq_pck);
    buffer_counter++;
}

uint32_t count_files_in_sd(const char *fsrc)
{   
    DIR *d = opendir(fsrc);
    struct dirent *p;
    uint32_t counter = 0;
    
    while ((p = readdir(d)) != NULL) {
        if(strcmp(p->d_name, ".Trash-1000"))
            counter++;
    }
    closedir(d);
    return counter;
}

void freq_channel1_ISR()
{
    pulse_counter1++;
}

void freq_channel2_ISR()
{
    pulse_counter2++;
}

void toggle_logging()
{
    running = !running;
    start.fall(NULL);
}