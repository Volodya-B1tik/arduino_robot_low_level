#define EnB 0 // Мотор левой гусеницы
#define In_4 1 //
#define In_3 2 //
#define In_2 3 //
#define In_1 4 //
#define EnA 5 // Мотор правой гусеницы
//#define pi 3.141592653589793238462643383279

//IMU
unsigned long currentMillis;
unsigned long prevMillis;

//right enco
#define ENC_A_l  19
#define ENC_B_l  18
#define ENC_A_r  2
#define ENC_B_r  3
volatile boolean state_a_l = 0;
volatile boolean state_b_l = 0;
volatile boolean state_a_r = 0;
volatile boolean state_b_r = 0;

// Encoder position
volatile int enc_pos_l = 0;
int enc_pos_prev_l = 0;
int enc_pos_change_l = 0;

volatile int enc_pos_r = 0;
int enc_pos_prev_r = 0;
int enc_pos_change_r = 0;
/////////////////////////////////////////////////

#define trigPin_left 10
#define echoPin_left 11
#define trigPin_mid 6
#define echoPin_mid 7
#define trigPin_right 8
#define echoPin_right  9

// Энкодер правой гусеницы

int duration_left , distance_left, duration_mid , distance_mid, duration_right , distance_right;


//int N=4000;// Мощность двигателя
double gradus; // Градус поворота
float radian; // градус в радианах

int left = 0, mid = 0, right = 0;

int sonar_data[3];
#define DEBUG true
