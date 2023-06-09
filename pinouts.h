#define EnB 0 // Мотор левой гусеницы
#define In_4 1 //
#define In_3 2 //
#define In_2 3 //
#define In_1 4 //
#define EnA 5 // Мотор правой гусеницы
#define pi 3.141592653589793238462643383279

//right enco
#define inB 4
#define inA 5
volatile long pause1    = 25;  // Пауза для борьбы с дребезгом
volatile long lastTurn1 = 0;   // Переменная для хранения времени последнего изменения

volatile int count1 = 0;       // Счетчик оборотов
int actualcount1    = 0;       // Временная переменная определяющая изменение основного счетчика

volatile int state1 = 0;       // Статус одного шага - от 0 до 4 в одну сторону, от 0 до -4 - в другую

volatile int inAValue = 0;   // Переменные хранящие состояние пина, для экономии времени
volatile int inBValue = 0;   // Переменные хранящие состояние пина, для экономии времени


// Left Enco
#define inB1 3
#define inA1 2
volatile long pause    = 25;  // Пауза для борьбы с дребезгом
volatile long lastTurn = 0;   // Переменная для хранения времени последнего изменения

volatile int count = 0;       // Счетчик оборотов
int actualcount    = 0;       // Временная переменная определяющая изменение основного счетчика

volatile int state = 0;       // Статус одного шага - от 0 до 4 в одну сторону, от 0 до -4 - в другую

volatile int inA1Value = 0;   // Переменные хранящие состояние пина, для экономии времени
volatile int inB1Value = 0;   // Переменные хранящие состояние пина, для экономии времени

/////////////////////////////////////////////////

#define trigPin_left 10
#define echoPin_left 11
//int trigPin_mid = 8;
//int echoPin_mid = 9;
//int trigPin_right = 6;
//int echoPin_right = 7;
#define trigPin_mid 6
#define echoPin_mid 7
#define trigPin_right 8
#define echoPin_right  9

// Энкодер правой гусеницы
int counter_right = 0;
int current_state_right;
int pre_state_right;

// Энкодер левой гусеницы
float counter_left = 0;
float current_state_left;
float pre_state_left;

int duration_left , distance_left, duration_mid , distance_mid, duration_right , distance_right;
//int N=4000;// Мощность двигателя
double gradus; // Градус поворота
float radian; // градус в радианах


int sonar_data[3];
#define DEBUG true
